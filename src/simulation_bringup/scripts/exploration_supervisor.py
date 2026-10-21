#!/usr/bin/env python3
import csv
import json
import math
import os
import time
from pathlib import Path

import rclpy
from frontier_msgs.msg import FrontierArrayExtended
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Float32MultiArray, String


class ExplorationSupervisor(Node):
    def __init__(self):
        super().__init__('exploration_supervisor')
        self.declare_parameter('robot_count', 3)
        self.declare_parameter('global_map_topic', '/global_map')
        self.declare_parameter('frontier_topic', '/frontiers_info')
        self.declare_parameter('stop_topic', '/exploration_stop')
        self.declare_parameter('coverage_target', 0.95)
        self.declare_parameter('max_mission_time_sec', 0.0)
        self.declare_parameter('no_frontier_hold_sec', 10.0)
        self.declare_parameter('check_period_sec', 1.0)
        self.declare_parameter('roi_min_x', -17.4)
        self.declare_parameter('roi_min_y', -17.4)
        self.declare_parameter('roi_max_x', 17.6)
        self.declare_parameter('roi_max_y', 17.6)
        self.declare_parameter('static_map_yaml', '')
        self.declare_parameter('static_map_count_mode', 'free_only')
        self.declare_parameter('coverage_total_override', 0)
        self.declare_parameter('max_odom_step_m', 1.0)
        self.declare_parameter('csv_path', os.path.join(os.getcwd(), 'data', 'metrics', 'energy_mission_metrics.csv'))
        self.declare_parameter('summary_path', os.path.join(os.getcwd(), 'data', 'metrics', 'energy_mission_summary.json'))
        self.declare_parameter('coverage_timeseries_path', '')
        self.declare_parameter('battery_timeseries_path', '')
        self.declare_parameter('robot_timeseries_path', '')
        self.declare_parameter('charging_events_path', '')
        self.declare_parameter('return_events_path', '')
        self.declare_parameter('task_events_path', '')

        self.robot_count = int(self.get_parameter('robot_count').value)
        self.coverage_target = float(self.get_parameter('coverage_target').value)
        self.max_mission_time_sec = float(self.get_parameter('max_mission_time_sec').value)
        self.no_frontier_hold_sec = float(self.get_parameter('no_frontier_hold_sec').value)
        self.check_period_sec = float(self.get_parameter('check_period_sec').value)
        self.roi_min_x = float(self.get_parameter('roi_min_x').value)
        self.roi_min_y = float(self.get_parameter('roi_min_y').value)
        self.roi_max_x = float(self.get_parameter('roi_max_x').value)
        self.roi_max_y = float(self.get_parameter('roi_max_y').value)
        self.static_map_yaml = str(self.get_parameter('static_map_yaml').value).strip()
        self.static_map_count_mode = str(self.get_parameter('static_map_count_mode').value).strip().lower()
        self.coverage_total_override = int(self.get_parameter('coverage_total_override').value)
        self.max_odom_step_m = float(self.get_parameter('max_odom_step_m').value)
        self.csv_path = str(self.get_parameter('csv_path').value)
        self.summary_path = str(self.get_parameter('summary_path').value)
        self.coverage_timeseries_path = str(self.get_parameter('coverage_timeseries_path').value).strip()
        if not self.coverage_timeseries_path:
            self.coverage_timeseries_path = str(Path(self.csv_path).with_name('coverage_timeseries.csv'))
        self.battery_timeseries_path = str(self.get_parameter('battery_timeseries_path').value).strip()
        if not self.battery_timeseries_path:
            self.battery_timeseries_path = str(Path(self.csv_path).with_name('battery_timeseries.csv'))
        self.robot_timeseries_path = str(self.get_parameter('robot_timeseries_path').value).strip()
        if not self.robot_timeseries_path:
            self.robot_timeseries_path = str(Path(self.csv_path).with_name('robot_timeseries.csv'))
        self.charging_events_path = str(self.get_parameter('charging_events_path').value).strip()
        if not self.charging_events_path:
            self.charging_events_path = str(Path(self.csv_path).with_name('charging_events.jsonl'))
        self.return_events_path = str(self.get_parameter('return_events_path').value).strip()
        if not self.return_events_path:
            self.return_events_path = str(Path(self.csv_path).with_name('return_events.jsonl'))
        self.task_events_path = str(self.get_parameter('task_events_path').value).strip()
        if not self.task_events_path:
            self.task_events_path = str(Path(self.csv_path).with_name('task_events.jsonl'))

        self.map_msg = None
        self.static_map_meta = None
        self._coverage_roi_indices = None
        self._coverage_roi_total = None
        self.start_time = time.monotonic()
        self.stopped = False
        self.frontier_count = None
        self.last_frontier_msg_time = self.start_time
        self.no_frontier_since = None
        self.t90_sec = None
        self.t95_sec = None
        self.path_lengths = [0.0] * self.robot_count
        self.last_odom_xy = [None] * self.robot_count
        self.failed = [False] * self.robot_count
        self.failure_time_sec = [None] * self.robot_count
        self.final_battery = [None] * self.robot_count
        self.current_battery = [None] * self.robot_count
        self.initial_battery = [None] * self.robot_count
        self.min_battery = [None] * self.robot_count
        self.battery_capacity_ah = [None] * self.robot_count
        self.previous_battery = [None] * self.robot_count
        self.consumed_ratio = [0.0] * self.robot_count
        self.charged_ratio = [0.0] * self.robot_count
        self.consumed_ah = [0.0] * self.robot_count
        self.charged_ah = [0.0] * self.robot_count
        self.mode_by_robot = ['EXPLORING'] * self.robot_count
        self.mode_time_sec = [dict() for _ in range(self.robot_count)]
        self.exploration_path_m = [0.0] * self.robot_count
        self.return_path_m = [0.0] * self.robot_count
        self.low_battery_metrics = [dict() for _ in range(self.robot_count)]
        self.last_low_battery_event = [''] * self.robot_count
        self.task_event_counts = [dict() for _ in range(self.robot_count)]
        self.charger_status = {}
        self.last_charger_signature = None
        self.charger_occupied_time_sec = 0.0
        self.charger_queue_time_sec = 0.0
        self.max_charger_queue_length = 0
        self.charger_contention_samples = 0
        self.charger_status_samples = 0
        self.last_metrics_tick = time.monotonic()
        self.weight_stats = [self._new_weight_stats() for _ in range(self.robot_count)]

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        stop_qos = QoSProfile(depth=1)
        stop_qos.reliability = ReliabilityPolicy.RELIABLE
        stop_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.create_subscription(
            OccupancyGrid, str(self.get_parameter('global_map_topic').value), self.map_callback, map_qos)
        self.create_subscription(
            FrontierArrayExtended, str(self.get_parameter('frontier_topic').value), self.frontier_callback, 10)
        for rid in range(self.robot_count):
            self.create_subscription(
                Odometry, f'/robot_{rid}/odom',
                lambda msg, robot_id=rid: self.odom_callback(msg, robot_id), 20)
            self.create_subscription(
                Bool, f'/robot_{rid}/failed',
                lambda msg, robot_id=rid: self.failed_callback(msg, robot_id),
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.create_subscription(
                BatteryState, f'/robot_{rid}/battery_state',
                lambda msg, robot_id=rid: self.battery_callback(msg, robot_id), 10)
            self.create_subscription(
                String, f'/robot_{rid}/low_battery_metrics',
                lambda msg, robot_id=rid: self.low_battery_callback(msg, robot_id), 20)
            self.create_subscription(
                String, f'/robot_{rid}/task_metrics',
                lambda msg, robot_id=rid: self.task_event_callback(msg, robot_id), 20)
            self.create_subscription(
                Float32MultiArray, f'/robot_{rid}/rl_adjust_weights',
                lambda msg, robot_id=rid: self.weight_callback(msg, robot_id), 20)

        self.create_subscription(String, '/charging_station/status', self.charger_status_callback, 20)

        self.stop_pub = self.create_publisher(
            Bool, str(self.get_parameter('stop_topic').value), stop_qos)
        self.timer = self.create_timer(self.check_period_sec, self.on_timer)
        max_time_desc = 'disabled' if self.max_mission_time_sec <= 0.0 else f'{self.max_mission_time_sec:.1f}s'
        self.get_logger().info(
            f'Energy mission supervisor started: target={self.coverage_target:.2f}, '
            f'Tmax={max_time_desc}')
        self._try_load_static_map()

    def map_callback(self, msg):
        self.map_msg = msg
        if self.static_map_meta is not None and self._coverage_roi_indices is None:
            self._build_static_roi_mask()

    @staticmethod
    def _read_simple_yaml_map(path):
        data = {}
        for raw in path.read_text(encoding='utf-8').splitlines():
            line = raw.strip()
            if not line or line.startswith('#') or ':' not in line:
                continue
            key, value = line.split(':', 1)
            data[key.strip()] = value.strip()
        image = data.get('image', '').strip().strip('"').strip("'")
        resolution = float(data.get('resolution', '0.05'))
        origin_text = data.get('origin', '[0.0, 0.0, 0.0]').strip()
        negate = int(float(data.get('negate', '0')))
        occupied_thresh = float(data.get('occupied_thresh', '0.65'))
        free_thresh = float(data.get('free_thresh', '0.196'))
        if not image:
            raise RuntimeError(f'Invalid map yaml, missing image: {path}')
        if not (origin_text.startswith('[') and origin_text.endswith(']')):
            raise RuntimeError(f'Invalid origin in map yaml: {path}')
        origin_vals = [float(x.strip()) for x in origin_text[1:-1].split(',')]
        if len(origin_vals) < 2:
            raise RuntimeError(f'Invalid origin in map yaml: {path}')
        return {
            'image_path': (path.parent / image).resolve(),
            'resolution': resolution,
            'origin_x': origin_vals[0],
            'origin_y': origin_vals[1],
            'negate': negate,
            'occupied_thresh': occupied_thresh,
            'free_thresh': free_thresh,
        }

    @staticmethod
    def _read_pgm(path):
        with open(path, 'rb') as handle:
            magic = handle.readline().strip()
            if magic not in (b'P5', b'P2'):
                raise RuntimeError(f'Unsupported PGM format: {magic!r} ({path})')

            def tokens():
                while True:
                    line = handle.readline()
                    if not line:
                        return
                    line = line.strip()
                    if not line or line.startswith(b'#'):
                        continue
                    for token in line.split():
                        yield token

            token_iter = tokens()
            width = int(next(token_iter))
            height = int(next(token_iter))
            maxval = int(next(token_iter))
            if magic == b'P5':
                raw = handle.read(width * height)
                if len(raw) != width * height:
                    raise RuntimeError(f'PGM size mismatch: {path}')
                pixels = list(raw)
            else:
                rest = list(token_iter)
                if len(rest) != width * height:
                    raise RuntimeError(f'PGM size mismatch: {path}')
                pixels = [int(token) for token in rest]
        return width, height, maxval, pixels

    def _try_load_static_map(self):
        if not self.static_map_yaml:
            return
        yaml_path = Path(self.static_map_yaml).expanduser().resolve()
        if not yaml_path.exists():
            self.get_logger().warn(f'static_map_yaml not found: {yaml_path}')
            return
        try:
            meta = self._read_simple_yaml_map(yaml_path)
            width, height, maxval, pixels = self._read_pgm(meta['image_path'])
        except Exception as exc:
            self.get_logger().warn(f'Failed to load static map yaml/image: {exc}')
            return
        meta.update({'width': width, 'height': height, 'maxval': maxval, 'pixels': pixels})
        self.static_map_meta = meta
        self.get_logger().info(
            f'Loaded static map for fixed coverage denominator: yaml={yaml_path} '
            f'image={meta["image_path"]} size={width}x{height} '
            f'res={meta["resolution"]:.3f} mode={self.static_map_count_mode}')

    def _static_cell_state(self, sx, sy):
        meta = self.static_map_meta
        if meta is None or sx < 0 or sx >= meta['width'] or sy < 0 or sy >= meta['height']:
            return None
        pixel = meta['pixels'][sy * meta['width'] + sx]
        occ = float(pixel) / float(meta['maxval'])
        if meta['negate'] == 0:
            occ = 1.0 - occ
        if occ > meta['occupied_thresh']:
            return 'occupied'
        if occ < meta['free_thresh']:
            return 'free'
        return 'unknown'

    @staticmethod
    def world_to_grid(map_msg, wx, wy):
        res = float(map_msg.info.resolution)
        ox = float(map_msg.info.origin.position.x)
        oy = float(map_msg.info.origin.position.y)
        return int(math.floor((wx - ox) / res)), int(math.floor((wy - oy) / res))

    def _build_static_roi_mask(self):
        if self.map_msg is None or self.static_map_meta is None:
            return
        msg = self.map_msg
        width, height = int(msg.info.width), int(msg.info.height)
        gx0, gy0 = self.world_to_grid(msg, self.roi_min_x, self.roi_min_y)
        gx1, gy1 = self.world_to_grid(msg, self.roi_max_x, self.roi_max_y)
        min_gx = max(0, min(gx0, gx1))
        max_gx = min(width - 1, max(gx0, gx1))
        min_gy = max(0, min(gy0, gy1))
        max_gy = min(height - 1, max(gy0, gy1))
        indices = []
        for gy in range(min_gy, max_gy + 1):
            row_offset = gy * width
            wy = msg.info.origin.position.y + (gy + 0.5) * msg.info.resolution
            for gx in range(min_gx, max_gx + 1):
                wx = msg.info.origin.position.x + (gx + 0.5) * msg.info.resolution
                sx = int(math.floor((wx - self.static_map_meta['origin_x']) / self.static_map_meta['resolution']))
                sy = int(math.floor((wy - self.static_map_meta['origin_y']) / self.static_map_meta['resolution']))
                state = self._static_cell_state(sx, sy)
                if self.static_map_count_mode == 'free_only':
                    keep = state == 'free'
                elif self.static_map_count_mode == 'known_non_occupied':
                    keep = state in ('free', 'unknown')
                else:
                    keep = state is not None and state != 'occupied'
                if keep:
                    indices.append(row_offset + gx)
        self._coverage_roi_indices = indices
        self._coverage_roi_total = len(indices)
        self.get_logger().info(
            f'Built static ROI coverage mask: total_explorable_cells={self._coverage_roi_total} '
            f'within ROI=({self.roi_min_x:.2f},{self.roi_min_y:.2f})-({self.roi_max_x:.2f},{self.roi_max_y:.2f})')

    def frontier_callback(self, msg):
        self.frontier_count = len(msg.frontiers_extended)
        self.last_frontier_msg_time = time.monotonic()
        if self.frontier_count > 0:
            self.no_frontier_since = None

    @staticmethod
    def _new_weight_stats():
        return {
            'count': 0, 'sum': [0.0] * 4, 'sum_sq': [0.0] * 4,
            'min': [float('inf')] * 4, 'max': [float('-inf')] * 4,
            'simplex_error_sum': 0.0,
            'battery_bins': {
                'high': {'count': 0, 'sum': [0.0] * 4},
                'medium': {'count': 0, 'sum': [0.0] * 4},
                'low': {'count': 0, 'sum': [0.0] * 4},
            },
        }

    @staticmethod
    def _append_jsonl(path_text, payload):
        path = Path(path_text)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open('a', encoding='utf-8') as handle:
            handle.write(json.dumps(payload, ensure_ascii=False) + '\n')

    @staticmethod
    def _parse_json_message(msg):
        try:
            value = json.loads(msg.data)
            return value if isinstance(value, dict) else None
        except Exception:
            return None

    def low_battery_callback(self, msg, robot_id):
        payload = self._parse_json_message(msg)
        if payload is None:
            return
        self.low_battery_metrics[robot_id] = payload
        event = str(payload.get('event', ''))
        if event and event != self.last_low_battery_event[robot_id]:
            self.last_low_battery_event[robot_id] = event
            self._append_jsonl(self.return_events_path, {
                'recorded_at_unix': time.time(), 'runtime_sec': time.monotonic() - self.start_time,
                'robot_id': robot_id, **payload,
            })

    def task_event_callback(self, msg, robot_id):
        payload = self._parse_json_message(msg)
        if payload is None:
            return
        event = str(payload.get('event', 'unknown'))
        counts = self.task_event_counts[robot_id]
        counts[event] = int(counts.get(event, 0)) + 1
        self._append_jsonl(self.task_events_path, {
            'recorded_at_unix': time.time(), 'runtime_sec': time.monotonic() - self.start_time,
            'robot_id': robot_id, **payload,
        })

    def charger_status_callback(self, msg):
        payload = self._parse_json_message(msg)
        if payload is None:
            return
        waiting = payload.get('waiting', [])
        queue_len = len(waiting) if isinstance(waiting, list) else int(payload.get('waiting_robot_count', 0))
        self.max_charger_queue_length = max(self.max_charger_queue_length, queue_len)
        signature = (
            bool(payload.get('occupied', False)), int(payload.get('owner_robot_id', -1)),
            tuple(waiting) if isinstance(waiting, list) else queue_len,
            tuple(payload.get('reservations', [])),
        )
        if signature != self.last_charger_signature:
            self.last_charger_signature = signature
            self._append_jsonl(self.charging_events_path, {
                'recorded_at_unix': time.time(), 'runtime_sec': time.monotonic() - self.start_time,
                **payload,
            })
        self.charger_status = payload
        modes = payload.get('mode_by_robot', {})
        for rid in range(self.robot_count):
            self.mode_by_robot[rid] = str(modes.get(str(rid), modes.get(rid, self.mode_by_robot[rid])))

    def weight_callback(self, msg, robot_id):
        values = [float(value) for value in msg.data[:4]]
        if len(values) != 4 or not all(math.isfinite(value) for value in values):
            return
        stats = self.weight_stats[robot_id]
        stats['count'] += 1
        stats['simplex_error_sum'] += abs(sum(values) - 1.0)
        battery = self.current_battery[robot_id]
        bin_name = 'high' if battery is not None and battery > 0.70 else (
            'medium' if battery is not None and battery > 0.40 else 'low')
        bin_stats = stats['battery_bins'][bin_name]
        bin_stats['count'] += 1
        for idx, value in enumerate(values):
            stats['sum'][idx] += value
            stats['sum_sq'][idx] += value * value
            stats['min'][idx] = min(stats['min'][idx], value)
            stats['max'][idx] = max(stats['max'][idx], value)
            bin_stats['sum'][idx] += value

    def battery_callback(self, msg, robot_id):
        percentage = float(msg.percentage)
        capacity = float(msg.design_capacity if msg.design_capacity > 0.0 else msg.capacity)
        if math.isfinite(capacity) and capacity > 0.0:
            self.battery_capacity_ah[robot_id] = capacity
        previous = self.previous_battery[robot_id]
        if previous is None:
            self.initial_battery[robot_id] = percentage
            self.min_battery[robot_id] = percentage
        else:
            delta = percentage - previous
            capacity_ah = self.battery_capacity_ah[robot_id]
            if delta < 0.0:
                self.consumed_ratio[robot_id] += -delta
                if capacity_ah is not None:
                    self.consumed_ah[robot_id] += -delta * capacity_ah
            elif delta > 0.02:
                self.charged_ratio[robot_id] += delta
                if capacity_ah is not None:
                    self.charged_ah[robot_id] += delta * capacity_ah
        self.previous_battery[robot_id] = percentage
        self.min_battery[robot_id] = percentage if self.min_battery[robot_id] is None else min(
            self.min_battery[robot_id], percentage)
        self.current_battery[robot_id] = percentage
        self.final_battery[robot_id] = percentage
        if not msg.present:
            self.mark_failed(robot_id)

    def failed_callback(self, msg, robot_id):
        if msg.data:
            self.mark_failed(robot_id)

    def mark_failed(self, robot_id):
        if self.failed[robot_id]:
            return
        self.failed[robot_id] = True
        self.failure_time_sec[robot_id] = time.monotonic() - self.start_time
        self.get_logger().warn(
            f'robot_{robot_id} failed at {self.failure_time_sec[robot_id]:.1f}s; '
            f'ATL frozen at {self.path_lengths[robot_id]:.2f}m')

    def odom_callback(self, msg, robot_id):
        xy = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))
        prev = self.last_odom_xy[robot_id]
        self.last_odom_xy[robot_id] = xy
        if prev is None or self.failed[robot_id]:
            return
        dist = math.hypot(xy[0] - prev[0], xy[1] - prev[1])
        if 1e-5 < dist <= self.max_odom_step_m:
            self.path_lengths[robot_id] += dist
            mode = self.mode_by_robot[robot_id].upper()
            if 'RETURN' in mode:
                self.return_path_m[robot_id] += dist
            elif mode not in ('WAITING', 'WAITING_FOR_CHARGER', 'CHARGING', 'FAILED'):
                self.exploration_path_m[robot_id] += dist

    def compute_roi_coverage(self):
        if self.map_msg is None:
            return None, 0, 0
        msg = self.map_msg

        if self.static_map_meta is not None:
            if self._coverage_roi_indices is None:
                self._build_static_roi_mask()
            indices = self._coverage_roi_indices or []
            total = int(self._coverage_roi_total or 0)
            if total <= 0:
                return None, 0, 0
            known = 0
            for idx in indices:
                if 0 <= idx < len(msg.data) and msg.data[idx] != -1:
                    known += 1
            if self.coverage_total_override > 0:
                total = self.coverage_total_override
            return min(1.0, known / max(1, total)), known, total

        res = float(msg.info.resolution)
        ox = float(msg.info.origin.position.x)
        oy = float(msg.info.origin.position.y)
        width, height = int(msg.info.width), int(msg.info.height)
        x0 = max(0, int(math.floor((self.roi_min_x - ox) / res)))
        x1 = min(width - 1, int(math.floor((self.roi_max_x - ox) / res)))
        y0 = max(0, int(math.floor((self.roi_min_y - oy) / res)))
        y1 = min(height - 1, int(math.floor((self.roi_max_y - oy) / res)))
        if x0 > x1 or y0 > y1:
            return None, 0, 0
        known = 0
        total = 0
        for y in range(y0, y1 + 1):
            offset = y * width
            for x in range(x0, x1 + 1):
                total += 1
                known += int(msg.data[offset + x] != -1)
        if self.coverage_total_override > 0:
            total = self.coverage_total_override
        return min(1.0, known / max(1, total)), known, total

    def write_timeseries(self, runtime_sec, coverage, known, total, stop_reason=''):
        path = Path(self.coverage_timeseries_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        fields = ['runtime_sec', 'coverage_ratio', 'known_cells', 'total_roi_cells',
                  'ATL_m', 'NFR', 'frontier_count', 'stop_reason']
        row = {
            'runtime_sec': f'{runtime_sec:.3f}',
            'coverage_ratio': f'{coverage:.6f}',
            'known_cells': known,
            'total_roi_cells': total,
            'ATL_m': f'{sum(self.path_lengths):.3f}',
            'NFR': sum(self.failed),
            'frontier_count': self.frontier_count if self.frontier_count is not None else '',
            'stop_reason': stop_reason,
        }
        new_file = not path.exists()
        with path.open('a', newline='') as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            if new_file:
                writer.writeheader()
            writer.writerow(row)

    def write_battery_timeseries(self, runtime_sec, coverage, stop_reason=''):
        path = Path(self.battery_timeseries_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        robot_fields = [f'robot_{rid}_battery' for rid in range(self.robot_count)]
        fields = ['runtime_sec', 'coverage_ratio', 'mean_battery', 'min_battery', 'max_battery',
                  'NFR', 'stop_reason'] + robot_fields
        known_batteries = [battery for battery in self.current_battery if battery is not None]
        row = {
            'runtime_sec': f'{runtime_sec:.3f}',
            'coverage_ratio': f'{coverage:.6f}',
            'mean_battery': '' if not known_batteries else f'{sum(known_batteries) / len(known_batteries):.6f}',
            'min_battery': '' if not known_batteries else f'{min(known_batteries):.6f}',
            'max_battery': '' if not known_batteries else f'{max(known_batteries):.6f}',
            'NFR': sum(self.failed),
            'stop_reason': stop_reason,
        }
        for rid in range(self.robot_count):
            battery = self.current_battery[rid]
            row[f'robot_{rid}_battery'] = '' if battery is None else f'{battery:.6f}'
        new_file = not path.exists()
        with path.open('a', newline='') as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            if new_file:
                writer.writeheader()
            writer.writerow(row)

    def write_robot_timeseries(self, runtime_sec, coverage, stop_reason=''):
        path = Path(self.robot_timeseries_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        fields = [
            'runtime_sec', 'robot_id', 'coverage_ratio', 'battery', 'mode',
            'trajectory_m', 'exploration_path_m', 'return_path_m',
            'consumed_ratio', 'consumed_ah', 'charged_ratio', 'charged_ah',
            'predicted_return_distance_m', 'predicted_return_energy',
            'predicted_wait_time_sec', 'predicted_wait_energy',
            'predicted_required_energy', 'predicted_safety_slack', 'stop_reason',
        ]
        new_file = not path.exists()
        with path.open('a', newline='') as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            if new_file:
                writer.writeheader()
            for rid in range(self.robot_count):
                metric = self.low_battery_metrics[rid]
                writer.writerow({
                    'runtime_sec': f'{runtime_sec:.3f}', 'robot_id': rid,
                    'coverage_ratio': f'{coverage:.6f}',
                    'battery': '' if self.current_battery[rid] is None else f'{self.current_battery[rid]:.6f}',
                    'mode': self.mode_by_robot[rid],
                    'trajectory_m': f'{self.path_lengths[rid]:.3f}',
                    'exploration_path_m': f'{self.exploration_path_m[rid]:.3f}',
                    'return_path_m': f'{self.return_path_m[rid]:.3f}',
                    'consumed_ratio': f'{self.consumed_ratio[rid]:.6f}',
                    'consumed_ah': f'{self.consumed_ah[rid]:.6f}',
                    'charged_ratio': f'{self.charged_ratio[rid]:.6f}',
                    'charged_ah': f'{self.charged_ah[rid]:.6f}',
                    'predicted_return_distance_m': metric.get('last_estimated_return_distance_m', ''),
                    'predicted_return_energy': metric.get('predicted_return_energy', ''),
                    'predicted_wait_time_sec': metric.get('predicted_wait_time_sec', ''),
                    'predicted_wait_energy': metric.get('predicted_wait_energy', ''),
                    'predicted_required_energy': metric.get('predicted_required_energy', ''),
                    'predicted_safety_slack': metric.get('predicted_safety_slack', ''),
                    'stop_reason': stop_reason,
                })

    def _weight_summary(self, rid):
        stats = self.weight_stats[rid]
        count = int(stats['count'])
        if count <= 0:
            return None
        mean = [value / count for value in stats['sum']]
        std = [math.sqrt(max(0.0, stats['sum_sq'][idx] / count - mean[idx] ** 2)) for idx in range(4)]
        bins = {}
        for name, item in stats['battery_bins'].items():
            n = int(item['count'])
            bins[name] = {'count': n, 'mean': None if n <= 0 else [value / n for value in item['sum']]}
        return {
            'count': count, 'mean': mean, 'std': std,
            'min': stats['min'], 'max': stats['max'],
            'mean_simplex_error': stats['simplex_error_sum'] / count,
            'battery_bins': bins,
        }

    def _update_mode_and_charger_integrals(self, now):
        dt = max(0.0, min(5.0, now - self.last_metrics_tick))
        self.last_metrics_tick = now
        for rid, mode in enumerate(self.mode_by_robot):
            times = self.mode_time_sec[rid]
            times[mode] = float(times.get(mode, 0.0)) + dt
        if self.charger_status:
            self.charger_status_samples += 1
            if bool(self.charger_status.get('occupied', False)):
                self.charger_occupied_time_sec += dt
            waiting = self.charger_status.get('waiting', [])
            queue_len = len(waiting) if isinstance(waiting, list) else int(
                self.charger_status.get('waiting_robot_count', 0))
            self.charger_queue_time_sec += dt * queue_len
            if queue_len > 0 or int(self.charger_status.get('returning_robot_count', 0)) > 1:
                self.charger_contention_samples += 1

    def finish(self, reason, runtime_sec, coverage, known, total):
        if self.stopped:
            return
        self.stopped = True
        total_consumed_ah = sum(self.consumed_ah)
        total_charged_ah = sum(self.charged_ah)
        total_path = sum(self.path_lengths)
        charge_counts = [int(metric.get('charge_count', 0)) for metric in self.low_battery_metrics]
        return_failures = sum(
            counts.get('return_home_failed', 0) + counts.get('return_home_dispatch_failed', 0)
            for counts in self.task_event_counts)
        return_successes = sum(counts.get('return_home_reached', 0) for counts in self.task_event_counts)
        team_mode_times = {}
        for times in self.mode_time_sec:
            for mode, seconds in times.items():
                team_mode_times[mode] = float(team_mode_times.get(mode, 0.0)) + float(seconds)
        total_waiting_time = sum(
            seconds for mode, seconds in team_mode_times.items() if 'WAIT' in mode.upper())
        total_returning_time = sum(
            seconds for mode, seconds in team_mode_times.items() if 'RETURN' in mode.upper())
        total_charging_time = sum(
            seconds for mode, seconds in team_mode_times.items() if 'CHARG' in mode.upper() and 'READY' not in mode.upper())
        payload = {
            'type': 'energy_constrained_mission_summary',
            'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
            'stop_reason': reason,
            'runtime_sec': round(runtime_sec, 3),
            'FCR': round(coverage, 6),
            'T90_sec': None if self.t90_sec is None else round(self.t90_sec, 3),
            'T95_sec': None if self.t95_sec is None else round(self.t95_sec, 3),
            'T90_display': '>Tmax' if self.t90_sec is None else f'{self.t90_sec:.3f}',
            'T95_display': '>Tmax' if self.t95_sec is None else f'{self.t95_sec:.3f}',
            'ATL_m': round(total_path, 3),
            'NFR': int(sum(self.failed)),
            'known_cells': int(known),
            'total_roi_cells': int(total),
            'efficiency_metrics': {
                'coverage_per_second': coverage / max(runtime_sec, 1e-9),
                'coverage_per_meter': coverage / max(total_path, 1e-9),
                'coverage_per_energy_ah': None if total_consumed_ah <= 0.0 else coverage / total_consumed_ah,
                'energy_ah_per_coverage': None if coverage <= 0.0 else total_consumed_ah / coverage,
            },
            'energy_metrics': {
                'team_consumed_ratio_sum': sum(self.consumed_ratio),
                'team_consumed_ah': total_consumed_ah,
                'team_charged_ratio_sum': sum(self.charged_ratio),
                'team_charged_ah': total_charged_ah,
            },
            'charging_metrics': {
                'charge_count_total': sum(charge_counts),
                'return_success_count': return_successes,
                'return_failure_count': return_failures,
                'charger_occupied_time_sec': self.charger_occupied_time_sec,
                'charger_utilization': self.charger_occupied_time_sec / max(runtime_sec, 1e-9),
                'queue_robot_time_sec': self.charger_queue_time_sec,
                'max_queue_length': self.max_charger_queue_length,
                'contention_sample_count': self.charger_contention_samples,
                'team_waiting_time_sec': total_waiting_time,
                'mean_waiting_time_per_charge_sec': total_waiting_time / max(1, sum(charge_counts)),
                'team_returning_time_sec': total_returning_time,
                'team_charging_time_sec': total_charging_time,
                'team_mode_time_sec': team_mode_times,
            },
            'rl_weight_metrics': [self._weight_summary(rid) for rid in range(self.robot_count)],
            'robot_metrics': [
                {
                    'robot_id': rid,
                    'failed': self.failed[rid],
                    'failure_time_sec': self.failure_time_sec[rid],
                    'active_trajectory_length_m': round(self.path_lengths[rid], 3),
                    'exploration_trajectory_length_m': round(self.exploration_path_m[rid], 3),
                    'return_trajectory_length_m': round(self.return_path_m[rid], 3),
                    'final_battery': self.final_battery[rid],
                    'initial_battery': self.initial_battery[rid],
                    'minimum_battery': self.min_battery[rid],
                    'battery_capacity_ah': self.battery_capacity_ah[rid],
                    'consumed_ratio': self.consumed_ratio[rid],
                    'consumed_ah': self.consumed_ah[rid],
                    'charged_ratio': self.charged_ratio[rid],
                    'charged_ah': self.charged_ah[rid],
                    'mode_time_sec': self.mode_time_sec[rid],
                    'low_battery_metrics': self.low_battery_metrics[rid],
                    'task_event_counts': self.task_event_counts[rid],
                }
                for rid in range(self.robot_count)
            ],
        }
        summary_path = Path(self.summary_path)
        summary_path.parent.mkdir(parents=True, exist_ok=True)
        summary_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding='utf-8')
        csv_path = Path(self.csv_path)
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        new_file = not csv_path.exists()
        with csv_path.open('a', newline='') as handle:
            writer = csv.DictWriter(handle, fieldnames=list(payload.keys())[:-1])
            if new_file:
                writer.writeheader()
            writer.writerow({key: value for key, value in payload.items() if key != 'robot_metrics'})
        self.write_timeseries(runtime_sec, coverage, known, total, reason)
        self.write_battery_timeseries(runtime_sec, coverage, reason)
        self.write_robot_timeseries(runtime_sec, coverage, reason)
        stop = Bool()
        stop.data = True
        self.stop_pub.publish(stop)
        self.get_logger().warn(
            f'Mission stopped: reason={reason} FCR={coverage:.3f} ATL={sum(self.path_lengths):.1f}m '
            f'NFR={sum(self.failed)}')

    def on_timer(self):
        if self.stopped:
            return
        now = time.monotonic()
        self._update_mode_and_charger_integrals(now)
        coverage, known, total = self.compute_roi_coverage()
        if coverage is None:
            return
        runtime_sec = now - self.start_time
        if self.t90_sec is None and coverage >= 0.90:
            self.t90_sec = runtime_sec
        if self.t95_sec is None and coverage >= 0.95:
            self.t95_sec = runtime_sec
        self.write_timeseries(runtime_sec, coverage, known, total)
        self.write_battery_timeseries(runtime_sec, coverage)
        self.write_robot_timeseries(runtime_sec, coverage)

        reason = None
        if coverage >= self.coverage_target:
            reason = 'coverage_target_reached'
        elif self.max_mission_time_sec > 0.0 and runtime_sec >= self.max_mission_time_sec:
            reason = 'max_mission_time'
        elif all(self.failed):
            reason = 'all_robots_failed'
        elif self.frontier_count == 0 or now - self.last_frontier_msg_time >= self.no_frontier_hold_sec:
            if self.no_frontier_since is None:
                self.no_frontier_since = now
            elif now - self.no_frontier_since >= self.no_frontier_hold_sec:
                reason = 'no_reachable_frontier'
        if reason:
            self.finish(reason, runtime_sec, coverage, known, total)


def main(args=None):
    rclpy.init(args=args)
    node = ExplorationSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
