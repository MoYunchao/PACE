#!/usr/bin/env python3
"""Logical single-slot charger used by online inference and navigation.

Each robot returns to and waits at its own initial pose, avoiding physical
co-location.  Those poses are separate access points to one shared logical
charging resource.  Arrival is reported by each navigator; this node serializes
access and holds the single slot for a fixed duration.  The navigator resets
its battery as soon as access is granted, while completion releases the
10-second logical hold.
"""
from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Float64, String

try:
    import yaml
except Exception:  # pragma: no cover
    yaml = None


class ChargingStationCoordinator(Node):
    def __init__(self) -> None:
        super().__init__('charging_station_coordinator')
        self.declare_parameter('robot_count', 3)
        self.declare_parameter('robot_namespace', 'robot_')
        self.declare_parameter('pose_path', '')
        self.declare_parameter('hold_duration_sec', 10.0)
        self.declare_parameter('publish_rate_hz', 2.0)

        self.robot_count = int(self.get_parameter('robot_count').value)
        self.robot_namespace = str(self.get_parameter('robot_namespace').value)
        self.hold_duration_sec = max(0.1, float(self.get_parameter('hold_duration_sec').value))
        self.access_points = self._load_access_points(
            str(self.get_parameter('pose_path').value)
        )

        self.owner_id: Optional[int] = None
        self.owner_started = 0.0
        self.owner_completion_sent = False
        self.reservations: List[int] = []
        self.waiting: List[int] = []
        self.mode: Dict[int, str] = {rid: 'EXPLORING' for rid in range(self.robot_count)}
        self.battery: Dict[int, float] = {rid: 1.0 for rid in range(self.robot_count)}
        self.metrics: Dict[int, Dict] = {rid: {} for rid in range(self.robot_count)}

        self.command_pubs = []
        self.wait_pubs = []
        self.available_pubs = []
        for rid in range(self.robot_count):
            name = f'{self.robot_namespace}{rid}'
            self.command_pubs.append(
                self.create_publisher(String, f'/{name}/charge_command', 10)
            )
            self.wait_pubs.append(
                self.create_publisher(Float64, f'/{name}/predicted_charger_wait_sec', 10)
            )
            self.available_pubs.append(
                self.create_publisher(Bool, f'/{name}/available_for_exploration', 10)
            )
            self.create_subscription(
                String, f'/{name}/task_metrics',
                lambda msg, idx=rid: self._task_event(idx, msg), 20)
            self.create_subscription(
                String, f'/{name}/low_battery_metrics',
                lambda msg, idx=rid: self._energy_metrics(idx, msg), 20)
            self.create_subscription(
                BatteryState, f'/{name}/battery_state',
                lambda msg, idx=rid: self._battery(idx, msg), 10)

        self.status_pub = self.create_publisher(String, '/charging_station/status', 10)
        rate = max(0.2, float(self.get_parameter('publish_rate_hz').value))
        self.timer = self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f'Logical charger ready: access_points={self.access_points} '
            f'hold={self.hold_duration_sec:.1f}s capacity=1'
        )

    def _load_access_points(self, raw_path: str) -> Dict[int, Dict[str, float]]:
        if yaml is None or not raw_path:
            return {}
        path = Path(raw_path).expanduser().resolve()
        if not path.is_file():
            self.get_logger().warning(f'Pose file missing: {path}')
            return {}
        with path.open('r', encoding='utf-8') as stream:
            data = yaml.safe_load(stream) or {}
        robot_poses = data.get('robot_poses', {})
        points: Dict[int, Dict[str, float]] = {}
        for rid in range(self.robot_count):
            pose = robot_poses.get(f'{self.robot_namespace}{rid}', {})
            if 'x' in pose and 'y' in pose:
                points[rid] = {'x': float(pose['x']), 'y': float(pose['y'])}
        return points

    @staticmethod
    def _json(raw: str) -> Dict:
        try:
            value = json.loads(raw)
            return value if isinstance(value, dict) else {}
        except Exception:
            return {}

    def _battery(self, rid: int, msg: BatteryState) -> None:
        if msg.present:
            self.battery[int(rid)] = float(max(0.0, min(1.0, msg.percentage)))

    def _energy_metrics(self, rid: int, msg: String) -> None:
        data = self._json(msg.data)
        if data:
            self.metrics[int(rid)] = data

    def _task_event(self, rid: int, msg: String) -> None:
        data = self._json(msg.data)
        event = str(data.get('event', ''))
        rid = int(rid)
        if event in ('return_home_triggered', 'return_home_started'):
            if rid not in self.reservations:
                self.reservations.append(rid)
            self.mode[rid] = 'RETURNING'
        elif event == 'return_home_reached':
            if rid in self.reservations:
                self.reservations.remove(rid)
            if rid not in self.waiting and rid != self.owner_id:
                self.waiting.append(rid)
            self.mode[rid] = 'WAITING'
        elif event == 'charge_hold_started':
            self.mode[rid] = 'CHARGING'
        elif event == 'recharge_completed':
            self._release(rid)
            self.mode[rid] = 'READY'
        elif event in ('recharge_failed', 'return_home_failed'):
            if rid == self.owner_id and event == 'recharge_failed':
                self._release(rid)

    def _priority(self, rid: int):
        slack = float(self.metrics.get(rid, {}).get('predicted_safety_slack', 1.0))
        return (slack, self.battery.get(rid, 1.0), rid)

    def _grant_next(self) -> None:
        if self.owner_id is not None or not self.waiting:
            return
        self.waiting.sort(key=self._priority)
        rid = int(self.waiting.pop(0))
        self.owner_id = rid
        self.owner_started = time.monotonic()
        self.owner_completion_sent = False
        self.mode[rid] = 'CHARGING'
        msg = String()
        msg.data = 'CHARGE_GRANTED'
        self.command_pubs[rid].publish(msg)
        self.get_logger().info(f'Charger granted to robot_{rid}')

    def _release(self, rid: int) -> None:
        if self.owner_id == int(rid):
            self.get_logger().info(f'Charger released by robot_{rid}')
            self.owner_id = None
            self.owner_started = 0.0
            self.owner_completion_sent = False

    def _wait_for(self, rid: int, now: float) -> float:
        remaining = 0.0
        if self.owner_id is not None:
            remaining = max(0.0, self.hold_duration_sec - (now - self.owner_started))
        ahead = 0
        ordered = sorted(self.waiting, key=self._priority)
        if rid in ordered:
            ahead = ordered.index(rid)
        elif rid in self.reservations:
            ahead = len(ordered) + sum(1 for item in self.reservations if item != rid)
        return remaining + ahead * self.hold_duration_sec

    def _tick(self) -> None:
        now = time.monotonic()
        if self.owner_id is not None:
            elapsed = now - self.owner_started
            if elapsed >= self.hold_duration_sec and not self.owner_completion_sent:
                msg = String()
                msg.data = 'CHARGE_COMPLETE'
                self.command_pubs[self.owner_id].publish(msg)
                self.owner_completion_sent = True
        self._grant_next()

        waits: Dict[int, float] = {}
        for rid in range(self.robot_count):
            waits[rid] = self._wait_for(rid, now)
            wait_msg = Float64()
            wait_msg.data = waits[rid]
            self.wait_pubs[rid].publish(wait_msg)
            available = Bool()
            available.data = self.mode.get(rid, 'EXPLORING') in ('EXPLORING', 'READY')
            self.available_pubs[rid].publish(available)

        remaining = 0.0
        if self.owner_id is not None:
            remaining = max(0.0, self.hold_duration_sec - (now - self.owner_started))
        payload = {
            'schema_version': 'logical_charger_v2',
            'access_point_type': 'per_robot_initial_pose',
            'access_point_by_robot': self.access_points,
            'occupied': self.owner_id is not None,
            'owner_robot_id': -1 if self.owner_id is None else self.owner_id,
            'remaining_hold_time_sec': remaining,
            'hold_duration_sec': self.hold_duration_sec,
            'reservations': list(self.reservations),
            'waiting': list(self.waiting),
            'returning_robot_count': len(self.reservations),
            'reserved_robot_count': len(self.reservations),
            'waiting_robot_count': len(self.waiting),
            'charging_robot_count': 0 if self.owner_id is None else 1,
            'estimated_wait_time_by_robot': waits,
            'mode_by_robot': self.mode,
            'battery_by_robot': self.battery,
        }
        status = String()
        status.data = json.dumps(payload, separators=(',', ':'))
        self.status_pub.publish(status)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ChargingStationCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
