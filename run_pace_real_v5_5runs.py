#!/usr/bin/env python3
"""PACE full method: real map, V5 checkpoint, 0.40 Ah per robot, five runs.

Wait for a mission summary, stop this launch and its descendants, then wait
30 seconds before the next run. Results: output/pace_real_v5_batches/.
Usage: python3 -u run_pace_real_v5_5runs.py [--dry-run] [--timeout-sec 1800]
"""
import argparse
from datetime import datetime
import signal
import time

from run_semantic_maze40_4runs import ROOT, run_once, save_batch

MODEL = 'models/inference/mappo/mappo_v5_battery040_090_seed42/models/best_model.pt'
COMMAND = [
    'bash', 'start_all_mappo.sh', '--map', 'real', MODEL,
    'robot0_battery_capacity_ah:=0.40', 'robot1_battery_capacity_ah:=0.40',
    'robot2_battery_capacity_ah:=0.40', 'discharge_during_return:=false',
]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout-sec', type=float, default=1800,
                        help='Stop the batch if one run has no summary within this time (default: 1800)')
    parser.add_argument('--dry-run', action='store_true', help='Print the five-run plan without starting ROS')
    args = parser.parse_args()
    if not 0 < args.timeout_sec < float('inf'):
        parser.error('--timeout-sec must be finite and positive')
    print('Command:', ' '.join(COMMAND), flush=True)
    print('Runs: 5; wait 30 seconds after cleanup between runs.', flush=True)
    if args.dry_run:
        return
    if not (ROOT / 'install/setup.bash').is_file():
        parser.error('Build the workspace first (install/setup.bash is missing)')
    model_path = MODEL
    if not (ROOT / model_path).is_file():
        parser.error(f'Model is missing: {model_path}')
    # A separate session for each launch lets Ctrl+C stop its descendants too.
    def on_terminate(signum, frame):
        raise KeyboardInterrupt
    previous_handler = signal.signal(signal.SIGTERM, on_terminate)
    batch_dir = ROOT / 'output/pace_real_v5_batches' / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    batch_dir.mkdir(parents=True)
    manifest = {'command': COMMAND, 'repeats': 5, 'map': 'real', 'model': MODEL,
                'battery_capacity_ah': [0.40, 0.40, 0.40], 'pause_seconds': 30,
                'status': 'running', 'runs': []}
    manifest_path = batch_dir / 'batch_summary.json'
    save_batch(manifest_path, manifest)
    print('Batch results:', batch_dir, flush=True)
    try:
        for repeat in range(1, 6):
            directory = batch_dir / f'run_{repeat:02d}'
            row = {'repeat': repeat, 'directory': str(directory), 'status': 'running'}
            manifest['runs'].append(row)
            save_batch(manifest_path, manifest)
            print(f'[{repeat}/5] Starting; log: {directory / "console.log"}', flush=True)
            try:
                row.update(run_once(COMMAND, directory, args.timeout_sec))
                row['status'] = 'completed'
            except BaseException as exc:
                row.update(status='interrupted' if isinstance(exc, KeyboardInterrupt) else 'error',
                           error=str(exc) or type(exc).__name__)
                raise
            finally:
                save_batch(manifest_path, manifest)
            print(f'[{repeat}/5] Finished: {row["stop_reason"]}; processes stopped.', flush=True)
            if repeat < 5:
                print('Waiting 30 seconds before the next run...', flush=True)
                time.sleep(30)
        manifest['status'] = 'completed'
    except KeyboardInterrupt:
        manifest['status'] = 'interrupted'
        print('Interrupted; this experiment has been stopped. No next run will start.', flush=True)
    except Exception as exc:
        manifest['status'] = 'error'
        print(f'Batch stopped: {exc}', flush=True)
    finally:
        save_batch(manifest_path, manifest)
        signal.signal(signal.SIGTERM, previous_handler)
    print('Batch summary:', manifest_path, flush=True)
    if manifest['status'] != 'completed':
        raise SystemExit(1)


if __name__ == '__main__':
    main()
