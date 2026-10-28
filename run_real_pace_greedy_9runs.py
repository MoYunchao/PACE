#!/usr/bin/env python3
"""real map: five V5 PACE runs at 0.40 Ah, then four Greedy runs at 0.50 Ah.

Run: python3 -u run_real_pace_greedy_9runs.py
Greedy only: add --method greedy. PACE only: add --method pace.
After a mission summary, stop the current launch and its descendants, then wait
30 seconds before the next run. Aborts on launch error, missing summary timeout,
or failed process cleanup. Ctrl+C stops the current run and the batch.
Results: output/real_pace_greedy_batches/<timestamp>/<method>/run_<nn>/.
"""
import argparse
from datetime import datetime
import signal
import time

from run_semantic_maze40_4runs import ROOT, run_once, save_batch
from run_pace_real_v5_5runs import COMMAND as PACE_COMMAND, MODEL

GREEDY_COMMAND = [
    'bash', 'start_all_greedy.sh', '--map', 'real',
    'robot0_battery_capacity_ah:=0.50', 'robot1_battery_capacity_ah:=0.50',
    'robot2_battery_capacity_ah:=0.50',
]


def build_plan(method):
    plan = []
    if method in ('all', 'pace'):
        plan.extend(('pace', repeat, list(PACE_COMMAND), [0.40] * 3)
                    for repeat in range(1, 6))
    if method in ('all', 'greedy'):
        plan.extend(('greedy', repeat, list(GREEDY_COMMAND), [0.50] * 3)
                    for repeat in range(1, 5))
    return plan


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout-sec', type=float, default=1800,
                        help='Stop the batch if one run has no summary within this time (default: 1800)')
    parser.add_argument('--dry-run', action='store_true', help='Print the selected run plan without starting ROS')
    parser.add_argument('--method', choices=('all', 'pace', 'greedy'), default='all',
                        help='Run both groups, PACE only, or Greedy only')
    args = parser.parse_args()
    if not 0 < args.timeout_sec < float('inf'):
        parser.error('--timeout-sec must be finite and positive')
    plan = build_plan(args.method)
    for index, (method, repeat, command, capacities) in enumerate(plan, 1):
        print(f'[{index}/{len(plan)}] {method} #{repeat}: ' + ' '.join(command), flush=True)
    print('Wait 30 seconds after cleanup between runs.', flush=True)
    if args.dry_run:
        return
    if not (ROOT / 'install/setup.bash').is_file():
        parser.error('Build the workspace first (install/setup.bash is missing)')
    if args.method != 'greedy' and not (ROOT / MODEL).is_file():
        parser.error(f'Model is missing: {MODEL}')
    # A separate session for each launch lets Ctrl+C stop its descendants too.
    def on_terminate(signum, frame):
        raise KeyboardInterrupt
    previous_handler = signal.signal(signal.SIGTERM, on_terminate)
    batch_dir = ROOT / 'output/real_pace_greedy_batches' / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    batch_dir.mkdir(parents=True)
    manifest = {'map': 'real', 'method': args.method, 'pause_seconds': 30,
                'timeout_sec': args.timeout_sec, 'planned_runs': len(plan),
                'status': 'running', 'runs': []}
    manifest_path = batch_dir / 'batch_summary.json'
    save_batch(manifest_path, manifest)
    print('Batch results:', batch_dir, flush=True)
    try:
        for index, (method, repeat, command, capacities) in enumerate(plan, 1):
            directory = batch_dir / method / f'run_{repeat:02d}'
            row = {'index': index, 'method': method, 'repeat': repeat,
                   'command': command, 'battery_capacity_ah': capacities,
                   'model': MODEL if method == 'pace' else None,
                   'directory': str(directory), 'status': 'running'}
            manifest['runs'].append(row)
            save_batch(manifest_path, manifest)
            print(f'[{index}/{len(plan)}] Starting {method} #{repeat}; log: {directory / "console.log"}', flush=True)
            try:
                row.update(run_once(command, directory, args.timeout_sec))
                row['status'] = 'completed'
            except BaseException as exc:
                row.update(status='interrupted' if isinstance(exc, KeyboardInterrupt) else 'error',
                           error=str(exc) or type(exc).__name__)
                raise
            finally:
                save_batch(manifest_path, manifest)
            print(f'[{index}/{len(plan)}] Finished: {row["stop_reason"]}; processes stopped.', flush=True)
            if index < len(plan):
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
