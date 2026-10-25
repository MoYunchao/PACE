"""Reuse the existing experiment cleanup to run additional fixed batch plans."""
import argparse
from datetime import datetime
import signal
import time

from run_semantic_maze40_4runs import COMMAND, ROOT, run_once, save_batch


def build_command(scene, mode, capacity):
    command = list(COMMAND)
    command[command.index('--map') + 1] = scene
    command[command.index('--mode') + 1] = mode
    for index, argument in enumerate(command):
        if argument.startswith(('robot0_battery_capacity_ah:=', 'robot1_battery_capacity_ah:=',
                                'robot2_battery_capacity_ah:=')):
            command[index] = argument.split(':=', 1)[0] + ':=' + capacity
    return command


def run_plan(scene, modes, capacity, output_name):
    parser = argparse.ArgumentParser(
        description=f'{scene}: run each selected ablation four times, with 30s pauses.')
    parser.add_argument('--mode', choices=modes,
                        help='Run just this mode; default is all modes in this plan')
    parser.add_argument('--timeout-sec', type=float, default=1800)
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()
    if not 0 < args.timeout_sec < float('inf'):
        parser.error('--timeout-sec must be finite and positive')
    selected = [args.mode] if args.mode else list(modes)
    commands = {mode: build_command(scene, mode, capacity) for mode in selected}
    total = len(selected) * 4
    for mode, command in commands.items():
        print(f'{mode} (4 runs):', ' '.join(command), flush=True)
    print(f'Total: {total} runs; wait 30 seconds after cleanup between runs.', flush=True)
    if args.dry_run:
        return
    if not (ROOT / 'install/setup.bash').is_file():
        parser.error('Build the workspace first (install/setup.bash is missing)')
    model = COMMAND[COMMAND.index('--model') + 1]
    if not (ROOT / model).is_file():
        parser.error(f'Model is missing: {model}')

    def on_terminate(signum, frame):
        raise KeyboardInterrupt
    previous_handler = signal.signal(signal.SIGTERM, on_terminate)
    batch_dir = ROOT / 'output' / output_name / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    batch_dir.mkdir(parents=True)
    manifest = {'map': scene, 'modes': selected, 'battery_capacity_ah': capacity,
                'commands': commands, 'repeats_per_mode': 4, 'pause_seconds': 30,
                'status': 'running', 'runs': []}
    manifest_path = batch_dir / 'batch_summary.json'
    save_batch(manifest_path, manifest)
    print('Batch results:', batch_dir, flush=True)
    try:
        index = 0
        for mode, command in commands.items():
            for repeat in range(1, 5):
                index += 1
                directory = batch_dir / mode / f'run_{repeat:02d}'
                row = {'index': index, 'mode': mode, 'repeat': repeat,
                       'directory': str(directory), 'status': 'running'}
                manifest['runs'].append(row)
                save_batch(manifest_path, manifest)
                print(f'[{index}/{total}] {mode} #{repeat}; log: {directory / "console.log"}', flush=True)
                try:
                    row.update(run_once(command, directory, args.timeout_sec))
                    row['status'] = 'completed'
                except BaseException as exc:
                    row.update(status='interrupted' if isinstance(exc, KeyboardInterrupt) else 'error',
                               error=str(exc) or type(exc).__name__)
                    raise
                finally:
                    save_batch(manifest_path, manifest)
                print(f'[{index}/{total}] Finished: {row["stop_reason"]}; processes stopped.', flush=True)
                if index < total:
                    print('Waiting 30 seconds...', flush=True)
                    time.sleep(30)
        manifest['status'] = 'completed'
    except KeyboardInterrupt:
        manifest['status'] = 'interrupted'
        print('Interrupted; current experiment stopped. No next run will start.', flush=True)
    except Exception as exc:
        manifest['status'] = 'error'
        print(f'Batch stopped: {exc}', flush=True)
    finally:
        save_batch(manifest_path, manifest)
        signal.signal(signal.SIGTERM, previous_handler)
    print('Batch summary:', manifest_path, flush=True)
    if manifest['status'] != 'completed':
        raise SystemExit(1)
