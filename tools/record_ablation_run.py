#!/usr/bin/env python3
"""Validate and record a fixed-weight ablation before starting Gazebo."""
import ast
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys


def main():
    run_dir, scene, mode, *arguments = sys.argv[1:]
    if scene not in ('default', 'maze40', 'real') or mode not in (
            'full', 'semantic', 'distance', 'gain', 'risk'):
        raise SystemExit('Invalid scene or ablation mode')
    overrides = {}
    for argument in arguments:
        if ':=' not in argument:
            raise SystemExit(f'Expected a launch argument of the form name:=value: {argument}')
        key, value = argument.split(':=', 1)
        if key in overrides:
            raise SystemExit(f'Duplicate launch argument is not allowed: {key}')
        overrides[key] = value
    root = Path(__file__).resolve().parents[1]
    launch = root / 'src/pace_ablation/launch/feature_ablation.launch.py'
    defaults, allowed = {}, set()
    for node in ast.walk(ast.parse(launch.read_text())):
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
                and node.func.id == 'DeclareLaunchArgument'):
            key = node.args[0].value
            allowed.add(key)
            for kw in node.keywords:
                if kw.arg == 'default_value' and isinstance(kw.value, ast.Constant):
                    defaults[key] = kw.value.value
    unknown = set(overrides) - allowed
    if unknown:
        raise SystemExit(f'Unknown launch arguments: {sorted(unknown)}')
    effective = {**defaults, **overrides}
    for rid in range(3):
        key = f'robot{rid}_battery_capacity_ah'
        try:
            value = float(effective[key])
        except (KeyError, ValueError):
            raise SystemExit(f'Provide a positive capacity in Ah: {key}:=VALUE')
        if not math.isfinite(value) or value <= 0:
            raise SystemExit(f'Capacity must be finite and positive: {key}')
    if effective['discharge_during_return'] not in ('true', 'false'):
        raise SystemExit('discharge_during_return must be true or false')
    for key in ('slam_delay_sec', 'multi_explore_delay_sec', 'fuzzy_delay_sec',
                'rl_delay_sec', 'goal_delay_sec'):
        value = float(effective[key])
        if not math.isfinite(value) or value < 0:
            raise SystemExit(f'Delay must be finite and nonnegative: {key}')
    def git(*args):
        return subprocess.run(['git', '-C', str(root), *args], capture_output=True,
                              text=True, check=False).stdout.strip()
    files = ('src/pace_ablation/src/feature_ablation_node.cpp',
             'src/pace_ablation/launch/feature_ablation.launch.py',
             'src/fuzzy_decision/src/fuzzy_decision_node.cpp')
    record = {
        'timestamp_utc': datetime.now(timezone.utc).isoformat(),
        'git_revision': git('rev-parse', 'HEAD') or None,
        'git_dirty': bool(git('status', '--porcelain')),
        'map': scene, 'ablation_mode': mode,
        'weight_mode': 'fixed', 'weights': [0.25] * 4,
        'mappo_inference_enabled': False, 'model_path': None,
        'reference_values': [0, 0.25, 0.5, 0.75, 1] if mode != 'full' else [],
        'aggregation': 'mean_of_final_scores' if mode != 'full' else 'original_score',
        'effective_arguments': effective,
        'source_sha256': {name: hashlib.sha256((root / name).read_bytes()).hexdigest()
                          for name in files},
        'command': ['ros2', 'launch', 'pace_ablation', 'feature_ablation.launch.py', *arguments],
        'scope': 'Frontier scoring only; navigation, feasibility filtering and energy safety remain enabled.',
    }
    target = Path(run_dir)
    target.mkdir(parents=True, exist_ok=True)
    (target / 'run_config.json').write_text(json.dumps(record, indent=2) + '\n')


if __name__ == '__main__':
    main()
