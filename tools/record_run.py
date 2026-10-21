#!/usr/bin/env python3
"""Record the launch command, defaults, overrides and checkpoint identity."""
import ast
import hashlib
import json
from pathlib import Path
import subprocess
import sys
from datetime import datetime, timezone


def main():
    run_dir, map_name, model_path, launch_file, *arguments = sys.argv[1:]
    root = Path(__file__).resolve().parents[1]
    model = Path(model_path).resolve()
    launch = root / 'src/simulation_bringup/launch' / launch_file
    defaults = {}
    for node in ast.walk(ast.parse(launch.read_text())):
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
                and node.func.id == 'DeclareLaunchArgument' and node.args
                and isinstance(node.args[0], ast.Constant)):
            for keyword in node.keywords:
                if keyword.arg == 'default_value' and isinstance(keyword.value, ast.Constant):
                    defaults[node.args[0].value] = keyword.value.value
    overrides = dict(arg.split(':=', 1) for arg in arguments if ':=' in arg)
    effective = {**defaults, **overrides}
    revision = subprocess.run(
        ['git', '-C', str(root), 'rev-parse', 'HEAD'],
        capture_output=True, text=True, check=False,
    )
    config_path = model.parent.parent / 'config.json'
    if not config_path.is_file():
        config_path = model.parent / 'config.json'
    record = {
        'timestamp_utc': datetime.now(timezone.utc).isoformat(),
        'git_revision': revision.stdout.strip() or None,
        'map': map_name,
        'model_path': str(model),
        'model_sha256': hashlib.sha256(model.read_bytes()).hexdigest(),
        'launch_defaults': defaults,
        'launch_overrides': overrides,
        'effective_literal_arguments': effective,
        'command': ['ros2', 'launch', 'simulation_bringup', launch_file, *arguments],
        'note': 'Initial SOC must be checked in runtime metrics; capacity is measured in Ah.',
    }
    target = Path(run_dir)
    target.mkdir(parents=True, exist_ok=True)
    (target / 'run_config.json').write_text(json.dumps(record, indent=2) + '\n')
    if config_path.is_file():
        (target / 'model_config.json').write_bytes(config_path.read_bytes())


if __name__ == '__main__':
    main()
