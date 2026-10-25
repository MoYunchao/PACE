"""Check the remaining batch plans without starting ROS or waiting between runs."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tools import ablation_batch_plan as plan


class BatchPlanTests(unittest.TestCase):
    def check_plan(self, scene, modes, capacity, selected=None, fail=False):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            (root / 'install').mkdir()
            (root / 'install/setup.bash').touch()
            model = root / plan.COMMAND[plan.COMMAND.index('--model') + 1]
            model.parent.mkdir(parents=True)
            model.touch()
            calls, pauses = [], []
            def run(command, directory, timeout):
                calls.append((list(command), directory))
                if fail:
                    raise TimeoutError('simulated timeout')
                return {'stop_reason': 'coverage_target_reached', 'target_reached': True}
            argv = ['batch'] + (['--mode', selected] if selected else [])
            with patch.object(plan, 'ROOT', root), patch.object(plan, 'run_once', run), \
                    patch.object(plan.time, 'sleep', pauses.append), patch.object(sys, 'argv', argv):
                if fail:
                    with self.assertRaises(SystemExit):
                        plan.run_plan(scene, modes, capacity, 'batch_results')
                else:
                    plan.run_plan(scene, modes, capacity, 'batch_results')
            manifest = json.loads(next((root / 'output').rglob('batch_summary.json')).read_text())
            expected_modes = [selected] if selected else modes
            expected_count = 1 if fail else 4 * len(expected_modes)
            self.assertEqual(len(calls), expected_count)
            self.assertEqual(len(set(directory for _, directory in calls)), expected_count)
            self.assertEqual(pauses, [] if fail else [30] * (expected_count - 1))
            for command, _ in calls:
                self.assertEqual(command[command.index('--map') + 1], scene)
                self.assertEqual(command[command.index('--model') + 1],
                                 plan.COMMAND[plan.COMMAND.index('--model') + 1])
                for rid in range(3):
                    self.assertIn(f'robot{rid}_battery_capacity_ah:={capacity}', command)
            self.assertEqual(manifest['status'], 'error' if fail else 'completed')
            if not fail:
                self.assertEqual([r['mode'] for r in manifest['runs']],
                                 [mode for mode in expected_modes for _ in range(4)])

    def test_maze40_risk_four_runs(self):
        self.check_plan('maze40', ['risk'], '0.40')

    def test_default_four_modes_sixteen_runs(self):
        self.check_plan('default', ['semantic', 'distance', 'gain', 'risk'], '0.30')

    def test_select_one_default_mode(self):
        self.check_plan('default', ['semantic', 'distance', 'gain', 'risk'], '0.30', selected='gain')

    def test_error_stops_before_second_run(self):
        self.check_plan('maze40', ['risk'], '0.40', fail=True)


if __name__ == '__main__':
    unittest.main()
