"""Check scene navigation parity without needing a ROS installation."""
import importlib.util
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch

import yaml


ROOT = Path(__file__).resolve().parents[3]


class Action:
    def __init__(self, *args, **kwargs):
        self.args, self.kwargs = args, kwargs


class Configuration:
    def __init__(self, name):
        self.name = name


class Context:
    def __init__(self, values):
        self.launch_configurations = values

    def perform_substitution(self, value):
        return self.launch_configurations[value.name]


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def stubs():
    modules = {name: types.ModuleType(name) for name in (
        'ament_index_python', 'ament_index_python.packages', 'launch', 'launch.actions',
        'launch.launch_description_sources', 'launch.substitutions', 'launch_ros',
        'launch_ros.actions')}
    modules['ament_index_python.packages'].get_package_share_directory = lambda name: str(ROOT / 'src' / name)
    modules['launch'].LaunchDescription = Action
    for name in ('DeclareLaunchArgument', 'IncludeLaunchDescription', 'OpaqueFunction', 'TimerAction'):
        setattr(modules['launch.actions'], name, type(name, (Action,), {}))
    modules['launch.launch_description_sources'].PythonLaunchDescriptionSource = Action
    modules['launch.substitutions'].LaunchConfiguration = Configuration
    modules['launch_ros.actions'].Node = type('Node', (Action,), {})
    return modules


def navigator_parameters(node, context):
    params = node.kwargs['parameters'][0]
    result = {}
    for key, value in params.items():
        if isinstance(value, Configuration):
            value = yaml.safe_load(context.perform_substitution(value))
        result[key] = value
    return result


class LaunchParityTests(unittest.TestCase):
    def test_navigation_and_charger_configuration_match_original_scenes(self):
        with patch.dict(sys.modules, stubs()):
            new = load(ROOT / 'src/pace_ablation/launch/feature_ablation.launch.py', 'ablation_launch')
            for scene in new.SCENES:
                for rid in range(3):
                    with self.subTest(scene=scene, robot=rid):
                        if scene == 'default':
                            filename = f'robot_{rid}_fuzzy_decision.launch.py'
                        else:
                            filename = f'robot_fuzzy_decision_{scene}.launch.py'
                        old = load(ROOT / 'src/simulation_bringup/launch' / filename,
                                   f'original_{scene}_{rid}')
                        values = {
                            'map_variant': scene, 'ablation_mode': 'full',
                            'robot_name': f'robot_{rid}', 'robot_id': str(rid),
                            'home_pose_path': str(ROOT / 'src/simulation_bringup/config' / new.SCENES[scene][2]),
                            'weight_topic': f'/robot_{rid}/rl_adjust_weights',
                            'wait_for_rl_weights': 'false', 'discharge_during_return': 'false',
                            'astar_unknown_cost_multiplier': '100.0', 'astar_max_expansions': '150000',
                            'metrics_csv_path': '/tmp/test_metrics.csv',
                            'metrics_summary_path': '/tmp/test_summary.json',
                            'mappo_model_path': str(ROOT / 'models/inference/mappo/mappo_v5_battery040_090_seed42/models/best_model.pt'),
                            'infer_save_root': '/tmp/test_ablation',
                            'robot0_battery_capacity_ah': '.3', 'robot1_battery_capacity_ah': '.32',
                            'robot2_battery_capacity_ah': '.31', 'slam_delay_sec': '6',
                            'multi_explore_delay_sec': '26', 'fuzzy_delay_sec': '34',
                            'rl_delay_sec': '36', 'goal_delay_sec': '40',
                        }
                        context = Context(values)
                        description = old.generate_launch_description()
                        opaque = next(a for a in description.args[0] if type(a).__name__ == 'OpaqueFunction')
                        old_nodes = opaque.kwargs['function'](context)
                        old_nav = next(n for n in old_nodes if n.kwargs.get('executable') == 'minimal_task_navigator')
                        actions = new.build_experiment(context)
                        timers = [a for a in actions if type(a).__name__ == 'TimerAction']
                        self.assertEqual([t.kwargs['period'] for t in timers], [6, 26, 34, 36, 40])
                        new_nodes = timers[2].kwargs['actions']
                        new_nav = next(n for n in new_nodes if n.kwargs.get('executable') == 'minimal_task_navigator'
                                       and n.kwargs['namespace'] == f'robot_{rid}')
                        self.assertEqual(navigator_parameters(old_nav, context),
                                         navigator_parameters(new_nav, context))
                        self.assertEqual(old_nav.kwargs['remappings'], new_nav.kwargs['remappings'])
                        coordination = timers[3].kwargs['actions'][0]
                        args = dict(coordination.kwargs['launch_arguments'])
                        self.assertEqual(args['enable_weight_node'], 'true')
                        self.assertEqual(args['policy_mode'], 'mappo')
                        self.assertEqual(args['model_path'], values['mappo_model_path'])
                        self.assertEqual(args['weight_topic_template'], '/{robot_name}/rl_adjust_weights')
                        self.assertEqual(args['infer_save_root'], values['infer_save_root'])
                        self.assertEqual(args['home_pose_path'], values['home_pose_path'])
                        scorer = next(n for n in new_nodes if n.kwargs.get('executable') == 'feature_ablation_node'
                                      and n.kwargs['namespace'] == f'robot_{rid}')
                        self.assertTrue(scorer.kwargs['parameters'][0]['wait_for_rl_weights'])
                        self.assertEqual(scorer.kwargs['parameters'][0]['weight_topic'],
                                         f'/robot_{rid}/rl_adjust_weights')


if __name__ == '__main__':
    unittest.main()
