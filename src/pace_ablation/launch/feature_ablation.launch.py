"""Independent fixed-weight feature ablations; reuse PACE mapping and navigation."""
import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


SCENES = {
    'default': ('simulation_bringup.launch.py', 'multi_robots_explore.launch.py',
                'robot_initial_poses.yaml'),
    'maze40': ('maze40_simulation_bringup.launch.py', 'maze40_multi_robots_explore.launch.py',
               'robot_initial_poses_maze40.yaml'),
    'real': ('real_simulation_bringup.launch.py', 'real_multi_robots_explore.launch.py',
             'robot_initial_poses_real.yaml'),
}
MODES = ('full', 'semantic', 'distance', 'gain', 'risk')


def build_experiment(context):
    cfg = context.launch_configurations
    scene, mode = cfg['map_variant'], cfg['ablation_mode']
    if scene not in SCENES or mode not in MODES:
        raise ValueError('Invalid map_variant or ablation_mode')
    capacities = [cfg[f'robot{rid}_battery_capacity_ah'] for rid in range(3)]
    if any(not math.isfinite(float(value)) or float(value) <= 0 for value in capacities):
        raise ValueError('All three battery capacities must be finite and positive (Ah)')
    if cfg['discharge_during_return'].lower() not in ('true', 'false'):
        raise ValueError('discharge_during_return must be true or false')

    sim = get_package_share_directory('simulation_bringup')
    rl = get_package_share_directory('rl_training')
    env_file, explore_file, poses_file = SCENES[scene]
    poses_path = os.path.join(sim, 'config', poses_file)
    with open(poses_path, encoding='utf-8') as handle:
        poses = yaml.safe_load(handle)['robot_poses']

    def include(package_path, filename, arguments=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(package_path, 'launch', filename)),
            launch_arguments=(arguments or {}).items(),
        )

    slams, fuzzy_and_navigators = [], []
    for rid in range(3):
        name = f'robot_{rid}'
        slams.append(include(sim, f'robot_{rid}_slam.launch.py', {
            'use_sim_time': 'true', 'robot_name': name,
            'battery_design_capacity_ah': capacities[rid],
        }))
        fuzzy_and_navigators.append(Node(
            package='pace_ablation', executable='feature_ablation_node',
            name='fuzzy_decision_node', namespace=name, output='screen',
            parameters=[{
                'use_sim_time': True, 'robot_id': rid, 'ablation_mode': mode,
                'weight_extremely_high': 10.0, 'weight_high': 8.0,
                'weight_medium': 5.0, 'weight_low': 2.0,
                'local_map_topic': f'/{name}/map', 'local_map_frame': f'{name}/map',
                'wait_for_rl_weights': False,
            }],
            remappings=[('frontiers_info', '/frontiers_info'),
                        ('/frontier_priority_fuzzy', 'frontier_priority_fuzzy')],
        ))
        nav = {
            'use_sim_time': True, 'target_frame': f'{name}/map',
            'local_map_frame': f'{name}/map', 'global_frame': 'global_map',
            'base_frame': f'{name}/base_link', 'global_map_topic': '/global_map',
            'local_map_topic': f'/{name}/map', 'enable_low_battery_handling': True,
            'discharge_during_return': cfg['discharge_during_return'].lower() == 'true',
            'return_battery_per_meter': 0.0045, 'wait_battery_per_second': 0.0,
            'return_safety_margin_base': 0.04, 'return_safety_margin_gain': 0.20,
            'return_docking_reserve': 0.005, 'low_battery_check_threshold': 0.0,
            'low_battery_threshold': 0.0,
            'home_x': float(poses[name]['x']), 'home_y': float(poses[name]['y']),
            'battery_reset_service': 'reset_battery',
            'battery_discharge_service': 'set_discharge_enabled',
            'low_battery_metrics_topic': 'low_battery_metrics',
            'enable_frontier_disappearance_switch': True, 'frontier_topic': '/frontiers_info',
            'frontier_missing_hold_sec': 2.0, 'frontier_position_tolerance_m': 1.0,
            'frontier_disappearance_ignore_near_goal_m': 3.0,
        }
        # Preserve each original scene's navigator settings. The real-map entry
        # relies on the navigator defaults, just as the original launch does.
        if scene == 'default':
            nav.update(astar_unknown_cost_multiplier=100.0, astar_max_expansions=150000)
        elif scene == 'maze40':
            nav.update(astar_unknown_cost_multiplier=25.0, astar_max_expansions=250000)
        fuzzy_and_navigators.append(Node(
            package='multi_robot_navigation', executable='minimal_task_navigator',
            name='task_point_navigator', namespace=name, output='screen',
            parameters=[nav], remappings=[('/task_point', 'task_point')],
        ))

    exploration = include(sim, explore_file, {
        'robot_count': '3', 'robot_namespace': 'robot_', 'global_frame': 'global_map',
        'metrics_csv_path': cfg['metrics_csv_path'],
        'metrics_summary_path': cfg['metrics_summary_path'],
    })
    # Keep the original global-state and shared-charger nodes, with no actor.
    resource_coordination = include(rl, 'rl_training.launch.py', {
        'robot_count': '3', 'robot_namespace': 'robot_', 'enable_weight_node': 'false',
        'home_pose_path': poses_path,
    })
    goal_selector = Node(
        package='goal_select', executable='multi_robot_goal_selector_node',
        name='multi_robot_goal_selector_node', output='screen',
        parameters=[{'position_tolerance': 1.0, 'startup_warmup_sec': 8.0,
                     'require_all_robots_ready': True}],
    )
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', os.path.join(sim, 'config', 'fuzzy_decision.rviz')],
    )
    return [
        include(sim, env_file),
        TimerAction(period=float(cfg['slam_delay_sec']), actions=slams),
        TimerAction(period=float(cfg['multi_explore_delay_sec']), actions=[exploration]),
        TimerAction(period=float(cfg['fuzzy_delay_sec']), actions=fuzzy_and_navigators),
        TimerAction(period=float(cfg['rl_delay_sec']), actions=[resource_coordination]),
        TimerAction(period=float(cfg['goal_delay_sec']), actions=[goal_selector, rviz]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_variant', default_value='default', choices=list(SCENES)),
        DeclareLaunchArgument('ablation_mode', default_value='full', choices=list(MODES)),
        # Explicit capacities avoid accidentally comparing different energy budgets.
        DeclareLaunchArgument('robot0_battery_capacity_ah'),
        DeclareLaunchArgument('robot1_battery_capacity_ah'),
        DeclareLaunchArgument('robot2_battery_capacity_ah'),
        DeclareLaunchArgument('discharge_during_return', default_value='false'),
        DeclareLaunchArgument('metrics_csv_path'),
        DeclareLaunchArgument('metrics_summary_path'),
        DeclareLaunchArgument('slam_delay_sec', default_value='6.0'),
        DeclareLaunchArgument('multi_explore_delay_sec', default_value='26.0'),
        DeclareLaunchArgument('fuzzy_delay_sec', default_value='34.0'),
        DeclareLaunchArgument('rl_delay_sec', default_value='36.0'),
        DeclareLaunchArgument('goal_delay_sec', default_value='40.0'),
        OpaqueFunction(function=build_experiment),
    ])
