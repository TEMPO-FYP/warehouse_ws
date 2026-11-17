import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetParameter
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def _build_nav2_nodes(context, default_override_file):
    """Build Nav2 server nodes with optional namespace and static warehouse map override."""
    ns = LaunchConfiguration('namespace').perform(context).strip('/')
    frame_prefix = f'{ns}/' if ns else ''
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file').perform(context)
    use_static_map = (
        LaunchConfiguration('use_static_map').perform(context).strip().lower() == 'true'
    )

    param_rewrites = {
        'use_sim_time': use_sim_time,
        'robot_base_frame': f'{frame_prefix}base_footprint',
        'local_frame': f'{frame_prefix}odom',
    }

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=ns,
            param_rewrites=param_rewrites,
            convert_types=True,
        ),
        allow_substs=True,
    )
    nav_params = [configured_params]
    if use_static_map:
        configured_override = ParameterFile(
            RewrittenYaml(
                source_file=default_override_file,
                root_key=ns,
                param_rewrites={'use_sim_time': use_sim_time},
                convert_types=True,
            ),
            allow_substs=True,
        )
        nav_params.append(configured_override)

    # Inject per-robot costmap global_frame for odom-based local_costmap
    if frame_prefix:
        nav_params.append({
            'local_costmap': {
                'local_costmap': {
                    'ros__parameters': {
                        'global_frame': f'{frame_prefix}odom',
                        'robot_base_frame': f'{frame_prefix}base_footprint',
                    }
                }
            }
        })

    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'velocity_smoother',
    ]

    nodes = [
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=nav_params,
            remappings=[('cmd_vel', 'cmd_vel_nav')],
        ),
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=nav_params,
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=nav_params,
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=nav_params,
            remappings=[('cmd_vel', 'cmd_vel_nav')],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=nav_params,
        ),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=nav_params,
            remappings=[
                ('cmd_vel', 'cmd_vel_nav'),
                ('cmd_vel_smoothed', 'cmd_vel'),
            ],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[
                {'autostart': autostart},
                {'node_names': lifecycle_nodes},
            ],
        ),
    ]

    group_actions = [SetParameter('use_sim_time', use_sim_time)]
    if ns:
        group_actions.append(PushRosNamespace(ns))
    group_actions.extend(nodes)

    return [GroupAction(actions=group_actions)]


def generate_launch_description():
    """Launch the Nav2 navigation stack configured for b2_cargo."""
    pkg_ctrl = get_package_share_directory('b2_cargo_control')
    default_params_file = os.path.join(pkg_ctrl, 'config', 'nav2', 'nav2_params.yaml')
    default_override_file = os.path.join(pkg_ctrl, 'config', 'nav2', 'nav2_warehouse_override.yaml')

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        DeclareLaunchArgument('namespace', default_value='', description='Optional ROS namespace'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation clock'),
        DeclareLaunchArgument('autostart', default_value='true', description='Autostart Nav2 lifecycle'),
        DeclareLaunchArgument('params_file', default_value=default_params_file, description='Nav2 params YAML'),
        DeclareLaunchArgument('use_static_map', default_value='false', description='Enable shared /map static_layer'),
        OpaqueFunction(function=_build_nav2_nodes, args=[default_override_file]),
    ])
