import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def _resolve_namespace(raw_ns: str) -> str:
    clean = raw_ns.strip().strip('/')
    return clean or 'b1z1'


def _build_nav2_nodes(context, default_override_file: str):
    """Build namespaced Nav2 server nodes with rewritten frame names and optional static map override."""
    ns = _resolve_namespace(LaunchConfiguration('namespace').perform(context))
    raw_prefix = LaunchConfiguration('prefix').perform(context).strip()
    prefix = raw_prefix if raw_prefix else f'{ns}_'
    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    autostart = ParameterValue(LaunchConfiguration('autostart'), value_type=bool)
    params_file = LaunchConfiguration('params_file').perform(context)
    use_static_map = (
        LaunchConfiguration('use_static_map').perform(context).strip().lower() == 'true'
    )

    odom_frame = f'{prefix}odom'
    base_frame = f'{prefix}base_footprint'
    scan_topic = f'/{ns}/scan'
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=ns,
        param_rewrites={
            'bt_navigator.ros__parameters.robot_base_frame': base_frame,
            'local_costmap.local_costmap.ros__parameters.global_frame': odom_frame,
            'local_costmap.local_costmap.ros__parameters.robot_base_frame': base_frame,
            'local_costmap.local_costmap.ros__parameters.obstacle_layer.scan.topic': scan_topic,
            'global_costmap.global_costmap.ros__parameters.robot_base_frame': base_frame,
            'global_costmap.global_costmap.ros__parameters.obstacle_layer.scan.topic': scan_topic,
            'behavior_server.ros__parameters.local_frame': odom_frame,
            'behavior_server.ros__parameters.robot_base_frame': base_frame,
        },
        convert_types=True,
    )

    nav_params = [configured_params]
    if use_static_map:
        configured_override = RewrittenYaml(
            source_file=default_override_file,
            root_key=ns,
            param_rewrites={},
            convert_types=True,
        )
        nav_params.append(configured_override)

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

    return [
        GroupAction(
            actions=[
                PushRosNamespace(ns),
                SetParameter('use_sim_time', use_sim_time),
                *nodes,
            ]
        )
    ]


def generate_launch_description():
    """Launch the Nav2 navigation stack configured for Unitree B1Z1."""
    pkg_b1z1_control = get_package_share_directory('b1z1_control')
    default_params_file = os.path.join(pkg_b1z1_control, 'config', 'nav2', 'nav2_params.yaml')
    default_override_file = os.path.join(
        pkg_b1z1_control, 'config', 'nav2', 'nav2_warehouse_override.yaml'
    )

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        DeclareLaunchArgument(
            'namespace',
            default_value='b1z1',
            description='ROS namespace for this robot Nav2 stack',
        ),
        DeclareLaunchArgument(
            'prefix',
            default_value='',
            description='TF link prefix (defaults to <namespace>_ when empty)',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true',
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup the Nav2 lifecycle nodes',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Full path to the Nav2 parameters YAML file',
        ),
        DeclareLaunchArgument(
            'use_static_map',
            default_value='true',
            description='Enable global_costmap static_layer for the shared /map topic',
        ),
        OpaqueFunction(function=_build_nav2_nodes, args=[default_override_file]),
    ])
