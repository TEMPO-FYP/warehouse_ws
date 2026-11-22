import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    """Spawn b2_cargo into a running Gazebo world and start CHAMP control and Nav2."""
    pkg_desc = get_package_share_directory('b2_cargo_description')
    pkg_ctrl = get_package_share_directory('b2_cargo_control')

    default_controllers_yaml = os.path.join(pkg_ctrl, 'config', 'ros2_controllers.yaml')

    spawn_period = PythonExpression(["float('", LaunchConfiguration('spawn_delay'), "')"])
    nav2_period = PythonExpression([
        "float('", LaunchConfiguration('spawn_delay'), "') + 10.0",
    ])

    spawn_launch = TimerAction(
        period=spawn_period,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_desc, 'launch', 'spawn_b2_cargo.launch.py')
                ),
                launch_arguments={
                    'name': LaunchConfiguration('name'),
                    'namespace': LaunchConfiguration('namespace'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'controllers_yaml': LaunchConfiguration('controllers_yaml'),
                    'x': LaunchConfiguration('x'),
                    'y': LaunchConfiguration('y'),
                    'z': LaunchConfiguration('z'),
                    'yaw': LaunchConfiguration('yaw'),
                    'spawn_test_cargo': LaunchConfiguration('spawn_test_cargo'),
                }.items(),
            )
        ],
    )

    control_launch = TimerAction(
        period=spawn_period,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_ctrl, 'launch', 'b2_cargo_control.launch.py')
                ),
                launch_arguments={
                    'namespace': LaunchConfiguration('namespace'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'controllers_yaml': LaunchConfiguration('controllers_yaml'),
                }.items(),
            )
        ],
    )

    nav2_launch = TimerAction(
        period=nav2_period,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_ctrl, 'launch', 'b2_cargo_nav2.launch.py')
                ),
                launch_arguments={
                    'namespace': LaunchConfiguration('namespace'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'use_static_map': LaunchConfiguration('use_static_map'),
                }.items(),
                condition=IfCondition(LaunchConfiguration('use_nav2')),
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'name',
            default_value='b2_cargo',
            description='Model name in Gazebo',
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='b2_cargo',
            description='ROS namespace for this robot instance',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock across spawned nodes',
        ),
        DeclareLaunchArgument(
            'controllers_yaml',
            default_value=default_controllers_yaml,
            description='ros2_controllers YAML passed into GazeboSimROS2ControlPlugin',
        ),
        DeclareLaunchArgument('x', default_value='2.0', description='Spawn X position'),
        DeclareLaunchArgument('y', default_value='16.0', description='Spawn Y position'),
        DeclareLaunchArgument('z', default_value='0.64', description='Spawn Z position'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn Yaw rotation'),
        DeclareLaunchArgument(
            'spawn_test_cargo',
            default_value='false',
            description='Spawn sample small_box payload centered in cargo_rack',
        ),
        DeclareLaunchArgument(
            'use_nav2',
            default_value='true',
            description='Launch Nav2 navigation stack alongside Gazebo and controllers',
        ),
        DeclareLaunchArgument(
            'use_static_map',
            default_value='true',
            description='Enable global_costmap static_layer for the shared /map topic',
        ),
        DeclareLaunchArgument(
            'spawn_delay',
            default_value='0.0',
            description='Seconds to wait before spawning b2_cargo',
        ),
        spawn_launch,
        control_launch,
        nav2_launch,
    ])
