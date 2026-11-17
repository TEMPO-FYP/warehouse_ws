import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def _resolve_controllers_file(base_yaml_path: str, ns: str) -> str:
    """Wrap ros2_controllers.yaml with namespace key when launching into a sub-namespace."""
    if not ns:
        return base_yaml_path
    with open(base_yaml_path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)
    namespaced = {ns: data}
    tmp = tempfile.NamedTemporaryFile(
        mode='w',
        prefix=f'b2_cargo_controllers_{ns}_',
        suffix='.yaml',
        delete=False,
        encoding='utf-8',
    )
    yaml.safe_dump(namespaced, tmp, sort_keys=False)
    tmp.close()
    return tmp.name


def _setup_bringup(context, pkg_desc, pkg_ctrl):
    ns = LaunchConfiguration('namespace').perform(context).strip('/')
    base_controllers_yaml = os.path.join(pkg_ctrl, 'config', 'ros2_controllers.yaml')
    controllers_file = _resolve_controllers_file(base_controllers_yaml, ns)

    spawn_period = float(LaunchConfiguration('spawn_delay').perform(context))
    nav2_period = spawn_period + 10.0

    spawn_launch = TimerAction(
        period=spawn_period,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_desc, 'launch', 'spawn_b2_cargo.launch.py')
                ),
                launch_arguments={
                    'name': LaunchConfiguration('name'),
                    'namespace': ns,
                    'controllers_file': controllers_file,
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
                launch_arguments={'namespace': ns}.items(),
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
                    'namespace': ns,
                    'use_static_map': LaunchConfiguration('use_static_map'),
                }.items(),
                condition=IfCondition(LaunchConfiguration('use_nav2')),
            )
        ],
    )

    return [spawn_launch, control_launch, nav2_launch]


def generate_launch_description():
    """Spawn b2_cargo into a running Gazebo world and start CHAMP control, EKF, and Nav2."""
    pkg_desc = get_package_share_directory('b2_cargo_description')
    pkg_ctrl = get_package_share_directory('b2_cargo_control')

    return LaunchDescription([
        DeclareLaunchArgument('name', default_value='b2_cargo', description='Model name in Gazebo'),
        DeclareLaunchArgument('namespace', default_value='', description='Optional ROS namespace'),
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
        OpaqueFunction(function=_setup_bringup, args=[pkg_desc, pkg_ctrl]),
    ])
