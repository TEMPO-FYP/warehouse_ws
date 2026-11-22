import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_namespace(raw_ns: str) -> str:
    clean = raw_ns.strip().strip('/')
    return clean or 'b2_cargo'


def _launch_setup(context):
    ns = _resolve_namespace(LaunchConfiguration('namespace').perform(context))
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).strip().lower() == 'true'
    rviz_config = LaunchConfiguration('rviz_config').perform(context).strip()

    arguments = ['-d', rviz_config] if rviz_config else []

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        namespace=ns,
        name='rviz2',
        output='screen',
        arguments=arguments,
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return [rviz_node]


def generate_launch_description():
    """Launch RViz2 configured for b2_cargo TF, RobotModel, LiDAR, and Nav2 visualization."""
    pkg_ctrl = get_package_share_directory('b2_cargo_control')
    default_rviz_config = os.path.join(pkg_ctrl, 'config', 'rviz', 'b2_cargo.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='b2_cargo',
            description='ROS namespace for b2_cargo RViz2',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz_config,
            description='Path to an .rviz config file',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
