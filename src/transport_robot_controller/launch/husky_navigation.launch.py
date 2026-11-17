"""
Nav2 for the Husky A200 transport robot (a200_0001) in the warehouse.

Starts, under the robot's namespace:
  * Localization: map_server with warehouse_sim/maps/warehouse_map.yaml
    (Gazebo world coordinates) + AMCL with the initial pose preset to the
    spawn pose (config/husky_localization.yaml).
  * Navigation: clearpath_nav2_demos' A200 Nav2 stack (planner, MPPI
    controller, behaviours, bt_navigator -> /a200_0001/navigate_to_pose).

Run after warehouse.launch.py has spawned the robot and started its
platform + sensors (the lidar scan must be flowing for AMCL).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    """Bring up localization on the warehouse map and Nav2 for a200_0001."""
    pkg_self = get_package_share_directory('transport_robot_controller')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_clearpath_nav2 = get_package_share_directory('clearpath_nav2_demos')
    pkg_warehouse_sim = get_package_share_directory('warehouse_sim')

    namespace = LaunchConfiguration('namespace')
    map_yaml = LaunchConfiguration('map')
    setup_path = LaunchConfiguration('setup_path')
    use_sim_time = LaunchConfiguration('use_sim_time')

    args = [
        DeclareLaunchArgument(
            'namespace', default_value='a200_0001',
            description='Robot namespace (must match husky_transport.yaml)'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(pkg_warehouse_sim, 'maps', 'warehouse_map.yaml'),
            description='Map yaml (in Gazebo world coordinates)'),
        DeclareLaunchArgument(
            'setup_path', default_value='/root/clearpath/',
            description='Clearpath setup path containing robot.yaml'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true', choices=['true', 'false'],
            description='Use Gazebo /clock'),
    ]

    # Same structure as clearpath_nav2_demos/launch/localization.launch.py,
    # but with our map and AMCL params (initial pose preset).
    localization = GroupAction([
        PushRosNamespace(namespace),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2_bringup, 'launch', 'localization_launch.py')),
            launch_arguments=[
                ('namespace', namespace),
                ('map', map_yaml),
                ('use_sim_time', use_sim_time),
                ('params_file',
                 os.path.join(pkg_self, 'config', 'husky_localization.yaml')),
                ('autostart', 'true'),
                ('use_composition', 'False'),
                ('use_respawn', 'false'),
            ],
        ),
    ])

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_clearpath_nav2, 'launch', 'nav2.launch.py')),
        launch_arguments=[
            ('setup_path', setup_path),
            ('use_sim_time', use_sim_time),
        ],
    )

    return LaunchDescription(args + [localization, navigation])
