# Copyright 2026 TEMPO Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Nav2 navigation stack for Clearpath Husky A200 using shared /map and ground-truth odom."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace
from nav2_common.launch import RewrittenYaml


def _resolve_namespace(raw_ns: str) -> str:
    clean = raw_ns.strip().strip('/')
    return clean or 'a200_0001'


def _build_nav2(context, pkg_nav2_bringup: str, pkg_clearpath_nav2: str):
    ns = _resolve_namespace(LaunchConfiguration('namespace').perform(context))
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).strip()
    max_speed = LaunchConfiguration('max_speed').perform(context).strip()

    scan_topic = f'/{ns}/sensors/lidar2d_0/scan'
    odom_topic = f'/{ns}/ground_truth/odom'

    nav2_params = RewrittenYaml(
        source_file=os.path.join(pkg_clearpath_nav2, 'config', 'a200', 'nav2.yaml'),
        param_rewrites={
            'local_costmap.local_costmap.ros__parameters.voxel_layer.scan.topic': scan_topic,
            'global_costmap.global_costmap.ros__parameters.obstacle_layer.scan.topic': scan_topic,
            'collision_monitor.ros__parameters.scan.topic': scan_topic,
            'controller_server.ros__parameters.FollowPath.motion_model': 'DiffDrive',
            'controller_server.ros__parameters.FollowPath.TwirlingCritic.enabled': 'false',
            'controller_server.ros__parameters.FollowPath.vx_max': max_speed,
            'global_costmap.global_costmap.ros__parameters.rolling_window': 'false',
            'local_costmap.local_costmap.ros__parameters.static_layer.map_topic': '/map',
            'global_costmap.global_costmap.ros__parameters.static_layer.map_topic': '/map',
            'bt_navigator.ros__parameters.odom_topic': odom_topic,
            'controller_server.ros__parameters.odom_topic': odom_topic,
            'velocity_smoother.ros__parameters.odom_topic': odom_topic,
            'bt_navigator.ros__parameters.default_server_timeout': '1000',
            'controller_server.ros__parameters.progress_checker.movement_time_allowance': '30.0',
            'controller_server.ros__parameters.costmap_update_timeout': '2.0',
        },
        convert_types=True,
    )

    navigation = GroupAction([
        PushRosNamespace(ns),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
            ),
            launch_arguments=[
                ('namespace', ns),
                ('use_sim_time', use_sim_time),
                ('params_file', nav2_params),
                ('autostart', 'true'),
                ('use_composition', 'False'),
                ('use_respawn', 'false'),
            ],
        ),
    ])

    return [navigation]


def generate_launch_description():
    """Bring up namespaced Nav2 for Husky A200 against shared /map."""
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_clearpath_nav2 = get_package_share_directory('clearpath_nav2_demos')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='a200_0001',
            description='ROS namespace for this Husky Nav2 stack',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=['true', 'false'],
            description='Use simulation clock',
        ),
        DeclareLaunchArgument(
            'max_speed',
            default_value='1.0',
            description='Max forward speed in m/s',
        ),
        OpaqueFunction(function=_build_nav2, args=[pkg_nav2_bringup, pkg_clearpath_nav2]),
    ])
