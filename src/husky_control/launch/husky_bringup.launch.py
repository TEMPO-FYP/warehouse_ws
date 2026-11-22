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

"""Orchestrate Husky A200 spawning, Clearpath control, ground-truth odom, and Nav2."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    """Spawn Husky A200 into a running Gazebo world and start Nav2."""
    pkg_husky_control = get_package_share_directory('husky_control')

    nav2_period = PythonExpression([
        "float('", LaunchConfiguration('spawn_delay'), "') + 12.0",
    ])

    spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_husky_control, 'launch', 'spawn_husky.launch.py')
        ),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
            'name': LaunchConfiguration('name'),
            'world_name': LaunchConfiguration('world_name'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'yaw': LaunchConfiguration('yaw'),
            'spawn_delay': LaunchConfiguration('spawn_delay'),
        }.items(),
    )

    nav2_launch = TimerAction(
        period=nav2_period,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_husky_control, 'launch', 'husky_nav2.launch.py')
                ),
                launch_arguments={
                    'namespace': LaunchConfiguration('namespace'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'max_speed': LaunchConfiguration('max_speed'),
                }.items(),
                condition=IfCondition(LaunchConfiguration('use_nav2')),
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'name',
            default_value='a200_0001',
            description='Model name in Gazebo',
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='a200_0001',
            description='ROS namespace for this Husky instance',
        ),
        DeclareLaunchArgument(
            'world_name',
            default_value='default',
            description='Gazebo world name hosting /world/<world_name>/create',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock',
        ),
        DeclareLaunchArgument('x', default_value='0.0', description='Spawn X position'),
        DeclareLaunchArgument('y', default_value='-5.0', description='Spawn Y position'),
        DeclareLaunchArgument('z', default_value='0.15', description='Spawn Z position'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn Yaw rotation'),
        DeclareLaunchArgument(
            'use_nav2',
            default_value='true',
            description='Launch Nav2 navigation stack for Husky',
        ),
        DeclareLaunchArgument(
            'max_speed',
            default_value='1.0',
            description='Max forward speed in m/s',
        ),
        DeclareLaunchArgument(
            'spawn_delay',
            default_value='0.0',
            description='Seconds to wait after /create is ready before spawning Husky',
        ),
        spawn_launch,
        nav2_launch,
    ])
