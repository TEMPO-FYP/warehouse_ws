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

"""Launch file to spawn bcr_arm into Gazebo and start ros2_control + MoveIt 2."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    """Spawn bcr_arm into a running Gazebo world and start control and MoveIt 2."""
    pkg_desc = get_package_share_directory('bcr_arm_description')
    pkg_ctrl = get_package_share_directory('bcr_arm_control')
    pkg_moveit = get_package_share_directory('bcr_arm_moveit_config')

    default_controllers_yaml = os.path.join(pkg_ctrl, 'config', 'ros2_controllers.yaml')

    spawn_period = PythonExpression(["float('", LaunchConfiguration('spawn_delay'), "')"])
    move_group_period = PythonExpression([
        "float('", LaunchConfiguration('spawn_delay'), "') + 8.0",
    ])

    spawn_launch = TimerAction(
        period=spawn_period,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_desc, 'launch', 'spawn_bcr_arm.launch.py')
                ),
                launch_arguments={
                    'namespace': LaunchConfiguration('namespace'),
                    'prefix': LaunchConfiguration('prefix'),
                    'name': LaunchConfiguration('name'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'controllers_yaml': LaunchConfiguration('controllers_yaml'),
                    'x': LaunchConfiguration('x'),
                    'y': LaunchConfiguration('y'),
                    'z': LaunchConfiguration('z'),
                    'yaw': LaunchConfiguration('yaw'),
                    'use_pedestal': LaunchConfiguration('use_pedestal'),
                    'pedestal_height': LaunchConfiguration('pedestal_height'),
                }.items(),
            )
        ],
    )

    control_launch = TimerAction(
        period=spawn_period,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_ctrl, 'launch', 'bcr_arm_control.launch.py')
                ),
                launch_arguments={
                    'namespace': LaunchConfiguration('namespace'),
                    'prefix': LaunchConfiguration('prefix'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'x': LaunchConfiguration('x'),
                    'y': LaunchConfiguration('y'),
                    'z': LaunchConfiguration('z'),
                    'yaw': LaunchConfiguration('yaw'),
                }.items(),
            )
        ],
    )

    move_group_launch = TimerAction(
        period=move_group_period,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_moveit, 'launch', 'move_group.launch.py')
                ),
                launch_arguments={
                    'namespace': LaunchConfiguration('namespace'),
                    'prefix': LaunchConfiguration('prefix'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'use_pedestal': LaunchConfiguration('use_pedestal'),
                    'pedestal_height': LaunchConfiguration('pedestal_height'),
                }.items(),
                condition=IfCondition(LaunchConfiguration('use_moveit')),
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='bcr_arm',
            description='ROS namespace for this robot instance',
        ),
        DeclareLaunchArgument(
            'prefix',
            default_value='',
            description='TF/URDF/SRDF link prefix (defaults to <namespace>_ when empty)',
        ),
        DeclareLaunchArgument(
            'name',
            default_value='bcr_arm',
            description='Model name in Gazebo',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock across spawned nodes',
        ),
        DeclareLaunchArgument('x', default_value='1.75', description='Spawn X position'),
        DeclareLaunchArgument('y', default_value='13.08', description='Spawn Y position'),
        DeclareLaunchArgument('z', default_value='0.0', description='Spawn Z position'),
        DeclareLaunchArgument('yaw', default_value='3.14159', description='Spawn Yaw rotation'),
        DeclareLaunchArgument(
            'use_pedestal',
            default_value='true',
            description='Mount bcr_arm on a built-in pedestal link',
        ),
        DeclareLaunchArgument(
            'pedestal_height',
            default_value='0.55',
            description='Built-in pedestal height (m)',
        ),
        DeclareLaunchArgument(
            'use_moveit',
            default_value='true',
            description='Launch MoveIt 2 move_group alongside Gazebo and controllers',
        ),
        DeclareLaunchArgument(
            'spawn_delay',
            default_value='0.0',
            description='Seconds to wait before spawning bcr_arm',
        ),
        DeclareLaunchArgument(
            'controllers_yaml',
            default_value=default_controllers_yaml,
            description='ros2_controllers YAML passed into GazeboSimROS2ControlPlugin',
        ),
        spawn_launch,
        control_launch,
        move_group_launch,
    ])
