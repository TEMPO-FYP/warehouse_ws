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

"""Launch RViz2 configured for BCR arm MoveIt 2."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def _resolve_namespace(raw_ns: str) -> str:
    clean = raw_ns.strip().strip('/')
    return clean or 'bcr_arm'


def _launch_setup(context, urdf_xacro_path: str):
    ns = _resolve_namespace(LaunchConfiguration('namespace').perform(context))
    raw_prefix = LaunchConfiguration('prefix').perform(context).strip()
    prefix = raw_prefix if raw_prefix else f'{ns}_'
    use_sim_time = (
        LaunchConfiguration('use_sim_time').perform(context).strip().lower() == 'true'
    )
    use_pedestal = LaunchConfiguration('use_pedestal').perform(context).strip()
    pedestal_height = LaunchConfiguration('pedestal_height').perform(context).strip()
    rviz_config = LaunchConfiguration('rviz_config').perform(context).strip()

    moveit_config = (
        MoveItConfigsBuilder('bcr_arm', package_name='bcr_arm_moveit_config')
        .robot_description(
            file_path=urdf_xacro_path,
            mappings={
                'namespace': ns,
                'prefix': prefix,
                'use_pedestal': use_pedestal,
                'pedestal_height': pedestal_height,
                'add_world_joint': 'false',
            },
        )
        .robot_description_semantic(
            file_path='config/bcr_arm.srdf.xacro',
            mappings={'prefix': prefix},
        )
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )

    arguments = ['-d', rviz_config] if rviz_config else []

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        namespace=ns,
        name='rviz2',
        output='screen',
        arguments=arguments,
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {'use_sim_time': use_sim_time},
        ],
    )

    return [rviz_node]


def generate_launch_description():
    """Launch RViz2 with BCR arm MoveIt 2 parameters."""
    pkg_desc = get_package_share_directory('bcr_arm_description')
    urdf_xacro_path = os.path.join(pkg_desc, 'xacro', 'bcr_arm.urdf.xacro')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='bcr_arm',
            description='ROS namespace for MoveIt RViz2',
        ),
        DeclareLaunchArgument(
            'prefix',
            default_value='',
            description='TF/URDF/SRDF link prefix (defaults to <namespace>_ when empty)',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock',
        ),
        DeclareLaunchArgument(
            'use_pedestal',
            default_value='true',
            description='Include built-in pedestal link in MoveIt URDF',
        ),
        DeclareLaunchArgument(
            'pedestal_height',
            default_value='0.55',
            description='Built-in pedestal height (m)',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value='',
            description='Optional path to an .rviz config file',
        ),
        OpaqueFunction(function=_launch_setup, args=[urdf_xacro_path]),
    ])
