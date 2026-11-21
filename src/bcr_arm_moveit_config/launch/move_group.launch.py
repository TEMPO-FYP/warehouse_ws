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

"""Launch MoveIt 2 move_group for the BCR 7-DOF manipulator."""

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
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        namespace=ns,
        name='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {
                'use_sim_time': use_sim_time,
                'publish_planning_scene': True,
                'publish_geometry_updates': True,
                'publish_state_updates': True,
                'publish_transforms_updates': True,
                'trajectory_execution.allowed_execution_duration_scaling': 2.0,
                'trajectory_execution.allowed_goal_duration_margin': 1.0,
                'trajectory_execution.allowed_start_tolerance': 0.05,
            },
        ],
    )

    return [move_group_node]


def generate_launch_description():
    """Launch namespaced MoveIt 2 move_group for bcr_arm."""
    pkg_desc = get_package_share_directory('bcr_arm_description')
    urdf_xacro_path = os.path.join(pkg_desc, 'xacro', 'bcr_arm.urdf.xacro')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='bcr_arm',
            description='ROS namespace for move_group',
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
        OpaqueFunction(function=_launch_setup, args=[urdf_xacro_path]),
    ])
