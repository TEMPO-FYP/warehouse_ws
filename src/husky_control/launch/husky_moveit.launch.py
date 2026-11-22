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

"""Launch namespaced MoveIt 2 move_group for Clearpath Husky manipulators."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def _launch_setup(context):
    setup_path = LaunchConfiguration('setup_path').perform(context).strip()
    ns = LaunchConfiguration('namespace').perform(context).strip().strip('/') or 'a200_0000'
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).strip().lower() == 'true'

    patched_urdf_path = os.path.join(setup_path, 'robot_abs.urdf')
    if os.path.isfile(patched_urdf_path):
        with open(patched_urdf_path, 'r', encoding='utf-8') as f:
            urdf_xml = f.read()
    else:
        urdf_xml = xacro.process_file(
            os.path.join(setup_path, 'robot.urdf.xacro'),
            mappings={
                'is_sim': 'true',
                'namespace': ns,
                'use_manipulation_controllers': 'true',
                'gazebo_controllers': os.path.join(
                    setup_path, 'platform', 'config', 'control.yaml'
                ),
            },
        ).toxml()

    with open(os.path.join(setup_path, 'robot.srdf'), 'r', encoding='utf-8') as f:
        srdf_xml = f.read()

    moveit_yaml = os.path.join(setup_path, 'manipulators', 'config', 'moveit.yaml')

    return [
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            namespace=ns,
            parameters=[
                moveit_yaml,
                {'robot_description': urdf_xml},
                {'robot_description_semantic': srdf_xml},
                {
                    'use_sim_time': use_sim_time,
                    'trajectory_execution.allowed_execution_duration_scaling': 3.0,
                    'trajectory_execution.allowed_goal_duration_margin': 3.0,
                    'trajectory_execution.allowed_start_tolerance': 0.05,
                    'robot_description_kinematics.arm_0.kinematics_solver_timeout': 0.05,
                    'publish_planning_scene': True,
                    'publish_geometry_updates': True,
                    'publish_state_updates': True,
                    'publish_transforms_updates': True,
                },
            ],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
                ('joint_states', 'platform/joint_states'),
            ],
        )
    ]


def generate_launch_description():
    """Create the MoveIt 2 launch description for Clearpath Husky manipulators."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'setup_path',
            default_value='/tmp/clearpath_a200_0000',
            description='Generated Clearpath setup directory',
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='a200_0000',
            description='ROS namespace for this Husky manipulator instance',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=['true', 'false'],
            description='Use simulation clock',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
