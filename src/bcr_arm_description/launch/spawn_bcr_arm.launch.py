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

"""Launch file for spawning the stationary BCR 7-DOF arm into Gazebo Sim."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _resolve_namespace(raw_ns: str) -> str:
    clean = raw_ns.strip().strip('/')
    return clean or 'bcr_arm'


def _launch_setup(context, xacro_file: str):
    ns = _resolve_namespace(LaunchConfiguration('namespace').perform(context))
    raw_prefix = LaunchConfiguration('prefix').perform(context).strip()
    prefix = raw_prefix if raw_prefix else f'{ns}_'
    use_sim_time = (
        LaunchConfiguration('use_sim_time').perform(context).strip().lower() == 'true'
    )
    controllers_yaml = LaunchConfiguration('controllers_yaml').perform(context).strip()
    use_pedestal = LaunchConfiguration('use_pedestal').perform(context).strip()
    pedestal_height = LaunchConfiguration('pedestal_height').perform(context).strip()

    base_cmd = [
        'xacro ', xacro_file,
        ' namespace:=', ns,
        ' prefix:=', prefix,
        ' use_pedestal:=', use_pedestal,
        ' pedestal_height:=', pedestal_height,
    ]
    if controllers_yaml:
        base_cmd.extend([' controllers_yaml:=', controllers_yaml])

    rsp_description = ParameterValue(
        Command([*base_cmd, ' add_world_joint:=false']), value_type=str
    )
    gz_description = Command([*base_cmd, ' add_world_joint:=true'])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=ns,
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': rsp_description, 'use_sim_time': use_sim_time}],
    )

    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        namespace=ns,
        name='bcr_arm_spawner',
        output='screen',
        arguments=[
            '-name', LaunchConfiguration('name'),
            '-string', gz_description,
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw'),
        ],
    )

    return [
        robot_state_publisher_node,
        spawn_entity_node,
    ]


def generate_launch_description():
    """Spawn bcr_arm into Gazebo and start its namespaced robot_state_publisher."""
    pkg_arm = get_package_share_directory('bcr_arm_description')
    pkg_robotiq = get_package_share_directory('robotiq_description')
    xacro_file = os.path.join(pkg_arm, 'xacro', 'bcr_arm.urdf.xacro')

    arm_parent = os.path.dirname(pkg_arm)
    robotiq_parent = os.path.dirname(pkg_robotiq)
    existing_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join(
            filter(
                None,
                [arm_parent, pkg_arm, robotiq_parent, pkg_robotiq, existing_resource_path],
            )
        ),
    )

    return LaunchDescription([
        gz_resource_path,
        DeclareLaunchArgument(
            'namespace',
            default_value='bcr_arm',
            description='ROS namespace for this robot instance',
        ),
        DeclareLaunchArgument(
            'prefix',
            default_value='',
            description='TF/URDF link prefix (defaults to <namespace>_ when empty)',
        ),
        DeclareLaunchArgument(
            'name',
            default_value='bcr_arm',
            description='Model name in Gazebo',
        ),
        DeclareLaunchArgument(
            'x',
            default_value='1.75',
            description='Spawn X position in map frame',
        ),
        DeclareLaunchArgument(
            'y',
            default_value='13.08',
            description='Spawn Y position in map frame',
        ),
        DeclareLaunchArgument(
            'z',
            default_value='0.0',
            description='Spawn Z position in map frame',
        ),
        DeclareLaunchArgument(
            'yaw',
            default_value='3.14159',
            description='Spawn Yaw rotation in map frame',
        ),
        DeclareLaunchArgument(
            'use_pedestal',
            default_value='true',
            description='Mount bcr_arm on a built-in pedestal link',
        ),
        DeclareLaunchArgument(
            'pedestal_height',
            default_value='0.55',
            description='Height of the built-in pedestal (m)',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock',
        ),
        DeclareLaunchArgument(
            'controllers_yaml',
            default_value='',
            description='ros2_controllers YAML passed into GazeboSimROS2ControlPlugin',
        ),
        OpaqueFunction(function=_launch_setup, args=[xacro_file]),
    ])
