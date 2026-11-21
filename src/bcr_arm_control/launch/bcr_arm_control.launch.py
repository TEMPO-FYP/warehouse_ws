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

"""Launch file for BCR arm static TF placement and ros2_control spawners."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_namespace(raw_ns: str) -> str:
    clean = raw_ns.strip().strip('/')
    return clean or 'bcr_arm'


def _launch_setup(context):
    ns = _resolve_namespace(LaunchConfiguration('namespace').perform(context))
    raw_prefix = LaunchConfiguration('prefix').perform(context).strip()
    prefix = raw_prefix if raw_prefix else f'{ns}_'
    use_sim_time = (
        LaunchConfiguration('use_sim_time').perform(context).strip().lower() == 'true'
    )
    x = LaunchConfiguration('x').perform(context).strip()
    y = LaunchConfiguration('y').perform(context).strip()
    z = LaunchConfiguration('z').perform(context).strip()
    yaw = LaunchConfiguration('yaw').perform(context).strip()

    static_tf_map_mount = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        namespace=ns,
        name='static_tf_map_to_mount',
        output='screen',
        arguments=[
            '--x', x, '--y', y, '--z', z,
            '--roll', '0', '--pitch', '0', '--yaw', yaw,
            '--frame-id', 'map', '--child-frame-id', f'{prefix}mount_link',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    cm_path = f'/{ns}/controller_manager'

    def make_spawner(controller_name: str) -> Node:
        return Node(
            package='controller_manager',
            executable='spawner',
            namespace=ns,
            name=f'spawner_{controller_name}',
            output='screen',
            arguments=[
                controller_name,
                '--controller-manager', cm_path,
                '--controller-manager-timeout', '30',
                '--service-call-timeout', '60',
                '--switch-timeout', '60',
            ],
        )

    jsb_spawner = make_spawner('joint_state_broadcaster')
    arm_spawner = make_spawner('arm_trajectory_controller')
    gripper_spawner = make_spawner('gripper_controller')

    spawn_arm_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[arm_spawner],
        )
    )
    spawn_gripper_after_arm = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_spawner,
            on_exit=[gripper_spawner],
        )
    )

    return [
        static_tf_map_mount,
        jsb_spawner,
        spawn_arm_after_jsb,
        spawn_gripper_after_arm,
    ]


def generate_launch_description():
    """Publish map->mount TF and spawn ros2_control controllers for bcr_arm."""
    return LaunchDescription([
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
            'use_sim_time',
            default_value='true',
            description='Use simulation clock',
        ),
        DeclareLaunchArgument('x', default_value='1.75', description='Mount X in map frame'),
        DeclareLaunchArgument('y', default_value='13.08', description='Mount Y in map frame'),
        DeclareLaunchArgument('z', default_value='0.0', description='Mount Z in map frame'),
        DeclareLaunchArgument(
            'yaw', default_value='3.14159', description='Mount Yaw in map frame'
        ),
        OpaqueFunction(function=_launch_setup),
    ])
