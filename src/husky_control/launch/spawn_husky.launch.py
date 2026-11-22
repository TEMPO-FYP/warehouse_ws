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

"""Generate Clearpath assets, spawn Husky A200 into Gazebo, and start platform/odom bridges."""

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_namespace(raw_ns: str) -> str:
    clean = raw_ns.strip().strip('/')
    return clean or 'a200_0001'


def _setup_spawn(context, pkg_husky_control: str, pkg_husky_prefix: str):
    ns = _resolve_namespace(LaunchConfiguration('namespace').perform(context))
    name = LaunchConfiguration('name').perform(context).strip() or ns
    world_name = LaunchConfiguration('world_name').perform(context).strip() or 'default'
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).strip().lower() == 'true'
    x = LaunchConfiguration('x').perform(context).strip()
    y = LaunchConfiguration('y').perform(context).strip()
    z = LaunchConfiguration('z').perform(context).strip()
    yaw = LaunchConfiguration('yaw').perform(context).strip()
    timeout_sec = LaunchConfiguration('timeout_sec').perform(context).strip()
    spawn_delay = LaunchConfiguration('spawn_delay').perform(context).strip()
    config_yaml = LaunchConfiguration('config_yaml').perform(context).strip()

    setup_dir = f'/tmp/clearpath_{ns}'
    spawn_script = os.path.join(
        pkg_husky_prefix, 'lib', 'husky_control', 'prepare_and_spawn_husky.sh'
    )
    if not os.path.isfile(spawn_script):
        spawn_script = os.path.join(
            pkg_husky_control, 'scripts', 'prepare_and_spawn_husky.sh'
        )

    pkg_clearpath_desc = get_package_share_directory('clearpath_platform_description')
    pkg_clearpath_gz = get_package_share_directory('clearpath_gz')
    ros_share_path = os.path.dirname(pkg_clearpath_desc)
    existing_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join(filter(None, [
            ros_share_path,
            setup_dir,
            os.path.join(pkg_clearpath_gz, 'models'),
            os.path.join(pkg_clearpath_gz, 'meshes'),
            existing_gz_path,
        ])),
    )

    spawn_proc = ExecuteProcess(
        cmd=[
            'bash',
            spawn_script,
            config_yaml,
            ns,
            name,
            world_name,
            x,
            y,
            z,
            yaw,
            timeout_sec,
            spawn_delay,
        ],
        output='screen',
    )

    platform_launch = os.path.join(
        setup_dir, 'platform', 'launch', 'platform-service.launch.py'
    )
    sensors_launch = os.path.join(
        setup_dir, 'sensors', 'launch', 'sensors-service.launch.py'
    )

    platform_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_platform_velocity_controller',
        namespace=ns,
        arguments=[
            'platform_velocity_controller',
            '--controller-manager', f'/{ns}/controller_manager',
            '--controller-manager-timeout', '120',
            '--switch-timeout', '120',
            '--service-call-timeout', '120',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    gt_odom_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='husky_gt_odom_bridge',
        namespace=ns,
        output='screen',
        arguments=[
            f'/{ns}/ground_truth/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            f'/{ns}/ground_truth/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ],
        remappings=[(f'/{ns}/ground_truth/tf', f'/{ns}/tf')],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_to_odom',
        namespace=ns,
        output='screen',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'map', '--child-frame-id', 'odom',
        ],
        remappings=[('/tf_static', f'/{ns}/tf_static')],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    def _on_spawn_exit(event, _ctx):
        if event.returncode != 0:
            return [LogInfo(msg=f'[spawn_husky] Spawn failed for {name}; skipping platform launch.')]
        return [
            IncludeLaunchDescription(PythonLaunchDescriptionSource(platform_launch)),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(sensors_launch)),
            platform_controller_spawner,
            gt_odom_bridge,
            static_tf_map_odom,
        ]

    on_spawn_handler = RegisterEventHandler(
        OnProcessExit(target_action=spawn_proc, on_exit=_on_spawn_exit)
    )

    return [gz_resource_path, spawn_proc, on_spawn_handler]


def generate_launch_description():
    """Spawn Husky A200 into Gazebo and launch Clearpath services + ground-truth odometry."""
    pkg_husky_control = get_package_share_directory('husky_control')
    pkg_husky_prefix = get_package_prefix('husky_control')
    default_config_yaml = os.path.join(pkg_husky_control, 'config', 'husky_transport.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='a200_0001',
            description='ROS namespace for this Husky instance',
        ),
        DeclareLaunchArgument(
            'name',
            default_value='a200_0001',
            description='Model name in Gazebo',
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
            'timeout_sec',
            default_value='120',
            description='Seconds to wait for Gazebo /world/<world_name>/create service',
        ),
        DeclareLaunchArgument(
            'spawn_delay',
            default_value='0.0',
            description='Seconds to wait after /create is ready before spawning Husky',
        ),
        DeclareLaunchArgument(
            'config_yaml',
            default_value=default_config_yaml,
            description='Path to Clearpath robot.yaml configuration template',
        ),
        OpaqueFunction(function=_setup_spawn, args=[pkg_husky_control, pkg_husky_prefix]),
    ])
