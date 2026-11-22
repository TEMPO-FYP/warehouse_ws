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

"""Combined simulation, bcr_arm bringup, and optional TEMPO mission pipeline launcher."""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _spawn_test_targets(context, pkg_warehouse_sim: str):
    """Optionally spawn a test target box reachable by bcr_arm."""
    if LaunchConfiguration('spawn_test_target').perform(context).strip().lower() != 'true':
        return []

    world_val = LaunchConfiguration('world').perform(context).strip().lower()
    x = float(LaunchConfiguration('x').perform(context))
    y = float(LaunchConfiguration('y').perform(context))
    yaw = float(LaunchConfiguration('yaw').perform(context))
    headless = LaunchConfiguration('headless').perform(context).strip().lower() == 'true'
    delay_sec = 0.0 if headless else float(LaunchConfiguration('spawn_delay').perform(context))

    box_sdf = os.path.join(pkg_warehouse_sim, 'models', 'graspable_box', 'model.sdf')

    if 'warehouse' in world_val:
        # Place graspable_box atop Packing_Station (surface z=0.914m) facing bcr_arm
        # at (1.75, 13.08)
        bx = x + 0.55 * math.cos(yaw)
        by = y + 0.55 * math.sin(yaw)
        spawners = [
            Node(
                package='ros_gz_sim',
                executable='create',
                name='graspable_box_spawner',
                output='screen',
                arguments=[
                    '-file', box_sdf,
                    '-name', 'graspable_box',
                    '-x', f'{bx:.4f}',
                    '-y', f'{by:.4f}',
                    '-z', '0.9450',
                    '-Y', f'{yaw:.4f}',
                ],
            )
        ]
    else:
        ped_sdf = os.path.join(pkg_warehouse_sim, 'models', 'test_pedestal', 'model.sdf')
        dist = 0.55
        px = x + dist * math.cos(yaw)
        py = y + dist * math.sin(yaw)
        spawners = [
            Node(
                package='ros_gz_sim',
                executable='create',
                name='test_pedestal_spawner',
                output='screen',
                arguments=[
                    '-file', ped_sdf,
                    '-name', 'test_pedestal',
                    '-x', f'{px:.4f}',
                    '-y', f'{py:.4f}',
                    '-z', '0.3500',
                    '-Y', f'{yaw:.4f}',
                ],
            ),
            Node(
                package='ros_gz_sim',
                executable='create',
                name='graspable_box_spawner',
                output='screen',
                arguments=[
                    '-file', box_sdf,
                    '-name', 'graspable_box',
                    '-x', f'{px:.4f}',
                    '-y', f'{py:.4f}',
                    '-z', '0.7310',
                    '-Y', f'{yaw:.4f}',
                ],
            ),
        ]

    return [TimerAction(period=delay_sec, actions=spawners)]


def generate_launch_description():
    """Launch warehouse_sim + bcr_arm_bringup (and optionally tempo_warehouse_pipeline)."""
    pkg_warehouse_sim = get_package_share_directory('warehouse_sim')
    pkg_bcr_arm_description = get_package_share_directory('bcr_arm_description')
    pkg_bcr_arm_control = get_package_share_directory('bcr_arm_control')
    pkg_robotiq_description = get_package_share_directory('robotiq_description')
    pkg_tempo_bringup = get_package_share_directory('tempo_bringup')

    default_map_yaml = os.path.join(pkg_warehouse_sim, 'maps', 'warehouse_map.yaml')

    arm_parent = os.path.dirname(pkg_bcr_arm_description)
    robotiq_parent = os.path.dirname(pkg_robotiq_description)
    existing_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join(
            filter(
                None,
                [
                    arm_parent,
                    pkg_bcr_arm_description,
                    robotiq_parent,
                    pkg_robotiq_description,
                    existing_gz_path,
                ],
            )
        ),
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='warehouse.sdf',
        description='SDF world name or path passed to warehouse_sim/warehouse.launch.py',
    )
    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='default',
        description='Gazebo world name used for /world/<world_name>/pose/info',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        choices=['true', 'false'],
        description='Use simulation clock across simulation, robot, and TEMPO nodes',
    )
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        choices=['true', 'false'],
        description='Run Gazebo Sim server headless (-s)',
    )
    use_static_map_arg = DeclareLaunchArgument(
        'use_static_map',
        default_value='auto',
        choices=['auto', 'true', 'false'],
        description='Serve shared /map via nav2_map_server (auto enables for warehouse)',
    )
    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml',
        default_value=default_map_yaml,
        description='Path to static 2D occupancy map YAML',
    )
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='bcr_arm',
        description='ROS namespace for the spawned bcr_arm instance',
    )
    prefix_arg = DeclareLaunchArgument(
        'prefix',
        default_value='',
        description='TF/URDF/SRDF link prefix (defaults to <namespace>_ when empty)',
    )
    name_arg = DeclareLaunchArgument(
        'name',
        default_value='bcr_arm',
        description='Model name in Gazebo',
    )
    x_arg = DeclareLaunchArgument('x', default_value='1.75', description='Spawn X position')
    y_arg = DeclareLaunchArgument('y', default_value='13.08', description='Spawn Y position')
    z_arg = DeclareLaunchArgument('z', default_value='0.0', description='Spawn Z position')
    yaw_arg = DeclareLaunchArgument(
        'yaw', default_value='3.14159', description='Spawn Yaw rotation'
    )
    use_pedestal_arg = DeclareLaunchArgument(
        'use_pedestal',
        default_value='true',
        description='Mount bcr_arm on a built-in pedestal link',
    )
    pedestal_height_arg = DeclareLaunchArgument(
        'pedestal_height',
        default_value='0.55',
        description='Built-in pedestal height (m)',
    )
    spawn_test_target_arg = DeclareLaunchArgument(
        'spawn_test_target',
        default_value='false',
        description='Spawn a graspable_box reachable by bcr_arm for standalone testing',
    )
    use_moveit_arg = DeclareLaunchArgument(
        'use_moveit',
        default_value='true',
        description='Launch MoveIt 2 move_group alongside Gazebo and controllers',
    )
    spawn_delay_arg = DeclareLaunchArgument(
        'spawn_delay',
        default_value='40.0',
        description=(
            'Seconds before spawning bcr_arm when Gazebo GUI is enabled (headless:=false). '
            'Ignored (0.0 s) when headless:=true.'
        ),
    )
    use_tempo_arg = DeclareLaunchArgument(
        'use_tempo',
        default_value='false',
        choices=['true', 'false'],
        description='Also launch tempo_warehouse_pipeline.launch.py',
    )

    effective_spawn_delay = PythonExpression([
        "'0.0' if '", LaunchConfiguration('headless'), "' == 'true' else '",
        LaunchConfiguration('spawn_delay'), "'",
    ])

    warehouse_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_warehouse_sim, 'launch', 'warehouse.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'world_name': LaunchConfiguration('world_name'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'headless': LaunchConfiguration('headless'),
            'use_static_map': LaunchConfiguration('use_static_map'),
            'map_yaml': LaunchConfiguration('map_yaml'),
        }.items(),
    )

    bcr_arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bcr_arm_control, 'launch', 'bcr_arm_bringup.launch.py')
        ),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
            'prefix': LaunchConfiguration('prefix'),
            'name': LaunchConfiguration('name'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'yaw': LaunchConfiguration('yaw'),
            'use_pedestal': LaunchConfiguration('use_pedestal'),
            'pedestal_height': LaunchConfiguration('pedestal_height'),
            'use_moveit': LaunchConfiguration('use_moveit'),
            'spawn_delay': effective_spawn_delay,
        }.items(),
    )

    tempo_pipeline_launch = TimerAction(
        period=PythonExpression(["float('", effective_spawn_delay, "') + 12.0"]),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_tempo_bringup, 'launch', 'tempo_warehouse_pipeline.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }.items(),
            )
        ],
        condition=IfCondition(LaunchConfiguration('use_tempo')),
    )

    return LaunchDescription([
        gz_resource_path,
        world_arg,
        world_name_arg,
        use_sim_time_arg,
        headless_arg,
        use_static_map_arg,
        map_yaml_arg,
        namespace_arg,
        prefix_arg,
        name_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        use_pedestal_arg,
        pedestal_height_arg,
        spawn_test_target_arg,
        use_moveit_arg,
        spawn_delay_arg,
        use_tempo_arg,
        warehouse_launch,
        bcr_arm_launch,
        OpaqueFunction(function=_spawn_test_targets, args=[pkg_warehouse_sim]),
        tempo_pipeline_launch,
    ])
