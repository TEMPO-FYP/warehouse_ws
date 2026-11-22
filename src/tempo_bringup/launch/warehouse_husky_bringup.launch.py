"""Combined simulation, Husky A200 bringup, and optional TEMPO mission pipeline launcher."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    """Launch warehouse_sim + husky_bringup (and optionally tempo_warehouse_pipeline)."""
    pkg_warehouse_sim = get_package_share_directory('warehouse_sim')
    pkg_husky_control = get_package_share_directory('husky_control')
    pkg_tempo_bringup = get_package_share_directory('tempo_bringup')

    default_map_yaml = os.path.join(pkg_warehouse_sim, 'maps', 'warehouse_map.yaml')

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
    record_arg = DeclareLaunchArgument(
        'record',
        default_value='false',
        choices=['true', 'false'],
        description='Record Gazebo Sim states (--record) to ~/.gz/sim/log for playback',
    )
    record_path_arg = DeclareLaunchArgument(
        'record_path',
        default_value='',
        description='Custom directory path for Gazebo Sim recording (--record-path)',
    )
    record_period_arg = DeclareLaunchArgument(
        'record_period',
        default_value='0.033',
        description='Seconds between recorded states (--record-period, 0 records every step)',
    )
    use_static_map_arg = DeclareLaunchArgument(
        'use_static_map',
        default_value='auto',
        choices=['auto', 'true', 'false'],
        description='Serve shared /map and enable Nav2 static_layer (auto enables for warehouse)',
    )
    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml',
        default_value=default_map_yaml,
        description='Path to static 2D occupancy map YAML',
    )
    name_arg = DeclareLaunchArgument(
        'name',
        default_value='a200_0001',
        description='Model name in Gazebo',
    )
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='a200_0001',
        description='ROS namespace for the spawned Husky instance',
    )
    x_arg = DeclareLaunchArgument('x', default_value='0.0', description='Spawn X position')
    y_arg = DeclareLaunchArgument('y', default_value='-5.0', description='Spawn Y position')
    z_arg = DeclareLaunchArgument('z', default_value='0.15', description='Spawn Z position')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn Yaw rotation')
    use_nav2_arg = DeclareLaunchArgument(
        'use_nav2',
        default_value='true',
        description='Launch Nav2 navigation stack alongside Gazebo and controllers',
    )
    max_speed_arg = DeclareLaunchArgument(
        'max_speed',
        default_value='1.0',
        description='Max forward speed in m/s for Husky Nav2',
    )
    spawn_delay_arg = DeclareLaunchArgument(
        'spawn_delay',
        default_value='30.0',
        description=(
            'Seconds before spawning Husky when Gazebo GUI is enabled (headless:=false). '
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
            'record': LaunchConfiguration('record'),
            'record_path': LaunchConfiguration('record_path'),
            'record_period': LaunchConfiguration('record_period'),
            'use_static_map': LaunchConfiguration('use_static_map'),
            'map_yaml': LaunchConfiguration('map_yaml'),
        }.items(),
    )

    husky_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_husky_control, 'launch', 'husky_bringup.launch.py')
        ),
        launch_arguments={
            'name': LaunchConfiguration('name'),
            'namespace': LaunchConfiguration('namespace'),
            'world_name': LaunchConfiguration('world_name'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'yaw': LaunchConfiguration('yaw'),
            'use_nav2': LaunchConfiguration('use_nav2'),
            'max_speed': LaunchConfiguration('max_speed'),
            'spawn_delay': effective_spawn_delay,
        }.items(),
    )

    tempo_pipeline_launch = TimerAction(
        period=PythonExpression(["float('", effective_spawn_delay, "') + 14.0"]),
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
        world_arg,
        world_name_arg,
        use_sim_time_arg,
        headless_arg,
        record_arg,
        record_path_arg,
        record_period_arg,
        use_static_map_arg,
        map_yaml_arg,
        name_arg,
        namespace_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        use_nav2_arg,
        max_speed_arg,
        spawn_delay_arg,
        use_tempo_arg,
        warehouse_launch,
        husky_launch,
        tempo_pipeline_launch,
    ])
