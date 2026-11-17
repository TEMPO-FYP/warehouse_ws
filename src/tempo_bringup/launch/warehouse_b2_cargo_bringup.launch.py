"""Combined simulation, b2_cargo bringup, and optional TEMPO mission pipeline launcher."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    """Launch warehouse_sim + b2_cargo_bringup (and optionally tempo_warehouse_pipeline)."""
    pkg_warehouse_sim = get_package_share_directory('warehouse_sim')
    pkg_b2_cargo_control = get_package_share_directory('b2_cargo_control')
    pkg_tempo_bringup = get_package_share_directory('tempo_bringup')

    default_map_yaml = os.path.join(pkg_warehouse_sim, 'maps', 'warehouse_map.yaml')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='warehouse.sdf',
        description='SDF world name or path passed to warehouse_sim/warehouse.launch.py',
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
        description='Serve shared /map and enable Nav2 static_layer (auto enables for warehouse)',
    )
    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml',
        default_value=default_map_yaml,
        description='Path to static 2D occupancy map YAML',
    )
    name_arg = DeclareLaunchArgument('name', default_value='b2_cargo', description='Model name in Gazebo')
    namespace_arg = DeclareLaunchArgument('namespace', default_value='', description='Optional ROS namespace')
    x_arg = DeclareLaunchArgument('x', default_value='2.0', description='Spawn X position')
    y_arg = DeclareLaunchArgument('y', default_value='16.0', description='Spawn Y position')
    z_arg = DeclareLaunchArgument('z', default_value='0.64', description='Spawn Z position')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn Yaw rotation')
    spawn_test_cargo_arg = DeclareLaunchArgument(
        'spawn_test_cargo',
        default_value='false',
        description='Spawn a sample small_box payload centered inside cargo_rack',
    )
    use_nav2_arg = DeclareLaunchArgument(
        'use_nav2',
        default_value='true',
        description='Launch Nav2 navigation stack alongside Gazebo and controllers',
    )
    spawn_delay_arg = DeclareLaunchArgument(
        'spawn_delay',
        default_value='40.0',
        description=(
            'Seconds to wait before spawning b2_cargo when Gazebo GUI is enabled (headless:=false). '
            'Ignored (0.0 s) when headless:=true.'
        ),
    )
    use_tempo_arg = DeclareLaunchArgument(
        'use_tempo',
        default_value='false',
        choices=['true', 'false'],
        description='Also launch tempo_warehouse_pipeline.launch.py with transport controller enabled',
    )

    nav2_use_static_map = PythonExpression([
        "'true' if ('", LaunchConfiguration('use_static_map'), "' == 'true' or ('",
        LaunchConfiguration('use_static_map'), "' == 'auto' and 'warehouse' in '",
        LaunchConfiguration('world'), "'.lower())) else 'false'",
    ])

    effective_spawn_delay = PythonExpression([
        "'4.0' if '", LaunchConfiguration('headless'), "' == 'true' else '",
        LaunchConfiguration('spawn_delay'), "'",
    ])

    warehouse_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_warehouse_sim, 'launch', 'warehouse.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'headless': LaunchConfiguration('headless'),
            'use_static_map': LaunchConfiguration('use_static_map'),
            'map_yaml': LaunchConfiguration('map_yaml'),
        }.items(),
    )

    b2_cargo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_b2_cargo_control, 'launch', 'b2_cargo_bringup.launch.py')
        ),
        launch_arguments={
            'name': LaunchConfiguration('name'),
            'namespace': LaunchConfiguration('namespace'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'yaw': LaunchConfiguration('yaw'),
            'spawn_test_cargo': LaunchConfiguration('spawn_test_cargo'),
            'use_nav2': LaunchConfiguration('use_nav2'),
            'use_static_map': nav2_use_static_map,
            'spawn_delay': effective_spawn_delay,
        }.items(),
    )

    tempo_pipeline_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tempo_bringup, 'launch', 'tempo_warehouse_pipeline.launch.py')
        ),
        launch_arguments={
            'use_mobile_controller': 'false',
            'use_transport_controller': 'true',
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_tempo')),
    )

    return LaunchDescription([
        world_arg,
        headless_arg,
        use_static_map_arg,
        map_yaml_arg,
        name_arg,
        namespace_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        spawn_test_cargo_arg,
        use_nav2_arg,
        spawn_delay_arg,
        use_tempo_arg,
        warehouse_launch,
        b2_cargo_launch,
        tempo_pipeline_launch,
    ])
