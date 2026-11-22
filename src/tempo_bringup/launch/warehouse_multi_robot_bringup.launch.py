"""Combined multi-robot warehouse simulation, robot bringup, and optional TEMPO pipeline."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    """Launch warehouse_sim with toggleable b1z1, b2_cargo, bcr_arm, husky_a200, and husky_ur5e."""
    pkg_warehouse_sim = get_package_share_directory('warehouse_sim')
    pkg_b1z1_description = get_package_share_directory('b1z1_description')
    pkg_b1z1_control = get_package_share_directory('b1z1_control')
    pkg_b2_cargo_description = get_package_share_directory('b2_cargo_description')
    pkg_b2_cargo_control = get_package_share_directory('b2_cargo_control')
    pkg_bcr_arm_description = get_package_share_directory('bcr_arm_description')
    pkg_bcr_arm_control = get_package_share_directory('bcr_arm_control')
    pkg_husky_control = get_package_share_directory('husky_control')
    pkg_robotiq_description = get_package_share_directory('robotiq_description')
    pkg_ur_description = get_package_share_directory('ur_description')
    pkg_tempo_bringup = get_package_share_directory('tempo_bringup')

    default_map_yaml = os.path.join(pkg_warehouse_sim, 'maps', 'warehouse_map.yaml')

    b1z1_parent = os.path.dirname(pkg_b1z1_description)
    b2_parent = os.path.dirname(pkg_b2_cargo_description)
    arm_parent = os.path.dirname(pkg_bcr_arm_description)
    robotiq_parent = os.path.dirname(pkg_robotiq_description)
    ur_parent = os.path.dirname(pkg_ur_description)
    existing_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join(filter(None, [
            b1z1_parent,
            pkg_b1z1_description,
            b2_parent,
            pkg_b2_cargo_description,
            arm_parent,
            pkg_bcr_arm_description,
            robotiq_parent,
            pkg_robotiq_description,
            ur_parent,
            pkg_ur_description,
            existing_gz_path,
        ])),
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
        description='Serve shared /map and enable Nav2 static_layer (auto enables for warehouse)',
    )
    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml',
        default_value=default_map_yaml,
        description='Path to static 2D occupancy map YAML',
    )
    spawn_delay_arg = DeclareLaunchArgument(
        'spawn_delay',
        default_value='40.0',
        description=(
            'Seconds to wait before spawning robots when Gazebo GUI is enabled (headless:=false). '
            'Ignored (0.0 s) when headless:=true.'
        ),
    )
    use_nav2_arg = DeclareLaunchArgument(
        'use_nav2',
        default_value='true',
        description='Launch Nav2 navigation stack for enabled robots',
    )
    use_moveit_arg = DeclareLaunchArgument(
        'use_moveit',
        default_value='true',
        description='Launch MoveIt 2 move_group for enabled manipulators (b1z1, bcr_arm, husky_ur5e)',
    )
    use_tempo_arg = DeclareLaunchArgument(
        'use_tempo',
        default_value='false',
        choices=['true', 'false'],
        description='Also launch tempo_warehouse_pipeline.launch.py',
    )

    spawn_b1z1_arg = DeclareLaunchArgument(
        'spawn_b1z1',
        default_value='true',
        choices=['true', 'false'],
        description='Spawn and bring up b1z1 (set false to test b2_cargo alone)',
    )
    b1z1_namespace_arg = DeclareLaunchArgument(
        'b1z1_namespace',
        default_value='b1z1',
        description='ROS namespace for b1z1',
    )
    b1z1_x_arg = DeclareLaunchArgument('b1z1_x', default_value='0.0', description='b1z1 Spawn X')
    b1z1_y_arg = DeclareLaunchArgument('b1z1_y', default_value='16.0', description='b1z1 Spawn Y')
    b1z1_z_arg = DeclareLaunchArgument('b1z1_z', default_value='0.65', description='b1z1 Spawn Z')
    b1z1_yaw_arg = DeclareLaunchArgument('b1z1_yaw', default_value='0.0', description='b1z1 Yaw')

    spawn_b2_cargo_arg = DeclareLaunchArgument(
        'spawn_b2_cargo',
        default_value='true',
        choices=['true', 'false'],
        description='Spawn and bring up b2_cargo',
    )
    b2_cargo_name_arg = DeclareLaunchArgument(
        'b2_cargo_name',
        default_value='b2_cargo',
        description='Gazebo model name for b2_cargo',
    )
    b2_cargo_namespace_arg = DeclareLaunchArgument(
        'b2_cargo_namespace',
        default_value='b2_cargo',
        description='ROS namespace for b2_cargo',
    )
    b2_cargo_x_arg = DeclareLaunchArgument(
        'b2_cargo_x', default_value='2.0', description='b2_cargo Spawn X'
    )
    b2_cargo_y_arg = DeclareLaunchArgument(
        'b2_cargo_y', default_value='16.0', description='b2_cargo Spawn Y'
    )
    b2_cargo_z_arg = DeclareLaunchArgument(
        'b2_cargo_z', default_value='0.64', description='b2_cargo Spawn Z'
    )
    b2_cargo_yaw_arg = DeclareLaunchArgument(
        'b2_cargo_yaw', default_value='0.0', description='b2_cargo Spawn Yaw'
    )
    spawn_test_cargo_arg = DeclareLaunchArgument(
        'spawn_test_cargo',
        default_value='false',
        description='Spawn a sample small_box payload centered inside b2_cargo cargo_rack',
    )

    spawn_bcr_arm_arg = DeclareLaunchArgument(
        'spawn_bcr_arm',
        default_value='true',
        choices=['true', 'false'],
        description='Spawn and bring up bcr_arm stationary manipulator',
    )
    bcr_arm_name_arg = DeclareLaunchArgument(
        'bcr_arm_name',
        default_value='bcr_arm',
        description='Gazebo model name for bcr_arm',
    )
    bcr_arm_namespace_arg = DeclareLaunchArgument(
        'bcr_arm_namespace',
        default_value='bcr_arm',
        description='ROS namespace for bcr_arm',
    )
    bcr_arm_x_arg = DeclareLaunchArgument(
        'bcr_arm_x', default_value='1.75', description='bcr_arm Spawn X'
    )
    bcr_arm_y_arg = DeclareLaunchArgument(
        'bcr_arm_y', default_value='13.08', description='bcr_arm Spawn Y'
    )
    bcr_arm_z_arg = DeclareLaunchArgument(
        'bcr_arm_z', default_value='0.0', description='bcr_arm Spawn Z'
    )
    bcr_arm_yaw_arg = DeclareLaunchArgument(
        'bcr_arm_yaw', default_value='3.14159', description='bcr_arm Spawn Yaw'
    )
    bcr_arm_use_pedestal_arg = DeclareLaunchArgument(
        'bcr_arm_use_pedestal',
        default_value='true',
        description='Mount bcr_arm on built-in pedestal',
    )
    bcr_arm_pedestal_height_arg = DeclareLaunchArgument(
        'bcr_arm_pedestal_height',
        default_value='0.55',
        description='bcr_arm built-in pedestal height (m)',
    )

    spawn_husky_arg = DeclareLaunchArgument(
        'spawn_husky',
        default_value='true',
        choices=['true', 'false'],
        description='Spawn and bring up Clearpath Husky A200 transport robot',
    )
    husky_name_arg = DeclareLaunchArgument(
        'husky_name',
        default_value='a200_0001',
        description='Gazebo model name for Husky A200',
    )
    husky_namespace_arg = DeclareLaunchArgument(
        'husky_namespace',
        default_value='a200_0001',
        description='ROS namespace for Husky A200',
    )
    husky_x_arg = DeclareLaunchArgument('husky_x', default_value='0.0', description='Husky Spawn X')
    husky_y_arg = DeclareLaunchArgument('husky_y', default_value='-5.0', description='Husky Spawn Y')
    husky_z_arg = DeclareLaunchArgument('husky_z', default_value='0.15', description='Husky Spawn Z')
    husky_yaw_arg = DeclareLaunchArgument('husky_yaw', default_value='0.0', description='Husky Yaw')
    husky_max_speed_arg = DeclareLaunchArgument(
        'husky_max_speed',
        default_value='1.0',
        description='Husky Nav2 max forward speed (m/s)',
    )

    spawn_husky_ur5e_arg = DeclareLaunchArgument(
        'spawn_husky_ur5e',
        default_value='true',
        choices=['true', 'false'],
        description='Spawn and bring up Clearpath Husky A200 + UR5e mobile manipulator',
    )
    husky_ur5e_name_arg = DeclareLaunchArgument(
        'husky_ur5e_name',
        default_value='a200_0000',
        description='Gazebo model name for Husky UR5e',
    )
    husky_ur5e_namespace_arg = DeclareLaunchArgument(
        'husky_ur5e_namespace',
        default_value='a200_0000',
        description='ROS namespace for Husky UR5e',
    )
    husky_ur5e_x_arg = DeclareLaunchArgument(
        'husky_ur5e_x', default_value='0.0', description='Husky UR5e Spawn X'
    )
    husky_ur5e_y_arg = DeclareLaunchArgument(
        'husky_ur5e_y', default_value='-3.0', description='Husky UR5e Spawn Y'
    )
    husky_ur5e_z_arg = DeclareLaunchArgument(
        'husky_ur5e_z', default_value='0.15', description='Husky UR5e Spawn Z'
    )
    husky_ur5e_yaw_arg = DeclareLaunchArgument(
        'husky_ur5e_yaw', default_value='0.0', description='Husky UR5e Yaw'
    )
    husky_ur5e_max_speed_arg = DeclareLaunchArgument(
        'husky_ur5e_max_speed',
        default_value='1.0',
        description='Husky UR5e Nav2 max forward speed (m/s)',
    )

    nav2_use_static_map = PythonExpression([
        "'true' if ('", LaunchConfiguration('use_static_map'), "' == 'true' or ('",
        LaunchConfiguration('use_static_map'), "' == 'auto' and 'warehouse' in '",
        LaunchConfiguration('world'), "'.lower())) else 'false'",
    ])

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

    b1z1_launch = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_b1z1_control, 'launch', 'b1z1_bringup.launch.py')
                ),
                launch_arguments={
                    'namespace': LaunchConfiguration('b1z1_namespace'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'x': LaunchConfiguration('b1z1_x'),
                    'y': LaunchConfiguration('b1z1_y'),
                    'z': LaunchConfiguration('b1z1_z'),
                    'yaw': LaunchConfiguration('b1z1_yaw'),
                    'use_moveit': LaunchConfiguration('use_moveit'),
                    'use_nav2': LaunchConfiguration('use_nav2'),
                    'use_static_map': nav2_use_static_map,
                    'spawn_delay': effective_spawn_delay,
                }.items(),
            )
        ],
        scoped=True,
        condition=IfCondition(LaunchConfiguration('spawn_b1z1')),
    )

    b2_cargo_launch = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_b2_cargo_control, 'launch', 'b2_cargo_bringup.launch.py')
                ),
                launch_arguments={
                    'name': LaunchConfiguration('b2_cargo_name'),
                    'namespace': LaunchConfiguration('b2_cargo_namespace'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'x': LaunchConfiguration('b2_cargo_x'),
                    'y': LaunchConfiguration('b2_cargo_y'),
                    'z': LaunchConfiguration('b2_cargo_z'),
                    'yaw': LaunchConfiguration('b2_cargo_yaw'),
                    'spawn_test_cargo': LaunchConfiguration('spawn_test_cargo'),
                    'use_nav2': LaunchConfiguration('use_nav2'),
                    'use_static_map': nav2_use_static_map,
                    'spawn_delay': effective_spawn_delay,
                }.items(),
            )
        ],
        scoped=True,
        condition=IfCondition(LaunchConfiguration('spawn_b2_cargo')),
    )

    bcr_arm_launch = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_bcr_arm_control, 'launch', 'bcr_arm_bringup.launch.py')
                ),
                launch_arguments={
                    'name': LaunchConfiguration('bcr_arm_name'),
                    'namespace': LaunchConfiguration('bcr_arm_namespace'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'x': LaunchConfiguration('bcr_arm_x'),
                    'y': LaunchConfiguration('bcr_arm_y'),
                    'z': LaunchConfiguration('bcr_arm_z'),
                    'yaw': LaunchConfiguration('bcr_arm_yaw'),
                    'use_pedestal': LaunchConfiguration('bcr_arm_use_pedestal'),
                    'pedestal_height': LaunchConfiguration('bcr_arm_pedestal_height'),
                    'use_moveit': LaunchConfiguration('use_moveit'),
                    'spawn_delay': effective_spawn_delay,
                }.items(),
            )
        ],
        scoped=True,
        condition=IfCondition(LaunchConfiguration('spawn_bcr_arm')),
    )

    husky_launch = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_husky_control, 'launch', 'husky_bringup.launch.py')
                ),
                launch_arguments={
                    'name': LaunchConfiguration('husky_name'),
                    'namespace': LaunchConfiguration('husky_namespace'),
                    'world_name': LaunchConfiguration('world_name'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'x': LaunchConfiguration('husky_x'),
                    'y': LaunchConfiguration('husky_y'),
                    'z': LaunchConfiguration('husky_z'),
                    'yaw': LaunchConfiguration('husky_yaw'),
                    'use_nav2': LaunchConfiguration('use_nav2'),
                    'max_speed': LaunchConfiguration('husky_max_speed'),
                    'spawn_delay': effective_spawn_delay,
                }.items(),
            )
        ],
        scoped=True,
        condition=IfCondition(LaunchConfiguration('spawn_husky')),
    )

    husky_ur5e_launch = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_husky_control, 'launch', 'husky_ur5e_bringup.launch.py')
                ),
                launch_arguments={
                    'name': LaunchConfiguration('husky_ur5e_name'),
                    'namespace': LaunchConfiguration('husky_ur5e_namespace'),
                    'world_name': LaunchConfiguration('world_name'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'x': LaunchConfiguration('husky_ur5e_x'),
                    'y': LaunchConfiguration('husky_ur5e_y'),
                    'z': LaunchConfiguration('husky_ur5e_z'),
                    'yaw': LaunchConfiguration('husky_ur5e_yaw'),
                    'use_nav2': LaunchConfiguration('use_nav2'),
                    'use_moveit': LaunchConfiguration('use_moveit'),
                    'max_speed': LaunchConfiguration('husky_ur5e_max_speed'),
                    'spawn_delay': effective_spawn_delay,
                }.items(),
            )
        ],
        scoped=True,
        condition=IfCondition(LaunchConfiguration('spawn_husky_ur5e')),
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
        spawn_delay_arg,
        use_nav2_arg,
        use_moveit_arg,
        use_tempo_arg,
        spawn_b1z1_arg,
        b1z1_namespace_arg,
        b1z1_x_arg,
        b1z1_y_arg,
        b1z1_z_arg,
        b1z1_yaw_arg,
        spawn_b2_cargo_arg,
        b2_cargo_name_arg,
        b2_cargo_namespace_arg,
        b2_cargo_x_arg,
        b2_cargo_y_arg,
        b2_cargo_z_arg,
        b2_cargo_yaw_arg,
        spawn_test_cargo_arg,
        spawn_bcr_arm_arg,
        bcr_arm_name_arg,
        bcr_arm_namespace_arg,
        bcr_arm_x_arg,
        bcr_arm_y_arg,
        bcr_arm_z_arg,
        bcr_arm_yaw_arg,
        bcr_arm_use_pedestal_arg,
        bcr_arm_pedestal_height_arg,
        spawn_husky_arg,
        husky_name_arg,
        husky_namespace_arg,
        husky_x_arg,
        husky_y_arg,
        husky_z_arg,
        husky_yaw_arg,
        husky_max_speed_arg,
        spawn_husky_ur5e_arg,
        husky_ur5e_name_arg,
        husky_ur5e_namespace_arg,
        husky_ur5e_x_arg,
        husky_ur5e_y_arg,
        husky_ur5e_z_arg,
        husky_ur5e_yaw_arg,
        husky_ur5e_max_speed_arg,
        warehouse_launch,
        b1z1_launch,
        b2_cargo_launch,
        bcr_arm_launch,
        husky_launch,
        husky_ur5e_launch,
        tempo_pipeline_launch,
    ])
