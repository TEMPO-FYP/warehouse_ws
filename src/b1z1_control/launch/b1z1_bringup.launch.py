import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    """Launch Gazebo Sim, spawn b1z1, start ros2_control + CHAMP + EKF, and optionally MoveIt 2."""
    pkg_warehouse_sim = get_package_share_directory('warehouse_sim')
    pkg_b1z1_description = get_package_share_directory('b1z1_description')
    pkg_b1z1_control = get_package_share_directory('b1z1_control')
    pkg_b1z1_moveit = get_package_share_directory('b1z1_moveit_config')

    default_map_yaml = os.path.join(pkg_b1z1_control, 'maps', 'warehouse_map.yaml')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty.sdf',
        description='SDF world name or path passed to warehouse_sim/warehouse.launch.py',
    )
    x_arg = DeclareLaunchArgument('x', default_value='0.0', description='Spawn X position')
    y_arg = DeclareLaunchArgument(
        'y',
        default_value=PythonExpression(
            ["'16.0' if 'warehouse' in '", LaunchConfiguration('world'), "' else '0.0'"]
        ),
        description='Spawn Y position',
    )
    z_arg = DeclareLaunchArgument('z', default_value='0.65', description='Spawn Z position')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn Yaw rotation')
    spawn_test_target_arg = DeclareLaunchArgument(
        'spawn_test_target',
        default_value=PythonExpression(
            ["'false' if 'warehouse' in '", LaunchConfiguration('world'), "' else 'true'"]
        ),
        description='Spawn static pedestal and graspable_box in front of b1z1',
    )
    test_target_dist_arg = DeclareLaunchArgument(
        'test_target_dist',
        default_value='0.73',
        description='Distance ahead of b1z1 to spawn optional test pedestal/box',
    )
    use_moveit_arg = DeclareLaunchArgument(
        'use_moveit',
        default_value='true',
        description='Launch MoveIt 2 move_group alongside Gazebo and controllers',
    )
    use_nav2_arg = DeclareLaunchArgument(
        'use_nav2',
        default_value='true',
        description='Launch Nav2 navigation stack alongside Gazebo and controllers',
    )
    use_static_map_arg = DeclareLaunchArgument(
        'use_static_map',
        default_value=PythonExpression(
            ["'true' if 'warehouse' in '", LaunchConfiguration('world'), "' else 'false'"]
        ),
        description='Use known static 2D map via nav2_map_server and static_layer',
    )
    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml',
        default_value=default_map_yaml,
        description='Path to static 2D occupancy map YAML',
    )
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo Sim server headless (-s)',
    )
    spawn_delay_arg = DeclareLaunchArgument(
        'spawn_delay',
        default_value='40.0',
        description=(
            'Seconds to wait before spawning b1z1 when the Gazebo GUI is enabled '
            '(headless:=false). The GUI takes ~15 s after the world loads to apply its '
            'initial world-state snapshot; a robot created before that finishes is never '
            'added to the Entity Tree / 3D scene. Ignored (0 s) when headless:=true.'
        ),
    )

    warehouse_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_warehouse_sim, 'launch', 'warehouse.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'headless': LaunchConfiguration('headless'),
        }.items(),
    )

    spawn_period = PythonExpression([
        "0.0 if '", LaunchConfiguration('headless'), "' == 'true' else float('",
        LaunchConfiguration('spawn_delay'), "')",
    ])

    spawn_launch = TimerAction(
        period=spawn_period,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_b1z1_description, 'launch', 'spawn_b1z1.launch.py')
                ),
                launch_arguments={
                    'x': LaunchConfiguration('x'),
                    'y': LaunchConfiguration('y'),
                    'z': LaunchConfiguration('z'),
                    'yaw': LaunchConfiguration('yaw'),
                    'bridge_clock': 'false',
                    'spawn_test_target': LaunchConfiguration('spawn_test_target'),
                    'test_target_dist': LaunchConfiguration('test_target_dist'),
                }.items(),
            )
        ],
    )

    # Controller spawners wait for /controller_manager, which only exists once b1z1 is
    # spawned (gz_ros2_control plugin), so start them together with the delayed spawn.
    control_launch = TimerAction(
        period=spawn_period,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_b1z1_control, 'launch', 'b1z1_control.launch.py')
                ),
            )
        ],
    )

    move_group_launch = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_b1z1_moveit, 'launch', 'move_group.launch.py')
                ),
                condition=IfCondition(LaunchConfiguration('use_moveit')),
            )
        ],
    )

    nav2_launch = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_b1z1_control, 'launch', 'b1z1_nav2.launch.py')
                ),
                launch_arguments={
                    'use_static_map': LaunchConfiguration('use_static_map'),
                    'map_yaml': LaunchConfiguration('map_yaml'),
                }.items(),
                condition=IfCondition(LaunchConfiguration('use_nav2')),
            )
        ],
    )

    return LaunchDescription([
        world_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        spawn_test_target_arg,
        test_target_dist_arg,
        use_moveit_arg,
        use_nav2_arg,
        use_static_map_arg,
        map_yaml_arg,
        headless_arg,
        spawn_delay_arg,
        warehouse_launch,
        spawn_launch,
        control_launch,
        move_group_launch,
        nav2_launch,
    ])
