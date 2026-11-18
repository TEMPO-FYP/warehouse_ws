import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    """Spawn b1z1 into a running Gazebo world and start control, MoveIt 2, and Nav2."""
    pkg_b1z1_description = get_package_share_directory('b1z1_description')
    pkg_b1z1_control = get_package_share_directory('b1z1_control')
    pkg_b1z1_moveit = get_package_share_directory('b1z1_moveit_config')

    x_arg = DeclareLaunchArgument('x', default_value='0.0', description='Spawn X position')
    y_arg = DeclareLaunchArgument('y', default_value='16.0', description='Spawn Y position')
    z_arg = DeclareLaunchArgument('z', default_value='0.65', description='Spawn Z position')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn Yaw rotation')
    spawn_test_target_arg = DeclareLaunchArgument(
        'spawn_test_target',
        default_value='false',
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
        default_value='true',
        description='Enable global_costmap static_layer for the shared /map topic',
    )
    spawn_delay_arg = DeclareLaunchArgument(
        'spawn_delay',
        default_value='0.0',
        description=(
            'Seconds to wait before spawning b1z1. Keep 0.0 when Gazebo is already running; '
            'set ~40.0 when launched simultaneously with Gazebo GUI.'
        ),
    )
    enable_video_cam_arg = DeclareLaunchArgument(
        'enable_video_cam',
        default_value='false',
        description='Attach third-person follow camera sensor to b1z1 for MP4 recording',
    )

    spawn_period = PythonExpression(["float('", LaunchConfiguration('spawn_delay'), "')"])
    move_group_period = PythonExpression([
        "float('", LaunchConfiguration('spawn_delay'), "') + 8.0",
    ])
    nav2_period = PythonExpression([
        "float('", LaunchConfiguration('spawn_delay'), "') + 10.0",
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
                    'spawn_test_target': LaunchConfiguration('spawn_test_target'),
                    'test_target_dist': LaunchConfiguration('test_target_dist'),
                    'enable_video_cam': LaunchConfiguration('enable_video_cam'),
                }.items(),
            )
        ],
    )

    # Controller spawners wait for /controller_manager, which only exists once b1z1 is
    # spawned (gz_ros2_control plugin), so start them together with spawn_launch.
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
        period=move_group_period,
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
        period=nav2_period,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_b1z1_control, 'launch', 'b1z1_nav2.launch.py')
                ),
                launch_arguments={
                    'use_static_map': LaunchConfiguration('use_static_map'),
                }.items(),
                condition=IfCondition(LaunchConfiguration('use_nav2')),
            )
        ],
    )

    return LaunchDescription([
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        spawn_test_target_arg,
        test_target_dist_arg,
        use_moveit_arg,
        use_nav2_arg,
        use_static_map_arg,
        spawn_delay_arg,
        enable_video_cam_arg,
        spawn_launch,
        control_launch,
        move_group_launch,
        nav2_launch,
    ])
