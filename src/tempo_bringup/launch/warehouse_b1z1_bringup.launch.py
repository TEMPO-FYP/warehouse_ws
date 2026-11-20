"""Combined simulation, B1Z1 bringup, and optional TEMPO mission pipeline launcher."""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _spawn_test_targets(context, pkg_warehouse_sim: str):
    """Optionally spawn test pedestals and boxes ahead of b1z1 after Gazebo initializes."""
    if LaunchConfiguration('spawn_test_target').perform(context).strip().lower() != 'true':
        return []

    x = float(LaunchConfiguration('x').perform(context))
    y = float(LaunchConfiguration('y').perform(context))
    yaw = float(LaunchConfiguration('yaw').perform(context))
    box_dist = float(LaunchConfiguration('test_target_dist').perform(context))
    headless = LaunchConfiguration('headless').perform(context).strip().lower() == 'true'
    delay_sec = 0.0 if headless else float(LaunchConfiguration('spawn_delay').perform(context))

    def make_spawner(node_name: str, model_name: str, model_dir: str, dist: float, pos_z: float, lat: float = 0.0):
        px = f'{x + dist * math.cos(yaw) - lat * math.sin(yaw):.4f}'
        py = f'{y + dist * math.sin(yaw) + lat * math.cos(yaw):.4f}'
        return Node(
            package='ros_gz_sim',
            executable='create',
            name=node_name,
            output='screen',
            arguments=[
                '-file', os.path.join(pkg_warehouse_sim, 'models', model_dir, 'model.sdf'),
                '-name', model_name,
                '-x', px,
                '-y', py,
                '-z', f'{pos_z:.4f}',
                '-Y', f'{yaw:.4f}',
            ],
        )

    spawners = [
        make_spawner('test_pedestal_spawner', 'test_pedestal', 'test_pedestal', dist=box_dist + 0.03, pos_z=0.35),
        make_spawner('graspable_box_spawner', 'graspable_box', 'graspable_box', dist=box_dist, pos_z=0.731),
        make_spawner('test_pedestal_2_spawner', 'test_pedestal_2', 'test_pedestal', dist=box_dist + 2.43, pos_z=0.35, lat=-1.80),
        make_spawner('tall_box_spawner', 'tall_box', 'tall_box', dist=box_dist + 2.41, pos_z=0.746, lat=-1.79),
    ]
    return [TimerAction(period=delay_sec, actions=spawners)]


def generate_launch_description():
    """Launch warehouse_sim + b1z1_bringup (and optionally tempo_warehouse_pipeline)."""
    pkg_warehouse_sim = get_package_share_directory('warehouse_sim')
    pkg_b1z1_description = get_package_share_directory('b1z1_description')
    pkg_b1z1_control = get_package_share_directory('b1z1_control')
    pkg_tempo_bringup = get_package_share_directory('tempo_bringup')

    default_map_yaml = os.path.join(pkg_warehouse_sim, 'maps', 'warehouse_map.yaml')

    # Ensure Gazebo server spawned in warehouse.launch.py can resolve b1z1_description meshes.
    b1z1_share_parent = os.path.dirname(pkg_b1z1_description)
    existing_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join(filter(None, [b1z1_share_parent, pkg_b1z1_description, existing_gz_path])),
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
    save_video_arg = DeclareLaunchArgument(
        'save_video',
        default_value='false',
        choices=['true', 'false'],
        description='Render follow camera and save an MP4 video directly to src/recordings/',
    )
    video_path_arg = DeclareLaunchArgument(
        'video_path',
        default_value='',
        description='Output MP4 path (relative paths save under src/recordings/)',
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
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='b1z1',
        description='ROS namespace for the spawned B1Z1 instance',
    )
    prefix_arg = DeclareLaunchArgument(
        'prefix',
        default_value='',
        description='TF/URDF link prefix (defaults to <namespace>_ when empty)',
    )
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
    spawn_delay_arg = DeclareLaunchArgument(
        'spawn_delay',
        default_value='40.0',
        description=(
            'Seconds to wait before spawning b1z1 when Gazebo GUI is enabled (headless:=false). '
            'Ignored (0.0 s) when headless:=true.'
        ),
    )
    use_tempo_arg = DeclareLaunchArgument(
        'use_tempo',
        default_value='false',
        choices=['true', 'false'],
        description='Also launch tempo_warehouse_pipeline.launch.py',
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
    enable_video_cam = PythonExpression([
        "'true' if ('", LaunchConfiguration('save_video'), "' == 'true' or '",
        LaunchConfiguration('video_path'), "'.strip() != '') else 'false'",
    ])
    video_topic_expr = PythonExpression([
        "'/' + '", LaunchConfiguration('namespace'), "'.strip().strip('/') + '/video_camera'",
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
            'save_video': LaunchConfiguration('save_video'),
            'video_path': LaunchConfiguration('video_path'),
            'video_topic': video_topic_expr,
            'use_static_map': LaunchConfiguration('use_static_map'),
            'map_yaml': LaunchConfiguration('map_yaml'),
        }.items(),
    )

    b1z1_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_b1z1_control, 'launch', 'b1z1_bringup.launch.py')
        ),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
            'prefix': LaunchConfiguration('prefix'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'yaw': LaunchConfiguration('yaw'),
            'enable_video_cam': enable_video_cam,
            'use_moveit': LaunchConfiguration('use_moveit'),
            'use_nav2': LaunchConfiguration('use_nav2'),
            'use_static_map': nav2_use_static_map,
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
        record_arg,
        record_path_arg,
        record_period_arg,
        save_video_arg,
        video_path_arg,
        use_static_map_arg,
        map_yaml_arg,
        namespace_arg,
        prefix_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        spawn_test_target_arg,
        test_target_dist_arg,
        use_moveit_arg,
        use_nav2_arg,
        spawn_delay_arg,
        use_tempo_arg,
        warehouse_launch,
        b1z1_launch,
        OpaqueFunction(function=_spawn_test_targets, args=[pkg_warehouse_sim]),
        tempo_pipeline_launch,
    ])
