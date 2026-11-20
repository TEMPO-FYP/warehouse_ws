import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_gz_sim(context, pkg_warehouse_sim, pkg_ros_gz_sim):
    """Resolve relative world filenames against warehouse_sim/worlds/ when present."""
    world_val = LaunchConfiguration('world').perform(context).strip()
    local_world = os.path.join(pkg_warehouse_sim, 'worlds', world_val)
    if not os.path.isabs(world_val) and os.path.isfile(local_world):
        resolved_world = local_world
    else:
        resolved_world = world_val

    headless = LaunchConfiguration('headless').perform(context)
    if headless == 'true':
        gz_flags = '-r -s'
    elif headless == 'false':
        gz_flags = '-r'
    else:
        raise ValueError(
            f"Invalid 'headless' value '{headless}'; expected 'true' or 'false'"
        )

    record = LaunchConfiguration('record').perform(context).strip().lower()
    record_path = LaunchConfiguration('record_path').perform(context).strip()
    record_period = LaunchConfiguration('record_period').perform(context).strip()
    save_video = LaunchConfiguration('save_video').perform(context).strip().lower()
    video_path = LaunchConfiguration('video_path').perform(context).strip()
    args_parts = [gz_flags]
    if headless == 'true' and (save_video == 'true' or video_path):
        args_parts.append('--headless-rendering')
    if record_path:
        args_parts.append(f'--record-path "{record_path}" --log-overwrite')
    elif record == 'true':
        args_parts.append('--record')
    if (record_path or record == 'true') and record_period and float(record_period) > 0.0:
        args_parts.append(f'--record-period {record_period}')
    args_parts.append(resolved_world)

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': ' '.join(args_parts)
            }.items(),
        )
    ]


def _launch_video_recorder(context):
    """Launch gz_video_recorder when save_video or video_path is enabled."""
    save_video = LaunchConfiguration('save_video').perform(context).strip().lower()
    video_path = LaunchConfiguration('video_path').perform(context).strip()
    if save_video != 'true' and not video_path:
        return []

    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).strip().lower() == 'true'
    video_topic = LaunchConfiguration('video_topic').perform(context).strip() or '/b1z1/video_camera'
    video_fps = float(LaunchConfiguration('video_fps').perform(context))
    return [
        Node(
            package='warehouse_sim',
            executable='gz_video_recorder',
            name='video_recorder',
            output='screen',
            parameters=[{
                'topic': video_topic,
                'output_path': video_path,
                'fps': video_fps,
                'use_sim_time': use_sim_time,
            }],
        )
    ]


def _launch_map_server(context):
    """Launch shared nav2_map_server when static warehouse map is enabled."""
    mode = LaunchConfiguration('use_static_map').perform(context)
    world_val = LaunchConfiguration('world').perform(context).strip().lower()
    if mode == 'false' or (mode == 'auto' and 'warehouse' not in world_val):
        return []

    map_yaml = LaunchConfiguration('map_yaml').perform(context).strip()
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).strip().lower() == 'true'
    return [
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'yaml_filename': map_yaml}],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart': True},
                {'node_names': ['map_server']},
            ],
        ),
    ]


def generate_launch_description():
    pkg_warehouse_sim = get_package_share_directory('warehouse_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Path to SDF world and shared static 2D map
    default_world_path = os.path.join(pkg_warehouse_sim, 'worlds', 'warehouse.sdf')
    default_map_yaml = os.path.join(pkg_warehouse_sim, 'maps', 'warehouse_map.yaml')
    models_path = os.path.join(pkg_warehouse_sim, 'models')
    worlds_path = os.path.join(pkg_warehouse_sim, 'worlds')

    # Declare world argument
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world_path,
        description='Path to the SDF world file to load'
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
        description='Use simulation clock across launched warehouse_sim nodes',
    )
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        choices=['true', 'false'],
        description='Run Gazebo Sim server headless (-s)'
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
    video_topic_arg = DeclareLaunchArgument(
        'video_topic',
        default_value='/b1z1/video_camera',
        description='Gazebo camera topic recorded by gz_video_recorder',
    )
    video_fps_arg = DeclareLaunchArgument(
        'video_fps',
        default_value='15.0',
        description='Output MP4 frame rate for gz_video_recorder',
    )
    use_static_map_arg = DeclareLaunchArgument(
        'use_static_map',
        default_value='auto',
        choices=['auto', 'true', 'false'],
        description='Serve shared /map via nav2_map_server (auto enables for warehouse worlds)',
    )
    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml',
        default_value=default_map_yaml,
        description='Path to the static 2D occupancy map YAML',
    )

    # Set GZ_SIM_RESOURCE_PATH so Gazebo Sim finds local models and sourced package:// assets
    existing_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    ament_share_paths = [
        os.path.join(p, 'share')
        for p in os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep)
        if p and os.path.isdir(os.path.join(p, 'share'))
    ]
    resource_paths = [
        models_path,
        worlds_path,
        *ament_share_paths,
        existing_resource_path,
    ]
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join(filter(None, resource_paths)),
    )

    # Launch Gazebo Sim unpaused (-r)
    gz_sim = OpaqueFunction(
        function=_launch_gz_sim,
        args=[pkg_warehouse_sim, pkg_ros_gz_sim],
    )

    # Bridge simulation clock to ROS 2
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )

    # Bridge Gazebo world entity poses to ROS 2 (/world/<world_name>/pose/info)
    pose_bridge = Node(
        package='warehouse_sim',
        executable='gz_pose_bridge',
        name='pose_bridge',
        output='screen',
        parameters=[{
            'world_name': LaunchConfiguration('world_name'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    map_server_action = OpaqueFunction(function=_launch_map_server)
    video_recorder_action = OpaqueFunction(function=_launch_video_recorder)

    return LaunchDescription([
        world_arg,
        world_name_arg,
        use_sim_time_arg,
        headless_arg,
        record_arg,
        record_path_arg,
        record_period_arg,
        save_video_arg,
        video_path_arg,
        video_topic_arg,
        video_fps_arg,
        use_static_map_arg,
        map_yaml_arg,
        gz_resource_path,
        gz_sim,
        clock_bridge,
        pose_bridge,
        map_server_action,
        video_recorder_action,
    ])
