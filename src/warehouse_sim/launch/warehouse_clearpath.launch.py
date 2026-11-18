"""
Launch the custom warehouse world and spawn the Clearpath Husky A200 + UR5e into it.

Why this file exists:
  `ros2 launch clearpath_gz simulation.launch.py world:=...` only accepts the
  built-in world names (construction, empty, office, orchard, pipeline,
  solar_farm, warehouse) and always appends ".sdf" to them, so it cannot load a
  world from an arbitrary path. This launch file starts Gazebo itself with
  *your* SDF and then reuses Clearpath's own robot_spawn.launch.py, which does
  the real work (generate description/params, ros2_control, bridges, spawn).

Place at:  warehouse_ws/src/warehouse_sim/launch/warehouse_clearpath.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def _default_world_file():
    return os.path.join(
        get_package_share_directory('warehouse_sim'), 'worlds', 'warehouse.sdf')


ARGUMENTS = [
    DeclareLaunchArgument(
        'world_file', default_value=_default_world_file(),
        description='Absolute path to your world SDF'),
    DeclareLaunchArgument(
        'world_name', default_value='default',
        description='MUST equal the name in <world name="..."> inside the SDF'),
    DeclareLaunchArgument(
        'setup_path', default_value=[EnvironmentVariable('HOME'), '/clearpath/'],
        description='Clearpath setup path (folder with robot.yaml)'),
    DeclareLaunchArgument(
        'headless', default_value='false', choices=['true', 'false'],
        description='Run Gazebo server only, without the GUI'),
    DeclareLaunchArgument(
        'rviz', default_value='false', choices=['true', 'false'],
        description='Start RViz'),
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false']),
    DeclareLaunchArgument(
        'bridge_pose', default_value='true', choices=['true', 'false'],
        description='Bridge /world/<name>/pose/info to ROS (needed by robot_controllers)'),
    DeclareLaunchArgument('x', default_value='0.0', description='Robot spawn x'),
    DeclareLaunchArgument('y', default_value='0.0', description='Robot spawn y'),
    DeclareLaunchArgument('z', default_value='0.15', description='Robot spawn z'),
    DeclareLaunchArgument('yaw', default_value='0.0', description='Robot spawn yaw'),
]


def launch_setup(context, *args, **kwargs):
    world_file = LaunchConfiguration('world_file').perform(context)
    if not os.path.isfile(world_file):
        raise RuntimeError(f'World file not found: {world_file}')

    headless = LaunchConfiguration('headless').perform(context) == 'true'

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_clearpath_gz = get_package_share_directory('clearpath_gz')
    pkg_warehouse = get_package_share_directory('warehouse_sim')

    # Gazebo resource path: your package (worlds/models/meshes), Clearpath's
    # meshes, and every sourced ROS package share dir (so package:// and
    # model:// URIs resolve for the robot and for the world).
    resource_dirs = [
        os.path.dirname(world_file),
        pkg_warehouse,
        os.path.join(pkg_warehouse, 'models'),
        os.path.join(pkg_clearpath_gz, 'worlds'),
        os.path.join(pkg_clearpath_gz, 'meshes'),
    ]
    resource_dirs += [
        os.path.join(p, 'share')
        for p in os.environ.get('AMENT_PREFIX_PATH', '').split(':') if p
    ]
    existing = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    if existing:
        resource_dirs.append(existing)

    gz_args = f'{world_file} -r -v 4'
    if headless:
        gz_args += ' -s --headless-rendering'

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments=[('gz_args', gz_args)],
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )

    # Gazebo world pose of every entity -> ROS (tf2_msgs/TFMessage). The
    # robot_controllers node looks up object / shelf poses from this topic.
    world_name = LaunchConfiguration('world_name').perform(context)
    pose_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='pose_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[f'/world/{world_name}/pose/info'
                   '@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
    )

    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_clearpath_gz, 'launch', 'robot_spawn.launch.py')),
        launch_arguments=[
            ('setup_path', LaunchConfiguration('setup_path')),
            ('world', LaunchConfiguration('world_name')),
            ('use_sim_time', LaunchConfiguration('use_sim_time')),
            ('rviz', LaunchConfiguration('rviz')),
            ('x', LaunchConfiguration('x')),
            ('y', LaunchConfiguration('y')),
            ('z', LaunchConfiguration('z')),
            ('yaw', LaunchConfiguration('yaw')),
        ],
    )

    actions = [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', ':'.join(resource_dirs)),
        gz_sim,
        clock_bridge,
        robot_spawn,
    ]
    if LaunchConfiguration('bridge_pose').perform(context) == 'true':
        actions.append(pose_bridge)
    return actions


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld