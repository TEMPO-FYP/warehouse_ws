"""
Launch the custom warehouse world and spawn the Clearpath Husky A200 + UR5e into it.

Why this file exists:
  `ros2 launch clearpath_gz simulation.launch.py world:=...` only accepts the
  built-in world names (construction, empty, office, orchard, pipeline,
  solar_farm, warehouse) and always appends ".sdf" to them, so it cannot load a
  world from an arbitrary path. This launch file starts Gazebo itself with
  *your* SDF and then reuses Clearpath's own robot_spawn.launch.py, which does
  the real work (generate description/params, ros2_control, bridges, spawn).

The robot is defined by warehouse_sim/config/mobile_manipulator.yaml, which is
copied to <setup_path>/robot.yaml before Clearpath's generators run. Each robot
needs its own setup_path, so this one defaults to ~/clearpath_mobile/ (the
transport Husky uses ~/clearpath/).

Clearpath's controller spawners give up after 10 s per controller switch (and
share a lock), which is too short when the sim runs well below real time
(Gazebo on CPU), leaving the wheels and arm without controllers. The
ensure_controllers node then loads/configures/activates whatever is not active
yet, with long timeouts and without the spawner lock.

A robot spawned while the Gazebo GUI is still loading the world exists on the
server but never shows up in the GUI. So, as on the transport-robot branch,
scripts/wait_for_gazebo.sh first waits for the world's /create service and
then gui_settle_sec (skipped when headless); the robot is spawned only once
it exits successfully.

Place at:  warehouse_ws/src/warehouse_sim/launch/warehouse_clearpath.launch.py
"""

import os
import shutil

from ament_index_python.packages import get_package_share_directory

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
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
import yaml


def _default_world_file():
    return os.path.join(
        get_package_share_directory('warehouse_sim'), 'worlds', 'warehouse.sdf')


def _default_robot_config():
    return os.path.join(
        get_package_share_directory('warehouse_sim'), 'config', 'mobile_manipulator.yaml')


# Controllers ensure_controllers brings up if Clearpath's spawners did not
ROBOT_CONTROLLERS = [
    'joint_state_broadcaster',
    'platform_velocity_controller',
    'arm_0_joint_trajectory_controller',
    'arm_0_gripper_controller',
]


ARGUMENTS = [
    DeclareLaunchArgument(
        'world_file', default_value=_default_world_file(),
        description='Absolute path to your world SDF'),
    DeclareLaunchArgument(
        'world_name', default_value='default',
        description='MUST equal the name in <world name="..."> inside the SDF'),
    DeclareLaunchArgument(
        'setup_path', default_value=[EnvironmentVariable('HOME'), '/clearpath_mobile/'],
        description='Clearpath setup path (folder with robot.yaml)'),
    DeclareLaunchArgument(
        'robot_config', default_value=_default_robot_config(),
        description='robot.yaml to copy into setup_path; empty to use the '
                    'robot.yaml already in setup_path'),
    DeclareLaunchArgument(
        'headless', default_value='false', choices=['true', 'false'],
        description='Run Gazebo server only, without the GUI'),
    DeclareLaunchArgument(
        'gui_settle_sec', default_value='30.0',
        description='Seconds to let the Gazebo GUI load the world before spawning '
                    'the robot (otherwise it is invisible in the GUI); ignored when headless'),
    DeclareLaunchArgument(
        'rviz', default_value='false', choices=['true', 'false'],
        description='Start RViz'),
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false']),
    DeclareLaunchArgument('x', default_value='0.0', description='Robot spawn x'),
    # (0, 0) is inside a mapped obstacle; (0, -3) is clear and 2 m from the
    # transport Husky's spawn at (0, -5).
    DeclareLaunchArgument('y', default_value='-3.0', description='Robot spawn y'),
    DeclareLaunchArgument('z', default_value='0.15', description='Robot spawn z'),
    DeclareLaunchArgument('yaw', default_value='0.0', description='Robot spawn yaw'),
]


def launch_setup(context, *args, **kwargs):
    world_file = LaunchConfiguration('world_file').perform(context)
    if not os.path.isfile(world_file):
        raise RuntimeError(f'World file not found: {world_file}')

    headless = LaunchConfiguration('headless').perform(context) == 'true'
    world_name = LaunchConfiguration('world_name').perform(context)
    gui_settle_sec = 0.0 if headless else float(
        LaunchConfiguration('gui_settle_sec').perform(context))

    # Put the robot definition in this robot's own setup folder
    setup_path = LaunchConfiguration('setup_path').perform(context)
    robot_config = LaunchConfiguration('robot_config').perform(context)
    robot_yaml = os.path.join(setup_path, 'robot.yaml')
    if robot_config:
        os.makedirs(setup_path, exist_ok=True)
        shutil.copyfile(robot_config, robot_yaml)
    if not os.path.isfile(robot_yaml):
        raise RuntimeError(f'No robot.yaml in setup_path: {robot_yaml}')
    with open(robot_yaml, 'r', encoding='utf-8') as f:
        namespace = yaml.safe_load(f)['system']['ros2']['namespace']

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

    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_clearpath_gz, 'launch', 'robot_spawn.launch.py')),
        launch_arguments=[
            ('setup_path', setup_path),
            ('world', LaunchConfiguration('world_name')),
            ('use_sim_time', LaunchConfiguration('use_sim_time')),
            ('rviz', LaunchConfiguration('rviz')),
            ('x', LaunchConfiguration('x')),
            ('y', LaunchConfiguration('y')),
            ('z', LaunchConfiguration('z')),
            ('yaw', LaunchConfiguration('yaw')),
        ],
    )

    # Spawn only once the world is up and the GUI has loaded it
    wait_for_gazebo = ExecuteProcess(
        cmd=['bash', os.path.join(pkg_warehouse, 'scripts', 'wait_for_gazebo.sh'),
             world_name, '120', str(int(gui_settle_sec))],
        output='screen',
    )

    def _on_wait_exit(event, _context):
        if event.returncode != 0:
            return [LogInfo(msg='[warehouse_clearpath] Gazebo world not ready; '
                                'robot not spawned.')]
        return [robot_spawn]

    spawn_when_ready = RegisterEventHandler(
        OnProcessExit(target_action=wait_for_gazebo, on_exit=_on_wait_exit)
    )

    ensure_controllers = Node(
        package='warehouse_sim',
        executable='ensure_controllers',
        namespace=namespace,
        output='screen',
        parameters=[{
            'controller_manager': f'/{namespace}/controller_manager',
            'controllers': ROBOT_CONTROLLERS,
            'timeout_sec': 900.0,
            'switch_timeout_sec': 120.0,
        }],
    )

    return [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', ':'.join(resource_dirs)),
        gz_sim,
        clock_bridge,
        wait_for_gazebo,
        spawn_when_ready,
        ensure_controllers,
    ]


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
