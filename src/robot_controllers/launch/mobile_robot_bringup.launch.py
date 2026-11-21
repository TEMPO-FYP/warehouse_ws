"""
Nav2, MoveIt and the controller for the TEMPO mobile manipulator (a200_0000).

Run it after (or while) the world and robot are launched by warehouse_sim:

  ros2 launch warehouse_sim warehouse_clearpath.launch.py                  # warehouse
  ros2 launch robot_controllers mobile_robot_bringup.launch.py world:=warehouse

It waits until the spawned robot is up (lidar scan, joint states, the four
ros2_control controllers), then starts, one stage after another (starting
everything at once floods DDS discovery and loses lifecycle service replies,
which leaves Nav2 half-configured):

Under the robot's namespace:
  * Localization: map_server with the world's map (Gazebo world coordinates)
    + AMCL with the initial pose preset to the spawn pose.
  * Navigation: Nav2 (planner, MPPI controller, behaviours, bt_navigator ->
    /a200_0000/navigate_to_pose) with clearpath_nav2_demos' A200 parameters.
  * MoveIt: move_group (-> /a200_0000/move_action) with the config
    Clearpath's generators wrote to <setup_path> when the robot was spawned.

  * mobile_robot_controller (controller:=true, the default).

world:=warehouse|empty picks the map and initial pose from config/worlds.yaml
(the spawn pose of warehouse_clearpath.launch.py); map:=, initial_x:=,
initial_y:= and initial_yaw:= override them. localization:=, nav2:=, moveit:=
and controller:= switch parts off.

The navigation part mirrors clearpath_nav2_demos/launch/nav2.launch.py, which
hard-codes its parameter file, so that timing parameters can be relaxed for a
simulation running well below real time (Gazebo on CPU):
  * bt_navigator default_server_timeout: Clearpath's 20 ms (wall time) is too
    short for the planner to acknowledge a replanning request on a loaded
    CPU with the full warehouse map; 5 s.
  * progress_checker movement_time_allowance and costmap_update_timeout.
  * transform tolerances (costmaps, MPPI, behaviours, collision monitor):
    TF lags behind when the EKF cannot keep its update rate.
  * lifecycle manager bond_timeout: with the CPU saturated, Nav2 servers can
    miss the default 4 s heartbeat and get shut down ("CRITICAL FAILURE ...
    IS DOWN"). nav2_bringup does not pass the params file to the lifecycle
    managers, so it is set with SetParameter.
It also raises MPPI's top speed (vx_max) to max_speed and tightens the goal
tolerance so the arm can reach objects after navigate_to.

MoveIt mirrors clearpath_manipulators/launch/moveit.launch.py (which also
hard-codes its parameters) and disables trajectory execution-duration
monitoring: MoveIt measures it in wall time, so in a slow simulation it would
cancel every arm motion part-way.

"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetParameter
from nav2_common.launch import RewrittenYaml
import xacro
import yaml


def load_world_presets():
    """Return config/worlds.yaml with package-relative paths resolved."""
    path = os.path.join(
        get_package_share_directory('robot_controllers'), 'config', 'worlds.yaml')
    with open(path, 'r', encoding='utf-8') as f:
        presets = yaml.safe_load(f)
    for preset in presets.values():
        pkg, rel = preset['map']
        preset['map'] = os.path.join(get_package_share_directory(pkg), rel)
    return presets


ROBOT_CONTROLLERS = [
    'joint_state_broadcaster',
    'platform_velocity_controller',
    'arm_0_joint_trajectory_controller',
    'arm_0_gripper_controller',
]
LOCALIZATION_NODES = ['map_server', 'amcl']
NAV2_NODES = ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator']


ARGUMENTS = [
    DeclareLaunchArgument(
        'world', default_value='warehouse', choices=['warehouse', 'empty'],
        description='World preset (map and initial pose) from config/worlds.yaml'),
    DeclareLaunchArgument(
        'namespace', default_value='a200_0000',
        description='Robot namespace (must match its robot.yaml)'),
    DeclareLaunchArgument(
        'setup_path',
        default_value=[EnvironmentVariable('HOME'), '/clearpath_mobile/'],
        description='Clearpath setup path used by warehouse_clearpath.launch.py'),
    DeclareLaunchArgument(
        'map', default_value='',
        description='Map yaml in Gazebo world coordinates (default: the world preset)'),
    DeclareLaunchArgument(
        'use_sim_time', default_value='true', choices=['true', 'false'],
        description='Use Gazebo /clock'),
    DeclareLaunchArgument(
        'scan_topic', default_value='',
        description='2D lidar scan topic (default /<namespace>/sensors/lidar2d_0/scan)'),
    DeclareLaunchArgument(
        'max_speed', default_value='1.0',
        description='Max forward speed in m/s (Clearpath default 0.5; the A200 '
                    'wheel controller and velocity smoother allow 1.0)'),
    DeclareLaunchArgument(
        'goal_tolerance', default_value='0.15',
        description='Nav2 xy and yaw goal tolerance (Clearpath default 0.25)'),
    DeclareLaunchArgument(
        'initial_x', default_value='',
        description='Initial x in the map (default: the world preset spawn pose)'),
    DeclareLaunchArgument('initial_y', default_value='', description='Initial y'),
    DeclareLaunchArgument('initial_yaw', default_value='', description='Initial yaw'),
    DeclareLaunchArgument(
        'localization', default_value='true', choices=['true', 'false'],
        description='Start map_server and AMCL'),
    DeclareLaunchArgument(
        'nav2', default_value='true', choices=['true', 'false'],
        description='Start Nav2 (needs localization)'),
    DeclareLaunchArgument(
        'moveit', default_value='true', choices=['true', 'false'],
        description='Start MoveIt (needed by pick and place)'),
    DeclareLaunchArgument(
        'controller', default_value='true', choices=['true', 'false'],
        description='Start mobile_robot_controller (TEMPO subtasks)'),
    DeclareLaunchArgument(
        'stage_timeout_sec', default_value='1800.0',
        description='Give up waiting for a stage after this long (wall time)'),
]


def launch_setup(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    pkg_self = get_package_share_directory('robot_controllers')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_clearpath_nav2 = get_package_share_directory('clearpath_nav2_demos')

    preset = load_world_presets()[arg('world')]
    namespace = arg('namespace')
    setup_path = arg('setup_path')
    use_sim_time = arg('use_sim_time')
    map_yaml = arg('map') or preset['map']
    scan_topic = arg('scan_topic') or f'/{namespace}/sensors/lidar2d_0/scan'
    initial = {
        'x': arg('initial_x') or str(preset['spawn']['x']),
        'y': arg('initial_y') or str(preset['spawn']['y']),
        'yaw': arg('initial_yaw') or str(preset['spawn']['yaw']),
    }
    timeout = float(arg('stage_timeout_sec'))

    # Applies to every node in the group; only the lifecycle managers use it.
    bond_timeout = SetParameter(name='bond_timeout', value=20.0)

    def localization_actions():
        amcl_params = RewrittenYaml(
            source_file=os.path.join(pkg_self, 'config', 'mobile_robot_localization.yaml'),
            param_rewrites={
                'amcl.ros__parameters.initial_pose.x': initial['x'],
                'amcl.ros__parameters.initial_pose.y': initial['y'],
                'amcl.ros__parameters.initial_pose.yaw': initial['yaw'],
                'amcl.ros__parameters.scan_topic': scan_topic,
            },
            convert_types=True,
        )
        # Same structure as clearpath_nav2_demos/launch/localization.launch.py,
        # but with our map and AMCL params (initial pose preset).
        return [GroupAction([
            PushRosNamespace(namespace),
            bond_timeout,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_nav2_bringup, 'launch', 'localization_launch.py')),
                launch_arguments=[
                    ('namespace', namespace),
                    ('map', map_yaml),
                    ('use_sim_time', use_sim_time),
                    ('params_file', amcl_params),
                    ('autostart', 'true'),
                    ('use_composition', 'False'),
                    ('use_respawn', 'false'),
                ],
            ),
        ])]

    def nav2_actions():
        goal_tolerance = arg('goal_tolerance')
        nav2_params = RewrittenYaml(
            source_file=os.path.join(pkg_clearpath_nav2, 'config', 'a200', 'nav2.yaml'),
            param_rewrites={
                # From clearpath_nav2_demos/launch/nav2.launch.py
                'local_costmap.local_costmap.ros__parameters.voxel_layer.scan.topic':
                    scan_topic,
                'global_costmap.global_costmap.ros__parameters.obstacle_layer.scan.topic':
                    scan_topic,
                'collision_monitor.ros__parameters.scan.topic': scan_topic,
                'controller_server.ros__parameters.FollowPath.motion_model': 'DiffDrive',
                'controller_server.ros__parameters.FollowPath.TwirlingCritic.enabled':
                    'false',
                'global_costmap.global_costmap.ros__parameters.rolling_window': 'false',
                # Top speed (Clearpath: 0.5 m/s)
                'controller_server.ros__parameters.FollowPath.vx_max': arg('max_speed'),
                # Stop close enough for the arm to reach (Clearpath: 0.25)
                'controller_server.ros__parameters.general_goal_checker.xy_goal_tolerance':
                    goal_tolerance,
                'controller_server.ros__parameters.general_goal_checker.yaw_goal_tolerance':
                    goal_tolerance,
                # Slow-sim tolerances (Clearpath: 20 ms, 10 s, 0.3 s)
                # (planning on the full warehouse map can take a few seconds)
                'bt_navigator.ros__parameters.default_server_timeout': '5000',
                'controller_server.ros__parameters.progress_checker.movement_time_allowance':
                    '30.0',
                'controller_server.ros__parameters.costmap_update_timeout': '2.0',
                # TF lags under load (Clearpath: 0.1, 0.1, 0.2 s)
                'controller_server.ros__parameters.FollowPath.transform_tolerance': '0.5',
                'behavior_server.ros__parameters.transform_tolerance': '0.5',
                'collision_monitor.ros__parameters.transform_tolerance': '0.5',
            },
            convert_types=True,
        )
        return [GroupAction([
            PushRosNamespace(namespace),
            bond_timeout,
            # Not in Clearpath's file, so set for every node (incl. the costmap
            # sub-nodes; default 0.3 s), which then tolerate TF lagging under load
            SetParameter(name='transform_tolerance', value=1.0),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')),
                launch_arguments=[
                    ('namespace', namespace),
                    ('use_sim_time', use_sim_time),
                    ('params_file', nav2_params),
                    ('autostart', 'true'),
                    ('use_composition', 'False'),
                    ('use_respawn', 'false'),
                ],
            ),
        ])]

    def moveit_actions():
        # Clearpath's generators write these into setup_path when the robot is
        # spawned, so they are read only once the robot is up
        moveit_yaml = os.path.join(setup_path, 'manipulators', 'config', 'moveit.yaml')
        if not os.path.isfile(moveit_yaml):
            return [LogInfo(msg=f'[bringup] {moveit_yaml} not found; MoveIt not started.')]
        return [Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='log',
            namespace=namespace,
            parameters=[
                moveit_yaml,
                {'robot_description': xacro.process_file(
                    os.path.join(setup_path, 'robot.urdf.xacro')).toxml()},
                {'robot_description_semantic': xacro.process_file(
                    os.path.join(setup_path, 'robot.srdf')).toxml()},
                {
                    'use_sim_time': use_sim_time == 'true',
                    # Measured in wall time; aborts arm motions in a slow sim
                    'trajectory_execution.execution_duration_monitoring': False,
                    # Joint states lag behind in a slow sim
                    'trajectory_execution.allowed_start_tolerance': 0.05,
                },
            ],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
                ('joint_states', 'platform/joint_states'),
            ],
        )]

    def controller_actions():
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_self, 'launch', 'mobile_robot_controller.launch.py')),
            launch_arguments=[('use_sim_time', use_sim_time)],
        )]

    def waiter(name, **params):
        return Node(
            package='robot_controllers',
            executable='wait_for_robot',
            name=f'wait_for_{name}',
            namespace=namespace,
            output='screen',
            parameters=[{'timeout_sec': timeout, **params}],
        )

    # Stages: (message, builder, waiter that must succeed before the next stage)
    stages = []
    if arg('localization') == 'true':
        stages.append(('Starting localization.', localization_actions, waiter(
            'localization', lifecycle_nodes=[f'/{namespace}/{n}' for n in LOCALIZATION_NODES])))
    if arg('nav2') == 'true':
        stages.append(('Starting Nav2.', nav2_actions, waiter(
            'nav2', lifecycle_nodes=[f'/{namespace}/{n}' for n in NAV2_NODES])))
    if arg('moveit') == 'true':
        stages.append(('Starting MoveIt.', moveit_actions, None))
    if arg('controller') == 'true':
        stages.append(('Starting mobile_robot_controller.', controller_actions, None))

    wait_robot = waiter(
        'robot',
        topics=['sensors/lidar2d_0/scan', 'platform/joint_states'],
        topic_types=['sensor_msgs/msg/LaserScan', 'sensor_msgs/msg/JointState'],
        controller_manager=f'/{namespace}/controller_manager',
        controllers=ROBOT_CONTROLLERS,
    )

    def chain(index):
        """Actions that start stage `index` (and every stage after it)."""
        if index >= len(stages):
            return []
        message, build, wait = stages[index]
        actions = [LogInfo(msg=f'[bringup] {message}'), *build()]
        if wait is None:
            return actions + chain(index + 1)
        return actions + [wait, after(wait, index + 1)]

    def after(waiting, index):
        def on_exit(event, _context):
            if event.returncode != 0:
                return [LogInfo(msg='[bringup] A stage did not become ready (see the '
                                    'wait_for_* messages above); stopping here.')]
            return chain(index)
        return RegisterEventHandler(OnProcessExit(target_action=waiting, on_exit=on_exit))

    return [
        LogInfo(msg='[bringup] Waiting for the robot (launch warehouse_sim/'
                    'warehouse_clearpath.launch.py if it is not running).'),
        wait_robot,
        after(wait_robot, 0),
    ]


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
