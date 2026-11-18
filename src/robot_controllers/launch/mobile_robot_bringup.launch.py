"""
Nav2 and MoveIt for the TEMPO mobile manipulator (mobile_robot_1, a200_0000).

Starts, under the robot's namespace:
  * Localization: map_server with warehouse_sim/maps/warehouse_map.yaml
    (Gazebo world coordinates) + AMCL with the initial pose preset to the
    spawn pose (config/mobile_robot_localization.yaml, overridden by the
    initial_x/initial_y/initial_yaw arguments).
  * Navigation: Nav2 (planner, MPPI controller, behaviours, bt_navigator ->
    /a200_0000/navigate_to_pose) with clearpath_nav2_demos' A200 parameters.
  * MoveIt: clearpath_manipulators' move_group (-> /a200_0000/move_action).

The navigation part mirrors clearpath_nav2_demos/launch/nav2.launch.py, which
hard-codes its parameter file, so that a few timing parameters can be relaxed
for a simulation running well below real time (Gazebo on CPU, ~0.1x):
  * bt_navigator default_server_timeout: Clearpath's 20 ms (wall time) is too
    short for the planner to acknowledge a replanning request, which aborts
    the whole goal.
  * progress_checker movement_time_allowance and costmap_update_timeout.
  * lifecycle manager bond_timeout: with the CPU saturated, Nav2 servers can
    miss the default 4 s heartbeat and get shut down ("CRITICAL FAILURE ...
    IS DOWN"). nav2_bringup does not pass the params file to the lifecycle
    managers, so it is set with SetParameter.
It also raises MPPI's top speed (vx_max) from 0.5 to max_speed (1.0 m/s).

Run after warehouse_clearpath.launch.py has spawned the robot (the lidar scan
must be flowing for AMCL).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import PushRosNamespace, SetParameter
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    """Bring up localization, Nav2 and MoveIt for a200_0000."""
    pkg_self = get_package_share_directory('robot_controllers')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_clearpath_nav2 = get_package_share_directory('clearpath_nav2_demos')
    pkg_clearpath_manipulators = get_package_share_directory('clearpath_manipulators')
    pkg_warehouse_sim = get_package_share_directory('warehouse_sim')

    namespace = LaunchConfiguration('namespace')
    setup_path = LaunchConfiguration('setup_path')
    map_yaml = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    scan_topic = LaunchConfiguration('scan_topic')
    max_speed = LaunchConfiguration('max_speed')

    args = [
        DeclareLaunchArgument(
            'namespace', default_value='a200_0000',
            description='Robot namespace (must match its robot.yaml)'),
        DeclareLaunchArgument(
            'setup_path',
            default_value=[EnvironmentVariable('HOME'), '/clearpath_mobile/'],
            description='Clearpath setup path used by warehouse_clearpath.launch.py'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(pkg_warehouse_sim, 'maps', 'warehouse_map.yaml'),
            description='Map yaml (in Gazebo world coordinates)'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true', choices=['true', 'false'],
            description='Use Gazebo /clock'),
        DeclareLaunchArgument(
            'scan_topic', default_value='/a200_0000/sensors/lidar2d_0/scan',
            description='2D lidar scan topic used by the costmaps'),
        DeclareLaunchArgument(
            'max_speed', default_value='1.0',
            description='Max forward speed in m/s (Clearpath default 0.5; the A200 '
                        'wheel controller and velocity smoother allow 1.0)'),
        DeclareLaunchArgument(
            'initial_x', default_value='0.0',
            description='Spawn x (must match warehouse_clearpath.launch.py)'),
        DeclareLaunchArgument(
            'initial_y', default_value='-3.0',
            description='Spawn y (must match warehouse_clearpath.launch.py)'),
        DeclareLaunchArgument(
            'initial_yaw', default_value='0.0',
            description='Spawn yaw (must match warehouse_clearpath.launch.py)'),
        DeclareLaunchArgument(
            'moveit', default_value='true', choices=['true', 'false'],
            description='Start MoveIt (needed by pick and place)'),
    ]

    amcl_params = RewrittenYaml(
        source_file=os.path.join(pkg_self, 'config', 'mobile_robot_localization.yaml'),
        param_rewrites={
            'amcl.ros__parameters.initial_pose.x': LaunchConfiguration('initial_x'),
            'amcl.ros__parameters.initial_pose.y': LaunchConfiguration('initial_y'),
            'amcl.ros__parameters.initial_pose.yaw': LaunchConfiguration('initial_yaw'),
        },
        convert_types=True,
    )

    # Same structure as clearpath_nav2_demos/launch/localization.launch.py,
    # but with our map and AMCL params (initial pose preset).
    # Applies to every node in the group; only the lifecycle managers use it.
    bond_timeout = SetParameter(name='bond_timeout', value=20.0)

    localization = GroupAction([
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
    ])

    # Clearpath's A200 Nav2 parameters, with the rewrites its nav2.launch.py
    # applies for a diff-drive platform on a static map, plus slow-sim timing.
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
            'controller_server.ros__parameters.FollowPath.TwirlingCritic.enabled': 'false',
            'global_costmap.global_costmap.ros__parameters.rolling_window': 'false',
            # Top speed (Clearpath: 0.5 m/s)
            'controller_server.ros__parameters.FollowPath.vx_max': max_speed,
            # Slow-sim tolerances (Clearpath: 20 ms, 10 s, 0.3 s)
            'bt_navigator.ros__parameters.default_server_timeout': '1000',
            'controller_server.ros__parameters.progress_checker.movement_time_allowance':
                '30.0',
            'controller_server.ros__parameters.costmap_update_timeout': '2.0',
        },
        convert_types=True,
    )

    navigation = GroupAction([
        PushRosNamespace(namespace),
        bond_timeout,
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
    ])

    # move_group for the arm; reads <setup_path>/robot.yaml and the MoveIt
    # config Clearpath's generators wrote there when the robot was spawned.
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_clearpath_manipulators, 'launch', 'moveit.launch.py')),
        launch_arguments=[
            ('setup_path', setup_path),
            ('use_sim_time', use_sim_time),
        ],
        condition=IfCondition(LaunchConfiguration('moveit')),
    )

    return LaunchDescription(args + [localization, navigation, moveit])
