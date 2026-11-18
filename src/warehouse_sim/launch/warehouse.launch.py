import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_warehouse_sim = get_package_share_directory('warehouse_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_clearpath_gz = get_package_share_directory('clearpath_gz')
    pkg_clearpath_description = get_package_share_directory('clearpath_platform_description')

    # Path to SDF world
    default_world_path = os.path.join(pkg_warehouse_sim, 'worlds', 'warehouse.sdf')
    models_path = os.path.join(pkg_warehouse_sim, 'models')
    worlds_path = os.path.join(pkg_warehouse_sim, 'worlds')
    clearpath_gz_models = os.path.join(pkg_clearpath_gz, 'models')
    clearpath_gz_meshes = os.path.join(pkg_clearpath_gz, 'meshes')

    # /root/clearpath is generated at container startup by entrypoint.sh
    # It is a container-internal convention, not a host path
    clearpath_setup_path = '/root/clearpath'

    clearpath_gz_launch = '/root/clearpath/platform/launch/platform-service.launch.py'
    clearpath_sensors_launch = '/root/clearpath/sensors/launch/sensors-service.launch.py'

    # Resolve spawn script via ament_index
    spawn_script = os.path.join(pkg_warehouse_sim, 'scripts', 'spawn_robot.sh')

    # os.path.dirname of a package share dir gives the ROS share root e.g. /opt/ros/jazzy/share
    ros_share_path = os.path.dirname(pkg_clearpath_description)

    # Declare world argument
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world_path,
        description='Path to the SDF world file to load'
    )

    # Set GZ_SIM_RESOURCE_PATH so Gazebo Sim finds all local models and Clearpath assets
    existing_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join(filter(None, [
            ros_share_path,
            models_path,
            worlds_path,
            clearpath_setup_path,
            clearpath_gz_models,
            clearpath_gz_meshes,
            existing_resource_path,
        ]))
    )

    # Launch Gazebo Sim unpaused (-r)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r ', LaunchConfiguration('world')]
        }.items(),
    )

    # Spawning before the GUI has loaded the world leaves the robot invisible
    # in the GUI (it still exists on the server). Use 0 for headless runs.
    gui_settle_arg = DeclareLaunchArgument(
        'gui_settle_sec',
        default_value='30',
        description='Seconds to wait after the Gazebo server is up before spawning the Husky',
    )

    # Spawn the Husky. The script polls until Gazebo's /world/default/create
    # service is up, then waits gui_settle_sec for the GUI.
    # /tmp/robot_abs.sdf is generated at container startup by entrypoint.sh
    spawn_husky = ExecuteProcess(
        cmd=['bash', spawn_script, '/tmp/robot_abs.sdf', 'a200_0001', 'default',
             '120', LaunchConfiguration('gui_settle_sec')],
        output='screen',
    )

    # Bring up the Clearpath platform (controllers, bridges) only once the
    # robot actually exists in the world.
    def _on_spawn_exit(event, _context):
        if event.returncode != 0:
            return [LogInfo(msg='[warehouse.launch] Husky spawn failed; '
                                'skipping Clearpath platform launch.')]
        return [
            IncludeLaunchDescription(PythonLaunchDescriptionSource(clearpath_gz_launch)),
            # Bridges sensors from husky_transport.yaml (2D lidar) into ROS
            IncludeLaunchDescription(PythonLaunchDescriptionSource(clearpath_sensors_launch)),
            platform_controller_spawner,
        ]

    # Clearpath's spawner gives up after 10 s per controller switch. With the
    # sim running well below real time (e.g. on CPU), activating
    # joint_state_broadcaster takes longer than that, so the spawner exits and
    # never loads platform_velocity_controller: no wheel odometry, no odom TF
    # (EKF), and the robot ignores velocity commands. Spawn it again here with
    # timeouts that tolerate a slow sim. If Clearpath's spawner already loaded
    # it, this one just logs an error and exits.
    platform_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace='a200_0001',
        arguments=[
            'platform_velocity_controller',
            '--controller-manager', '/a200_0001/controller_manager',
            '--controller-manager-timeout', '120',
            '--switch-timeout', '120',
            '--service-call-timeout', '120',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    clearpath_launch = RegisterEventHandler(
        OnProcessExit(target_action=spawn_husky, on_exit=_on_spawn_exit)
    )

    # Bridge simulation clock to ROS 2
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )

    return LaunchDescription([
        world_arg,
        gui_settle_arg,
        gz_resource_path,
        gz_sim,
        clock_bridge,
        spawn_husky,
        clearpath_launch
    ])