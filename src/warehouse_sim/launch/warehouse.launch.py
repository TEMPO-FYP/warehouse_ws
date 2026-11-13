import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# testing xyz

def generate_launch_description():
    pkg_warehouse_sim = get_package_share_directory('warehouse_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Path to SDF world
    default_world_path = os.path.join(pkg_warehouse_sim, 'worlds', 'warehouse.sdf')
    models_path = os.path.join(pkg_warehouse_sim, 'models')
    worlds_path = os.path.join(pkg_warehouse_sim, 'worlds')

    # Resolve b1z1_description paths if available
    try:
        pkg_b1z1 = get_package_share_directory('b1z1_description')
        b1z1_paths = [os.path.dirname(pkg_b1z1), pkg_b1z1]
    except Exception:
        b1z1_paths = []

    # Declare world argument
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world_path,
        description='Path to the SDF world file to load'
    )

    # Set GZ_SIM_RESOURCE_PATH so Gazebo Sim finds all local models
    existing_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join(filter(None, [models_path, worlds_path] + b1z1_paths + [existing_resource_path]))
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
        gz_resource_path,
        gz_sim,
        clock_bridge,
    ])
