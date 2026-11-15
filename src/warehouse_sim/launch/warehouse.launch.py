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

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': f'{gz_flags} {resolved_world}'
            }.items(),
        )
    ]


def generate_launch_description():
    pkg_warehouse_sim = get_package_share_directory('warehouse_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Path to SDF world
    default_world_path = os.path.join(pkg_warehouse_sim, 'worlds', 'warehouse.sdf')
    models_path = os.path.join(pkg_warehouse_sim, 'models')
    worlds_path = os.path.join(pkg_warehouse_sim, 'worlds')

    # Resolve b1z1_description parent share path for package:// URI resolution
    pkg_b1z1 = get_package_share_directory('b1z1_description')
    b1z1_share_parent = os.path.dirname(pkg_b1z1)

    # Declare world argument
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world_path,
        description='Path to the SDF world file to load'
    )
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        choices=['true', 'false'],
        description='Run Gazebo Sim server headless (-s)'
    )

    # Set GZ_SIM_RESOURCE_PATH so Gazebo Sim finds all local models
    existing_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    resource_paths = [
        models_path,
        worlds_path,
        b1z1_share_parent,
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

    # Bridge Gazebo world entity poses to ROS 2 (/world/default/pose/info)
    pose_bridge = Node(
        package='warehouse_sim',
        executable='gz_pose_bridge',
        name='pose_bridge',
        output='screen',
    )

    return LaunchDescription([
        world_arg,
        headless_arg,
        gz_resource_path,
        gz_sim,
        clock_bridge,
        pose_bridge,
    ])
