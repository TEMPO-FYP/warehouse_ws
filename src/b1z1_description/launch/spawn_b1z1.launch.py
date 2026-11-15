import math
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _spawn_test_targets(context, pkg_b1z1):
    """Optionally spawn a pedestal and scaled graspable box ahead of b1z1."""
    if LaunchConfiguration('spawn_test_target').perform(context).strip().lower() != 'true':
        return []

    x = float(LaunchConfiguration('x').perform(context))
    y = float(LaunchConfiguration('y').perform(context))
    yaw = float(LaunchConfiguration('yaw').perform(context))

    def make_spawner(node_name, model_name, sdf_rel_path, dist, pos_z):
        px = f'{x + dist * math.cos(yaw):.4f}'
        py = f'{y + dist * math.sin(yaw):.4f}'
        return Node(
            package='ros_gz_sim',
            executable='create',
            name=node_name,
            output='screen',
            arguments=[
                '-file', os.path.join(pkg_b1z1, sdf_rel_path),
                '-name', model_name,
                '-x', px,
                '-y', py,
                '-z', f'{pos_z:.4f}',
                '-Y', f'{yaw:.4f}',
            ],
        )

    box_dist = float(LaunchConfiguration('test_target_dist').perform(context))
    return [
        make_spawner(
            'test_pedestal_spawner',
            'test_pedestal',
            os.path.join('models', 'test_pedestal', 'model.sdf'),
            dist=box_dist + 0.05,
            pos_z=0.35,
        ),
        make_spawner(
            'graspable_box_spawner',
            'graspable_box',
            os.path.join('models', 'graspable_box', 'model.sdf'),
            dist=box_dist,
            pos_z=0.731,
        ),
    ]


def generate_launch_description():
    pkg_b1z1 = get_package_share_directory('b1z1_description')
    xacro_file = os.path.join(pkg_b1z1, 'xacro', 'b1z1.urdf.xacro')
    bridge_config = os.path.join(pkg_b1z1, 'config', 'b1z1_bridge.yaml')

    # Ensure Gazebo Sim finds meshes in install share directory
    pkg_share_parent = os.path.dirname(pkg_b1z1)
    existing_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join(filter(None, [pkg_share_parent, pkg_b1z1, existing_resource_path]))
    )

    # Launch arguments for spawn pose and optional test pick target
    name_arg = DeclareLaunchArgument('name', default_value='b1z1', description='Model name in Gazebo')
    x_arg = DeclareLaunchArgument('x', default_value='0.0', description='Spawn X position')
    y_arg = DeclareLaunchArgument('y', default_value='16.0', description='Spawn Y position (staging area)')
    z_arg = DeclareLaunchArgument('z', default_value='0.65', description='Spawn Z position (clears bent crouch legs before settling)')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn Yaw rotation')
    test_target_arg = DeclareLaunchArgument(
        'spawn_test_target',
        default_value='false',
        description='Spawn static pedestal and scaled graspable_box in front of b1z1',
    )
    test_target_dist_arg = DeclareLaunchArgument(
        'test_target_dist',
        default_value='0.73',
        description='Forward offset from b1z1 spawn for optional test target box',
    )

    # Generate robot_description from xacro
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='b1z1_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    # Spawn entity into Gazebo Sim Harmonic
    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='b1z1_spawner',
        output='screen',
        arguments=[
            '-name', LaunchConfiguration('name'),
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw')
        ]
    )

    # ROS 2 <-> Gazebo Bridge for IMU and Gazebo Odometry
    b1z1_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='b1z1_bridge',
        output='screen',
        parameters=[{'config_file': bridge_config}]
    )

    return LaunchDescription([
        gz_resource_path,
        name_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        test_target_arg,
        test_target_dist_arg,
        robot_state_publisher_node,
        spawn_entity_node,
        b1z1_bridge_node,
        OpaqueFunction(function=_spawn_test_targets, args=[pkg_b1z1]),
    ])
