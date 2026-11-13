import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_b1z1 = get_package_share_directory('b1z1_description')
    xacro_file = os.path.join(pkg_b1z1, 'xacro', 'b1z1.urdf.xacro')

    # Ensure Gazebo Sim finds meshes in install share directory
    pkg_share_parent = os.path.dirname(pkg_b1z1)
    existing_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join(filter(None, [pkg_share_parent, pkg_b1z1, existing_resource_path]))
    )

    # Launch arguments for spawn pose
    name_arg = DeclareLaunchArgument('name', default_value='b1z1', description='Model name in Gazebo')
    x_arg = DeclareLaunchArgument('x', default_value='0.0', description='Spawn X position')
    y_arg = DeclareLaunchArgument('y', default_value='16.0', description='Spawn Y position (staging area)')
    z_arg = DeclareLaunchArgument('z', default_value='0.70', description='Spawn Z position')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn Yaw rotation')

    # Generate robot_description from xacro
    robot_description = Command(['xacro ', xacro_file])

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

    # ROS 2 <-> Gazebo Bridge for Joint States
    joint_state_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='b1z1_joint_state_bridge',
        output='screen',
        arguments=[
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model'
        ]
    )

    return LaunchDescription([
        gz_resource_path,
        name_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        robot_state_publisher_node,
        spawn_entity_node,
        joint_state_bridge
    ])
