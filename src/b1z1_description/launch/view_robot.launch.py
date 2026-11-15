import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_b1z1 = get_package_share_directory('b1z1_description')

    # Path to xacro
    default_xacro_path = os.path.join(pkg_b1z1, 'xacro', 'b1z1.urdf.xacro')
    default_rviz_path = os.path.join(pkg_b1z1, 'rviz', 'b1z1.rviz')

    # Launch configuration
    model_arg = DeclareLaunchArgument(
        'model',
        default_value=default_xacro_path,
        description='Absolute path to robot xacro/urdf file'
    )

    rviz_arg = DeclareLaunchArgument(
        'rvizconfig',
        default_value=default_rviz_path,
        description='Absolute path to rviz config file'
    )

    # Command substitution to run xacro
    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]), value_type=str)

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Joint State Publisher GUI Node (provides interactive joint angle sliders)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # RViz2 Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')]
    )

    return LaunchDescription([
        model_arg,
        rviz_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
