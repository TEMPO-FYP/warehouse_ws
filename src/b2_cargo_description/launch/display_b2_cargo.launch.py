import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_desc = get_package_share_directory('b2_cargo_description')
    xacro_file = os.path.join(pkg_desc, 'xacro', 'b2_cargo.urdf.xacro')
    rviz_config = os.path.join(pkg_desc, 'config', 'display_b2_cargo.rviz')

    use_gui = LaunchConfiguration('gui')
    use_rviz = LaunchConfiguration('use_rviz')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str,
    )

    standing_zeros = {
        'FL_hip_joint': 0.0,
        'FR_hip_joint': 0.0,
        'RL_hip_joint': 0.0,
        'RR_hip_joint': 0.0,
        'FL_thigh_joint': 0.67,
        'FR_thigh_joint': 0.67,
        'RL_thigh_joint': 0.67,
        'RR_thigh_joint': 0.67,
        'FL_calf_joint': -1.30,
        'FR_calf_joint': -1.30,
        'RL_calf_joint': -1.30,
        'RR_calf_joint': -1.30,
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Launch joint_state_publisher_gui sliders for leg joints',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz2 preconfigured for b2_cargo inspection',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_base_link_standing',
            arguments=[
                '--x', '0.0',
                '--y', '0.0',
                '--z', '0.5891',
                '--frame-id', 'world',
                '--child-frame-id', 'base_link',
            ],
            output='screen',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            condition=IfCondition(use_gui),
            parameters=[{
                'robot_description': robot_description,
                'zeros': standing_zeros,
            }],
            output='screen',
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            condition=UnlessCondition(use_gui),
            parameters=[{
                'robot_description': robot_description,
                'zeros': standing_zeros,
            }],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            condition=IfCondition(use_rviz),
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
