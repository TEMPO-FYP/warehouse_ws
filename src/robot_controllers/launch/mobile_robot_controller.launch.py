"""
Mobile robot controller.

Starts mobile_robot_controller, which executes navigate_to, pick and place
subtasks via Nav2, MoveIt and the gripper controller. It reads Gazebo entity
poses from /world/<world>/pose/info directly over gz-transport (world.name in
the controller config), so no ros_gz_bridge is needed.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Bring up the mobile robot controller."""
    use_sim_time = LaunchConfiguration('use_sim_time')

    args = [
        DeclareLaunchArgument(
            'use_sim_time', default_value='true', choices=['true', 'false'],
            description='Use Gazebo /clock'),
    ]

    controller = Node(
        package='robot_controllers',
        executable='mobile_robot_controller',
        name='mobile_robot_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription(args + [controller])
