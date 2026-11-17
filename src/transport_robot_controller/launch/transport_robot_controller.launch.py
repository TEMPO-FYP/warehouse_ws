"""
Transport robot controller with its own Gazebo world-pose bridge.

Starts:
  * ros_gz_bridge: /world/<world>/pose/info (gz.msgs.Pose_V) ->
    tf2_msgs/TFMessage, used to resolve Gazebo entity names to poses.
  * transport_robot_controller: executes navigate_to subtasks via Nav2.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Bring up the pose bridge and the transport robot controller."""
    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')

    args = [
        DeclareLaunchArgument(
            'world', default_value='default',
            description='Gazebo world name (must match world.name in the controller config)'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true', choices=['true', 'false'],
            description='Use Gazebo /clock'),
    ]

    pose_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='transport_pose_bridge',
        output='screen',
        arguments=[
            ['/world/', world, '/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    controller = Node(
        package='transport_robot_controller',
        executable='transport_robot_controller',
        name='transport_robot_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription(args + [pose_bridge, controller])
