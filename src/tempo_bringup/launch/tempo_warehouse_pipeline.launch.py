"""Launch the TEMPO planning, allocation, dispatching, and robot control pipeline."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch llm_planner, solver_node, dispatcher_node, and robot controllers."""
    use_mobile = DeclareLaunchArgument(
        'use_mobile_controller',
        default_value='true',
        choices=['true', 'false'],
        description='Launch mobile_robot_controller for B1Z1',
    )
    use_transport = DeclareLaunchArgument(
        'use_transport_controller',
        default_value='false',
        choices=['true', 'false'],
        description='Launch transport_robot_controller for b2_cargo',
    )

    planner_node = Node(
        package='llm_planner',
        executable='planner_node',
        name='llm_planner_node',
        output='screen',
    )

    solver_node = Node(
        package='solver_node',
        executable='solver_node',
        name='solver_node',
        output='screen',
    )

    dispatcher_node = Node(
        package='dispatcher_node',
        executable='dispatcher_node',
        name='dispatcher_node',
        output='screen',
    )

    mobile_controller_node = Node(
        package='robot_controllers',
        executable='mobile_robot_controller',
        name='mobile_robot_controller',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_mobile_controller')),
    )

    transport_controller_node = Node(
        package='robot_controllers',
        executable='transport_robot_controller',
        name='transport_robot_controller',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_transport_controller')),
    )

    return LaunchDescription([
        use_mobile,
        use_transport,
        planner_node,
        solver_node,
        dispatcher_node,
        mobile_controller_node,
        transport_controller_node,
    ])
