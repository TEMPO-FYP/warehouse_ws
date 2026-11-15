"""Launch the TEMPO planning, allocation, dispatching, and robot control pipeline."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch llm_planner, solver_node, dispatcher_node, and mobile_robot_controller."""
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

    robot_controller_node = Node(
        package='robot_controllers',
        executable='mobile_robot_controller',
        name='mobile_robot_controller',
        output='screen',
    )

    return LaunchDescription([
        planner_node,
        solver_node,
        dispatcher_node,
        robot_controller_node,
    ])
