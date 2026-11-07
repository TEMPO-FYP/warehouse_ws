"""Launch file for the TEMPO LLM Planner node with 'plan_mission' Action Server."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for the TEMPO LLM Planner node."""
    planner_node = Node(
        package='llm_planner',
        executable='planner_node',
        name='llm_planner_node',
        output='screen',
    )

    return LaunchDescription(
        [
            planner_node,
        ]
    )
