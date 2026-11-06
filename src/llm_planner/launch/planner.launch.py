"""
planner.launch.py

ROS 2 Launch file for the TEMPO LLM Planner node.
Launches the node listening on '/mission/instruction' and publishing to '/mission/plan'.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    planner_node = Node(
        package="llm_planner",
        executable="planner_node",
        name="llm_planner_node",
        output="screen",
    )

    return LaunchDescription(
        [
            planner_node,
        ]
    )
