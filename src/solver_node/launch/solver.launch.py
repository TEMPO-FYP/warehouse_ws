"""Launch file for the TEMPO Task Allocation Solver node."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for the TEMPO Solver node."""
    solver_node = Node(
        package='solver_node',
        executable='solver_node',
        name='solver_node',
        output='screen',
    )

    return LaunchDescription(
        [
            solver_node,
        ]
    )
