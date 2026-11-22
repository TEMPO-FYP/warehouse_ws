"""Launch the TEMPO planning, allocation, dispatching, and robot control pipeline."""

from launch.actions import DeclareLaunchArgument
from launch.launch_description import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Launch llm_planner, solver_node, dispatcher_node, and mobile_robot_controller."""
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        choices=['true', 'false'],
        description='Use simulation clock across TEMPO pipeline nodes',
    )
    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)

    planner_node = Node(
        package='llm_planner',
        executable='planner_node',
        name='llm_planner_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    solver_node = Node(
        package='solver_node',
        executable='solver_node',
        name='solver_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    dispatcher_node = Node(
        package='dispatcher_node',
        executable='dispatcher_node',
        name='dispatcher_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    robot_controller_node = Node(
        package='robot_controllers',
        executable='mobile_robot_controller',
        name='mobile_robot_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        use_sim_time_arg,
        planner_node,
        solver_node,
        dispatcher_node,
        robot_controller_node,
    ])
