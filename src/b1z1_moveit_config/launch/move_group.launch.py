import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg_b1z1_description = get_package_share_directory('b1z1_description')
    urdf_xacro_path = os.path.join(pkg_b1z1_description, 'xacro', 'b1z1.urdf.xacro')

    moveit_config = (
        MoveItConfigsBuilder('b1z1', package_name='b1z1_moveit_config')
        .robot_description(file_path=urdf_xacro_path)
        .robot_description_semantic(file_path='config/b1z1.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {
                'use_sim_time': True,
                'publish_planning_scene': True,
                'publish_geometry_updates': True,
                'publish_state_updates': True,
                'publish_transforms_updates': True,
                'trajectory_execution.allowed_execution_duration_scaling': 2.0,
                'trajectory_execution.allowed_goal_duration_margin': 1.0,
                'trajectory_execution.allowed_start_tolerance': 0.05,
            },
        ],
    )

    return LaunchDescription([
        move_group_node,
    ])
