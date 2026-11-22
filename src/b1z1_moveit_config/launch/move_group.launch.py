import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def _resolve_namespace(raw_ns: str) -> str:
    clean = raw_ns.strip().strip('/')
    return clean or 'b1z1'


def _launch_setup(context, urdf_xacro_path: str):
    ns = _resolve_namespace(LaunchConfiguration('namespace').perform(context))
    raw_prefix = LaunchConfiguration('prefix').perform(context).strip()
    prefix = raw_prefix if raw_prefix else f'{ns}_'
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).strip().lower() == 'true'
    enable_video_cam = LaunchConfiguration('enable_video_cam').perform(context).strip()

    moveit_config = (
        MoveItConfigsBuilder('b1z1', package_name='b1z1_moveit_config')
        .robot_description(
            file_path=urdf_xacro_path,
            mappings={
                'namespace': ns,
                'prefix': prefix,
                'enable_video_cam': enable_video_cam,
            },
        )
        .robot_description_semantic(
            file_path='config/b1z1.srdf.xacro',
            mappings={'prefix': prefix},
        )
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        namespace=ns,
        name='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {
                'use_sim_time': use_sim_time,
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

    return [move_group_node]


def generate_launch_description():
    pkg_b1z1_description = get_package_share_directory('b1z1_description')
    urdf_xacro_path = os.path.join(pkg_b1z1_description, 'xacro', 'b1z1.urdf.xacro')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='b1z1',
            description='ROS namespace for move_group',
        ),
        DeclareLaunchArgument(
            'prefix',
            default_value='',
            description='TF/URDF/SRDF link prefix (defaults to <namespace>_ when empty)',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock',
        ),
        DeclareLaunchArgument(
            'enable_video_cam',
            default_value='false',
            description='Include optional third-person follow camera link in MoveIt robot_description',
        ),
        OpaqueFunction(function=_launch_setup, args=[urdf_xacro_path]),
    ])
