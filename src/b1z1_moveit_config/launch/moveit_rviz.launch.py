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
    rviz_config = LaunchConfiguration('rviz_config').perform(context).strip()

    moveit_config = (
        MoveItConfigsBuilder('b1z1', package_name='b1z1_moveit_config')
        .robot_description(
            file_path=urdf_xacro_path,
            mappings={'namespace': ns, 'prefix': prefix},
        )
        .robot_description_semantic(
            file_path='config/b1z1.srdf.xacro',
            mappings={'prefix': prefix},
        )
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )

    arguments = ['-d', rviz_config] if rviz_config else []

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        namespace=ns,
        name='rviz2',
        output='screen',
        arguments=arguments,
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {'use_sim_time': use_sim_time},
        ],
    )

    return [rviz_node]


def generate_launch_description():
    """Launch RViz2 with B1Z1 MoveIt 2 parameters (URDF, SRDF, kinematics, OMPL)."""
    pkg_b1z1_description = get_package_share_directory('b1z1_description')
    urdf_xacro_path = os.path.join(pkg_b1z1_description, 'xacro', 'b1z1.urdf.xacro')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='b1z1',
            description='ROS namespace for MoveIt RViz2',
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
            'rviz_config',
            default_value='',
            description='Optional path to an .rviz config file',
        ),
        OpaqueFunction(function=_launch_setup, args=[urdf_xacro_path]),
    ])
