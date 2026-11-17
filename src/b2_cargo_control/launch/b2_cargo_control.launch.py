import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _setup_control_nodes(context):
    pkg_desc = get_package_share_directory('b2_cargo_description')
    pkg_ctrl = get_package_share_directory('b2_cargo_control')

    ns = LaunchConfiguration('namespace').perform(context).strip('/')
    frame_prefix = f'{ns}/' if ns else ''
    ns_prefix = f'/{ns}' if ns else ''
    cm_name = f'{ns_prefix}/controller_manager'

    xacro_file = os.path.join(pkg_desc, 'xacro', 'b2_cargo.urdf.xacro')
    joints_yaml = os.path.join(pkg_ctrl, 'config', 'champ', 'joints.yaml')
    links_yaml = os.path.join(pkg_ctrl, 'config', 'champ', 'links.yaml')
    gait_yaml = os.path.join(pkg_ctrl, 'config', 'champ', 'gait.yaml')
    ekf_yaml = os.path.join(pkg_ctrl, 'config', 'ekf', 'odom_ekf.yaml')

    robot_description_str = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' namespace:=', ns,
            ' frame_prefix:=', frame_prefix,
        ]),
        value_type=str,
    )

    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_to_odom',
        namespace=ns,
        output='screen',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'map', '--child-frame-id', f'{frame_prefix}odom',
        ],
        parameters=[{'use_sim_time': True}],
    )

    static_tf_footprint_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_footprint_to_base',
        namespace=ns,
        output='screen',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', f'{frame_prefix}base_footprint',
            '--child-frame-id', f'{frame_prefix}base_link',
        ],
        parameters=[{'use_sim_time': True}],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_joint_state_broadcaster',
        namespace=ns,
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', cm_name,
            '--controller-manager-timeout', '45',
            '--service-call-timeout', '15',
            '--switch-timeout', '30',
        ],
    )

    leg_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_leg_trajectory_controller',
        namespace=ns,
        output='screen',
        arguments=[
            'leg_trajectory_controller',
            '--controller-manager', cm_name,
            '--controller-manager-timeout', '45',
            '--service-call-timeout', '15',
            '--switch-timeout', '30',
        ],
    )

    champ_controller_node = Node(
        package='champ_base',
        executable='quadruped_controller_node',
        name='quadruped_controller_node',
        namespace=ns,
        output='screen',
        parameters=[
            joints_yaml,
            links_yaml,
            gait_yaml,
            {
                'use_sim_time': True,
                'gazebo': True,
                'publish_foot_contacts': False,
                'publish_joint_states': False,
                'publish_joint_control': True,
                'joint_controller_topic': f'{ns_prefix}/leg_trajectory_controller/joint_trajectory',
                'loop_rate': 250.0,
                'joint_command_time': 0.04,
                'urdf': robot_description_str,
            },
        ],
        remappings=[('cmd_vel/smooth', 'cmd_vel')],
    )

    ekf_filter_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        namespace=ns,
        output='screen',
        parameters=[
            ekf_yaml,
            {
                'odom_frame': f'{frame_prefix}odom',
                'base_link_frame': f'{frame_prefix}base_footprint',
                'world_frame': f'{frame_prefix}odom',
                'odom0': f'{ns_prefix}/odom_gz',
                'imu0': f'{ns_prefix}/imu/data',
            },
        ],
        remappings=[('odometry/filtered', 'odom')],
    )

    spawn_jsb_after_legs = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=leg_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    return [
        static_tf_map_odom,
        static_tf_footprint_base,
        leg_controller_spawner,
        champ_controller_node,
        ekf_filter_node,
        spawn_jsb_after_legs,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Optional ROS namespace for multi-robot bringup',
        ),
        OpaqueFunction(function=_setup_control_nodes),
    ])
