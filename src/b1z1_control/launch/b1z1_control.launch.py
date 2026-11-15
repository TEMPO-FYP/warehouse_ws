import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_b1z1_description = get_package_share_directory('b1z1_description')
    pkg_b1z1_control = get_package_share_directory('b1z1_control')

    xacro_file = os.path.join(pkg_b1z1_description, 'xacro', 'b1z1.urdf.xacro')
    joints_yaml = os.path.join(pkg_b1z1_control, 'config', 'champ', 'joints.yaml')
    links_yaml = os.path.join(pkg_b1z1_control, 'config', 'champ', 'links.yaml')
    gait_yaml = os.path.join(pkg_b1z1_control, 'config', 'champ', 'gait.yaml')
    ekf_yaml = os.path.join(pkg_b1z1_control, 'config', 'ekf', 'odom_ekf.yaml')

    robot_description_str = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    # Static transforms so REP-105 tree map -> odom -> base_footprint -> base_link is continuous
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_to_odom',
        output='screen',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'map', '--child-frame-id', 'odom',
        ],
        parameters=[{'use_sim_time': True}],
    )

    static_tf_footprint_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_footprint_to_base',
        output='screen',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_footprint', '--child-frame-id', 'base_link',
        ],
        parameters=[{'use_sim_time': True}],
    )

    # ros2_control controller spawners (waits for /controller_manager from gz_ros2_control)
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_joint_state_broadcaster',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
            '--service-call-timeout', '60',
            '--switch-timeout', '60',
        ],
    )

    leg_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_leg_trajectory_controller',
        output='screen',
        arguments=[
            'leg_trajectory_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
            '--service-call-timeout', '60',
            '--switch-timeout', '60',
        ],
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_arm_trajectory_controller',
        output='screen',
        arguments=[
            'arm_trajectory_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
            '--service-call-timeout', '60',
            '--switch-timeout', '60',
        ],
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_gripper_controller',
        output='screen',
        arguments=[
            'gripper_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
            '--service-call-timeout', '60',
            '--switch-timeout', '60',
        ],
    )

    # CHAMP Quadruped Controller (publishes leg JointTrajectory from /cmd_vel)
    champ_controller_node = Node(
        package='champ_base',
        executable='quadruped_controller_node',
        name='quadruped_controller_node',
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
                'joint_controller_topic': '/leg_trajectory_controller/joint_trajectory',
                'loop_rate': 250.0,
                'joint_command_time': 0.01667,
                'urdf': robot_description_str,
            },
        ],
        remappings=[('cmd_vel/smooth', '/cmd_vel')],
    )

    # Robot Localization EKF fusing /odom_gz and /imu/data -> /odom + TF odom->base_footprint
    ekf_filter_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_yaml],
        remappings=[('odometry/filtered', '/odom')],
    )

    spawn_secondary_controllers_after_legs = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=leg_controller_spawner,
            on_exit=[
                joint_state_broadcaster_spawner,
                arm_controller_spawner,
                gripper_controller_spawner,
            ],
        )
    )

    return LaunchDescription([
        static_tf_map_odom,
        static_tf_footprint_base,
        leg_controller_spawner,
        champ_controller_node,
        ekf_filter_node,
        spawn_secondary_controllers_after_legs,
    ])
