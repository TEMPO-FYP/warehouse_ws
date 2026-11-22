import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _resolve_namespace(raw_ns: str) -> str:
    clean = raw_ns.strip().strip('/')
    return clean or 'b2_cargo'


def _setup_control_nodes(context, pkg_desc: str, pkg_ctrl: str):
    ns = _resolve_namespace(LaunchConfiguration('namespace').perform(context))
    frame_prefix = f'{ns}/'
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).strip().lower() == 'true'
    controllers_yaml = LaunchConfiguration('controllers_yaml').perform(context).strip()
    cm_name = f'/{ns}/controller_manager'

    xacro_file = os.path.join(pkg_desc, 'xacro', 'b2_cargo.urdf.xacro')
    joints_yaml = os.path.join(pkg_ctrl, 'config', 'champ', 'joints.yaml')
    links_yaml = os.path.join(pkg_ctrl, 'config', 'champ', 'links.yaml')
    gait_yaml = os.path.join(pkg_ctrl, 'config', 'champ', 'gait.yaml')

    xacro_cmd = [
        'xacro ', xacro_file,
        ' namespace:=', ns,
        ' frame_prefix:=', frame_prefix,
    ]
    if controllers_yaml:
        xacro_cmd.extend([' controllers_file:=', controllers_yaml])
    robot_description_str = ParameterValue(Command(xacro_cmd), value_type=str)

    # Static TFs so REP-105 tree map -> <ns>/odom -> <ns>/base_footprint -> <ns>/base_link links
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
        parameters=[{'use_sim_time': use_sim_time}],
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
        parameters=[{'use_sim_time': use_sim_time}],
    )

    def make_spawner(controller_name: str, extra_name: str) -> Node:
        return Node(
            package='controller_manager',
            executable='spawner',
            name=f'spawner_{extra_name}',
            namespace=ns,
            output='screen',
            arguments=[
                controller_name,
                '--controller-manager', cm_name,
                '--controller-manager-timeout', '45',
                '--service-call-timeout', '15',
                '--switch-timeout', '30',
            ],
        )

    joint_state_broadcaster_spawner = make_spawner(
        'joint_state_broadcaster', 'joint_state_broadcaster'
    )
    leg_controller_spawner = make_spawner(
        'leg_trajectory_controller', 'leg_trajectory_controller'
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
                'use_sim_time': use_sim_time,
                'gazebo': True,
                'publish_foot_contacts': False,
                'publish_joint_states': False,
                'publish_joint_control': True,
                'joint_controller_topic': f'/{ns}/leg_trajectory_controller/joint_trajectory',
                'loop_rate': 250.0,
                'joint_command_time': 0.04,
                'urdf': robot_description_str,
            },
        ],
        remappings=[('cmd_vel/smooth', f'/{ns}/cmd_vel')],
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
        spawn_jsb_after_legs,
    ]


def generate_launch_description():
    """Start static transforms, ros2_control spawners, and CHAMP controller for b2_cargo."""
    pkg_desc = get_package_share_directory('b2_cargo_description')
    pkg_ctrl = get_package_share_directory('b2_cargo_control')
    default_controllers_yaml = os.path.join(pkg_ctrl, 'config', 'ros2_controllers.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='b2_cargo',
            description='ROS namespace for this robot instance',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock',
        ),
        DeclareLaunchArgument(
            'controllers_yaml',
            default_value=default_controllers_yaml,
            description='ros2_controllers YAML passed into GazeboSimROS2ControlPlugin',
        ),
        OpaqueFunction(function=_setup_control_nodes, args=[pkg_desc, pkg_ctrl]),
    ])
