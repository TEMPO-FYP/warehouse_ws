import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def _resolve_namespace(raw_ns: str) -> str:
    clean = raw_ns.strip().strip('/')
    return clean or 'b1z1'


def _load_prefixed_links(links_yaml_path: str, prefix: str) -> dict:
    """Load CHAMP links.yaml and prepend `prefix` to every link name."""
    with open(links_yaml_path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}

    root_params = data.get('/**', {}).get('ros__parameters', {})
    raw_map = root_params.get('links_map', {})
    prefixed = {}
    for key, val in raw_map.items():
        if isinstance(val, str):
            prefixed[key] = f'{prefix}{val}'
        elif isinstance(val, list):
            prefixed[key] = [f'{prefix}{item}' for item in val]
        else:
            prefixed[key] = val
    return {'links_map': prefixed}


def _launch_setup(context, pkg_b1z1_description: str, pkg_b1z1_control: str):
    ns = _resolve_namespace(LaunchConfiguration('namespace').perform(context))
    raw_prefix = LaunchConfiguration('prefix').perform(context).strip()
    prefix = raw_prefix if raw_prefix else f'{ns}_'
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).strip().lower() == 'true'
    enable_video_cam = LaunchConfiguration('enable_video_cam').perform(context).strip()
    controllers_yaml = LaunchConfiguration('controllers_yaml').perform(context).strip()

    xacro_file = os.path.join(pkg_b1z1_description, 'xacro', 'b1z1.urdf.xacro')
    joints_yaml = os.path.join(pkg_b1z1_control, 'config', 'champ', 'joints.yaml')
    links_yaml = os.path.join(pkg_b1z1_control, 'config', 'champ', 'links.yaml')
    gait_yaml = os.path.join(pkg_b1z1_control, 'config', 'champ', 'gait.yaml')

    xacro_cmd = [
        'xacro ', xacro_file,
        ' namespace:=', ns,
        ' prefix:=', prefix,
        ' enable_video_cam:=', enable_video_cam,
    ]
    if controllers_yaml:
        xacro_cmd.extend([' controllers_yaml:=', controllers_yaml])
    robot_description_str = ParameterValue(Command(xacro_cmd), value_type=str)

    # Static transforms so REP-105 tree map -> <prefix>odom -> <prefix>base_footprint -> <prefix>base_link is continuous
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        namespace=ns,
        name='static_tf_map_to_odom',
        output='screen',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'map', '--child-frame-id', f'{prefix}odom',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    static_tf_footprint_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        namespace=ns,
        name='static_tf_footprint_to_base',
        output='screen',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', f'{prefix}base_footprint', '--child-frame-id', f'{prefix}base_link',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    cm_path = f'/{ns}/controller_manager'

    def make_spawner(controller_name: str) -> Node:
        return Node(
            package='controller_manager',
            executable='spawner',
            namespace=ns,
            name=f'spawner_{controller_name}',
            output='screen',
            arguments=[
                controller_name,
                '--controller-manager', cm_path,
                '--controller-manager-timeout', '30',
                '--service-call-timeout', '60',
                '--switch-timeout', '60',
            ],
        )

    joint_state_broadcaster_spawner = make_spawner('joint_state_broadcaster')
    leg_controller_spawner = make_spawner('leg_trajectory_controller')
    arm_controller_spawner = make_spawner('arm_trajectory_controller')
    gripper_controller_spawner = make_spawner('gripper_controller')

    # CHAMP Quadruped Controller (publishes leg JointTrajectory from /<ns>/cmd_vel)
    champ_controller_node = Node(
        package='champ_base',
        executable='quadruped_controller_node',
        namespace=ns,
        name='quadruped_controller_node',
        output='screen',
        parameters=[
            joints_yaml,
            _load_prefixed_links(links_yaml, prefix),
            gait_yaml,
            {
                'use_sim_time': use_sim_time,
                'gazebo': True,
                'publish_foot_contacts': False,
                'publish_joint_states': False,
                'publish_joint_control': True,
                'joint_controller_topic': f'/{ns}/leg_trajectory_controller/joint_trajectory',
                'loop_rate': 250.0,
                'joint_command_time': 0.01667,
                'urdf': robot_description_str,
            },
        ],
        remappings=[('cmd_vel/smooth', f'/{ns}/cmd_vel')],
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

    return [
        static_tf_map_odom,
        static_tf_footprint_base,
        leg_controller_spawner,
        champ_controller_node,
        spawn_secondary_controllers_after_legs,
    ]


def generate_launch_description():
    pkg_b1z1_description = get_package_share_directory('b1z1_description')
    pkg_b1z1_control = get_package_share_directory('b1z1_control')
    default_controllers_yaml = os.path.join(pkg_b1z1_control, 'config', 'ros2_controllers.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='b1z1',
            description='ROS namespace for this robot instance',
        ),
        DeclareLaunchArgument(
            'prefix',
            default_value='',
            description='TF/URDF link prefix (defaults to <namespace>_ when empty)',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock',
        ),
        DeclareLaunchArgument(
            'enable_video_cam',
            default_value='false',
            description='Attach third-person follow camera sensor in URDF for CHAMP lookup alignment',
        ),
        DeclareLaunchArgument(
            'controllers_yaml',
            default_value=default_controllers_yaml,
            description='ros2_controllers configuration YAML passed into GazeboSimROS2ControlPlugin',
        ),
        OpaqueFunction(
            function=_launch_setup,
            args=[pkg_b1z1_description, pkg_b1z1_control],
        ),
    ])
