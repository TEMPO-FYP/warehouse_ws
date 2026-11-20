import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _resolve_namespace(raw_ns: str) -> str:
    clean = raw_ns.strip().strip('/')
    return clean or 'b1z1'


def _launch_setup(context, xacro_file: str):
    ns = _resolve_namespace(LaunchConfiguration('namespace').perform(context))
    raw_prefix = LaunchConfiguration('prefix').perform(context).strip()
    prefix = raw_prefix if raw_prefix else f'{ns}_'
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).strip().lower() == 'true'
    controllers_yaml = LaunchConfiguration('controllers_yaml').perform(context).strip()
    enable_video_cam = LaunchConfiguration('enable_video_cam').perform(context).strip()

    xacro_cmd = [
        'xacro ', xacro_file,
        ' namespace:=', ns,
        ' prefix:=', prefix,
        ' enable_video_cam:=', enable_video_cam,
    ]
    if controllers_yaml:
        xacro_cmd.extend([' controllers_yaml:=', controllers_yaml])

    robot_description = ParameterValue(Command(xacro_cmd), value_type=str)

    # Do not pass frame_prefix: all URDF/SRDF links already carry `prefix` for MoveIt 2.
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=ns,
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
    )

    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        namespace=ns,
        name='b1z1_spawner',
        output='screen',
        arguments=[
            '-name', LaunchConfiguration('name'),
            '-topic', f'/{ns}/robot_description',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw'),
        ],
    )

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        namespace=ns,
        name='b1z1_bridge',
        output='screen',
        arguments=[
            f'/{ns}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            f'/{ns}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            f'/{ns}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        remappings=[(f'/{ns}/tf', '/tf')],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return [
        robot_state_publisher_node,
        spawn_entity_node,
        bridge_node,
    ]


def generate_launch_description():
    pkg_b1z1 = get_package_share_directory('b1z1_description')
    xacro_file = os.path.join(pkg_b1z1, 'xacro', 'b1z1.urdf.xacro')

    # Ensure Gazebo Sim finds meshes in install share directory
    pkg_share_parent = os.path.dirname(pkg_b1z1)
    existing_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join(filter(None, [pkg_share_parent, pkg_b1z1, existing_resource_path]))
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='b1z1',
        description='ROS namespace for this robot instance',
    )
    prefix_arg = DeclareLaunchArgument(
        'prefix',
        default_value='',
        description='TF/URDF link prefix (defaults to <namespace>_ when empty)',
    )
    name_arg = DeclareLaunchArgument('name', default_value='b1z1', description='Model name in Gazebo')
    x_arg = DeclareLaunchArgument('x', default_value='0.0', description='Spawn X position')
    y_arg = DeclareLaunchArgument('y', default_value='16.0', description='Spawn Y position (staging area)')
    z_arg = DeclareLaunchArgument('z', default_value='0.65', description='Spawn Z position (clears bent crouch legs before settling)')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn Yaw rotation')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock',
    )
    controllers_yaml_arg = DeclareLaunchArgument(
        'controllers_yaml',
        default_value='',
        description='Optional ros2_controllers YAML path passed into GazeboSimROS2ControlPlugin',
    )
    enable_video_cam_arg = DeclareLaunchArgument(
        'enable_video_cam',
        default_value='false',
        description='Attach third-person follow camera sensor to b1z1 for MP4 recording',
    )

    return LaunchDescription([
        gz_resource_path,
        namespace_arg,
        prefix_arg,
        name_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        use_sim_time_arg,
        controllers_yaml_arg,
        enable_video_cam_arg,
        OpaqueFunction(function=_launch_setup, args=[xacro_file]),
    ])
