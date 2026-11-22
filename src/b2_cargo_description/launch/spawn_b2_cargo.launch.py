import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _resolve_namespace(raw_ns: str) -> str:
    clean = raw_ns.strip().strip('/')
    return clean or 'b2_cargo'


def _spawn_test_cargo(context):
    """Optionally spawn a sample warehouse box inside b2_cargo's crate after standing settle."""
    if LaunchConfiguration('spawn_test_cargo').perform(context).strip().lower() != 'true':
        return []

    pkg_warehouse_sim = get_package_share_directory('warehouse_sim')
    sdf_path = os.path.join(pkg_warehouse_sim, 'models', 'small_box', 'model.sdf')
    x = float(LaunchConfiguration('x').perform(context))
    y = float(LaunchConfiguration('y').perform(context))
    z = float(LaunchConfiguration('z').perform(context))
    yaw = float(LaunchConfiguration('yaw').perform(context))

    # Wait ~3.5s so leg_trajectory_controller settles trunk near z~0.56-0.59 m before dropping box
    cx = x - 0.02 * math.cos(yaw)
    cy = y - 0.02 * math.sin(yaw)
    cz = min(z, 0.58) + 0.220

    return [
        TimerAction(
            period=3.5,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    name='b2_test_cargo_spawner',
                    output='screen',
                    arguments=[
                        '-file', sdf_path,
                        '-name', f"{LaunchConfiguration('name').perform(context)}_test_cargo",
                        '-x', f'{cx:.4f}',
                        '-y', f'{cy:.4f}',
                        '-z', f'{cz:.4f}',
                        '-Y', f'{yaw:.4f}',
                    ],
                )
            ],
        )
    ]


def _setup_nodes(context, xacro_file: str):
    ns = _resolve_namespace(LaunchConfiguration('namespace').perform(context))
    frame_prefix = f'{ns}/'
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).strip().lower() == 'true'
    controllers_yaml = LaunchConfiguration('controllers_yaml').perform(context).strip()
    if not controllers_yaml:
        controllers_yaml = LaunchConfiguration('controllers_file').perform(context).strip()

    xacro_cmd = [
        'xacro ', xacro_file,
        ' namespace:=', ns,
        ' frame_prefix:=', frame_prefix,
    ]
    if controllers_yaml:
        xacro_cmd.extend([' controllers_file:=', controllers_yaml])

    robot_description = ParameterValue(Command(xacro_cmd), value_type=str)

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=ns,
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'frame_prefix': frame_prefix,
            'use_sim_time': use_sim_time,
        }],
    )

    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='b2_cargo_spawner',
        namespace=ns,
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
        name='b2_cargo_bridge',
        namespace=ns,
        output='screen',
        arguments=[
            f'/{ns}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            f'/{ns}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            f'/{ns}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        remappings=[(f'/{ns}/tf', '/tf')],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return [rsp_node, spawn_node, bridge_node]


def generate_launch_description():
    """Spawn b2_cargo into Gazebo and start its state publisher and Gazebo bridge."""
    pkg_desc = get_package_share_directory('b2_cargo_description')
    xacro_file = os.path.join(pkg_desc, 'xacro', 'b2_cargo.urdf.xacro')

    pkg_share_parent = os.path.dirname(pkg_desc)
    existing_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join(filter(None, [pkg_share_parent, pkg_desc, existing_resource_path])),
    )

    return LaunchDescription([
        gz_resource_path,
        DeclareLaunchArgument(
            'name',
            default_value='b2_cargo',
            description='Model name in Gazebo',
        ),
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
            default_value='',
            description='Optional ros2_controllers YAML for GazeboSimROS2ControlPlugin',
        ),
        DeclareLaunchArgument(
            'controllers_file',
            default_value='',
            description='Legacy alias for controllers_yaml',
        ),
        DeclareLaunchArgument('x', default_value='2.0', description='Spawn X position'),
        DeclareLaunchArgument('y', default_value='16.0', description='Spawn Y position'),
        DeclareLaunchArgument('z', default_value='0.64', description='Spawn Z position'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn Yaw rotation'),
        DeclareLaunchArgument(
            'spawn_test_cargo',
            default_value='false',
            description='Spawn a small_box payload centered inside cargo_rack',
        ),
        OpaqueFunction(function=_setup_nodes, args=[xacro_file]),
        OpaqueFunction(function=_spawn_test_cargo),
    ])
