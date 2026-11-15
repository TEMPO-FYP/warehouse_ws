import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_warehouse_sim = get_package_share_directory("warehouse_sim")
    pkg_bcr_arm_desc = get_package_share_directory("bcr_arm_description")
    pkg_bcr_arm_gazebo = get_package_share_directory("bcr_arm_gazebo")
    pkg_robotiq_desc = get_package_share_directory("robotiq_description")
    pkg_bcr_arm_moveit = get_package_share_directory("bcr_arm_moveit_config")

    # Paths for Gazebo resource discovery
    warehouse_models = os.path.join(pkg_warehouse_sim, "models")
    warehouse_worlds = os.path.join(pkg_warehouse_sim, "worlds")
    bcr_arm_models = os.path.join(pkg_bcr_arm_gazebo, "models")
    bcr_arm_parent = os.path.dirname(pkg_bcr_arm_desc)
    robotiq_parent = os.path.dirname(pkg_robotiq_desc)

    resource_paths = [
        warehouse_models,
        warehouse_worlds,
        bcr_arm_models,
        bcr_arm_parent,
        robotiq_parent,
    ]

    existing_gz_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    all_gz_paths = [p for p in resource_paths if p]
    if existing_gz_resource_path:
        all_gz_paths.append(existing_gz_resource_path)
    gz_resource_path_str = os.pathsep.join(all_gz_paths)

    set_gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=gz_resource_path_str,
    )

    # Launch arguments
    default_world = os.path.join(pkg_warehouse_sim, "worlds", "test_shelf_world.sdf")
    default_controllers = os.path.join(pkg_bcr_arm_moveit, "config", "ros2_controllers.yaml")

    world_arg = DeclareLaunchArgument(
        "world",
        default_value=default_world,
        description="Path to the SDF world file to load",
    )

    controllers_arg = DeclareLaunchArgument(
        "ros2_controllers_path",
        default_value=default_controllers,
        description="Path to the ros2_controllers.yaml file",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    world_lc = LaunchConfiguration("world")
    controllers_lc = LaunchConfiguration("ros2_controllers_path")
    use_sim_time_lc = LaunchConfiguration("use_sim_time")

    # Robot description via Xacro
    robot_description_command = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("bcr_arm_description"),
                    "urdf",
                    "robots",
                    "bcr_arm.urdf.xacro",
                ]
            ),
            " ",
            "use_gazebo:=true",
            " ",
            "add_world:=true",
            " ",
            "ros2_controllers_path:=",
            controllers_lc,
        ]
    )

    robot_description_param = {
        "robot_description": ParameterValue(robot_description_command, value_type=str)
    }

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description_param, {"use_sim_time": use_sim_time_lc}],
    )

    # Gazebo Sim Launch
    gazebo_sim_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={"gz_args": ["-r -v 4 ", world_lc]}.items(),
    )

    # Clock Bridge Node
    clock_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        parameters=[{"use_sim_time": use_sim_time_lc}],
    )

    # Spawn Robot Entity in Gazebo Sim
    spawn_entity_node = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            robot_description_command,
            "-name",
            "bcr_arm",
            "-allow_renaming",
            "true",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.0",
        ],
    )

    # Controller Spawners (sequenced via OnProcessExit to prevent controller_manager lock timeout)
    spawn_joint_state_broadcaster_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            ["/controller_manager"],
        ],
        output="screen",
    )

    spawn_joint_trajectory_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager",
            ["/controller_manager"],
        ],
        output="screen",
    )

    spawn_gripper_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
            "--controller-manager",
            ["/controller_manager"],
        ],
        output="screen",
    )

    delay_jsb_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity_node,
            on_exit=[spawn_joint_state_broadcaster_node],
        )
    )

    delay_jtc_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster_node,
            on_exit=[spawn_joint_trajectory_controller_node],
        )
    )

    delay_gripper_after_jtc = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_trajectory_controller_node,
            on_exit=[spawn_gripper_controller_node],
        )
    )

    return LaunchDescription(
        [
            set_gz_resource_path,
            world_arg,
            controllers_arg,
            use_sim_time_arg,
            robot_state_publisher_node,
            gazebo_sim_node,
            clock_bridge_node,
            spawn_entity_node,
            delay_jsb_after_spawn,
            delay_jtc_after_jsb,
            delay_gripper_after_jtc,
        ]
    )
