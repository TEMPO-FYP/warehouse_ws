#!/usr/bin/env python3
"""
mobile_robot_controller.py

ROS 2 Controller Node for the TEMPO mobile manipulator (Husky A200 + UR5e + Robotiq 2F-85).

Subscribes to /mission/subtask (std_msgs/String, JSON), checks whether the
assigned robot matches this controller's instance, and executes the skill by
calling the appropriate ROS 2 interfaces:

  navigate_to ->  Nav2 NavigateToPose action
  pick ->  MoveIt2 MoveGroup action + gripper close
  place ->  MoveIt2 MoveGroup action + gripper open

Real-time object and location poses are looked up from the Gazebo world pose
topic /world/default/pose/info

Topics / Actions
----------------
Subscribed:
  /mission/subtask                              (std_msgs/String)
  /world/<world>/pose/info                      (gz.msgs.Pose_V via ros_gz_bridge)

Published:
  /mission/feedback                             (std_msgs/String)

Action Clients:
  /<namespace>/navigate_to_pose                 (nav2_msgs/NavigateToPose)
  /<namespace>/move_group                       (moveit_msgs/MoveGroup)
  /<namespace>/robotiq_gripper_controller/gripper_cmd  (control_msgs/GripperCommand)
"""

import json
import math
import os
from typing import Any, Dict, Optional, Tuple

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import GripperCommand
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    MotionPlanRequest,
    MoveItErrorCodes,
    PositionConstraint,
)
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, Quaternion


# Constants

ROBOT_TYPE = "mobile_robot"


# Config loader

def _load_config() -> Dict[str, Any]:
    """
    Loads controller_config.yaml from the package share directory.
    Returns the parsed config dictionary.
    """
    share_dir = get_package_share_directory("robot_controllers")
    config_path = os.path.join(share_dir, "config", "controller_config.yaml")
    with open(config_path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


# Pose helpers

def _apply_standoff(
    x: float,
    y: float,
    z: float,
    yaw: float,
    standoff: float,
) -> Tuple[float, float, float]:
    """
    Applies a standoff offset in front of the target pose.

    The offset is applied along the direction the target is facing (yaw),
    so the robot stops `standoff` metres in front of the target rather
    than driving into it.
    """
    offset_x = x - standoff * math.cos(yaw)
    offset_y = y - standoff * math.sin(yaw)
    return offset_x, offset_y, z


def _yaw_to_quaternion(yaw: float) -> Quaternion:
    """Converts a yaw angle (radians) to a geometry_msgs Quaternion."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


# Mobile robot controller node

class MobileRobotController(Node):
    """
    Executes navigate_to, pick, and place skills for a Husky A200 mobile
    manipulator by calling Nav2, MoveIt2, and gripper action servers.

    Lifecycle
    ---------
    1. /mission/subtask arrives -> _subtask_callback
    2. Robot type and instance checked -> _handle_subtask
    3. Skill dispatched -> _execute_navigate_to /
                           _execute_pick /
                           _execute_place
    4. Feedback published -> _publish_feedback
    """

    def __init__(self) -> None:
        super().__init__("mobile_robot_controller")

        # Load config
        config = _load_config()

        self._robot_instances: Dict[str, str] = config["robot_instances"]
        self._standoff: float = config["navigation"]["standoff_distance"]
        self._nav_timeout: float = config["navigation"]["nav_timeout_sec"]
        self._move_group_name: str = config["manipulation"]["move_group_name"]
        self._gripper_open: float = config["manipulation"]["gripper_open_position"]
        self._gripper_closed: float = config["manipulation"]["gripper_closed_position"]
        self._moveit_timeout: float = config["manipulation"]["moveit_timeout_sec"]
        self._world_name: str = config["world"]["name"]
        self._cb_group = ReentrantCallbackGroup()

        # World pose registry
        # Populated by _pose_callback from /world/default/pose/info
        # Keys are Gazebo entity names, values are (x, y, z, yaw) tuples
        self._pose_registry: Dict[str, Tuple[float, float, float, float]] = {}

        # Subscriptions
        self.create_subscription(
            String,
            "/mission/subtask",
            self._subtask_callback,
            10,
            callback_group=self._cb_group,
        )

        # Gazebo world pose topic (bridged via ros_gz_bridge as tf2_msgs/msg/TFMessage
        # /world/default/pose/info publishes all entity poses in the world
        self.create_subscription(
            TFMessage,
            f"/world/{self._world_name}/pose/info",
            self._pose_callback,
            10,
            callback_group=self._cb_group,
        )

        # Publisher
        self._feedback_pub = self.create_publisher(
            String,
            "/mission/feedback",
            10,
        )

        # Action clients
        # Keyed by robot instance id e.g. "mobile_robot_1"
        self._nav_clients: Dict[str, ActionClient] = {}
        self._moveit_clients: Dict[str, ActionClient] = {}
        self._gripper_clients: Dict[str, ActionClient] = {}

        self._init_action_clients()

        self.get_logger().info(
            "MobileRobotController ready.\n"
            "  -> Subscribing to: /mission/subtask\n"
            f"  -> Subscribing to: /world/{self._world_name}/pose/info\n"
            "  -> Publishing to:  /mission/feedback\n"
            f"  -> Managing instances: {list(self._robot_instances.keys())}"
        )

    def _init_action_clients(self) -> None:
        """
        Initialises Nav2, MoveIt2, and gripper action clients for every
        robot instance declared in controller_config.yaml.
        """
        for instance_id, namespace in self._robot_instances.items():
            self._nav_clients[instance_id] = ActionClient(
                self,
                NavigateToPose,
                f"{namespace}/navigate_to_pose",
                callback_group=self._cb_group,
            )
            self._moveit_clients[instance_id] = ActionClient(
                self,
                MoveGroup,
                f"{namespace}/move_group",
                callback_group=self._cb_group,
            )
            self._gripper_clients[instance_id] = ActionClient(
                self,
                GripperCommand,
                f"{namespace}/robotiq_gripper_controller/gripper_cmd",
                callback_group=self._cb_group,
            )
            self.get_logger().info(
                f"[init] Action clients ready for {instance_id} "
                f"(namespace={namespace})"
            )

    # World pose subscription

    def _pose_callback(self, msg: TFMessage) -> None:
        """
        Updates the pose registry from the Gazebo world pose topic.

        /world/default/pose/info is bridged via ros_gz_bridge as
        tf2_msgs/TFMessage. Each transform's child_frame_id is the
        Gazebo entity name and the transform gives its pose.
        """
        for transform in msg.transforms:
            name = transform.child_frame_id
            if not name:
                continue

            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z

            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w
            yaw = 2.0 * math.atan2(qz, qw)

            self._pose_registry[name] = (x, y, z, yaw)

    def _get_pose(self, entity_name: str) -> Optional[Tuple[float, float, float, float]]:
        """
        Looks up an entity pose from the registry.
        Returns (x, y, z, yaw) or None if not found.
        """
        pose = self._pose_registry.get(entity_name)
        if pose is None:
            self.get_logger().error(
                f"[get_pose] Entity '{entity_name}' not found in pose registry. "
                f"Known entities: {sorted(self._pose_registry.keys())[:10]}..."
            )
        return pose

    # Subtask subscription

    def _subtask_callback(self, msg: String) -> None:
        """
        Receives a subtask from /mission/subtask and delegates to
        _handle_subtask. Exceptions are caught so a bad message never
        crashes the node.
        """
        try:
            subtask = json.loads(msg.data.strip())
            self._handle_subtask(subtask)
        except Exception as exc:
            self.get_logger().error(
                f"[subtask_callback] Unhandled exception: {exc}"
            )

    def _handle_subtask(self, subtask: Dict[str, Any]) -> None:
        """
        Validates the subtask and dispatches to the correct skill handler.

        Checks:
        1. Robot type matches ROBOT_TYPE (mobile_robot)
        2. Robot instance is known in config
        3. Skill is supported
        """
        plan_id = subtask.get("plan_id", "")
        task_id = subtask.get("task_id", "")
        robot = subtask.get("robot", "")
        skills = subtask.get("required_skills", [])
        args = subtask.get("args", {})

        self.get_logger().info(
            f"[handle_subtask] Received subtask {task_id} "
            f"(robot={robot}, skills={skills})"
        )

        # Check robot type
        if not robot.startswith(ROBOT_TYPE):
            self.get_logger().info(
                f"[handle_subtask] Subtask {task_id} is assigned to '{robot}' "
                f"which is not a '{ROBOT_TYPE}', ignoring."
            )
            return

        # Check robot instance is known
        if robot not in self._robot_instances:
            self.get_logger().error(
                f"[handle_subtask] Robot instance '{robot}' not in config. "
                f"Known instances: {list(self._robot_instances.keys())}"
            )
            self._publish_feedback(
                plan_id, task_id, robot,
                status="failed",
                stage="config_error",
                detail=f"Unknown robot instance: {robot}",
            )
            return

        # Check skill
        skill = skills[0] if skills else None
        if skill is None:
            self.get_logger().error(
                f"[handle_subtask] Subtask {task_id} has no required_skills."
            )
            self._publish_feedback(
                plan_id, task_id, robot,
                status="failed",
                stage="config_error",
                detail="No required_skills specified.",
            )
            return

        # Dispatch to skill handler
        if skill == "navigate_to":
            self._execute_navigate_to(plan_id, task_id, robot, args)
        elif skill == "pick":
            self._execute_pick(plan_id, task_id, robot, args)
        elif skill == "place":
            self._execute_place(plan_id, task_id, robot, args)
        else:
            self.get_logger().error(
                f"[handle_subtask] Unknown skill '{skill}' for subtask {task_id}."
            )
            self._publish_feedback(
                plan_id, task_id, robot,
                status="failed",
                stage="config_error",
                detail=f"Unknown skill: {skill}",
            )

    # Skill: navigate_to

    def _execute_navigate_to(
        self,
        plan_id: str,
        task_id: str,
        robot: str,
        args: Dict[str, Any],
    ) -> None:
        """
        Executes navigate_to skill by sending a NavigateToPose goal to Nav2.

        Args expected:
          location (str): Gazebo entity name to navigate to
                          e.g. "Picking_Shelves.001"

        A standoff offset is applied so the robot stops in front of
        the target rather than driving into it.
        """
        location = args.get("location")
        if not location:
            self._publish_feedback(
                plan_id, task_id, robot,
                status="failed",
                stage="navigate_to",
                detail="Missing 'location' in args.",
            )
            return

        pose = self._get_pose(location)
        if pose is None:
            self._publish_feedback(
                plan_id, task_id, robot,
                status="failed",
                stage="navigate_to",
                detail=f"Entity '{location}' not in pose registry.",
            )
            return

        x, y, z, yaw = pose
        nav_x, nav_y, _ = _apply_standoff(x, y, z, yaw, self._standoff)

        self.get_logger().info(
            f"[navigate_to] Task {task_id}: navigating to '{location}' "
            f"with standoff {self._standoff}m -> ({nav_x:.2f}, {nav_y:.2f})"
        )

        # Build NavigateToPose goal
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = nav_x
        goal.pose.pose.position.y = nav_y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation = _yaw_to_quaternion(yaw)

        nav_client = self._nav_clients[robot]

        # Publish intermediate feedback
        self._publish_feedback(
            plan_id, task_id, robot,
            status="success",
            stage="navigating",
            detail=f"Navigating to '{location}'.",
        )

        if not nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                f"[navigate_to] Nav2 action server not available for {robot}."
            )
            self._publish_feedback(
                plan_id, task_id, robot,
                status="failed",
                stage="navigate_to",
                detail="Nav2 action server not available.",
            )
            return

        send_goal_future = nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(
            self, send_goal_future, timeout_sec=self._nav_timeout
        )

        goal_handle = send_goal_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error(
                f"[navigate_to] Nav2 rejected goal for task {task_id}."
            )
            self._publish_feedback(
                plan_id, task_id, robot,
                status="failed",
                stage="navigate_to",
                detail="Nav2 goal rejected.",
            )
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=self._nav_timeout
        )

        result = result_future.result()
        if result is None:
            goal_handle.cancel_goal_async()
            self._publish_feedback(
                plan_id, task_id, robot,
                status="failed",
                stage="navigate_to",
                detail="Nav2 result timed out.",
            )
            return

        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                f"[navigate_to] Task {task_id}: navigation failed "
                f"with status {result.status}."
            )
            self._publish_feedback(
                plan_id, task_id, robot,
                status="failed",
                stage="navigate_to",
                detail=f"Navigation failed with status {result.status}.",
            )
            return

        self.get_logger().info(
            f"[navigate_to] Task {task_id}: navigation complete."
        )
        self._publish_feedback(
            plan_id, task_id, robot,
            status="success",
            stage="complete",
            detail=f"Reached '{location}'.",
        )

    # Skill: pick

    def _execute_pick(
        self,
        plan_id: str,
        task_id: str,
        robot: str,
        args: Dict[str, Any],
    ) -> None:
        """
        Executes pick skill:
        1. Opens gripper
        2. Moves arm to object pose via MoveIt2
        3. Closes gripper

        Args expected:
          object (str): Gazebo entity name of object to pick
                        e.g. "small_box.019_5"
        """
        object_name = args.get("object")
        if not object_name:
            self._publish_feedback(
                plan_id, task_id, robot,
                status="failed",
                stage="pick",
                detail="Missing 'object' in args.",
            )
            return

        pose = self._get_pose(object_name)
        if pose is None:
            self._publish_feedback(
                plan_id, task_id, robot,
                status="failed",
                stage="pick",
                detail=f"Object '{object_name}' not in pose registry.",
            )
            return

        x, y, z, _ = pose

        self.get_logger().info(
            f"[pick] Task {task_id}: picking '{object_name}' "
            f"at ({x:.2f}, {y:.2f}, {z:.2f})"
        )

        self._publish_feedback(
            plan_id, task_id, robot,
            status="success",
            stage="pick_in_progress",
            detail=f"Picking '{object_name}'.",
        )

        # Step 1 : Open gripper
        if not self._send_gripper_goal(robot, self._gripper_open, task_id):
            self._publish_feedback(
                plan_id, task_id, robot,
                status="failed",
                stage="pick",
                detail="Failed to open gripper.",
            )
            return

        # Step 2 : Move arm to object
        if not self._send_moveit_goal(robot, x, y, z, task_id):
            self._publish_feedback(
                plan_id, task_id, robot,
                status="failed",
                stage="pick",
                detail=f"MoveIt2 failed to reach '{object_name}'.",
            )
            return

        # Step 3 : Close gripper
        if not self._send_gripper_goal(robot, self._gripper_closed, task_id):
            self._publish_feedback(
                plan_id, task_id, robot,
                status="failed",
                stage="pick",
                detail="Failed to close gripper.",
            )
            return

        self.get_logger().info(
            f"[pick] Task {task_id}: pick complete."
        )
        self._publish_feedback(
            plan_id, task_id, robot,
            status="success",
            stage="complete",
            detail=f"Picked '{object_name}'.",
        )

    # Skill: place

    def _execute_place(
        self,
        plan_id: str,
        task_id: str,
        robot: str,
        args: Dict[str, Any],
    ) -> None:
        """
        Executes place skill:
        1. Moves arm to target location via MoveIt2
        2. Opens gripper to release object

        Args expected:
          location (str): Gazebo entity name of target location
                          e.g. "Picking_Shelves.001"
          object   (str): name of object being placed (for logging)
        """
        location = args.get("location")
        object_name = args.get("object", "unknown")

        if not location:
            self._publish_feedback(
                plan_id, task_id, robot,
                status="failed",
                stage="place",
                detail="Missing 'location' in args.",
            )
            return

        pose = self._get_pose(location)
        if pose is None:
            self._publish_feedback(
                plan_id, task_id, robot,
                status="failed",
                stage="place",
                detail=f"Location '{location}' not in pose registry.",
            )
            return

        x, y, z, _ = pose

        self.get_logger().info(
            f"[place] Task {task_id}: placing '{object_name}' "
            f"at '{location}' ({x:.2f}, {y:.2f}, {z:.2f})"
        )

        self._publish_feedback(
            plan_id, task_id, robot,
            status="success",
            stage="place_in_progress",
            detail=f"Placing '{object_name}' at '{location}'.",
        )

        # Step 1 : Move arm to target location
        if not self._send_moveit_goal(robot, x, y, z, task_id):
            self._publish_feedback(
                plan_id, task_id, robot,
                status="failed",
                stage="place",
                detail=f"MoveIt2 failed to reach '{location}'.",
            )
            return

        # Step 2 : Open gripper to release
        if not self._send_gripper_goal(robot, self._gripper_open, task_id):
            self._publish_feedback(
                plan_id, task_id, robot,
                status="failed",
                stage="place",
                detail="Failed to open gripper.",
            )
            return

        self.get_logger().info(
            f"[place] Task {task_id}: place complete."
        )
        self._publish_feedback(
            plan_id, task_id, robot,
            status="success",
            stage="complete",
            detail=f"Placed '{object_name}' at '{location}'.",
        )

    # MoveIt2 helper

    def _send_moveit_goal(
        self,
        robot: str,
        x: float,
        y: float,
        z: float,
        task_id: str,
    ) -> bool:
        """
        Sends a MoveGroup goal to MoveIt2 to move the arm to (x, y, z).
        Returns True on success, False on failure.

        Uses a position constraint on the end effector link with a small
        tolerance sphere around the target point.
        """
        moveit_client = self._moveit_clients[robot]
        namespace = self._robot_instances[robot]

        if not moveit_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                f"[moveit] MoveIt2 action server not available for {robot}."
            )
            return False

        # Build position constraint
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = "world"
        position_constraint.link_name = f"{namespace.strip('/')}/arm_0_tool0"
        position_constraint.target_point_offset = Point(x=0.0, y=0.0, z=0.0)

        # Tolerance sphere around target
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01]  # 1cm tolerance

        bounding_volume = BoundingVolume()
        bounding_volume.primitives = [sphere]
        bounding_volume.primitive_poses = [
            self._make_pose(x, y, z)
        ]

        position_constraint.constraint_region = bounding_volume
        position_constraint.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints = [position_constraint]

        # Build MoveGroup goal
        motion_request = MotionPlanRequest()
        motion_request.group_name = self._move_group_name
        motion_request.goal_constraints = [constraints]
        motion_request.num_planning_attempts = 5
        motion_request.allowed_planning_time = 10.0
        motion_request.max_velocity_scaling_factor = 0.5
        motion_request.max_acceleration_scaling_factor = 0.5

        goal = MoveGroup.Goal()
        goal.request = motion_request
        goal.planning_options.plan_only = False

        send_goal_future = moveit_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(
            self, send_goal_future, timeout_sec=self._moveit_timeout
        )

        goal_handle = send_goal_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error(
                f"[moveit] MoveGroup goal rejected for task {task_id}."
            )
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=self._moveit_timeout
        )

        result = result_future.result()
        if result is None:
            goal_handle.cancel_goal_async()
            self.get_logger().error(
                f"[moveit] MoveGroup result timed out for task {task_id}."
            )
            return False

        error_code = result.result.error_code.val
        if error_code != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                f"[moveit] MoveGroup failed for task {task_id} "
                f"with error code {error_code}."
            )
            return False

        return True

    # Gripper helper

    def _send_gripper_goal(
        self,
        robot: str,
        position: float,
        task_id: str,
    ) -> bool:
        """
        Sends a GripperCommand goal to open or close the gripper.
        position: 0.0 = fully open, 0.8 = fully closed
        Returns True on success, False on failure.
        """
        gripper_client = self._gripper_clients[robot]

        if not gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                f"[gripper] Gripper action server not available for {robot}."
            )
            return False

        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = 50.0

        send_goal_future = gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(
            self, send_goal_future, timeout_sec=10.0
        )

        goal_handle = send_goal_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error(
                f"[gripper] Gripper goal rejected for task {task_id}."
            )
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=10.0
        )

        result = result_future.result()
        if result is None:
            goal_handle.cancel_goal_async()
            self.get_logger().error(
                f"[gripper] Gripper result timed out for task {task_id}."
            )
            return False

        return result.result.reached_goal or result.result.stalled

    # Feedback publisher

    def _publish_feedback(
        self,
        plan_id: str,
        task_id: str,
        robot: str,
        status: str,
        stage: str,
        detail: str = "",
    ) -> None:
        """Publishes a feedback message to /mission/feedback."""
        payload = json.dumps({
            "plan_id": plan_id,
            "task_id": task_id,
            "robot": robot,
            "status": status,
            "stage": stage,
            "detail": detail,
        })
        msg = String()
        msg.data = payload
        self._feedback_pub.publish(msg)
        self.get_logger().info(
            f"[feedback] task={task_id} status={status} "
            f"stage={stage} detail={detail}"
        )

    # Pose builder helper

    def _make_pose(self, x: float, y: float, z: float):
        """Builds a geometry_msgs/Pose at (x, y, z) with identity orientation."""
        from geometry_msgs.msg import Pose
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0
        return pose


# Entry point

def main(args=None) -> None:
    rclpy.init(args=args)
    node = MobileRobotController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()