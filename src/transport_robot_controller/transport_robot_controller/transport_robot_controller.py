#!/usr/bin/env python3
"""
ROS 2 controller node for the TEMPO transport robot.

The transport robot is a Clearpath Husky A200 with no manipulator.

Subscribes to /mission/subtask (std_msgs/String, JSON), keeps only subtasks
assigned to a transport_robot instance it manages, and executes them:

  navigate_to ->  Nav2 NavigateToPose action

Any other skill is reported as failed, since this platform cannot manipulate.

Goal poses are resolved first from `named_locations` in the config, then from
the Gazebo world pose topic /world/<world>/pose/info (with a standoff applied).
That topic is read directly over gz-transport: ros_gz_bridge's Pose_V ->
TFMessage conversion drops the entity names, which are needed here.

Each robot instance has its own worker thread and FIFO queue, so a robot never
runs two subtasks at once, and executor threads are never blocked waiting on
action results.

Topics / Actions
----------------
Subscribed:
  /mission/subtask                 (std_msgs/String)
  /world/<world>/pose/info         (gz.msgs.Pose_V, Gazebo topic via gz-transport)

Published:
  /mission/feedback                (std_msgs/String)

Action Clients:
  /<namespace>/navigate_to_pose    (nav2_msgs/NavigateToPose)
"""

import json
import math
import os
import queue
import threading
from typing import Any, Dict, Optional, Tuple

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Quaternion
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
from gz.transport13 import SubscribeOptions
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from std_msgs.msg import String
import yaml


ROBOT_TYPE = 'transport_robot'
SUPPORTED_SKILLS = ('navigate_to',)

Pose2D = Tuple[float, float, float]  # x, y, yaw


# Config loader

def load_config(path: Optional[str] = None) -> Dict[str, Any]:
    """Load transport_controller_config.yaml (package share dir by default)."""
    if path is None:
        share_dir = get_package_share_directory('transport_robot_controller')
        path = os.path.join(share_dir, 'config', 'transport_controller_config.yaml')
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


# Pose helpers

def approach_pose(
    x: float,
    y: float,
    yaw: float,
    standoff: float,
    axis: str = 'y',
    robot_xy: Optional[Tuple[float, float]] = None,
) -> Pose2D:
    """
    Compute a goal `standoff` metres from an entity, facing it.

    The goal lies on the entity's local `axis` ('x' or 'y'), on whichever
    side (+/-) is closer to the robot's current position (the + side if that
    is unknown). The returned heading points from the goal at the entity.

    Shelves in warehouse.sdf open on their local +/-Y faces, and are lined up
    along their local X, so the default axis is 'y'; stepping along local X
    (the entity's yaw) would land in the neighbouring shelf.
    """
    if axis == 'x':
        ux, uy = math.cos(yaw), math.sin(yaw)
    elif axis == 'y':
        ux, uy = -math.sin(yaw), math.cos(yaw)
    else:
        raise ValueError(f"approach axis must be 'x' or 'y', got {axis!r}")

    candidates = [
        (x + standoff * ux, y + standoff * uy),
        (x - standoff * ux, y - standoff * uy),
    ]
    if robot_xy is not None:
        rx, ry = robot_xy
        candidates.sort(key=lambda c: math.hypot(c[0] - rx, c[1] - ry))
    gx, gy = candidates[0]
    return gx, gy, math.atan2(y - gy, x - gx)


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Convert a yaw angle (radians) to a geometry_msgs Quaternion."""
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Extract yaw (rotation about z, ZYX convention), valid for tilted entities too."""
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def wait_for_future(future: Future, timeout_sec: float) -> bool:
    """
    Block the calling (non-executor) thread until `future` is done.

    Unlike rclpy.spin_until_future_complete, this does not spin the node a
    second time; the node's own executor completes the future.
    Returns True if the future finished within the timeout.
    """
    done = threading.Event()
    future.add_done_callback(lambda _: done.set())
    return done.wait(timeout_sec) or future.done()


# Transport robot controller node

class TransportRobotController(Node):
    """
    Execute navigate_to subtasks for Husky A200 transport robots via Nav2.

    Lifecycle
    ---------
    1. /mission/subtask arrives      -> _subtask_callback (filter + enqueue)
    2. Per-robot worker dequeues     -> _worker_loop
    3. Skill executed                -> _execute_navigate_to
    4. Feedback published            -> _publish_feedback
    """

    def __init__(self, config: Optional[Dict[str, Any]] = None) -> None:
        super().__init__('transport_robot_controller')

        config = config if config is not None else load_config()
        nav_cfg = config.get('navigation', {})

        self._robot_instances: Dict[str, str] = config['robot_instances']
        self._frame_id: str = nav_cfg.get('frame_id', 'map')
        self._standoff: float = float(nav_cfg.get('standoff_distance', 1.0))
        self._approach_axis: str = nav_cfg.get('approach_axis', 'y')
        self._approach_axis_overrides: Dict[str, str] = (
            nav_cfg.get('approach_axis_overrides') or {}
        )
        self._pose_period: float = float(config.get('world', {}).get(
            'pose_update_period_sec', 0.5))
        self._server_wait: float = float(nav_cfg.get('server_wait_sec', 5.0))
        self._nav_timeout: float = float(nav_cfg.get('nav_timeout_sec', 120.0))
        self._world_name: str = config.get('world', {}).get('name', 'default')
        self._named_locations: Dict[str, Pose2D] = {
            name: (float(p['x']), float(p['y']), float(p.get('yaw', 0.0)))
            for name, p in (config.get('named_locations') or {}).items()
        }

        self._cb_group = ReentrantCallbackGroup()

        # Gazebo entity name -> (x, y, yaw), refreshed by _pose_callback
        self._pose_registry: Dict[str, Pose2D] = {}
        self._pose_lock = threading.Lock()

        self.create_subscription(
            String, '/mission/subtask', self._subtask_callback, 10,
            callback_group=self._cb_group,
        )
        # Gazebo world poses, straight from gz-transport so entity names are
        # kept. The world publishes ~2200 poses at up to ~44 Hz, so gz-transport
        # throttles delivery to one message per pose_update_period_sec.
        pose_topic = f'/world/{self._world_name}/pose/info'
        self._pose_topic = pose_topic
        self._gz_node = GzNode()
        pose_opts = SubscribeOptions()
        pose_opts.msgs_per_sec = max(1, round(1.0 / self._pose_period))
        if not self._gz_node.subscribe(Pose_V, pose_topic, self._pose_callback, pose_opts):
            self.get_logger().error(f'Could not subscribe to Gazebo topic {pose_topic}')
        self._feedback_pub = self.create_publisher(String, '/mission/feedback', 10)

        # One Nav2 client, queue and worker thread per robot instance
        self._nav_clients: Dict[str, ActionClient] = {}
        self._queues: Dict[str, 'queue.Queue[Optional[Dict[str, Any]]]'] = {}
        self._workers: Dict[str, threading.Thread] = {}
        for instance_id, namespace in self._robot_instances.items():
            ns = '/' + namespace.strip('/')
            self._nav_clients[instance_id] = ActionClient(
                self, NavigateToPose, f'{ns}/navigate_to_pose',
                callback_group=self._cb_group,
            )
            self._queues[instance_id] = queue.Queue()
            worker = threading.Thread(
                target=self._worker_loop, args=(instance_id,),
                name=f'{instance_id}_worker', daemon=True,
            )
            self._workers[instance_id] = worker
            worker.start()

        self.get_logger().info(
            'TransportRobotController ready.\n'
            '  -> Subscribing to: /mission/subtask\n'
            f'  -> Subscribing to: /world/{self._world_name}/pose/info\n'
            '  -> Publishing to:  /mission/feedback\n'
            f'  -> Managing instances: {self._robot_instances}'
        )

    # Lifecycle

    def stop_workers(self) -> None:
        """Signal every worker thread to exit and wait for them."""
        for q in self._queues.values():
            q.put(None)
        for worker in self._workers.values():
            worker.join(timeout=2.0)

    def destroy_node(self) -> None:
        """Stop the Gazebo pose subscription, then destroy the ROS node."""
        self._gz_node.unsubscribe(self._pose_topic)
        super().destroy_node()

    def _wait_for_future_node_time(self, future: Future, timeout_sec: float) -> bool:
        """
        Like wait_for_future, but the timeout runs on the node's clock.

        With use_sim_time that is simulation time, so a sim running slower
        than real time (e.g. on CPU) does not make navigation time out early.
        """
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        deadline = self.get_clock().now() + Duration(seconds=timeout_sec)
        while not done.wait(0.1):
            if self.get_clock().now() >= deadline:
                return future.done()
        return True

    def _worker_loop(self, robot: str) -> None:
        """Execute queued subtasks for one robot strictly one at a time."""
        q = self._queues[robot]
        while True:
            subtask = q.get()
            if subtask is None:
                return
            try:
                self._execute_subtask(robot, subtask)
            except Exception as exc:  # never let one subtask kill the worker
                self.get_logger().error(f'[worker:{robot}] Unhandled exception: {exc}')
                self._publish_feedback(
                    subtask.get('plan_id', ''), subtask.get('task_id', ''), robot,
                    status='failed', stage='controller_error', detail=str(exc),
                )

    # World pose subscription

    def _pose_callback(self, msg: Pose_V) -> None:
        """Update the pose registry from the Gazebo pose topic (gz-transport thread)."""
        updates = {}
        for pose in msg.pose:
            if not pose.name:
                continue
            p, q = pose.position, pose.orientation
            updates[pose.name] = (p.x, p.y, quaternion_to_yaw(q.x, q.y, q.z, q.w))
        with self._pose_lock:
            self._pose_registry.update(updates)

    def _approach_axis_for(self, entity: str) -> str:
        """Return the approach axis for an entity, honouring name-prefix overrides."""
        for prefix, axis in self._approach_axis_overrides.items():
            if entity.startswith(prefix):
                return axis
        return self._approach_axis

    def resolve_goal(self, location: str, robot: Optional[str] = None) -> Optional[Pose2D]:
        """
        Resolve a location name to a navigation goal (x, y, yaw).

        Named locations from config are used as-is. Gazebo entities get a goal
        `standoff_distance` away on their approach axis, on the side nearest
        the robot (looked up in the same registry by its model name, which is
        its namespace), facing the entity.
        """
        if location in self._named_locations:
            return self._named_locations[location]

        robot_model = self._robot_instances.get(robot, '').strip('/') if robot else ''
        with self._pose_lock:
            pose = self._pose_registry.get(location)
            robot_pose = self._pose_registry.get(robot_model) if robot_model else None
            known = sorted(self._pose_registry.keys())[:10]
        if pose is None:
            self.get_logger().error(
                f"[resolve_goal] '{location}' is neither a named location nor a "
                f'known Gazebo entity. Some known entities: {known}...'
            )
            return None

        x, y, yaw = pose
        robot_xy = (robot_pose[0], robot_pose[1]) if robot_pose else None
        return approach_pose(
            x, y, yaw, self._standoff, self._approach_axis_for(location), robot_xy,
        )

    # Subtask subscription

    def _subtask_callback(self, msg: String) -> None:
        """Parse, filter, and enqueue a subtask; never blocks."""
        try:
            subtask = json.loads(msg.data.strip())
        except json.JSONDecodeError as exc:
            self.get_logger().error(f'[subtask_callback] Invalid JSON: {exc}')
            return
        if not isinstance(subtask, dict):
            self.get_logger().error('[subtask_callback] Subtask is not a JSON object.')
            return
        self.handle_subtask(subtask)

    def handle_subtask(self, subtask: Dict[str, Any]) -> None:
        """Validate ownership of a subtask and queue it on its robot's worker."""
        plan_id = subtask.get('plan_id', '')
        task_id = subtask.get('task_id', '')
        robot = subtask.get('robot', '')

        if not robot.startswith(ROBOT_TYPE):
            return  # another controller's subtask

        if robot not in self._robot_instances:
            self.get_logger().error(
                f"[handle_subtask] Robot instance '{robot}' not in config. "
                f'Known instances: {list(self._robot_instances)}'
            )
            self._publish_feedback(
                plan_id, task_id, robot, status='failed', stage='config_error',
                detail=f'Unknown robot instance: {robot}',
            )
            return

        self.get_logger().info(
            f'[handle_subtask] Queued subtask {task_id} for {robot} '
            f"(skills={subtask.get('required_skills', [])}, "
            f'queue depth={self._queues[robot].qsize()})'
        )
        self._queues[robot].put(subtask)

    def _execute_subtask(self, robot: str, subtask: Dict[str, Any]) -> None:
        """Dispatch one subtask to its skill handler (worker thread)."""
        plan_id = subtask.get('plan_id', '')
        task_id = subtask.get('task_id', '')
        skills = subtask.get('required_skills') or []
        args = subtask.get('args') or {}

        skill = skills[0] if skills else None
        if skill not in SUPPORTED_SKILLS:
            self._publish_feedback(
                plan_id, task_id, robot, status='failed', stage='config_error',
                detail=(f'Unsupported skill {skill!r} for {ROBOT_TYPE}; '
                        f'supported: {list(SUPPORTED_SKILLS)}'),
            )
            return

        self._execute_navigate_to(plan_id, task_id, robot, args)

    # Skill: navigate_to

    def _execute_navigate_to(
        self, plan_id: str, task_id: str, robot: str, args: Dict[str, Any],
    ) -> None:
        """Send a NavigateToPose goal to the robot's Nav2 stack and wait for it."""
        def fail(detail: str) -> None:
            self.get_logger().error(f'[navigate_to] Task {task_id}: {detail}')
            self._publish_feedback(
                plan_id, task_id, robot, status='failed', stage='navigate_to',
                detail=detail,
            )

        location = args.get('location')
        if not location:
            fail("Missing 'location' in args.")
            return

        goal_pose = self.resolve_goal(location, robot)
        if goal_pose is None:
            fail(f"Cannot resolve location '{location}'.")
            return
        gx, gy, gyaw = goal_pose

        nav_client = self._nav_clients[robot]
        if not nav_client.wait_for_server(timeout_sec=self._server_wait):
            fail(f'Nav2 action server not available for {robot}.')
            return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self._frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = gx
        goal.pose.pose.position.y = gy
        goal.pose.pose.orientation = yaw_to_quaternion(gyaw)

        self.get_logger().info(
            f"[navigate_to] Task {task_id}: {robot} -> '{location}' "
            f'({gx:.2f}, {gy:.2f}, yaw={gyaw:.2f}) in {self._frame_id}'
        )
        self._publish_feedback(
            plan_id, task_id, robot, status='in_progress', stage='navigating',
            detail=f"Navigating to '{location}'.",
        )

        send_goal_future = nav_client.send_goal_async(goal)
        if not wait_for_future(send_goal_future, self._server_wait):
            fail('Timed out sending goal to Nav2.')
            return

        goal_handle = send_goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            fail('Nav2 rejected the goal.')
            return

        result_future = goal_handle.get_result_async()
        if not self._wait_for_future_node_time(result_future, self._nav_timeout):
            goal_handle.cancel_goal_async()
            fail(f'Navigation timed out after {self._nav_timeout:.0f}s; goal cancelled.')
            return

        result = result_future.result()
        if result is None or result.status != GoalStatus.STATUS_SUCCEEDED:
            status = result.status if result is not None else 'unknown'
            fail(f'Navigation failed with status {status}.')
            return

        self.get_logger().info(f'[navigate_to] Task {task_id}: navigation complete.')
        self._publish_feedback(
            plan_id, task_id, robot, status='success', stage='complete',
            detail=f"Reached '{location}'.",
        )

    # Feedback publisher

    def _publish_feedback(
        self, plan_id: str, task_id: str, robot: str,
        status: str, stage: str, detail: str = '',
    ) -> None:
        """Publish a feedback message to /mission/feedback."""
        msg = String()
        msg.data = json.dumps({
            'plan_id': plan_id,
            'task_id': task_id,
            'robot': robot,
            'status': status,
            'stage': stage,
            'detail': detail,
        })
        self._feedback_pub.publish(msg)
        self.get_logger().info(
            f'[feedback] task={task_id} status={status} stage={stage} detail={detail}'
        )


# Entry point

def main(args=None) -> None:
    """Run the transport robot controller."""
    rclpy.init(args=args)
    node = TransportRobotController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_workers()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
