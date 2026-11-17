# Copyright 2026 TEMPO Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""TEMPO navigation-only controller node for the b2_cargo transport quadruped."""

import json
import math
import os
import threading
import time
from typing import Any, Dict, Optional, Tuple

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from robot_controllers.map_standoff import OccupancyMap
from robot_controllers.z1_ik import quaternion_to_yaw
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
import yaml


ROBOT_TYPE = 'quadruped_transport_robot'

PoseEntry = Tuple[float, float, float, Tuple[float, float, float, float]]


def _load_config() -> Dict[str, Any]:
    pkg_dir = get_package_share_directory('robot_controllers')
    config_path = os.path.join(pkg_dir, 'config', 'controller_config.yaml')
    with open(config_path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def _yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def _yaw_quat_tuple(yaw: float) -> Tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class TransportRobotController(Node):
    """Execute TEMPO quadruped_transport_robot navigation subtasks on b2_cargo via Nav2."""

    def __init__(self) -> None:
        """Initialise subscriptions, Nav2 action clients, and static warehouse map."""
        super().__init__(
            'transport_robot_controller',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
        )

        config = _load_config()
        self._robot_instances: Dict[str, str] = config.get(
            'transport_instances',
            {'quadruped_transport_robot_1': ''},
        )

        nav_cfg = config['navigation']
        self._standoff: float = float(nav_cfg['standoff_distance'])
        self._nav_timeout: float = float(nav_cfg['nav_timeout_sec'])
        self._clearance_radius: float = float(
            nav_cfg.get('transport_clearance_radius', 0.42)
        )
        self._named_locations: Dict[str, Tuple[float, float, float]] = {
            k: (float(v[0]), float(v[1]), float(v[2]) if len(v) > 2 else 0.0)
            for k, v in nav_cfg.get('named_locations', {}).items()
        }

        world_cfg = config['world']
        self._world_name: str = world_cfg['name']
        self._robot_model_name: str = world_cfg.get('transport_model_name', 'b2_cargo')

        self._occ_map = self._load_occupancy_map(
            nav_cfg.get('map_package', 'warehouse_sim'),
            nav_cfg.get('map_yaml_relpath', 'maps/warehouse_map.yaml'),
        )

        self._cb_group = ReentrantCallbackGroup()
        self._pose_lock = threading.Lock()
        self._pose_registry: Dict[str, PoseEntry] = {}
        self._robot_locks: Dict[str, threading.Lock] = {
            inst: threading.Lock() for inst in self._robot_instances
        }

        self.create_subscription(
            String,
            '/mission/subtask',
            self._subtask_callback,
            10,
            callback_group=self._cb_group,
        )
        self.create_subscription(
            TFMessage,
            f'/world/{self._world_name}/pose/info',
            self._pose_callback,
            10,
            callback_group=self._cb_group,
        )
        self._feedback_pub = self.create_publisher(String, '/mission/feedback', 10)

        self._nav_clients: Dict[str, ActionClient] = {}
        for instance_id, namespace in self._robot_instances.items():
            ns_prefix = namespace.rstrip('/')
            nav_topic = f'{ns_prefix}/navigate_to_pose' if ns_prefix else '/navigate_to_pose'
            self._nav_clients[instance_id] = ActionClient(
                self, NavigateToPose, nav_topic, callback_group=self._cb_group
            )
            self.get_logger().info(f'[init] {instance_id}: nav={nav_topic}')

    def _load_occupancy_map(
        self,
        pkg_name: str,
        rel_path: str,
    ) -> Optional[OccupancyMap]:
        try:
            map_yaml = os.path.join(get_package_share_directory(pkg_name), rel_path)
            if os.path.isfile(map_yaml):
                self.get_logger().info(f'[init] Loaded standoff map from {map_yaml}')
                return OccupancyMap(map_yaml)
        except Exception as exc:
            self.get_logger().warn(f'[init] Could not load static standoff map: {exc}')
        return None

    def _pose_callback(self, msg: TFMessage) -> None:
        with self._pose_lock:
            for transform in msg.transforms:
                name = transform.child_frame_id
                if not name:
                    continue
                tr = transform.transform.translation
                rot = transform.transform.rotation
                self._pose_registry[name] = (
                    tr.x,
                    tr.y,
                    tr.z,
                    (rot.x, rot.y, rot.z, rot.w),
                )

    def _get_robot_pose(self) -> Optional[PoseEntry]:
        with self._pose_lock:
            return (
                self._pose_registry.get(self._robot_model_name)
                or self._pose_registry.get(f'{self._robot_model_name}::base_link')
            )

    def _resolve_target_pose(
        self,
        entity_name: str,
    ) -> Tuple[Optional[PoseEntry], bool]:
        """Return (pose_entry, is_named_free_zone) for a Gazebo entity or named zone."""
        named = self._named_locations.get(entity_name)
        deadline = time.monotonic() + (0.0 if named is not None else 1.5)
        while True:
            with self._pose_lock:
                entry = self._pose_registry.get(entity_name)
            if entry is not None:
                return entry, False
            if named is not None or time.monotonic() >= deadline:
                break
            time.sleep(0.1)

        if named is not None:
            nx, ny, nyaw = named
            return (nx, ny, 0.0, _yaw_quat_tuple(nyaw)), True

        with self._pose_lock:
            sample = sorted(self._pose_registry.keys())[:10]
        self.get_logger().error(
            f"[pose] '{entity_name}' not found in /world/{self._world_name}/pose/info "
            f'or named_locations. Sample entities: {sample}'
        )
        return None, False

    def _wait_future(self, future: Any, timeout_sec: float) -> bool:
        """Block until an rclpy Future completes or timeout_sec elapses on /clock."""
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        start_ns = self.get_clock().now().nanoseconds
        if start_ns <= 0:
            return done.wait(timeout=timeout_sec)
        target_ns = start_ns + int(timeout_sec * 1e9)
        wall_deadline = time.monotonic() + max(30.0, timeout_sec * 10.0)
        while time.monotonic() < wall_deadline:
            if done.wait(timeout=0.05):
                return True
            if self.get_clock().now().nanoseconds >= target_ns:
                break
        return done.is_set()

    def _subtask_callback(self, msg: String) -> None:
        try:
            subtask = json.loads(msg.data.strip())
            self._handle_subtask(subtask)
        except Exception as exc:
            self.get_logger().error(f'[subtask] Unhandled exception: {exc}')

    def _handle_subtask(self, subtask: Dict[str, Any]) -> None:
        plan_id = subtask.get('plan_id', '')
        task_id = subtask.get('task_id', '')
        robot = subtask.get('robot', '')
        skills = subtask.get('required_skills', [])
        args = subtask.get('args', {})

        if not robot.startswith(ROBOT_TYPE):
            return

        if robot not in self._robot_instances:
            self._publish_feedback(
                plan_id, task_id, robot,
                status='failed',
                stage='config_error',
                detail=f'Unknown robot instance: {robot}',
            )
            return

        skill = skills[0] if skills else None
        if not skill:
            self._publish_feedback(
                plan_id, task_id, robot,
                status='failed',
                stage='config_error',
                detail='No required_skills specified.',
            )
            return

        with self._robot_locks[robot]:
            if skill == 'navigate_to':
                self._execute_navigate_to(plan_id, task_id, robot, args)
            else:
                self._publish_feedback(
                    plan_id, task_id, robot,
                    status='failed',
                    stage='unsupported_skill',
                    detail=(
                        f"Transport robot '{robot}' supports navigation only; "
                        f"unsupported skill '{skill}'."
                    ),
                )

    def _compute_nav_goal(
        self,
        tx: float,
        ty: float,
        target_yaw: float,
        preferred_dist: float,
        is_named_free_zone: bool,
    ) -> Tuple[float, float, float]:
        robot_pose = self._get_robot_pose()
        rx = robot_pose[0] if robot_pose is not None else None
        ry = robot_pose[1] if robot_pose is not None else None

        if is_named_free_zone:
            if self._occ_map is None or self._occ_map.is_world_free(
                tx, ty, self._clearance_radius
            ):
                return tx, ty, target_yaw
            return self._occ_map.find_standoff_pose(
                tx, ty, target_yaw, rx, ry, preferred_dist, self._clearance_radius
            )

        if self._occ_map is not None:
            return self._occ_map.find_standoff_pose(
                tx, ty, target_yaw, rx, ry, preferred_dist, self._clearance_radius
            )

        approach = (
            math.atan2(ry - ty, rx - tx)
            if rx is not None and ry is not None
            else (target_yaw + math.pi)
        )
        gx = tx + preferred_dist * math.cos(approach)
        gy = ty + preferred_dist * math.sin(approach)
        return gx, gy, math.atan2(ty - gy, tx - gx)

    def _send_nav_goal(
        self,
        robot: str,
        gx: float,
        gy: float,
        gyaw: float,
        task_id: str,
    ) -> Tuple[bool, str]:
        nav_client = self._nav_clients[robot]
        if not nav_client.wait_for_server(timeout_sec=5.0):
            return False, 'Nav2 action server not available.'

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = gx
        goal.pose.pose.position.y = gy
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation = _yaw_to_quaternion(gyaw)

        send_future = nav_client.send_goal_async(goal)
        if not self._wait_future(send_future, timeout_sec=10.0):
            return False, f'Timed out sending Nav2 goal for {task_id}.'

        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            return False, f'Nav2 rejected goal ({gx:.2f}, {gy:.2f}) for {task_id}.'

        result_future = goal_handle.get_result_async()
        if not self._wait_future(result_future, timeout_sec=self._nav_timeout):
            goal_handle.cancel_goal_async()
            return False, f'Nav2 timed out after {self._nav_timeout}s for {task_id}.'

        wrapped = result_future.result()
        if wrapped is None or wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            status = wrapped.status if wrapped is not None else -1
            return False, f'Nav2 failed with status {status}.'
        return True, ''

    def _execute_navigate_to(
        self,
        plan_id: str,
        task_id: str,
        robot: str,
        args: Dict[str, Any],
    ) -> None:
        location = args.get('location')
        if not location:
            self._publish_feedback(
                plan_id, task_id, robot,
                status='failed', stage='navigate_to', detail="Missing 'location' in args.",
            )
            return

        pose, is_named_zone = self._resolve_target_pose(location)
        if pose is None:
            self._publish_feedback(
                plan_id, task_id, robot,
                status='failed', stage='navigate_to',
                detail=f"Entity '{location}' not found in pose registry or named_locations.",
            )
            return

        tx, ty, _, quat = pose
        target_yaw = quaternion_to_yaw(*quat)
        gx, gy, gyaw = self._compute_nav_goal(
            tx, ty, target_yaw, self._standoff, is_named_zone
        )

        self.get_logger().info(
            f"[navigate_to] {task_id} -> '{location}' at ({tx:.2f}, {ty:.2f}) "
            f'goal=({gx:.2f}, {gy:.2f}, yaw={gyaw:.2f})'
        )
        self._publish_feedback(
            plan_id, task_id, robot,
            status='in_progress', stage='navigating', detail=f"Navigating to '{location}'.",
        )

        ok, err = self._send_nav_goal(robot, gx, gy, gyaw, task_id)
        if not ok:
            self._publish_feedback(
                plan_id, task_id, robot,
                status='failed', stage='navigate_to', detail=err,
            )
            return

        self._publish_feedback(
            plan_id, task_id, robot,
            status='success', stage='complete', detail=f"Reached '{location}'.",
        )

    def _publish_feedback(
        self,
        plan_id: str,
        task_id: str,
        robot: str,
        status: str,
        stage: str,
        detail: str = '',
    ) -> None:
        payload = json.dumps({
            'plan_id': plan_id,
            'task_id': task_id,
            'robot': robot,
            'status': status,
            'stage': stage,
            'detail': detail,
        })
        msg = String()
        msg.data = payload
        self._feedback_pub.publish(msg)
        self.get_logger().info(
            f'[feedback] task={task_id} status={status} stage={stage} detail={detail}'
        )


def main(args: Any = None) -> None:
    """Run the TEMPO transport robot controller node."""
    rclpy.init(args=args)
    node = TransportRobotController()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
