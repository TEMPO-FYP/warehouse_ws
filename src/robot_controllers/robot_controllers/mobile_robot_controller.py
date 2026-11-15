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

"""TEMPO mobile manipulator controller node for Unitree B1Z1."""

import json
import math
import os
import threading
import time
from typing import Any, Dict, Optional, Tuple

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    MoveItErrorCodes,
)
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from robot_controllers.map_standoff import OccupancyMap
from robot_controllers.z1_ik import (
    quaternion_to_yaw,
    solve_z1_ik,
    transform_world_to_base,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
import yaml


ROBOT_TYPE = 'mobile_robot'

PoseEntry = Tuple[float, float, float, Tuple[float, float, float, float]]

PRE_FOLD_JOINTS = {
    'joint1': 0.0,
    'joint2': 0.8,
    'joint3': -0.95,
    'joint4': -0.95,
    'joint5': 0.0,
    'joint6': 1.57,
}

# Retract with side-pinch roll first, then roll joint6=0.0 so lower jaw cradles during walk.
# Compact tuck (x=0.36m, z=0.50m) tilted +23 deg upward keeps load centered on turns.
CARRY_RETRACT_JOINTS = {
    'joint1': 0.0,
    'joint2': 0.50,
    'joint3': -0.45,
    'joint4': -0.45,
    'joint5': 0.0,
    'joint6': 1.57,
}

CARRY_JOINTS = {
    'joint1': 0.0,
    'joint2': 0.50,
    'joint3': -0.45,
    'joint4': -0.45,
    'joint5': 0.0,
    'joint6': 0.0,
}

HOME_JOINTS = {
    'joint1': 0.0,
    'joint2': 0.5,
    'joint3': -0.5,
    'joint4': 0.0,
    'joint5': 0.0,
    'joint6': 0.0,
}


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


class MobileRobotController(Node):
    """Execute TEMPO mobile_robot subtasks on B1Z1 via Nav2 and MoveIt 2."""

    def __init__(self) -> None:
        """Initialise subscriptions, action clients, and static warehouse map."""
        super().__init__(
            'mobile_robot_controller',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
        )

        config = _load_config()
        self._robot_instances: Dict[str, str] = config['robot_instances']

        nav_cfg = config['navigation']
        self._standoff: float = float(nav_cfg['standoff_distance'])
        self._manip_standoff: float = float(nav_cfg.get('manipulation_standoff', 0.73))
        self._nav_timeout: float = float(nav_cfg['nav_timeout_sec'])
        self._clearance_radius: float = float(nav_cfg.get('robot_clearance_radius', 0.36))
        self._named_locations: Dict[str, Tuple[float, float, float]] = {
            k: (float(v[0]), float(v[1]), float(v[2]) if len(v) > 2 else 0.0)
            for k, v in nav_cfg.get('named_locations', {}).items()
        }

        manip_cfg = config['manipulation']
        self._move_group_name: str = manip_cfg['move_group_name']
        self._gripper_group_name: str = manip_cfg.get('gripper_group_name', 'z1_gripper')
        self._gripper_open: float = float(manip_cfg['gripper_open_position'])
        self._gripper_closed: float = float(manip_cfg['gripper_closed_position'])
        self._gripper_open_tol: float = float(manip_cfg.get('gripper_open_tolerance', 0.08))
        self._gripper_closed_tol: float = float(manip_cfg.get('gripper_closed_tolerance', 0.02))
        self._arm_tol: float = float(manip_cfg.get('arm_joint_tolerance', 0.005))
        self._moveit_timeout: float = float(manip_cfg['moveit_timeout_sec'])
        self._min_reach: float = float(manip_cfg.get('min_reach_m', 0.40))
        self._max_reach: float = float(manip_cfg.get('max_reach_m', 0.80))
        self._place_drop_standoff: float = float(manip_cfg.get('place_drop_standoff', 0.68))
        self._place_clear_z_offset: float = float(manip_cfg.get('place_clear_z_offset', 0.075))
        self._fixture_surface_z: Dict[str, float] = {
            k: float(v) for k, v in manip_cfg.get('fixture_surface_z', {}).items()
        }

        world_cfg = config['world']
        self._world_name: str = world_cfg['name']
        self._robot_model_name: str = world_cfg.get('robot_model_name', 'b1z1')

        self._occ_map = self._load_occupancy_map(
            nav_cfg.get('map_package', 'b1z1_control'),
            nav_cfg.get('map_yaml_relpath', 'maps/warehouse_map.yaml'),
        )

        self._cb_group = ReentrantCallbackGroup()
        self._pose_lock = threading.Lock()
        self._pose_registry: Dict[str, PoseEntry] = {}
        self._js_lock = threading.Lock()
        self._joint_positions: Dict[str, float] = {}
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
        self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10,
            callback_group=self._cb_group,
        )
        self._feedback_pub = self.create_publisher(String, '/mission/feedback', 10)
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self._nav_clients: Dict[str, ActionClient] = {}
        self._moveit_clients: Dict[str, ActionClient] = {}
        for instance_id, namespace in self._robot_instances.items():
            ns_prefix = namespace.rstrip('/')
            nav_topic = f'{ns_prefix}/navigate_to_pose' if ns_prefix else '/navigate_to_pose'
            move_topic = f'{ns_prefix}/move_action' if ns_prefix else '/move_action'
            self._nav_clients[instance_id] = ActionClient(
                self, NavigateToPose, nav_topic, callback_group=self._cb_group
            )
            self._moveit_clients[instance_id] = ActionClient(
                self, MoveGroup, move_topic, callback_group=self._cb_group
            )
            self.get_logger().info(
                f'[init] {instance_id}: nav={nav_topic}, moveit={move_topic}'
            )

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

    def _joint_state_callback(self, msg: JointState) -> None:
        with self._js_lock:
            for name, pos in zip(msg.name, msg.position):
                self._joint_positions[name] = float(pos)

    def _get_joint_position(self, joint_name: str) -> Optional[float]:
        with self._js_lock:
            return self._joint_positions.get(joint_name)

    def _sim_sleep(self, sim_sec: float) -> None:
        """Sleep for sim_sec simulated seconds using /clock with wall-time guard."""
        if sim_sec <= 0.0:
            return
        start_ns = self.get_clock().now().nanoseconds
        if start_ns <= 0:
            time.sleep(sim_sec)
            return
        target_ns = start_ns + int(sim_sec * 1e9)
        wall_deadline = time.monotonic() + max(15.0, sim_sec * 18.0)
        while self.get_clock().now().nanoseconds < target_ns and time.monotonic() < wall_deadline:
            time.sleep(0.05)

    def _wait_gripper_below(self, threshold: float, max_sim_sec: float) -> Optional[float]:
        """Wait until jointGripper opens past threshold or timeout expires."""
        start_ns = self.get_clock().now().nanoseconds
        target_ns = start_ns + int(max_sim_sec * 1e9) if start_ns > 0 else 0
        wall_deadline = time.monotonic() + max(15.0, max_sim_sec * 18.0)
        while time.monotonic() < wall_deadline:
            pos = self._get_joint_position('jointGripper')
            if pos is not None and pos <= threshold:
                return pos
            if target_ns > 0 and self.get_clock().now().nanoseconds >= target_ns:
                break
            time.sleep(0.05)
        return self._get_joint_position('jointGripper')

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
            elif skill == 'pick':
                self._execute_pick(plan_id, task_id, robot, args)
            elif skill == 'place':
                self._execute_place(plan_id, task_id, robot, args)
            else:
                self._publish_feedback(
                    plan_id, task_id, robot,
                    status='failed',
                    stage='config_error',
                    detail=f'Unknown skill: {skill}',
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
        pref_dist = (
            self._manip_standoff
            if any(location.startswith(p) for p in ('graspable_box', 'test_pedestal'))
            else self._standoff
        )
        gx, gy, gyaw = self._compute_nav_goal(
            tx, ty, target_yaw, pref_dist, is_named_zone
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

    def _drive_pulse(self, vx: float, wz: float, sim_dur: float) -> None:
        """Command a bounded-duration body velocity pulse in simulated time."""
        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.angular.z = float(wz)
        start_ns = self.get_clock().now().nanoseconds
        target_ns = start_ns + int(sim_dur * 1e9) if start_ns > 0 else 0
        wall_deadline = time.monotonic() + max(10.0, sim_dur * 18.0)
        while time.monotonic() < wall_deadline:
            self._cmd_vel_pub.publish(cmd)
            if target_ns > 0:
                if self.get_clock().now().nanoseconds >= target_ns:
                    break
            elif time.monotonic() >= (wall_deadline - max(10.0, sim_dur * 18.0) + sim_dur):
                break
            time.sleep(0.03)
        self._cmd_vel_pub.publish(Twist())
        self._sim_sleep(0.9)

    def _ensure_aligned(
        self,
        robot: str,
        tx: float,
        ty: float,
        task_id: str,
        max_dist: float,
        preferred_standoff: float,
        tight: bool = True,
    ) -> Tuple[bool, str]:
        """Reposition b1z1 so target sits inside the Z1 reach and alignment window."""
        robot_pose = self._get_robot_pose()
        if robot_pose is None:
            return False, 'Robot pose unavailable in /world/default/pose/info.'

        rx, ry, rz, rquat = robot_pose
        bx, by, _ = transform_world_to_base(tx, ty, rz, (rx, ry, rz), rquat)
        horiz = math.hypot(bx, by)

        if horiz > 1.25 or bx < 0.25:
            target_yaw = math.atan2(ty - ry, tx - rx)
            gx, gy, gyaw = self._compute_nav_goal(
                tx, ty, target_yaw, preferred_standoff, is_named_free_zone=False
            )
            self.get_logger().info(
                f'[{task_id}] Target in base_link ({bx:.2f}, {by:.2f}) far; '
                f'navigating to ({gx:.2f}, {gy:.2f}, yaw={gyaw:.2f})'
            )
            ok, err = self._send_nav_goal(robot, gx, gy, gyaw, f'{task_id}_align')
            if not ok:
                return False, f'Pre-manipulation alignment failed: {err}'
            self._sim_sleep(0.5)

        dist_tol = 0.085 if tight else 0.12
        lat_tol = 0.055 if tight else 0.16
        yaw_tol = 0.070 if tight else 0.14

        for step in range(8):
            robot_pose = self._get_robot_pose()
            if robot_pose is None:
                return False, 'Robot pose lost during fine alignment.'
            rx, ry, rz, rquat = robot_pose
            bx, by, _ = transform_world_to_base(tx, ty, rz, (rx, ry, rz), rquat)
            ryaw = quaternion_to_yaw(*rquat)
            target_yaw = math.atan2(ty - ry, tx - rx)
            yaw_err = math.atan2(math.sin(target_yaw - ryaw), math.cos(target_yaw - ryaw))
            ex = bx - preferred_standoff
            if abs(ex) <= dist_tol and abs(by) <= lat_tol and abs(yaw_err) <= yaw_tol:
                break
            # Use >= 0.55s (> 0.45s stance_duration) arcs so CHAMP completes full gait cycles.
            dur = max(0.55, min(1.2, max(abs(ex) / 0.18, abs(yaw_err) / 0.28)))
            vx = 0.0
            if ex > dist_tol:
                vx = min(0.24, max(0.14, ex / dur))
            elif ex < -dist_tol:
                vx = -min(0.12, max(0.08, -ex / dur))
            elif abs(yaw_err) > yaw_tol and ex > -0.02:
                vx = 0.06
            wz = 0.0
            if abs(yaw_err) > yaw_tol:
                wz = max(-0.32, min(0.32, yaw_err / dur))
            self._drive_pulse(vx, wz, dur)
            self.get_logger().info(
                f'[{task_id}] align pulse {step}: base=({bx:.3f}, {by:.3f}) yaw_err={yaw_err:.3f}'
            )

        self._cmd_vel_pub.publish(Twist())
        self._sim_sleep(0.3)

        robot_pose = self._get_robot_pose()
        if robot_pose is None:
            return False, 'Robot pose lost after alignment.'
        rx, ry, rz, rquat = robot_pose
        bx, by, _ = transform_world_to_base(tx, ty, rz, (rx, ry, rz), rquat)
        horiz = math.hypot(bx, by)
        self.get_logger().info(
            f'[{task_id}] Post-align base_link=({bx:.3f}, {by:.3f}), horiz={horiz:.3f}'
        )
        max_lat = 0.10 if tight else 0.22
        max_horiz = max_dist + 0.06 if tight else max_dist + 0.15
        if bx <= 0.35 or horiz > max_horiz or abs(by) > max_lat:
            return False, (
                f'Alignment outside manipulation window: '
                f'base=({bx:.3f}, {by:.3f}), horiz={horiz:.3f}'
            )
        return True, ''

    def _send_joint_goal(
        self,
        robot: str,
        group: str,
        targets: Dict[str, float],
        label: str,
        tol: float,
        vel_scale: float = 0.4,
    ) -> bool:
        client = self._moveit_clients[robot]
        if not client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f'[moveit] /move_action unavailable for {robot}.')
            return False

        req = MotionPlanRequest()
        req.group_name = group
        req.num_planning_attempts = 3
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = vel_scale
        req.max_acceleration_scaling_factor = vel_scale

        c = Constraints()
        for joint_name, val in targets.items():
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = float(val)
            jc.tolerance_above = tol
            jc.tolerance_below = tol
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        req.goal_constraints.append(c)

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 2

        send_future = client.send_goal_async(goal)
        if not self._wait_future(send_future, timeout_sec=15.0):
            self.get_logger().error(f'[moveit] Timed out sending [{label}].')
            return False

        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error(f'[moveit] Goal [{label}] rejected.')
            return False

        result_future = goal_handle.get_result_async()
        if not self._wait_future(result_future, timeout_sec=self._moveit_timeout):
            goal_handle.cancel_goal_async()
            self.get_logger().error(f'[moveit] Goal [{label}] result timed out.')
            return False

        wrapped = result_future.result()
        if wrapped is None or wrapped.result.error_code.val != MoveItErrorCodes.SUCCESS:
            code = wrapped.result.error_code.val if wrapped is not None else -1
            self.get_logger().error(f'[moveit] [{label}] failed with MoveItErrorCode={code}.')
            return False
        self.get_logger().info(f'[moveit] [{label}] SUCCEEDED.')
        return True

    def _to_robot_base(
        self, wx: float, wy: float, wz: float
    ) -> Optional[Tuple[float, float, float]]:
        robot_pose = self._get_robot_pose()
        if robot_pose is None:
            return None
        rx, ry, rz, rquat = robot_pose
        return transform_world_to_base(wx, wy, wz, (rx, ry, rz), rquat)

    def _infer_surface_z(self, location_name: str, default_z: float) -> float:
        for prefix, surf_z in self._fixture_surface_z.items():
            if location_name.startswith(prefix):
                return surf_z
        return default_z + 0.12

    def _run_joint_sequence(
        self,
        plan_id: str,
        task_id: str,
        robot: str,
        stage: str,
        steps: Any,
        context_label: str,
    ) -> bool:
        for group, targets, label, tol, pause_sim, vel_scale in steps:
            ok = self._send_joint_goal(
                robot, group, targets, f'{task_id}_{label}', tol, vel_scale=vel_scale
            )
            if not ok:
                self._publish_feedback(
                    plan_id, task_id, robot,
                    status='failed', stage=stage,
                    detail=f"MoveIt2 failed on {stage} step '{label}' ({context_label}).",
                )
                return False
            self._sim_sleep(pause_sim)
            if label == 'open':
                grip = self._wait_gripper_below(-1.25, 4.0)
                self.get_logger().info(f'[{task_id}] Gripper after open settle: {grip}')
        return True

    def _execute_pick(
        self,
        plan_id: str,
        task_id: str,
        robot: str,
        args: Dict[str, Any],
    ) -> None:
        object_name = args.get('object')
        if not object_name:
            self._publish_feedback(
                plan_id, task_id, robot,
                status='failed', stage='pick', detail="Missing 'object' in args.",
            )
            return

        pose, _ = self._resolve_target_pose(object_name)
        if pose is None:
            self._publish_feedback(
                plan_id, task_id, robot,
                status='failed', stage='pick',
                detail=f"Object '{object_name}' not found in pose registry.",
            )
            return

        wx, wy, wz, _ = pose
        self._publish_feedback(
            plan_id, task_id, robot,
            status='in_progress', stage='pick_in_progress',
            detail=f"Picking '{object_name}'.",
        )

        ok, err = self._ensure_aligned(
            robot, wx, wy, task_id, self._max_reach, self._manip_standoff, tight=True
        )
        if not ok:
            self._publish_feedback(
                plan_id, task_id, robot, status='failed', stage='pick', detail=err
            )
            return

        # Re-read target pose after alignment so base_link coordinates are fresh.
        pose, _ = self._resolve_target_pose(object_name)
        if pose is None:
            self._publish_feedback(
                plan_id, task_id, robot,
                status='failed', stage='pick',
                detail=f"Lost pose of '{object_name}'.",
            )
            return
        wx, wy, pre_pick_z, _ = pose
        base_pt = self._to_robot_base(wx, wy, pre_pick_z)
        if base_pt is None:
            self._publish_feedback(
                plan_id, task_id, robot,
                status='failed', stage='pick', detail='Robot base pose unavailable.',
            )
            return

        bx, by, bz = base_pt
        q_pre_grasp = solve_z1_ik(bx - 0.025, by, bz + 0.15)
        q_grasp = solve_z1_ik(bx - 0.025, by, bz + 0.018)
        q_lift = solve_z1_ik(bx - 0.045, by, bz + 0.22)

        grasp_steps = [
            (
                self._gripper_group_name,
                {'jointGripper': self._gripper_open},
                'open',
                self._gripper_open_tol,
                0.3,
                0.5,
            ),
            (self._move_group_name, PRE_FOLD_JOINTS, 'pre_fold', self._arm_tol, 0.3, 0.5),
            (self._move_group_name, q_pre_grasp, 'pre_grasp', self._arm_tol, 0.3, 0.4),
            (self._move_group_name, q_grasp, 'grasp', self._arm_tol, 0.4, 0.25),
            (
                self._gripper_group_name,
                {'jointGripper': self._gripper_closed},
                'close',
                self._gripper_closed_tol,
                2.0,
                0.4,
            ),
            (self._move_group_name, q_lift, 'lift', self._arm_tol, 0.5, 0.25),
        ]
        if not self._run_joint_sequence(
            plan_id, task_id, robot, 'pick', grasp_steps, object_name
        ):
            return

        lift_pose, _ = self._resolve_target_pose(object_name)
        lift_z = lift_pose[2] if lift_pose is not None else pre_pick_z
        grip_pos = self._get_joint_position('jointGripper')
        self.get_logger().info(
            f'[{task_id}] Post-lift verify: z={lift_z:.3f} (was {pre_pick_z:.3f}), '
            f'grip={grip_pos}'
        )
        if lift_z < pre_pick_z + 0.08:
            self._publish_feedback(
                plan_id, task_id, robot,
                status='failed', stage='pick',
                detail=(
                    f"Grasp verification failed for '{object_name}': "
                    f'z={lift_z:.3f}m vs pre-pick {pre_pick_z:.3f}m'
                ),
            )
            return

        carry_steps = [
            (
                self._move_group_name,
                CARRY_RETRACT_JOINTS,
                'carry_retract',
                self._arm_tol,
                0.3,
                0.25,
            ),
            (self._move_group_name, CARRY_JOINTS, 'carry_cradle', self._arm_tol, 0.5, 0.15),
        ]
        if not self._run_joint_sequence(
            plan_id, task_id, robot, 'pick', carry_steps, object_name
        ):
            return

        carry_pose, _ = self._resolve_target_pose(object_name)
        carry_z = carry_pose[2] if carry_pose is not None else pre_pick_z
        if carry_z < pre_pick_z + 0.08:
            self._publish_feedback(
                plan_id, task_id, robot,
                status='failed', stage='pick',
                detail=(
                    f"Carry verification failed for '{object_name}': "
                    f'dropped to z={carry_z:.3f}m during carry tuck'
                ),
            )
            return

        self._publish_feedback(
            plan_id, task_id, robot,
            status='success', stage='complete', detail=f"Picked '{object_name}'.",
        )

    def _execute_place(
        self,
        plan_id: str,
        task_id: str,
        robot: str,
        args: Dict[str, Any],
    ) -> None:
        location = args.get('location')
        object_name = args.get('object', 'object')
        if not location:
            self._publish_feedback(
                plan_id, task_id, robot,
                status='failed', stage='place', detail="Missing 'location' in args.",
            )
            return

        pose, is_named_zone = self._resolve_target_pose(location)
        if pose is None:
            self._publish_feedback(
                plan_id, task_id, robot,
                status='failed', stage='place',
                detail=f"Location '{location}' not found in pose registry or named_locations.",
            )
            return

        lx, ly, lz, _ = pose
        self._publish_feedback(
            plan_id, task_id, robot,
            status='in_progress', stage='place_in_progress',
            detail=f"Placing '{object_name}' at '{location}'.",
        )

        if not is_named_zone:
            large_fixture = any(
                location.startswith(p)
                for p in ('Packing_Station', 'Picking_Shelves', 'Pallet_Standard')
            )
            max_dist = 1.02 if large_fixture else self._max_reach
            pref_dist = self._standoff if large_fixture else self._manip_standoff
            ok, err = self._ensure_aligned(
                robot, lx, ly, task_id, max_dist, pref_dist, tight=not large_fixture
            )
            if not ok:
                self._publish_feedback(
                    plan_id, task_id, robot, status='failed', stage='place', detail=err
                )
                return

        robot_pose = self._get_robot_pose()
        if robot_pose is None:
            self._publish_feedback(
                plan_id, task_id, robot,
                status='failed', stage='place', detail='Robot base pose unavailable.',
            )
            return

        rx, ry, rz, rquat = robot_pose
        bx_raw, by_raw, _ = transform_world_to_base(lx, ly, lz, (rx, ry, rz), rquat)
        dist = math.hypot(bx_raw, by_raw)
        if dist > self._place_drop_standoff and dist > 1e-4:
            scale = self._place_drop_standoff / dist
            bx = bx_raw * scale
            by = by_raw * scale
        else:
            bx = max(0.55, bx_raw - 0.03)
            by = by_raw

        surface_z = (
            rz - 0.35
            if is_named_zone
            else self._infer_surface_z(location, lz)
        )
        drop_world_z = surface_z + self._place_clear_z_offset
        bz = drop_world_z - rz

        q_pre_place = solve_z1_ik(bx, by, bz + 0.14, j6=0.0)
        q_place_cradle = solve_z1_ik(bx, by, bz + 0.02, j6=0.0)
        q_place_tip = solve_z1_ik(bx, by, bz + 0.02, j6=1.57)
        q_retreat = solve_z1_ik(bx, by, bz + 0.16, j6=1.57)

        steps = [
            (self._move_group_name, q_pre_place, 'pre_place', self._arm_tol, 0.3, 0.25),
            (self._move_group_name, q_place_cradle, 'place_descend', self._arm_tol, 0.4, 0.25),
            (self._move_group_name, q_place_tip, 'place_tip', self._arm_tol, 0.3, 0.20),
            (
                self._gripper_group_name,
                {'jointGripper': self._gripper_open},
                'release',
                self._gripper_open_tol,
                1.0,
                0.5,
            ),
            (self._move_group_name, q_retreat, 'retreat', self._arm_tol, 0.3, 0.35),
            (self._move_group_name, HOME_JOINTS, 'stow', self._arm_tol, 0.5, 0.4),
        ]
        if not self._run_joint_sequence(plan_id, task_id, robot, 'place', steps, location):
            return

        if object_name != 'object':
            with self._pose_lock:
                obj_pose = self._pose_registry.get(object_name)
            if obj_pose is not None:
                ox, oy, oz, _ = obj_pose
                horiz_err = math.hypot(ox - lx, oy - ly)
                self.get_logger().info(
                    f'[{task_id}] Post-place verify: obj=({ox:.3f}, {oy:.3f}, {oz:.3f}) '
                    f'vs target=({lx:.3f}, {ly:.3f}, surf_z={surface_z:.3f}) '
                    f'horiz_err={horiz_err:.3f}'
                )
                if horiz_err > 1.25 or (not is_named_zone and oz < surface_z - 0.05):
                    self._publish_feedback(
                        plan_id, task_id, robot,
                        status='failed', stage='place',
                        detail=(
                            f"Place verification failed for '{object_name}' at '{location}': "
                            f'obj=({ox:.3f}, {oy:.3f}, {oz:.3f})'
                        ),
                    )
                    return

        self._publish_feedback(
            plan_id, task_id, robot,
            status='success', stage='complete',
            detail=f"Placed '{object_name}' at '{location}'.",
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
    """Run the TEMPO mobile robot controller node."""
    rclpy.init(args=args)
    node = MobileRobotController()
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
