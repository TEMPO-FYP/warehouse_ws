#!/usr/bin/env python3
"""
pick_and_place.py

End-to-end collision-aware front-side pick-and-place pipeline for `bcr_arm` +
Robotiq 2F-85 in Gazebo Harmonic and MoveIt 2, with TEMPO subtask worker support.

Implements:
- Phase 3: Pick sequence (pre-grasp standoff, straight Cartesian approach,
  gripper stall grasp + MoveIt attach, vertical micro-lift, Cartesian retract).
- Phase 4: Place sequence (free-space transit to pre-place, slow Cartesian
  descent, detach + release + scene update, post-place lift & return ready).
- TEMPO Integration: `TempoArmWorkerNode` subscribing to `/mission/subtask`
  and publishing `/mission/feedback` for natural-language grounded `pick` and
  `place` commands.
"""

import argparse
import json
import math
import os
import sys
import threading
import time
from typing import Any, Dict, List, Optional, Sequence, Tuple

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    PlanningSceneComponents,
    RobotState,
)
from moveit_msgs.srv import GetCartesianPath, GetMotionPlan, GetStateValidity
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from scipy.optimize import minimize
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from warehouse_sim.gripper_interface import GripperInterface
from warehouse_sim.load_test_shelf_scene import load_test_shelf_scene
from warehouse_sim.planning_scene_manager import (
    DEFAULT_GRIPPER_TOUCH_LINKS,
    PlanningSceneManager,
)


ARM_JOINTS: List[str] = [
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
    "joint7",
]

# Joint states from bcr_arm.srdf
UPRIGHT_JOINTS: List[float] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
READY_JOINTS: List[float] = [0.0347, 0.2729, -1.3531, 1.1348, 2.5328, -1.0486, -1.145]

# Canonical outward-wrist front-side seed for yaw=0 (approach along +X_world)
CANONICAL_FRONT_SEED: List[float] = [
    -2.3233,
    -0.7574,
    -1.3701,
    1.8900,
    2.4956,
    1.6287,
    0.3974,
]

# URDF joint limits (with slight margin on +-2.0 joints)
JOINT_BOUNDS: List[Tuple[float, float]] = [
    (-6.20, 6.20),
    (-1.97, 1.97),
    (-6.20, 6.20),
    (-1.97, 1.97),
    (-6.20, 6.20),
    (-1.97, 1.97),
    (-6.20, 6.20),
]

DEFAULT_PITCH_DOWN_RAD = math.radians(18.0)
GRIPPER_TCP_OFFSET_Z = 0.1246


def _rot_x(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]], dtype=float)


def _rot_y(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=float)


def _rot_z(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=float)


def _rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    return _rot_z(yaw) @ _rot_y(pitch) @ _rot_x(roll)


def rot_to_quat(rot: np.ndarray) -> Tuple[float, float, float, float]:
    """Convert a 3x3 rotation matrix into a normalized (x, y, z, w) quaternion."""
    tr = rot[0, 0] + rot[1, 1] + rot[2, 2]
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (rot[2, 1] - rot[1, 2]) / s
        y = (rot[0, 2] - rot[2, 0]) / s
        z = (rot[1, 0] - rot[0, 1]) / s
    elif rot[0, 0] > rot[1, 1] and rot[0, 0] > rot[2, 2]:
        s = math.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2]) * 2.0
        w = (rot[2, 1] - rot[1, 2]) / s
        x = 0.25 * s
        y = (rot[0, 1] + rot[1, 0]) / s
        z = (rot[0, 2] + rot[2, 0]) / s
    elif rot[1, 1] > rot[2, 2]:
        s = math.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2]) * 2.0
        w = (rot[0, 2] - rot[2, 0]) / s
        x = (rot[0, 1] + rot[1, 0]) / s
        y = 0.25 * s
        z = (rot[1, 2] + rot[2, 1]) / s
    else:
        s = math.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1]) * 2.0
        w = (rot[1, 0] - rot[0, 1]) / s
        x = (rot[0, 2] + rot[2, 0]) / s
        y = (rot[1, 2] + rot[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w], dtype=float)
    q /= np.linalg.norm(q)
    return float(q[0]), float(q[1]), float(q[2]), float(q[3])


def wrap_angle(angle: float) -> float:
    """Wrap angle to (-pi, pi]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def make_front_grasp_orientation(
    yaw: float = 0.0,
    pitch_down_rad: float = DEFAULT_PITCH_DOWN_RAD,
) -> np.ndarray:
    """
    Build the Convention A `grasp_tcp` rotation matrix for a front-side grasp.

    - +Z_tcp: approach axis along horizontal yaw (pitched slightly downward by pitch_down_rad)
    - +X_tcp: horizontal jaw opening axis (perpendicular to approach in XY plane)
    - +Y_tcp: upward normal axis (Z_tcp x X_tcp)
    """
    ca, sa = math.cos(pitch_down_rad), math.sin(pitch_down_rad)
    cy, sy = math.cos(yaw), math.sin(yaw)
    z_tcp = np.array([cy * ca, sy * ca, -sa], dtype=float)
    x_tcp = np.array([-sy, cy, 0.0], dtype=float)
    y_tcp = np.cross(z_tcp, x_tcp)
    return np.column_stack((x_tcp, y_tcp, z_tcp))


class BcrArmKinematics:
    """Exact FK and continuous-branch numerical IK for bcr_arm + Robotiq 2F-85 TCP."""

    def __init__(self) -> None:
        self._r_mount = _rpy_matrix(1.5707963, 0.0, 1.5707963)
        self._t_mount = np.eye(4, dtype=float)
        self._t_mount[:3, :3] = self._r_mount

        self._t_tcp_offset = np.eye(4, dtype=float)
        self._t_tcp_offset[2, 3] = GRIPPER_TCP_OFFSET_Z

        self._joint_specs = [
            (np.array([0.0, 0.0, 0.025]), "z"),
            (np.array([0.0, 0.0, 0.200]), "x"),
            (np.array([0.065, 0.0, 0.0]), "z"),
            (np.array([0.0, 0.0, 0.410]), "x"),
            (np.array([-0.065, 0.0, 0.0]), "z"),
            (np.array([0.0, 0.0, 0.310]), "x"),
            (np.array([0.060, 0.0, 0.0]), "z"),
        ]

    def fk(
        self, q: Sequence[float]
    ) -> Tuple[List[np.ndarray], np.ndarray, np.ndarray, np.ndarray]:
        """
        Compute forward kinematics for 7 joint angles.

        :return: (link_transforms, T_flange, T_gripper_base, T_grasp_tcp)
        """
        t_cur = np.eye(4, dtype=float)
        links: List[np.ndarray] = [t_cur.copy()]
        for (offset, axis), angle in zip(self._joint_specs, q):
            t_step = np.eye(4, dtype=float)
            t_step[:3, 3] = offset
            t_cur = t_cur @ t_step
            r_joint = np.eye(4, dtype=float)
            r_joint[:3, :3] = _rot_z(float(angle)) if axis == "z" else _rot_x(float(angle))
            t_cur = t_cur @ r_joint
            links.append(t_cur.copy())

        t_flange = t_cur.copy()
        t_base = t_flange @ self._t_mount
        t_tcp = t_base @ self._t_tcp_offset
        return links, t_flange, t_base, t_tcp

    def tcp_to_flange_pose(
        self,
        tcp_pos: Sequence[float],
        rot_tcp: np.ndarray,
    ) -> Pose:
        """Convert a desired `grasp_tcp` pose into the equivalent `flange` Pose."""
        p_tcp = np.asarray(tcp_pos, dtype=float)
        p_flange = p_tcp - GRIPPER_TCP_OFFSET_Z * rot_tcp[:, 2]
        r_flange = rot_tcp @ self._r_mount.T
        qx, qy, qz, qw = rot_to_quat(r_flange)

        pose = Pose()
        pose.position = Point(
            x=float(p_flange[0]),
            y=float(p_flange[1]),
            z=float(p_flange[2]),
        )
        pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        return pose

    def canonical_seed_for_yaw(self, yaw: float) -> np.ndarray:
        """Rotate the canonical elbow-up seed onto the specified approach yaw."""
        q_seed = np.array(CANONICAL_FRONT_SEED, dtype=float)
        q_seed[0] = wrap_angle(q_seed[0] + yaw)
        return q_seed

    def solve_ik(
        self,
        target_tcp_pos: Sequence[float],
        target_rot_tcp: np.ndarray,
        q_seed: Optional[Sequence[float]] = None,
        yaw: Optional[float] = None,
        reg_weight: float = 0.02,
    ) -> Tuple[np.ndarray, float, float]:
        """
        Solve 7-DOF IK for `grasp_tcp` pose, staying close to `q_seed`.

        :return: (q_solution, position_error_m, rotation_frobenius_error)
        """
        target_pos = np.asarray(target_tcp_pos, dtype=float)
        if yaw is None:
            yaw = math.atan2(target_pos[1], target_pos[0]) if (
                abs(target_pos[0]) > 1e-4 or abs(target_pos[1]) > 1e-4
            ) else 0.0

        seeds: List[np.ndarray] = []
        if q_seed is not None:
            seeds.append(np.asarray(q_seed, dtype=float))
        canonical = self.canonical_seed_for_yaw(yaw)
        seeds.append(canonical)

        best_q = seeds[0].copy()
        best_score = float("inf")
        best_pe = float("inf")
        best_re = float("inf")

        for idx, seed in enumerate(seeds):
            # Keep search near seed for continuity on continuous Cartesian segments
            local_bounds = []
            for j_idx, (low, high) in enumerate(JOINT_BOUNDS):
                if idx == 0 and q_seed is not None:
                    local_bounds.append((max(low, seed[j_idx] - 1.5), min(high, seed[j_idx] + 1.5)))
                else:
                    local_bounds.append((low, high))

            def _loss(q_vec: np.ndarray, s_ref: np.ndarray = seed) -> float:
                _, _, _, t_tcp = self.fk(q_vec)
                pos_err = float(np.sum((t_tcp[:3, 3] - target_pos) ** 2))
                rot_err = float(np.sum((t_tcp[:3, :3] - target_rot_tcp) ** 2))
                lim_pen = float(
                    ((q_vec[1] / 1.92) ** 8 + (q_vec[3] / 1.92) ** 8 + (q_vec[5] / 1.92) ** 8)
                    * 0.005
                )
                reg = float(reg_weight * np.sum((q_vec - s_ref) ** 2))
                return 2000.0 * pos_err + 200.0 * rot_err + reg + lim_pen

            res = minimize(
                _loss,
                seed,
                bounds=local_bounds,
                method="L-BFGS-B",
                options={"ftol": 1e-14, "gtol": 1e-10, "maxiter": 800},
            )
            _, _, _, t_sol = self.fk(res.x)
            pe = float(np.linalg.norm(t_sol[:3, 3] - target_pos))
            re = float(np.linalg.norm(t_sol[:3, :3] - target_rot_tcp))
            score = pe + 0.1 * re
            if score < best_score:
                best_score = score
                best_q = res.x.copy()
                best_pe = pe
                best_re = re
            if pe < 1e-3 and re < 1e-3:
                break

        return best_q, best_pe, best_re

    def interpolate_cartesian_path(
        self,
        start_q: Sequence[float],
        start_tcp_pos: Sequence[float],
        end_tcp_pos: Sequence[float],
        target_rot_tcp: np.ndarray,
        step_size: float = 0.008,
    ) -> Tuple[List[np.ndarray], float]:
        """
        Generate a dense joint-space trajectory tracking a straight Cartesian segment.

        :return: (list of 7-DOF joint waypoints, segment Euclidean length in meters)
        """
        p0 = np.asarray(start_tcp_pos, dtype=float)
        p1 = np.asarray(end_tcp_pos, dtype=float)
        dist = float(np.linalg.norm(p1 - p0))
        num_steps = max(5, int(math.ceil(dist / step_size)))

        q_cur = np.asarray(start_q, dtype=float)
        waypoints: List[np.ndarray] = [q_cur.copy()]
        for i in range(1, num_steps + 1):
            alpha = i / float(num_steps)
            p_i = (1.0 - alpha) * p0 + alpha * p1
            q_next, pe, re = self.solve_ik(
                p_i,
                target_rot_tcp,
                q_seed=q_cur,
                reg_weight=0.02,
            )
            if pe > 0.005 or re > 0.05:
                raise RuntimeError(
                    f"Cartesian IK diverged at step {i}/{num_steps}: "
                    f"pos_err={pe:.4f} m, rot_err={re:.4f}."
                )
            q_cur = q_next
            waypoints.append(q_cur.copy())

        return waypoints, dist


class MoveItTrajectoryHelper:
    """Interfaces with MoveIt 2 planning services and JointTrajectoryController."""

    def __init__(
        self,
        node: Node,
        kinematics: BcrArmKinematics,
        group_name: str = "bcr_arm",
        tip_link: str = "flange",
        planning_frame: str = "world",
        spin_locally: bool = True,
    ) -> None:
        self.node = node
        self.logger = node.get_logger()
        self.kinematics = kinematics
        self.group_name = group_name
        self.tip_link = tip_link
        self.planning_frame = planning_frame
        self.spin_locally = spin_locally

        self._lock = threading.Lock()
        self._latest_joint_state: Optional[JointState] = None
        self._current_arm_q: Optional[List[float]] = None

        self._js_sub = self.node.create_subscription(
            JointState,
            "/joint_states",
            self._joint_state_callback,
            20,
        )
        self._arm_traj_pub = self.node.create_publisher(
            JointTrajectory,
            "/joint_trajectory_controller/joint_trajectory",
            10,
        )
        self._arm_action_client = ActionClient(
            self.node,
            FollowJointTrajectory,
            "/joint_trajectory_controller/follow_joint_trajectory",
        )
        self._validity_client = self.node.create_client(
            GetStateValidity, "/check_state_validity"
        )
        self._plan_client = self.node.create_client(
            GetMotionPlan, "/plan_kinematic_path"
        )
        self._cartesian_client = self.node.create_client(
            GetCartesianPath, "/compute_cartesian_path"
        )

    def _step_wait(self, dt_sec: float = 0.02) -> None:
        if self.spin_locally:
            rclpy.spin_once(self.node, timeout_sec=dt_sec)
        else:
            time.sleep(dt_sec)

    def _sim_now_sec(self) -> float:
        return self.node.get_clock().now().nanoseconds / 1e9

    def _joint_state_callback(self, msg: JointState) -> None:
        name_to_idx = {name: i for i, name in enumerate(msg.name)}
        if not all(j in name_to_idx for j in ARM_JOINTS):
            return
        arm_q = [float(msg.position[name_to_idx[j]]) for j in ARM_JOINTS]
        with self._lock:
            self._latest_joint_state = msg
            self._current_arm_q = arm_q

    def wait_for_ready(self, timeout_sec: float = 10.0) -> bool:
        """Wait for /joint_states and core MoveIt / controller endpoints."""
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            with self._lock:
                if self._current_arm_q is not None:
                    break
            self._step_wait(0.02)

        with self._lock:
            if self._current_arm_q is None:
                self.logger.error("Timed out waiting for arm /joint_states.")
                return False

        self._validity_client.wait_for_service(timeout_sec=3.0)
        self._plan_client.wait_for_service(timeout_sec=3.0)
        self._cartesian_client.wait_for_service(timeout_sec=3.0)
        self._arm_action_client.wait_for_server(timeout_sec=3.0)
        return True

    def get_current_arm_joints(self) -> Optional[List[float]]:
        """Return latest 7 arm joint angles in radians."""
        with self._lock:
            return list(self._current_arm_q) if self._current_arm_q is not None else None

    def _build_robot_state(self, q_override: Optional[Sequence[float]] = None) -> RobotState:
        rs = RobotState()
        rs.is_diff = True
        js = JointState()
        with self._lock:
            latest = self._latest_joint_state
        if latest is not None:
            js.name = list(latest.name)
            js.position = list(latest.position)
        else:
            js.name = list(ARM_JOINTS)
            js.position = list(UPRIGHT_JOINTS)

        if q_override is not None:
            name_to_idx = {name: i for i, name in enumerate(js.name)}
            for j_name, val in zip(ARM_JOINTS, q_override):
                if j_name in name_to_idx:
                    js.position[name_to_idx[j_name]] = float(val)
                else:
                    js.name.append(j_name)
                    js.position.append(float(val))
        rs.joint_state = js
        return rs

    def _call_service_sync(self, client, request, timeout_sec: float = 8.0):
        if not client.service_is_ready():
            if not client.wait_for_service(timeout_sec=2.0):
                return None
        future = client.call_async(request)
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            self._step_wait(0.02)
        if not future.done():
            return None
        return future.result()

    def check_state_validity(self, q: Sequence[float]) -> Tuple[bool, List[str]]:
        """Check whether joint vector `q` is collision-free in the active MoveIt planning scene."""
        if not self._validity_client.service_is_ready():
            return True, []

        req = GetStateValidity.Request()
        req.robot_state = self._build_robot_state(q_override=q)
        req.group_name = self.group_name
        res = self._call_service_sync(self._validity_client, req, timeout_sec=3.0)
        if res is None:
            self.logger.warn("/check_state_validity timed out; assuming state valid.")
            return True, []

        if res.valid:
            return True, []

        contacts = [
            f"{c.contact_body_1}<->{c.contact_body_2} (depth={c.depth:.4f})"
            for c in res.contacts
        ]
        return False, contacts

    def _time_parameterize_waypoints(
        self,
        waypoints: Sequence[Sequence[float]],
        total_duration_sec: float,
    ) -> JointTrajectory:
        """Convert dense joint waypoints into a smooth cosine-eased JointTrajectory."""
        traj = JointTrajectory()
        traj.joint_names = list(ARM_JOINTS)
        n = len(waypoints)
        if n == 0:
            return traj
        if n == 1:
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in waypoints[0]]
            pt.velocities = [0.0] * len(ARM_JOINTS)
            pt.time_from_start = Duration(sec=0, nanosec=100_000_000)
            traj.points.append(pt)
            return traj

        q_arr = np.asarray(waypoints, dtype=float)
        # Stretch start and end intervals for gentle acceleration/deceleration
        t_arr = np.zeros(n, dtype=float)
        seg_dists = np.linalg.norm(np.diff(q_arr, axis=0), axis=1)
        total_q_dist = float(np.sum(seg_dists))
        if total_q_dist < 1e-6:
            t_arr = np.linspace(0.0, total_duration_sec, n)
        else:
            cum = np.concatenate(([0.0], np.cumsum(seg_dists))) / total_q_dist
            # Blend linear arc-length time with smoothstep for smooth ramp in/out
            s_smooth = 0.5 * cum + 0.5 * (3.0 * cum ** 2 - 2.0 * cum ** 3)
            t_arr = s_smooth * total_duration_sec

        # Enforce strictly increasing timestamps
        for i in range(1, n):
            if t_arr[i] <= t_arr[i - 1] + 0.005:
                t_arr[i] = t_arr[i - 1] + 0.005

        for i in range(n):
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in q_arr[i]]
            if i == 0 or i == n - 1:
                pt.velocities = [0.0] * len(ARM_JOINTS)
            else:
                dt = t_arr[i + 1] - t_arr[i - 1]
                vel = (q_arr[i + 1] - q_arr[i - 1]) / max(dt, 1e-3)
                pt.velocities = [float(v) for v in vel]

            t_val = float(t_arr[i])
            sec = int(t_val)
            nsec = int((t_val - sec) * 1e9)
            pt.time_from_start = Duration(sec=sec, nanosec=nsec)
            traj.points.append(pt)

        return traj

    def plan_free_space_to_joints(
        self,
        q_goal: Sequence[float],
        vel_scale: float = 0.35,
        acc_scale: float = 0.35,
        planning_time: float = 5.0,
    ) -> Optional[JointTrajectory]:
        """
        Plan a collision-free trajectory to `q_goal` using MoveIt `/plan_kinematic_path`.
        Falls back to collision-checked smooth joint interpolation if OMPL fails.
        """
        q_start = self.get_current_arm_joints()
        if q_start is None:
            return None

        if self._plan_client.service_is_ready():
            req = GetMotionPlan.Request()
            mpr = req.motion_plan_request
            mpr.group_name = self.group_name
            mpr.num_planning_attempts = 5
            mpr.allowed_planning_time = float(planning_time)
            mpr.max_velocity_scaling_factor = float(vel_scale)
            mpr.max_acceleration_scaling_factor = float(acc_scale)
            mpr.start_state = self._build_robot_state(q_override=q_start)

            goal_constraint = Constraints()
            for j_name, val in zip(ARM_JOINTS, q_goal):
                jc = JointConstraint()
                jc.joint_name = j_name
                jc.position = float(val)
                jc.tolerance_above = 0.005
                jc.tolerance_below = 0.005
                jc.weight = 1.0
                goal_constraint.joint_constraints.append(jc)
            mpr.goal_constraints = [goal_constraint]

            res = self._call_service_sync(self._plan_client, req, timeout_sec=planning_time + 3.0)
            if res is not None:
                mpr_resp = res.motion_plan_response
                if (
                    mpr_resp.error_code.val == MoveItErrorCodes.SUCCESS
                    and mpr_resp.trajectory.joint_trajectory.points
                ):
                    self.logger.info(
                        f"OMPL planned collision-free trajectory with "
                        f"{len(mpr_resp.trajectory.joint_trajectory.points)} points."
                    )
                    return mpr_resp.trajectory.joint_trajectory
                self.logger.warn(
                    f"OMPL /plan_kinematic_path returned code {mpr_resp.error_code.val} "
                    f"({mpr_resp.error_code.message}); checking fallback interpolation."
                )

        # Fallback: smooth joint-space interpolation verified by /check_state_validity
        max_delta = max(abs(g - s) for g, s in zip(q_goal, q_start))
        n_steps = max(12, int(math.ceil(max_delta / 0.08)))
        q_s = np.asarray(q_start, dtype=float)
        q_g = np.asarray(q_goal, dtype=float)
        waypoints = [(1.0 - i / float(n_steps)) * q_s + (i / float(n_steps)) * q_g for i in range(n_steps + 1)]

        for step_idx in range(1, len(waypoints), max(1, len(waypoints) // 10)):
            valid, contacts = self.check_state_validity(waypoints[step_idx])
            if not valid:
                self.logger.error(
                    f"Fallback free-space trajectory collided at step {step_idx}: {contacts}"
                )
                return None

        duration = max(1.5, max_delta / max(0.3, 1.2 * vel_scale))
        self.logger.info(
            f"Generated validated fallback joint-space trajectory ({len(waypoints)} pts, {duration:.2f}s)."
        )
        return self._time_parameterize_waypoints(waypoints, total_duration_sec=duration)

    def plan_cartesian_tcp(
        self,
        start_tcp_pos: Sequence[float],
        end_tcp_pos: Sequence[float],
        rot_tcp: np.ndarray,
        max_cartesian_speed: float = 0.045,
        avoid_collisions: bool = True,
        min_duration_sec: float = 1.5,
    ) -> Optional[JointTrajectory]:
        """
        Plan a straight-line Cartesian trajectory for `grasp_tcp` at <= `max_cartesian_speed`.

        Queries MoveIt `/compute_cartesian_path` first; if KDL achieves < 99% fraction,
        computes a continuous-branch IK path validated step-by-step via `/check_state_validity`.
        """
        q_start = self.get_current_arm_joints()
        if q_start is None:
            return None

        p0 = np.asarray(start_tcp_pos, dtype=float)
        p1 = np.asarray(end_tcp_pos, dtype=float)
        dist = float(np.linalg.norm(p1 - p0))
        required_duration = max(min_duration_sec, dist / max(0.01, max_cartesian_speed) * 1.30)

        if self._cartesian_client.service_is_ready():
            req = GetCartesianPath.Request()
            req.header.frame_id = self.planning_frame
            req.start_state = self._build_robot_state(q_override=q_start)
            req.group_name = self.group_name
            req.link_name = self.tip_link

            n_wp = max(4, int(math.ceil(dist / 0.02)))
            waypoints_pose = []
            for i in range(1, n_wp + 1):
                p_i = p0 + (i / float(n_wp)) * (p1 - p0)
                waypoints_pose.append(self.kinematics.tcp_to_flange_pose(p_i, rot_tcp))
            req.waypoints = waypoints_pose
            req.max_step = 0.01
            req.jump_threshold = 0.0
            req.revolute_jump_threshold = 0.5
            req.avoid_collisions = avoid_collisions
            req.max_velocity_scaling_factor = 0.2
            req.max_acceleration_scaling_factor = 0.2
            req.cartesian_speed_limited_link = self.tip_link
            req.max_cartesian_speed = float(max_cartesian_speed)

            res = self._call_service_sync(self._cartesian_client, req, timeout_sec=6.0)
            if (
                res is not None
                and res.error_code.val == MoveItErrorCodes.SUCCESS
                and res.fraction >= 0.99
                and len(res.solution.joint_trajectory.points) >= 2
            ):
                pts = [list(pt.positions) for pt in res.solution.joint_trajectory.points]
                last_t = (
                    res.solution.joint_trajectory.points[-1].time_from_start.sec
                    + res.solution.joint_trajectory.points[-1].time_from_start.nanosec * 1e-9
                )
                self.logger.info(
                    f"MoveIt /compute_cartesian_path succeeded (fraction={res.fraction:.3f}, "
                    f"{len(pts)} pts, duration={last_t:.2f}s)."
                )
                if last_t < required_duration:
                    return self._time_parameterize_waypoints(pts, required_duration)
                return res.solution.joint_trajectory

            frac = res.fraction if res is not None else 0.0
            code = res.error_code.val if res is not None else -1
            self.logger.warn(
                f"MoveIt /compute_cartesian_path incomplete (fraction={frac:.3f}, code={code}); "
                "using continuous-branch IK + MoveIt state-validity check."
            )

        # Continuous-branch IK Cartesian path + MoveIt collision validation
        q_waypoints, _ = self.kinematics.interpolate_cartesian_path(
            start_q=q_start,
            start_tcp_pos=p0,
            end_tcp_pos=p1,
            target_rot_tcp=rot_tcp,
            step_size=0.008,
        )

        if avoid_collisions:
            for idx in range(len(q_waypoints)):
                valid, contacts = self.check_state_validity(q_waypoints[idx])
                if not valid:
                    self.logger.error(
                        f"Cartesian path collision detected at waypoint {idx}/{len(q_waypoints)-1}: "
                        f"{contacts}"
                    )
                    return None

        self.logger.info(
            f"Generated validated Cartesian trajectory ({len(q_waypoints)} pts, "
            f"dist={dist:.3f}m, duration={required_duration:.2f}s, "
            f"avg_v={dist/required_duration:.3f}m/s)."
        )
        return self._time_parameterize_waypoints(q_waypoints, required_duration)

    def execute_trajectory(
        self,
        traj: JointTrajectory,
        goal_tolerance: float = 0.04,
    ) -> bool:
        """Execute a JointTrajectory via FollowJointTrajectory action (with topic fallback)."""
        if not traj.points:
            self.logger.warn("Empty trajectory passed to execute_trajectory.")
            return False

        target_q = np.asarray(traj.points[-1].positions, dtype=float)
        last_t = (
            traj.points[-1].time_from_start.sec
            + traj.points[-1].time_from_start.nanosec * 1e-9
        )

        # Ensure header.stamp is zero so controller starts immediately
        traj.header.stamp.sec = 0
        traj.header.stamp.nanosec = 0

        if self._arm_action_client.server_is_ready():
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = traj

            send_future = self._arm_action_client.send_goal_async(goal)
            deadline = time.monotonic() + 5.0
            while rclpy.ok() and not send_future.done() and time.monotonic() < deadline:
                self._step_wait(0.02)

            if send_future.done():
                goal_handle = send_future.result()
                if goal_handle is not None and goal_handle.accepted:
                    res_future = goal_handle.get_result_async()
                    res_deadline = time.monotonic() + max(10.0, last_t + 8.0)
                    while rclpy.ok() and not res_future.done() and time.monotonic() < res_deadline:
                        self._step_wait(0.02)
                    if res_future.done():
                        wrapped = res_future.result()
                        err_code = wrapped.result.error_code
                        if err_code == FollowJointTrajectory.Result.SUCCESSFUL:
                            return self._wait_joint_convergence(target_q, goal_tolerance, 3.0)
                        self.logger.warn(
                            f"FollowJointTrajectory finished with error_code={err_code} "
                            f"({wrapped.result.error_string}); checking convergence."
                        )
                        return self._wait_joint_convergence(target_q, goal_tolerance + 0.02, 2.0)
                else:
                    self.logger.warn("FollowJointTrajectory goal rejected; falling back to topic.")

        # Topic fallback
        self._arm_traj_pub.publish(traj)
        sim_start = self._sim_now_sec()
        wall_deadline = time.monotonic() + last_t + 5.0
        while rclpy.ok() and time.monotonic() < wall_deadline:
            self._step_wait(0.02)
            if sim_start > 0.0 and (self._sim_now_sec() - sim_start) >= last_t:
                break
        return self._wait_joint_convergence(target_q, goal_tolerance, 3.0)

    def _wait_joint_convergence(
        self,
        target_q: np.ndarray,
        tolerance: float,
        timeout_sec: float,
    ) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            cur = self.get_current_arm_joints()
            if cur is not None:
                err = float(np.max(np.abs(np.asarray(cur, dtype=float) - target_q)))
                if err <= tolerance:
                    return True
            self._step_wait(0.02)
        cur = self.get_current_arm_joints()
        max_err = (
            float(np.max(np.abs(np.asarray(cur, dtype=float) - target_q)))
            if cur is not None
            else float("inf")
        )
        self.logger.warn(f"Joint convergence ended with max_err={max_err:.4f} rad.")
        return max_err <= (tolerance + 0.03)


class PickAndPlacePipeline:
    """Autonomous collision-aware front-side pick-and-place pipeline."""

    def __init__(self, node: Node, spin_locally: bool = True) -> None:
        self.node = node
        self.logger = node.get_logger()
        self.spin_locally = spin_locally

        self.scene_manager = PlanningSceneManager(node, spin_locally=spin_locally)
        self.gripper = GripperInterface(node, spin_locally=spin_locally)
        self.kinematics = BcrArmKinematics()
        self.motion = MoveItTrajectoryHelper(
            node,
            self.kinematics,
            spin_locally=spin_locally,
        )
        self._held_objects: Dict[str, Tuple[float, float, float]] = {}

    def initialize(self, ensure_test_scene: bool = False) -> bool:
        """Wait for MoveIt, controllers, and joint states; optionally load test shelf scene."""
        if not self.scene_manager.wait_for_service(timeout_sec=10.0):
            return False
        if not self.gripper.wait_for_joint_state(timeout_sec=10.0):
            return False
        if not self.motion.wait_for_ready(timeout_sec=10.0):
            return False

        if ensure_test_scene:
            scene = self.scene_manager.get_planning_scene(
                components=PlanningSceneComponents.WORLD_OBJECT_NAMES
            )
            existing = (
                {obj.id for obj in scene.world.collision_objects}
                if scene is not None
                else set()
            )
            needed = {"ground_plane", "shelf_table", "pickable_cube", "place_surface"}
            if not needed.issubset(existing):
                self.logger.info("Populating test shelf planning scene...")
                if not load_test_shelf_scene(self.scene_manager, configure_acm=True, verify=True):
                    return False
        return True

    def _load_scene_graph_lookup(self) -> Dict[str, Any]:
        """Load TEMPO warehouse_scene_graph.json if available in workspace."""
        candidates = []
        try:
            from ament_index_python.packages import get_package_share_directory

            candidates.append(
                os.path.join(
                    get_package_share_directory("llm_planner"),
                    "knowledge_base",
                    "warehouse_scene_graph.json",
                )
            )
        except Exception:
            pass
        candidates.append(
            "/root/warehouse_ws/src/llm_planner/knowledge_base/warehouse_scene_graph.json"
        )
        candidates.append(
            "/home/zain/warehouse_ws/src/llm_planner/knowledge_base/warehouse_scene_graph.json"
        )

        for path in candidates:
            if os.path.isfile(path):
                with open(path, "r", encoding="utf-8") as f:
                    return json.load(f)
        return {}

    def resolve_entity_geometry(
        self, entity_id: str
    ) -> Optional[Dict[str, Any]]:
        """
        Resolve an entity ID into {center: (x, y, z), dimensions: (dx, dy, dz), top_z: float}
        from the live MoveIt planning scene or TEMPO warehouse_scene_graph.json.
        """
        scene = self.scene_manager.get_planning_scene(
            components=(
                PlanningSceneComponents.WORLD_OBJECT_NAMES
                | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
                | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
            )
        )
        if scene is not None:
            for obj in scene.world.collision_objects:
                if obj.id == entity_id and obj.primitives and obj.primitive_poses:
                    prim = obj.primitives[0]
                    obj_p = obj.pose.position
                    prim_p = obj.primitive_poses[0].position
                    q = obj.pose.orientation
                    # Rotate primitive local offset by obj.pose quaternion
                    qx, qy, qz, qw = q.x, q.y, q.z, q.w
                    r_obj = np.array(
                        [
                            [
                                1.0 - 2.0 * (qy * qy + qz * qz),
                                2.0 * (qx * qy - qz * qw),
                                2.0 * (qx * qz + qy * qw),
                            ],
                            [
                                2.0 * (qx * qy + qz * qw),
                                1.0 - 2.0 * (qx * qx + qz * qz),
                                2.0 * (qy * qz - qx * qw),
                            ],
                            [
                                2.0 * (qx * qz - qy * qw),
                                2.0 * (qy * qz + qx * qw),
                                1.0 - 2.0 * (qx * qx + qy * qy),
                            ],
                        ],
                        dtype=float,
                    )
                    p_world = np.array([obj_p.x, obj_p.y, obj_p.z], dtype=float) + (
                        r_obj @ np.array([prim_p.x, prim_p.y, prim_p.z], dtype=float)
                    )
                    dims = list(prim.dimensions)
                    if len(dims) == 2:  # cylinder [height, radius]
                        dims_xyz = (dims[1] * 2.0, dims[1] * 2.0, dims[0])
                    else:
                        dims_xyz = (float(dims[0]), float(dims[1]), float(dims[2]))
                    center = (float(p_world[0]), float(p_world[1]), float(p_world[2]))
                    return {
                        "id": entity_id,
                        "center": center,
                        "dimensions": dims_xyz,
                        "top_z": center[2] + dims_xyz[2] * 0.5,
                    }

        # Fallback: search TEMPO scene graph if present
        sg = self._load_scene_graph_lookup()
        zones = sg.get("warehouse", {}).get("zones", {})
        for zdata in zones.values():
            for free_obj in zdata.get("free_objects", []):
                if free_obj.get("id") == entity_id:
                    p = free_obj["pose"]
                    dims = (0.05, 0.05, 0.05)
                    return {
                        "id": entity_id,
                        "center": (float(p["x"]), float(p["y"]), float(p["z"])),
                        "dimensions": dims,
                        "top_z": float(p["z"]) + dims[2] * 0.5,
                    }
            fixture_dicts = [zdata.get("fixtures", {})]
            for adata in zdata.get("aisles", {}).values():
                fixture_dicts.append(adata.get("fixtures", {}))
            for fdict in fixture_dicts:
                if entity_id in fdict:
                    fpose = fdict[entity_id]["pose"]
                    dims = (0.30, 0.30, 0.40)
                    return {
                        "id": entity_id,
                        "center": (float(fpose["x"]), float(fpose["y"]), float(fpose["z"]) + 0.20),
                        "dimensions": dims,
                        "top_z": float(fpose["z"]) + 0.40,
                    }
                for fentry in fdict.values():
                    for child in fentry.get("children", []):
                        if child.get("id") == entity_id:
                            p = child["pose"]
                            dims = (0.05, 0.05, 0.05)
                            return {
                                "id": entity_id,
                                "center": (float(p["x"]), float(p["y"]), float(p["z"])),
                                "dimensions": dims,
                                "top_z": float(p["z"]) + dims[2] * 0.5,
                            }
        return None

    def move_to_ready(self) -> bool:
        """Move arm safely to the SRDF `ready` configuration if not already near it."""
        cur = self.motion.get_current_arm_joints()
        if cur is not None:
            diff = max(abs(a - b) for a, b in zip(cur, READY_JOINTS))
            if diff <= 0.08:
                return True

        self.logger.info("Moving arm to SRDF 'ready' configuration...")
        traj = self.motion.plan_free_space_to_joints(READY_JOINTS, vel_scale=0.35, acc_scale=0.35)
        if traj is None:
            return False
        return self.motion.execute_trajectory(traj)

    def pick_object(
        self,
        object_id: str = "pickable_cube",
        object_center: Tuple[float, float, float] = (0.50, 0.00, 0.425),
        object_dimensions: Tuple[float, float, float] = (0.05, 0.05, 0.05),
        approach_yaw: Optional[float] = None,
        standoff_dist: float = 0.08,
        retract_radius: float = 0.40,
        lift_dz: float = 0.043,
        support_surface_id: str = "shelf_table",
        place_surface_id: str = "place_surface",
    ) -> bool:
        """
        Execute Phase 3: Front-side Pick Sequence for `object_id` at `object_center`.
        """
        xc, yc, zc = float(object_center[0]), float(object_center[1]), float(object_center[2])
        yaw = (
            float(approach_yaw)
            if approach_yaw is not None
            else math.atan2(yc, xc) if (abs(xc) > 1e-4 or abs(yc) > 1e-4) else 0.0
        )

        # Pick slightly above nominal cube center (0.432m for 0.425m center) and
        # compensate -10mm along approach for the Robotiq 4-bar closing forward arc.
        z_pick = zc + 0.007
        rot_grasp = make_front_grasp_orientation(yaw=yaw, pitch_down_rad=DEFAULT_PITCH_DOWN_RAD)

        p_standoff = np.array(
            [
                xc - standoff_dist * math.cos(yaw),
                yc - standoff_dist * math.sin(yaw),
                z_pick,
            ],
            dtype=float,
        )
        p_grasp = np.array(
            [
                xc - 0.010 * math.cos(yaw),
                yc - 0.010 * math.sin(yaw),
                z_pick,
            ],
            dtype=float,
        )
        p_lift = np.array([p_grasp[0], p_grasp[1], z_pick + lift_dz], dtype=float)
        p_retract = np.array(
            [
                retract_radius * math.cos(yaw),
                retract_radius * math.sin(yaw),
                z_pick + lift_dz,
            ],
            dtype=float,
        )

        self.logger.info(
            f"=== PHASE 3 PICK '{object_id}': center={[xc, yc, zc]}, yaw={yaw:.3f} rad ==="
        )

        # Sync object box pose in MoveIt planning scene before ACM update and grasp
        self.scene_manager.add_box(
            name=object_id,
            dimensions=object_dimensions,
            position=(xc, yc, zc),
            orientation=(0.0, 0.0, 0.0, 1.0),
            frame_id="world",
        )

        # Ensure ACM allows gripper contact with target object and support surfaces
        extra_links = [s for s in (support_surface_id, place_surface_id) if s]
        self.scene_manager.allow_gripper_collisions(
            object_id, allowed=True, extra_links=extra_links
        )

        # Step 0: Open gripper and verify ready state
        if not self.gripper.open(duration_sec=1.0, wait=True):
            self.logger.error("Failed to open gripper prior to pick.")
            return False

        if not self.move_to_ready():
            self.logger.error("Failed to move to ready pose prior to pick.")
            return False

        # Subtask 3.1: Pre-Grasp Stand-off Pose (seed from grasp IK for branch continuity)
        self.logger.info(f"[Subtask 3.1] Planning to standoff {p_standoff.round(4).tolist()}...")
        q_grasp, g_pe, g_re = self.kinematics.solve_ik(p_grasp, rot_grasp, yaw=yaw)
        if g_pe > 0.005 or g_re > 0.05:
            self.logger.error(f"Grasp seed IK failed: pos_err={g_pe:.4f}, rot_err={g_re:.4f}")
            return False
        q_standoff, pe, re = self.kinematics.solve_ik(
            p_standoff, rot_grasp, q_seed=q_grasp, yaw=yaw
        )
        if pe > 0.005 or re > 0.05:
            self.logger.error(f"Stand-off IK failed: pos_err={pe:.4f}, rot_err={re:.4f}")
            return False

        valid, contacts = self.motion.check_state_validity(q_standoff)
        if not valid:
            self.logger.error(f"Stand-off state in collision: {contacts}")
            return False

        traj_standoff = self.motion.plan_free_space_to_joints(
            q_standoff, vel_scale=0.35, acc_scale=0.35
        )
        if traj_standoff is None or not self.motion.execute_trajectory(traj_standoff):
            self.logger.error("Failed to execute trajectory to standoff pose.")
            return False

        # Subtask 3.2: Straight Cartesian Approach (+Z_tcp)
        self.logger.info(f"[Subtask 3.2] Cartesian approach to {p_grasp.round(4).tolist()}...")
        traj_approach = self.motion.plan_cartesian_tcp(
            p_standoff, p_grasp, rot_grasp, max_cartesian_speed=0.045, avoid_collisions=True
        )
        if traj_approach is None or not self.motion.execute_trajectory(traj_approach):
            self.logger.error("Failed to execute straight Cartesian approach.")
            return False

        # Subtask 3.3: Close Gripper, Verify Stall, and Attach Object
        self.logger.info("[Subtask 3.3] Closing gripper and checking contact stall...")
        if not self.gripper.close(duration_sec=1.2, wait=True, verify_stall=True):
            self.logger.error(f"Gripper stall grasp verification failed for '{object_id}'.")
            return False

        if not self.scene_manager.attach_object(
            object_id,
            link_name="grasp_tcp",
            touch_links=list(DEFAULT_GRIPPER_TOUCH_LINKS),
        ):
            self.logger.error(f"Failed to attach '{object_id}' in MoveIt planning scene.")
            return False

        self._held_objects[object_id] = tuple(float(d) for d in object_dimensions)

        # Subtask 3.4: Vertical Micro-Lift & Cartesian Retract
        self.logger.info(f"[Subtask 3.4] Micro-lift to {p_lift.round(4).tolist()}...")
        traj_lift = self.motion.plan_cartesian_tcp(
            p_grasp, p_lift, rot_grasp, max_cartesian_speed=0.04, avoid_collisions=True
        )
        if traj_lift is None or not self.motion.execute_trajectory(traj_lift):
            self.logger.error("Failed to execute vertical micro-lift.")
            return False

        self.logger.info(f"[Subtask 3.4] Cartesian retract to {p_retract.round(4).tolist()}...")
        traj_retract = self.motion.plan_cartesian_tcp(
            p_lift, p_retract, rot_grasp, max_cartesian_speed=0.045, avoid_collisions=True
        )
        if traj_retract is None or not self.motion.execute_trajectory(traj_retract):
            self.logger.error("Failed to execute Cartesian retract.")
            return False

        self.logger.info(f"=== PHASE 3 PICK '{object_id}' COMPLETED ===")
        return True

    def place_object(
        self,
        object_id: str = "pickable_cube",
        place_surface_top_center: Tuple[float, float, float] = (0.00, 0.50, 0.40),
        object_dimensions: Optional[Tuple[float, float, float]] = None,
        place_yaw: Optional[float] = None,
        place_surface_id: str = "place_surface",
    ) -> bool:
        """
        Execute Phase 4: Place Sequence for `object_id` onto `place_surface_top_center`.
        """
        dims = object_dimensions or self._held_objects.get(object_id, (0.05, 0.05, 0.05))
        xd, yd, zd = (
            float(place_surface_top_center[0]),
            float(place_surface_top_center[1]),
            float(place_surface_top_center[2]),
        )
        yaw = (
            float(place_yaw)
            if place_yaw is not None
            else math.atan2(yd, xd) if (abs(xd) > 1e-4 or abs(yd) > 1e-4) else 0.0
        )

        rot_place = make_front_grasp_orientation(yaw=yaw, pitch_down_rad=DEFAULT_PITCH_DOWN_RAD)
        # Compensate for the closed Robotiq pad center (~11mm ahead of grasp_tcp along approach)
        place_x = xd - 0.011 * math.cos(yaw)
        place_y = yd - 0.011 * math.sin(yaw)
        p_pre_place = np.array([place_x, place_y, zd + 0.10], dtype=float)
        # Release slightly above exact surface center (dz/2 + 7mm) so cube settles softly
        p_place = np.array([place_x, place_y, zd + dims[2] * 0.5 + 0.007], dtype=float)
        p_post_place = np.array([place_x, place_y, zd + 0.125], dtype=float)

        self.logger.info(
            f"=== PHASE 4 PLACE '{object_id}': target_top={[xd, yd, zd]}, yaw={yaw:.3f} rad ==="
        )

        # Subtask 4.1: Free-Space Transit to Pre-Place Pose
        self.logger.info(f"[Subtask 4.1] Transit to pre-place {p_pre_place.round(4).tolist()}...")
        q_place_seed = self.kinematics.canonical_seed_for_yaw(yaw)
        q_pre_place, pe, re = self.kinematics.solve_ik(
            p_pre_place, rot_place, q_seed=q_place_seed, yaw=yaw
        )
        if pe > 0.005 or re > 0.05:
            self.logger.error(f"Pre-place IK failed: pos_err={pe:.4f}, rot_err={re:.4f}")
            return False

        valid, contacts = self.motion.check_state_validity(q_pre_place)
        if not valid:
            self.logger.error(f"Pre-place state in collision: {contacts}")
            return False

        traj_pre = self.motion.plan_free_space_to_joints(
            q_pre_place, vel_scale=0.30, acc_scale=0.30
        )
        if traj_pre is None or not self.motion.execute_trajectory(traj_pre):
            self.logger.error("Failed to transit to pre-place pose.")
            return False

        # Subtask 4.2: Cartesian Place Descent (v <= 0.05 m/s)
        self.logger.info(f"[Subtask 4.2] Cartesian descent to {p_place.round(4).tolist()}...")
        traj_desc = self.motion.plan_cartesian_tcp(
            p_pre_place, p_place, rot_place, max_cartesian_speed=0.04, avoid_collisions=True
        )
        if traj_desc is None or not self.motion.execute_trajectory(traj_desc):
            self.logger.error("Failed to execute Cartesian place descent.")
            return False

        # Subtask 4.3: Detach Object, Open Gripper, and Update Planning Scene
        self.logger.info("[Subtask 4.3] Detaching object and opening gripper...")
        if not self.scene_manager.detach_object(object_id):
            self.logger.error(f"Failed to detach '{object_id}' in MoveIt planning scene.")
            return False

        if not self.gripper.open(duration_sec=1.0, wait=True):
            self.logger.error("Failed to open gripper for release.")
            return False

        self._held_objects.pop(object_id, None)

        drop_center = (xd, yd, zd + dims[2] * 0.5)
        self.scene_manager.add_box(
            name=object_id,
            dimensions=dims,
            position=drop_center,
            orientation=(0.0, 0.0, 0.0, 1.0),
            frame_id="world",
        )
        if place_surface_id:
            self.scene_manager.allow_gripper_collisions(
                object_id, allowed=True, extra_links=[place_surface_id]
            )

        # Subtask 4.4: Post-Place Retreat & Return Ready
        self.logger.info(
            f"[Subtask 4.4] Post-place lift to {p_post_place.round(4).tolist()}..."
        )
        traj_post = self.motion.plan_cartesian_tcp(
            p_place, p_post_place, rot_place, max_cartesian_speed=0.045, avoid_collisions=True
        )
        if traj_post is None or not self.motion.execute_trajectory(traj_post):
            self.logger.error("Failed to execute post-place lift.")
            return False

        self.logger.info("[Subtask 4.4] Returning arm to SRDF 'ready' pose...")
        if not self.move_to_ready():
            self.logger.error("Failed to return to ready pose after place.")
            return False

        self.logger.info(f"=== PHASE 4 PLACE '{object_id}' COMPLETED ===")
        return True

    def run_pick_and_place(
        self,
        object_id: str = "pickable_cube",
        object_center: Tuple[float, float, float] = (0.50, 0.00, 0.425),
        object_dimensions: Tuple[float, float, float] = (0.05, 0.05, 0.05),
        place_target: Optional[Tuple[float, float, float]] = (0.00, 0.50, 0.40),
        approach_yaw: Optional[float] = None,
        support_surface_id: str = "shelf_table",
        place_surface_id: str = "place_surface",
    ) -> bool:
        """Execute full pick (and optional place) pipeline."""
        pick_ok = self.pick_object(
            object_id=object_id,
            object_center=object_center,
            object_dimensions=object_dimensions,
            approach_yaw=approach_yaw,
            support_surface_id=support_surface_id,
            place_surface_id=place_surface_id,
        )
        if not pick_ok:
            return False

        if place_target is None:
            return True

        return self.place_object(
            object_id=object_id,
            place_surface_top_center=place_target,
            object_dimensions=object_dimensions,
            place_surface_id=place_surface_id,
        )


class TempoArmWorkerNode(Node):
    """
    TEMPO Robot Execution Worker for `bcr_arm`.

    Subscribes to `/mission/subtask`, executes grounded `pick` and `place`
    subtasks via `PickAndPlacePipeline`, and reports terminal feedback to
    `/mission/feedback` for `dispatcher_node`.
    """

    def __init__(self) -> None:
        super().__init__(
            "tempo_arm_worker",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )
        self.declare_parameter("robot_id", "robotic_arm_1")
        self.declare_parameter("ensure_test_scene", True)
        self.robot_id = str(self.get_parameter("robot_id").value)
        ensure_scene = bool(self.get_parameter("ensure_test_scene").value)

        self.pipeline = PickAndPlacePipeline(self, spin_locally=False)
        self._feedback_pub = self.create_publisher(String, "/mission/feedback", 10)
        self._subtask_sub = self.create_subscription(
            String, "/mission/subtask", self._subtask_callback, 10
        )

        self._work_lock = threading.Lock()
        self._ready = False
        self._init_thread = threading.Thread(
            target=self._initialize_async, args=(ensure_scene,), daemon=True
        )
        self._init_thread.start()

    def _initialize_async(self, ensure_scene: bool) -> None:
        self._ready = self.pipeline.initialize(ensure_test_scene=ensure_scene)
        if self._ready:
            self.get_logger().info(
                f"TempoArmWorkerNode '{self.robot_id}' ready on /mission/subtask."
            )
        else:
            self.get_logger().error("TempoArmWorkerNode initialization failed.")

    def _publish_feedback(
        self,
        plan_id: str,
        task_id: str,
        stage: str,
        status: str,
        reason: str = "",
    ) -> None:
        payload: Dict[str, Any] = {
            "plan_id": plan_id,
            "task_id": task_id,
            "robot": self.robot_id,
            "stage": stage,
            "status": status,
        }
        if reason:
            payload["reason"] = reason
        msg = String()
        msg.data = json.dumps(payload)
        self._feedback_pub.publish(msg)

    def _subtask_callback(self, msg: String) -> None:
        try:
            task = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f"Invalid /mission/subtask JSON: {exc}")
            return

        assigned_robot = str(task.get("robot", "")).strip()
        if assigned_robot and assigned_robot not in (self.robot_id, "bcr_arm", "robotic_arm"):
            return

        skills = task.get("required_skills", [])
        if not skills or skills[0] not in ("pick", "place"):
            return

        threading.Thread(target=self._execute_subtask, args=(task,), daemon=True).start()

    def _execute_subtask(self, task: Dict[str, Any]) -> None:
        plan_id = str(task.get("plan_id", ""))
        task_id = str(task.get("task_id", ""))
        skill = task["required_skills"][0]
        args = task.get("args", {})

        with self._work_lock:
            if not self._ready:
                self._publish_feedback(
                    plan_id, task_id, "failed", "failed", "Arm worker not initialized."
                )
                return

            self._publish_feedback(plan_id, task_id, "in_progress", "ok")
            ok = False
            reason = ""

            if skill == "pick":
                obj_id = str(args.get("object", "pickable_cube"))
                info = self.pipeline.resolve_entity_geometry(obj_id)
                if info is None:
                    reason = f"Could not resolve 3D geometry for object '{obj_id}'."
                else:
                    ok = self.pipeline.pick_object(
                        object_id=obj_id,
                        object_center=info["center"],
                        object_dimensions=info["dimensions"],
                    )
                    if not ok:
                        reason = f"Pick sequence failed for '{obj_id}'."

            elif skill == "place":
                obj_id = str(args.get("object", "pickable_cube"))
                loc_id = str(args.get("location", "place_surface"))
                loc_info = self.pipeline.resolve_entity_geometry(loc_id)
                if loc_info is None:
                    reason = f"Could not resolve 3D geometry for location '{loc_id}'."
                else:
                    top_target = (
                        loc_info["center"][0],
                        loc_info["center"][1],
                        loc_info["top_z"],
                    )
                    ok = self.pipeline.place_object(
                        object_id=obj_id,
                        place_surface_top_center=top_target,
                        place_surface_id=loc_id,
                    )
                    if not ok:
                        reason = f"Place sequence failed for '{obj_id}' at '{loc_id}'."

            if ok:
                self._publish_feedback(plan_id, task_id, "complete", "ok")
            else:
                self.get_logger().error(reason or f"Subtask {task_id} failed.")
                self._publish_feedback(plan_id, task_id, "failed", "failed", reason)


def worker_main(args=None) -> None:
    """Entry point for TEMPO arm subtask execution worker."""
    rclpy.init(args=args)
    node = TempoArmWorkerNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None) -> None:
    """Standalone CLI entry point for test-world pick-and-place."""
    parser = argparse.ArgumentParser(
        description="Run autonomous front-side pick-and-place on bcr_arm."
    )
    parser.add_argument("--object-id", default="pickable_cube")
    parser.add_argument(
        "--cube-pos", type=float, nargs=3, default=[0.50, 0.00, 0.425],
        metavar=("X", "Y", "Z"),
    )
    parser.add_argument(
        "--cube-dims", type=float, nargs=3, default=[0.05, 0.05, 0.05],
        metavar=("DX", "DY", "DZ"),
    )
    parser.add_argument(
        "--place-pos", type=float, nargs=3, default=[0.00, 0.50, 0.40],
        metavar=("X", "Y", "Z"),
        help="Target drop surface top center (x, y, z_top).",
    )
    parser.add_argument("--pick-only", action="store_true")
    parser.add_argument("--skip-scene-load", action="store_true")
    parsed, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = Node(
        "pick_and_place_pipeline",
        parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
    )
    pipeline = PickAndPlacePipeline(node, spin_locally=True)

    success = False
    try:
        if not pipeline.initialize(ensure_test_scene=not parsed.skip_scene_load):
            node.get_logger().error("PickAndPlacePipeline initialization failed.")
            sys.exit(1)

        place_target = None if parsed.pick_only else tuple(parsed.place_pos)
        success = pipeline.run_pick_and_place(
            object_id=parsed.object_id,
            object_center=tuple(parsed.cube_pos),
            object_dimensions=tuple(parsed.cube_dims),
            place_target=place_target,
        )
        if success:
            node.get_logger().info("END-TO-END PICK-AND-PLACE PIPELINE SUCCEEDED.")
        else:
            node.get_logger().error("END-TO-END PICK-AND-PLACE PIPELINE FAILED.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if not success:
        sys.exit(1)


if __name__ == "__main__":
    main()
