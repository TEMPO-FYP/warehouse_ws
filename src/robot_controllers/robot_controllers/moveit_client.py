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

"""Shared MoveIt 2 action/service client helpers for heterogeneous manipulators."""

import math
from typing import Any, Callable, Dict, List, Optional, Sequence
import xml.etree.ElementTree as ET

from action_msgs.msg import GoalStatus
from control_msgs.action import GripperCommand
from geometry_msgs.msg import Pose, PoseStamped, Vector3
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
    RobotState,
    RobotTrajectory,
)
from moveit_msgs.srv import GetCartesianPath, GetPositionIK
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.callback_groups import CallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


def unwrap_ik_solution(
    ik_sol: Dict[str, float],
    current_joints: Dict[str, float],
) -> Dict[str, float]:
    """Unwrap `2*pi`-equivalent IK angles to the closest value near `current_joints`."""
    out: Dict[str, float] = {}
    for joint_name, val in ik_sol.items():
        if abs(val) > math.pi:
            wrapped = val - 2.0 * math.pi if val > 0.0 else val + 2.0 * math.pi
            cur = float(current_joints.get(joint_name, 0.0))
            if abs(wrapped - cur) < abs(val - cur):
                val = wrapped
        out[joint_name] = float(val)
    return out


def parse_srdf_group_states(srdf_xml: str) -> Dict[str, Dict[str, Dict[str, float]]]:
    """Parse `<group_state>` tags from SRDF XML into `{group: {state_name: {joint: val}}}`."""
    states: Dict[str, Dict[str, Dict[str, float]]] = {}
    if not srdf_xml or not srdf_xml.strip():
        return states
    root = ET.fromstring(srdf_xml)
    for gs in root.findall('.//group_state'):
        group = gs.get('group', '').strip()
        name = gs.get('name', '').strip()
        if not group or not name:
            continue
        joints: Dict[str, float] = {}
        for j in gs.findall('joint'):
            jname = j.get('name', '').strip()
            jval = j.get('value')
            if jname and jval is not None:
                joints[jname] = float(jval)
        if joints:
            states.setdefault(group, {})[name] = joints
    return states


def build_pose_constraints(
    link_name: str,
    pose_stamped: PoseStamped,
    pos_tol: float = 0.015,
    ori_tol: float = 0.35,
) -> Constraints:
    """Construct MoveIt PositionConstraint + OrientationConstraint for a target PoseStamped."""
    region = BoundingVolume()
    sphere = SolidPrimitive()
    sphere.type = SolidPrimitive.SPHERE
    sphere.dimensions = [float(pos_tol)]
    region.primitives = [sphere]
    region.primitive_poses = [pose_stamped.pose]

    pc = PositionConstraint()
    pc.header = pose_stamped.header
    pc.link_name = link_name
    pc.target_point_offset = Vector3(x=0.0, y=0.0, z=0.0)
    pc.constraint_region = region
    pc.weight = 1.0

    oc = OrientationConstraint()
    oc.header = pose_stamped.header
    oc.link_name = link_name
    oc.orientation = pose_stamped.pose.orientation
    oc.absolute_x_axis_tolerance = float(ori_tol)
    oc.absolute_y_axis_tolerance = float(ori_tol)
    oc.absolute_z_axis_tolerance = float(ori_tol)
    oc.weight = 1.0

    return Constraints(position_constraints=[pc], orientation_constraints=[oc])


def ensure_trajectory_timed(
    traj: RobotTrajectory,
    vel_scale: float = 0.25,
    fallback_joint_speed: float = 0.6,
) -> RobotTrajectory:
    """
    Ensure `traj.joint_trajectory.points` have monotonically increasing `time_from_start`.

    Scales existing TOTG timestamps by `1.0 / vel_scale` when requested, or assigns
    monotonic joint-distance timestamps if all points carry zero `time_from_start`.
    """
    points = traj.joint_trajectory.points
    if not points:
        return traj

    scale = max(0.05, min(1.0, float(vel_scale)))
    last_ns = points[-1].time_from_start.sec * 1_000_000_000 + points[-1].time_from_start.nanosec

    if last_ns > 0:
        if scale < 0.999:
            inv = 1.0 / scale
            for pt in points:
                sec = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
                pt.time_from_start = Duration(seconds=sec * inv).to_msg()
                if pt.velocities:
                    pt.velocities = [v * scale for v in pt.velocities]
                if pt.accelerations:
                    pt.accelerations = [a * (scale * scale) for a in pt.accelerations]
        return traj

    elapsed = 0.0
    eff_speed = max(0.05, fallback_joint_speed * scale)
    for idx, pt in enumerate(points):
        if idx > 0 and pt.positions and points[idx - 1].positions:
            max_dq = max(
                abs(a - b) for a, b in zip(pt.positions, points[idx - 1].positions)
            )
            elapsed += max(0.05, max_dq / eff_speed)
        elif idx > 0:
            elapsed += 0.05
        pt.time_from_start = Duration(seconds=elapsed).to_msg()
    return traj


class MoveItRobotClient:
    """Namespaced MoveIt 2 client supporting SRDF states, seeded IK, Pose & Cartesian execution."""

    def __init__(
        self,
        node: Node,
        namespace: str,
        wait_future_fn: Callable[[Any, float], bool],
        callback_group: CallbackGroup,
        moveit_timeout_sec: float = 45.0,
        gripper_action_name: Optional[str] = None,
    ) -> None:
        self._node = node
        self._logger = node.get_logger()
        ns = namespace.strip().strip('/')
        self._ns_prefix = f'/{ns}' if ns else ''
        self._wait_future = wait_future_fn
        self._timeout = float(moveit_timeout_sec)

        self._move_client = ActionClient(
            node, MoveGroup, f'{self._ns_prefix}/move_action', callback_group=callback_group
        )
        self._exec_client = ActionClient(
            node, ExecuteTrajectory, f'{self._ns_prefix}/execute_trajectory',
            callback_group=callback_group,
        )
        self._ik_client = node.create_client(
            GetPositionIK, f'{self._ns_prefix}/compute_ik', callback_group=callback_group
        )
        self._cart_client = node.create_client(
            GetCartesianPath, f'{self._ns_prefix}/compute_cartesian_path',
            callback_group=callback_group,
        )
        self._param_client = node.create_client(
            GetParameters, f'{self._ns_prefix}/move_group/get_parameters',
            callback_group=callback_group,
        )
        self._gripper_client: Optional[ActionClient] = None
        if gripper_action_name:
            action_path = gripper_action_name.lstrip('/')
            self._gripper_client = ActionClient(
                node, GripperCommand, f'{self._ns_prefix}/{action_path}',
                callback_group=callback_group,
            )

        self._srdf_states: Optional[Dict[str, Dict[str, Dict[str, float]]]] = None

    def _send_action_goal(
        self,
        client: ActionClient,
        goal: Any,
        label: str,
        server_label: str,
    ) -> Optional[Any]:
        if not client.wait_for_server(timeout_sec=5.0):
            self._logger.error(f'[moveit] {server_label} unavailable for [{label}].')
            return None

        send_future = client.send_goal_async(goal)
        if not self._wait_future(send_future, 15.0):
            self._logger.error(f'[moveit] Timed out sending [{label}].')
            return None

        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            self._logger.error(f'[moveit] Goal [{label}] rejected.')
            return None

        result_future = goal_handle.get_result_async()
        if not self._wait_future(result_future, self._timeout):
            goal_handle.cancel_goal_async()
            self._logger.error(f'[moveit] Goal [{label}] result timed out.')
            return None

        return result_future.result()

    def _call_service(
        self, client: Any, request: Any, label: str, timeout_sec: float
    ) -> Optional[Any]:
        if (
            not client.service_is_ready()
            and not client.wait_for_service(timeout_sec=min(5.0, timeout_sec))
        ):
            self._logger.warn(f'[moveit] Service unavailable for [{label}].')
            return None
        future = client.call_async(request)
        if not self._wait_future(future, timeout_sec):
            self._logger.warn(f'[moveit] Service call timed out for [{label}].')
            return None
        return future.result()

    def _ensure_srdf_states(self) -> Dict[str, Dict[str, Dict[str, float]]]:
        if self._srdf_states is not None:
            return self._srdf_states
        req = GetParameters.Request()
        req.names = ['robot_description_semantic']
        resp = self._call_service(self._param_client, req, 'get_srdf', timeout_sec=5.0)
        if resp is None or not resp.values or not resp.values[0].string_value:
            self._logger.warn(
                f"[moveit] Could not fetch 'robot_description_semantic' from "
                f'{self._ns_prefix}/move_group/get_parameters.'
            )
            return {}
        self._srdf_states = parse_srdf_group_states(resp.values[0].string_value)
        return self._srdf_states

    def get_group_state(self, group: str, state_name: str) -> Optional[Dict[str, float]]:
        """Look up a named SRDF `<group_state>` joint map for `group`."""
        return self._ensure_srdf_states().get(group, {}).get(state_name)

    def move_to_joints(
        self,
        group: str,
        targets: Dict[str, float],
        label: str,
        tol: float = 0.01,
        vel_scale: float = 0.4,
    ) -> bool:
        """Plan and execute a joint-space goal via MoveGroup."""
        req = MotionPlanRequest()
        req.group_name = group
        req.num_planning_attempts = 5
        req.allowed_planning_time = 8.0
        req.max_velocity_scaling_factor = float(vel_scale)
        req.max_acceleration_scaling_factor = float(vel_scale)

        c = Constraints()
        for joint_name, val in targets.items():
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = float(val)
            jc.tolerance_above = float(tol)
            jc.tolerance_below = float(tol)
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        req.goal_constraints.append(c)

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 2

        wrapped = self._send_action_goal(
            self._move_client, goal, label, f'{self._ns_prefix}/move_action'
        )
        if wrapped is None or wrapped.result.error_code.val != MoveItErrorCodes.SUCCESS:
            code = wrapped.result.error_code.val if wrapped is not None else -1
            self._logger.error(f'[moveit] [{label}] failed with MoveItErrorCode={code}.')
            return False
        self._logger.info(f'[moveit] [{label}] SUCCEEDED.')
        return True

    def move_to_named_state(
        self,
        group: str,
        state_name: str,
        label: str,
        tol: float = 0.01,
        vel_scale: float = 0.4,
    ) -> bool:
        """Execute a named SRDF `<group_state>` resolved from `robot_description_semantic`."""
        targets = self.get_group_state(group, state_name)
        if not targets:
            self._logger.error(
                f"[moveit] SRDF state '{state_name}' not found for group '{group}'."
            )
            return False
        return self.move_to_joints(group, targets, label, tol=tol, vel_scale=vel_scale)

    def move_gripper_command(self, position: float, max_effort: float, label: str) -> bool:
        """Execute a GripperCommand action goal (e.g., Robotiq 2F-85)."""
        if self._gripper_client is None:
            self._logger.error(f'[moveit] No GripperCommand client configured for [{label}].')
            return False
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(max_effort)
        wrapped = self._send_action_goal(
            self._gripper_client, goal, label, f'{self._ns_prefix}/gripper_cmd'
        )
        if wrapped is None or wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            self._logger.error(f'[moveit] GripperCommand [{label}] failed.')
            return False
        self._logger.info(f'[moveit] GripperCommand [{label}] SUCCEEDED.')
        return True

    def _build_robot_state(
        self,
        current_joints: Dict[str, float],
        seed_overrides: Optional[Dict[str, float]] = None,
    ) -> RobotState:
        merged = dict(current_joints)
        if seed_overrides:
            merged.update(seed_overrides)
        state = RobotState()
        state.is_diff = False
        state.joint_state.name = list(merged.keys())
        state.joint_state.position = [float(merged[k]) for k in state.joint_state.name]
        return state

    def compute_ik(
        self,
        group: str,
        ik_link_name: str,
        pose_stamped: PoseStamped,
        current_joints: Dict[str, float],
        seed_overrides: Optional[Dict[str, float]] = None,
        avoid_collisions: bool = True,
        timeout_sec: float = 3.0,
    ) -> Optional[Dict[str, float]]:
        """Query MoveIt `/compute_ik` (KDL) with full RobotState + optional seed overrides."""
        req = GetPositionIK.Request()
        req.ik_request.group_name = group
        req.ik_request.ik_link_name = ik_link_name
        req.ik_request.pose_stamped = pose_stamped
        req.ik_request.robot_state = self._build_robot_state(current_joints, seed_overrides)
        req.ik_request.avoid_collisions = bool(avoid_collisions)
        req.ik_request.timeout = Duration(seconds=float(timeout_sec)).to_msg()

        resp = self._call_service(self._ik_client, req, f'ik_{ik_link_name}', timeout_sec + 2.0)
        if resp is None or resp.error_code.val != MoveItErrorCodes.SUCCESS:
            code = resp.error_code.val if resp is not None else -1
            self._logger.warn(f'[moveit] /compute_ik failed (code={code}) for {ik_link_name}.')
            return None

        sol = resp.solution.joint_state
        return {n: float(p) for n, p in zip(sol.name, sol.position)}

    def is_ik_service_ready(self) -> bool:
        """Return True if the MoveIt `/compute_ik` service is currently ready."""
        return bool(self._ik_client.service_is_ready())

    def check_ik_reachable(
        self,
        group: str,
        ik_link_name: str,
        poses: Sequence[PoseStamped],
        current_joints: Dict[str, float],
        seed_overrides: Optional[Dict[str, float]] = None,
        avoid_collisions: bool = True,
        timeout_sec: float = 0.5,
    ) -> Optional[bool]:
        """Return True iff every pose has valid IK, False if any fails, None if unready."""
        if not self._ik_client.service_is_ready():
            return None

        active_seed = seed_overrides
        for pose in poses:
            seed_candidates: List[Optional[Dict[str, float]]] = (
                [active_seed, None] if active_seed is not None else [None]
            )
            solved: Optional[Dict[str, float]] = None
            for seed in seed_candidates:
                solved = self.compute_ik(
                    group,
                    ik_link_name,
                    pose,
                    current_joints,
                    seed_overrides=seed,
                    avoid_collisions=avoid_collisions,
                    timeout_sec=timeout_sec,
                )
                if solved:
                    break
            if not solved:
                return False
            active_seed = unwrap_ik_solution(solved, current_joints)
        return True

    def move_to_pose(
        self,
        group: str,
        link_name: str,
        pose_stamped: PoseStamped,
        label: str,
        current_joints: Dict[str, float],
        seed_overrides: Optional[Dict[str, float]] = None,
        pos_tol: float = 0.015,
        ori_tol: float = 0.35,
        vel_scale: float = 0.35,
        arm_tol: float = 0.01,
    ) -> bool:
        """
        Plan and execute a 6-DOF end-effector pose via seeded KDL IK.

        Falls back to PositionConstraint + OrientationConstraint in OMPL if `/compute_ik` fails.
        """
        seed_candidates: List[Optional[Dict[str, float]]] = (
            [seed_overrides, None] if seed_overrides is not None else [None]
        )
        for seed in seed_candidates:
            ik_sol = self.compute_ik(
                group, link_name, pose_stamped, current_joints, seed_overrides=seed
            )
            if not ik_sol:
                continue
            unwrapped = unwrap_ik_solution(ik_sol, current_joints)
            group_states = self._ensure_srdf_states().get(group, {})
            if seed_overrides:
                target_names: Sequence[str] = list(seed_overrides.keys())
            elif group_states:
                target_names = list(next(iter(group_states.values())).keys())
            else:
                target_names = list(unwrapped.keys())
            arm_targets = {k: unwrapped[k] for k in target_names if k in unwrapped}
            if arm_targets and self.move_to_joints(
                group, arm_targets, label, tol=arm_tol, vel_scale=vel_scale
            ):
                return True

        req = MotionPlanRequest()
        req.group_name = group
        req.num_planning_attempts = 5
        req.allowed_planning_time = 8.0
        req.max_velocity_scaling_factor = float(vel_scale)
        req.max_acceleration_scaling_factor = float(vel_scale)
        req.goal_constraints.append(
            build_pose_constraints(link_name, pose_stamped, pos_tol=pos_tol, ori_tol=ori_tol)
        )

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 2

        wrapped = self._send_action_goal(
            self._move_client, goal, label, f'{self._ns_prefix}/move_action'
        )
        if wrapped is None or wrapped.result.error_code.val != MoveItErrorCodes.SUCCESS:
            code = wrapped.result.error_code.val if wrapped is not None else -1
            self._logger.error(f'[moveit] [{label}] pose goal failed (code={code}).')
            return False
        self._logger.info(f'[moveit] [{label}] pose goal SUCCEEDED.')
        return True

    def execute_cartesian_path(
        self,
        group: str,
        link_name: str,
        waypoints: List[Pose],
        frame_id: str,
        label: str,
        current_joints: Dict[str, float],
        max_step: float = 0.01,
        revolute_jump_threshold: float = 1.5,
        avoid_collisions: bool = True,
        vel_scale: float = 0.25,
        min_fraction: float = 0.90,
    ) -> bool:
        """Compute a Cartesian path via `/compute_cartesian_path`, time-check, and execute it."""
        req = GetCartesianPath.Request()
        req.header.frame_id = frame_id
        req.header.stamp = self._node.get_clock().now().to_msg()
        req.start_state = self._build_robot_state(current_joints)
        req.group_name = group
        req.link_name = link_name
        req.waypoints = waypoints
        req.max_step = float(max_step)
        req.jump_threshold = 0.0
        req.prismatic_jump_threshold = 0.0
        req.revolute_jump_threshold = float(revolute_jump_threshold)
        req.avoid_collisions = bool(avoid_collisions)
        scale = max(0.05, min(1.0, float(vel_scale)))
        req.max_velocity_scaling_factor = scale
        req.max_acceleration_scaling_factor = scale

        resp = self._call_service(self._cart_client, req, label, timeout_sec=10.0)
        if resp is None or resp.error_code.val != MoveItErrorCodes.SUCCESS:
            code = resp.error_code.val if resp is not None else -1
            self._logger.warn(f'[moveit] Cartesian [{label}] service failed (code={code}).')
            return False
        if float(resp.fraction) < min_fraction or not resp.solution.joint_trajectory.points:
            self._logger.warn(
                f'[moveit] Cartesian [{label}] incomplete fraction={resp.fraction:.2f} '
                f'(min={min_fraction:.2f}).'
            )
            return False

        # MoveIt's TOTG already applied `scale`; pass vel_scale=1.0 to time-fill only if untimed.
        timed_traj = ensure_trajectory_timed(resp.solution, vel_scale=1.0)
        exec_goal = ExecuteTrajectory.Goal()
        exec_goal.trajectory = timed_traj

        wrapped = self._send_action_goal(
            self._exec_client, exec_goal, label, f'{self._ns_prefix}/execute_trajectory'
        )
        if wrapped is None or wrapped.result.error_code.val != MoveItErrorCodes.SUCCESS:
            code = wrapped.result.error_code.val if wrapped is not None else -1
            self._logger.warn(f'[moveit] ExecuteTrajectory [{label}] failed (code={code}).')
            return False
        self._logger.info(
            f'[moveit] Cartesian [{label}] SUCCEEDED (fraction={resp.fraction:.2f}).'
        )
        return True
