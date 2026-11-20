#!/usr/bin/env python3
"""Native MoveIt 2 pick-and-place verification script for Unitree B1Z1."""

import argparse
import math
import time
from typing import Dict, Optional
import xml.etree.ElementTree as ET

from geometry_msgs.msg import PoseStamped, Vector3
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    OrientationConstraint,
    PositionConstraint,
    RobotState,
)
from moveit_msgs.srv import GetCartesianPath, GetPositionIK
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from tf2_msgs.msg import TFMessage
import tf2_ros


def _rpy_to_quat(roll: float, pitch: float, yaw: float):
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _parse_srdf_states(srdf_xml: str) -> Dict[str, Dict[str, Dict[str, float]]]:
    states: Dict[str, Dict[str, Dict[str, float]]] = {}
    root = ET.fromstring(srdf_xml)
    for gs in root.findall('.//group_state'):
        group = gs.get('group', '').strip()
        name = gs.get('name', '').strip()
        joints = {
            j.get('name', '').strip(): float(j.get('value'))
            for j in gs.findall('joint')
            if j.get('name') and j.get('value') is not None
        }
        if group and name and joints:
            states.setdefault(group, {})[name] = joints
    return states


class B1Z1PickPlaceTester(Node):
    """Execute a native MoveIt 2 pre-grasp -> grasp -> lift -> place sequence."""

    def __init__(self, namespace: str, prefix: str, box_name: str, world_name: str) -> None:
        super().__init__(
            'b1z1_pick_place_tester',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
        )
        ns = namespace.strip().strip('/')
        self._ns_prefix = f'/{ns}' if ns else ''
        self._prefix = prefix if prefix else f'{ns}_'
        self._base_frame = f'{self._prefix}base_link'
        self._odom_frame = f'{self._prefix}odom'
        self._tcp_frame = f'{self._prefix}gripper_tcp'
        self._box_name = box_name

        self._move_client = ActionClient(self, MoveGroup, f'{self._ns_prefix}/move_action')
        self._exec_client = ActionClient(
            self, ExecuteTrajectory, f'{self._ns_prefix}/execute_trajectory'
        )
        self._ik_client = self.create_client(GetPositionIK, f'{self._ns_prefix}/compute_ik')
        self._cart_client = self.create_client(
            GetCartesianPath, f'{self._ns_prefix}/compute_cartesian_path'
        )
        self._param_client = self.create_client(
            GetParameters, f'{self._ns_prefix}/move_group/get_parameters'
        )
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._joint_positions: Dict[str, float] = {}
        self._entity_poses: Dict[str, tuple] = {}
        self._srdf_states: Dict[str, Dict[str, Dict[str, float]]] = {}

        self.create_subscription(JointState, f'{self._ns_prefix}/joint_states', self._js_cb, 10)
        self.create_subscription(
            TFMessage, f'/world/{world_name}/pose/info', self._pose_cb, 10
        )

    def _js_cb(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            self._joint_positions[name] = float(pos)

    def _pose_cb(self, msg: TFMessage) -> None:
        for tr in msg.transforms:
            if tr.child_frame_id:
                p = tr.transform.translation
                self._entity_poses[tr.child_frame_id] = (p.x, p.y, p.z)

    def wait_ready(self, timeout_sec: float = 30.0) -> bool:
        self.get_logger().info(
            f'Waiting for {self._ns_prefix}/move_action and TF {self._odom_frame}->{self._base_frame}...'
        )
        if not self._move_client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().error('move_action server did not become available')
            return False

        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._joint_positions and self._tf_buffer.can_transform(
                self._base_frame, self._odom_frame, rclpy.time.Time()
            ):
                break
        else:
            self.get_logger().error('Timed out waiting for joint_states or TF transform')
            return False

        if self._param_client.wait_for_service(timeout_sec=5.0):
            req = GetParameters.Request(names=['robot_description_semantic'])
            fut = self._param_client.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
            res = fut.result()
            if res and res.values and res.values[0].string_value:
                self._srdf_states = _parse_srdf_states(res.values[0].string_value)
        return True

    def world_to_base_link(self, wx: float, wy: float, wz: float):
        tf = self._tf_buffer.lookup_transform(
            self._base_frame, self._odom_frame, rclpy.time.Time(), timeout=Duration(seconds=2.0)
        )
        q = tf.transform.rotation
        t = tf.transform.translation
        tx = 2.0 * (q.y * wz - q.z * wy)
        ty = 2.0 * (q.z * wx - q.x * wz)
        tz = 2.0 * (q.x * wy - q.y * wx)
        vx = wx + q.w * tx + (q.y * tz - q.z * ty) + t.x
        vy = wy + q.w * ty + (q.z * tx - q.x * tz) + t.y
        vz = wz + q.w * tz + (q.x * ty - q.y * tx) + t.z
        return vx, vy, vz

    def _send_move_goal(self, req: MotionPlanRequest, label: str) -> bool:
        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 2
        fut = self._move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            self.get_logger().error(f'[{label}] goal rejected')
            return False
        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut, timeout_sec=30.0)
        wrapped = res_fut.result()
        if wrapped is None or wrapped.result.error_code.val != 1:
            code = wrapped.result.error_code.val if wrapped else -1
            self.get_logger().error(f'[{label}] failed with MoveItErrorCode={code}')
            return False
        self.get_logger().info(f'[{label}] SUCCEEDED')
        return True

    def move_named_state(self, group: str, state_name: str, label: str, tol: float = 0.01) -> bool:
        targets = self._srdf_states.get(group, {}).get(state_name)
        if not targets:
            self.get_logger().error(f"SRDF state '{state_name}' missing in group '{group}'")
            return False
        return self.move_joints(group, targets, label, tol=tol)

    def move_joints(self, group: str, targets: Dict[str, float], label: str, tol: float = 0.005) -> bool:
        req = MotionPlanRequest(group_name=group, num_planning_attempts=3, allowed_planning_time=5.0)
        req.max_velocity_scaling_factor = 0.45
        req.max_acceleration_scaling_factor = 0.45
        c = Constraints()
        for jname, val in targets.items():
            c.joint_constraints.append(
                JointConstraint(
                    joint_name=jname, position=float(val),
                    tolerance_above=tol, tolerance_below=tol, weight=1.0,
                )
            )
        req.goal_constraints.append(c)
        return self._send_move_goal(req, label)

    def _make_pose_stamped(self, bx: float, by: float, bz: float, roll: float) -> PoseStamped:
        yaw = math.atan2(by, max(bx - 0.15, 0.05))
        qx, qy, qz, qw = _rpy_to_quat(roll, 0.0, yaw)
        ps = PoseStamped()
        ps.header.frame_id = self._base_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = bx
        ps.pose.position.y = by
        ps.pose.position.z = bz
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        return ps

    def move_to_pose(self, ps: PoseStamped, label: str) -> bool:
        if self._ik_client.wait_for_service(timeout_sec=2.0):
            ik_req = GetPositionIK.Request()
            ik_req.ik_request.group_name = 'z1_arm'
            ik_req.ik_request.ik_link_name = self._tcp_frame
            ik_req.ik_request.pose_stamped = ps
            merged = dict(self._joint_positions)
            merged.update(self._srdf_states.get('z1_arm', {}).get('pre_fold', {}))
            rs = RobotState()
            rs.joint_state.name = list(merged.keys())
            rs.joint_state.position = [merged[k] for k in rs.joint_state.name]
            ik_req.ik_request.robot_state = rs
            ik_req.ik_request.avoid_collisions = True
            fut = self._ik_client.call_async(ik_req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
            resp = fut.result()
            if resp and resp.error_code.val == 1:
                arm_names = list(self._srdf_states.get('z1_arm', {}).get('home', {}).keys()) or [
                    f'joint{i}' for i in range(1, 7)
                ]
                sol = dict(zip(resp.solution.joint_state.name, resp.solution.joint_state.position))
                return self.move_joints('z1_arm', {k: sol[k] for k in arm_names if k in sol}, label)

        req = MotionPlanRequest(group_name='z1_arm', num_planning_attempts=5, allowed_planning_time=8.0)
        req.max_velocity_scaling_factor = 0.35
        req.max_acceleration_scaling_factor = 0.35
        sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.015])
        pc = PositionConstraint(
            header=ps.header, link_name=self._tcp_frame,
            target_point_offset=Vector3(x=0.0, y=0.0, z=0.0),
            constraint_region=BoundingVolume(primitives=[sphere], primitive_poses=[ps.pose]),
            weight=1.0,
        )
        oc = OrientationConstraint(
            header=ps.header, link_name=self._tcp_frame, orientation=ps.pose.orientation,
            absolute_x_axis_tolerance=0.35, absolute_y_axis_tolerance=0.35,
            absolute_z_axis_tolerance=0.35, weight=1.0,
        )
        req.goal_constraints.append(Constraints(position_constraints=[pc], orientation_constraints=[oc]))
        return self._send_move_goal(req, label)

    def move_cartesian(self, ps: PoseStamped, label: str) -> bool:
        if self._cart_client.wait_for_service(timeout_sec=2.0) and self._exec_client.wait_for_server(timeout_sec=2.0):
            req = GetCartesianPath.Request()
            req.header = ps.header
            rs = RobotState()
            rs.joint_state.name = list(self._joint_positions.keys())
            rs.joint_state.position = [self._joint_positions[k] for k in rs.joint_state.name]
            req.start_state = rs
            req.group_name = 'z1_arm'
            req.link_name = self._tcp_frame
            req.waypoints = [ps.pose]
            req.max_step = 0.01
            req.jump_threshold = 0.0
            req.prismatic_jump_threshold = 0.0
            req.revolute_jump_threshold = 1.5
            req.avoid_collisions = True
            req.max_velocity_scaling_factor = 0.35
            req.max_acceleration_scaling_factor = 0.35
            fut = self._cart_client.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=8.0)
            resp = fut.result()
            if resp and resp.error_code.val == 1 and resp.fraction >= 0.90 and resp.solution.joint_trajectory.points:
                pts = resp.solution.joint_trajectory.points
                last_ns = pts[-1].time_from_start.sec * 1_000_000_000 + pts[-1].time_from_start.nanosec
                if last_ns == 0:
                    for idx, pt in enumerate(pts):
                        pt.time_from_start = Duration(seconds=0.1 * idx).to_msg()
                goal = ExecuteTrajectory.Goal(trajectory=resp.solution)
                gf = self._exec_client.send_goal_async(goal)
                rclpy.spin_until_future_complete(self, gf, timeout_sec=10.0)
                gh = gf.result()
                if gh and gh.accepted:
                    rf = gh.get_result_async()
                    rclpy.spin_until_future_complete(self, rf, timeout_sec=30.0)
                    wr = rf.result()
                    if wr and wr.result.error_code.val == 1:
                        self.get_logger().info(f'[{label}] Cartesian SUCCEEDED')
                        return True
        return self.move_to_pose(ps, label)

    def spin_pause(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def get_box_z(self) -> Optional[float]:
        rclpy.spin_once(self, timeout_sec=0.2)
        pose = self._entity_poses.get(self._box_name)
        return pose[2] if pose else None


def main() -> int:
    parser = argparse.ArgumentParser(description='Native MoveIt 2 pick-and-place verification for B1Z1')
    parser.add_argument('--namespace', default='b1z1', help='Robot ROS namespace (default b1z1)')
    parser.add_argument('--prefix', default='', help='TF link prefix (default <namespace>_)')
    parser.add_argument('--world-name', default='default', help='Gazebo world name (default default)')
    parser.add_argument('--box-name', default='graspable_box', help='Target Gazebo box model name')
    parser.add_argument('--box-x', type=float, default=0.73, help='Box world X (default 0.73)')
    parser.add_argument('--box-y', type=float, default=16.00, help='Box world Y (default 16.00)')
    parser.add_argument('--box-z', type=float, default=0.731, help='Box world Z (default 0.731)')
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    tester = B1Z1PickPlaceTester(args.namespace, args.prefix, args.box_name, args.world_name)
    try:
        if not tester.wait_ready(timeout_sec=30.0):
            return 1

        bx, by, bz = tester.world_to_base_link(args.box_x, args.box_y, args.box_z)
        pre_z = tester.get_box_z() or args.box_z
        tester.get_logger().info(f'Resolved target box in {tester._base_frame}: x={bx:.3f}, y={by:.3f}, z={bz:.3f}')

        p_pre_grasp = tester._make_pose_stamped(bx - 0.025, by, bz + 0.15, math.pi / 2.0)
        p_grasp = tester._make_pose_stamped(bx - 0.025, by, bz + 0.022, math.pi / 2.0)
        p_lift = tester._make_pose_stamped(bx - 0.045, by, bz + 0.22, math.pi / 2.0)
        p_place = tester._make_pose_stamped(bx - 0.045, by + 0.22, bz + 0.10, math.pi / 2.0)

        for fn in (
            lambda: tester.move_named_state('z1_gripper', 'open', 'open_gripper', tol=0.08),
            lambda: tester.move_named_state('z1_arm', 'pre_fold', 'arm_pre_fold'),
            lambda: tester.move_to_pose(p_pre_grasp, 'pre_grasp'),
            lambda: tester.move_cartesian(p_grasp, 'grasp_approach'),
            lambda: tester.move_named_state('z1_gripper', 'close', 'close_gripper', tol=0.02),
        ):
            if not fn():
                return 1
            tester.spin_pause(0.8)

        grip_pos = tester._joint_positions.get('jointGripper', float('nan'))
        tester.get_logger().info(f'Gripper settled position after close: {grip_pos:.3f} rad')

        if not tester.move_cartesian(p_lift, 'lift_box'):
            return 1
        tester.spin_pause(1.0)

        post_z = tester.get_box_z()
        if post_z is not None and post_z < pre_z + 0.05:
            tester.get_logger().error(f'Box Z did not lift cleanly: {post_z:.3f} vs pre {pre_z:.3f}')
            return 1

        for fn in (
            lambda: tester.move_to_pose(p_place, 'place_pose'),
            lambda: tester.move_named_state('z1_gripper', 'open', 'release_box', tol=0.08),
            lambda: tester.move_named_state('z1_arm', 'home', 'return_home'),
        ):
            if not fn():
                return 1
            tester.spin_pause(0.5)

        tester.get_logger().info('Native MoveIt 2 pick-and-place verification PASSED.')
        return 0
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
