#!/usr/bin/env python3
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

"""Native MoveIt 2 pick-and-place verification script for the BCR 7-DOF arm."""

import argparse
import math
import time
from typing import Dict, Optional, Tuple
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Vector3
from gz.msgs10.empty_pb2 import Empty as GzEmpty
from gz.msgs10.stringmsg_pb2 import StringMsg as GzStringMsg
from gz.transport13 import Node as GzNode
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
from robot_controllers.geometry_utils import z_approach_quat_tuple
from robot_controllers.planning_scene_manager import PlanningSceneManager
from robot_controllers.sdf_scene_loader import (
    build_world_collision_objects,
    parse_world_sdf_includes,
)
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from tf2_msgs.msg import TFMessage
import tf2_ros


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


class BcrArmPickPlaceTester(Node):
    """Execute a native MoveIt 2 pre-grasp -> grasp -> lift -> place sequence."""

    def __init__(self, namespace: str, prefix: str, box_name: str, world_name: str) -> None:
        super().__init__(
            'bcr_arm_pick_place_tester',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
        )
        ns = namespace.strip().strip('/')
        self._ns_prefix = f'/{ns}' if ns else ''
        self._prefix = prefix if prefix else f'{ns}_'
        self._base_frame = f'{self._prefix}base_link'
        self._tcp_frame = f'{self._prefix}grasp_tcp'
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
        self._entity_full_poses: Dict[str, tuple] = {}
        self._srdf_states: Dict[str, Dict[str, Dict[str, float]]] = {}
        self._gz_node = GzNode()
        self._gz_attach_pub = self._gz_node.advertise(f'{self._ns_prefix}/attach', GzStringMsg)
        self._gz_detach_pub = self._gz_node.advertise(f'{self._ns_prefix}/detach', GzEmpty)
        self._psm = PlanningSceneManager(
            self,
            spin_locally=True,
            namespace=self._ns_prefix,
            touch_links=[
                f'{self._prefix}grasp_tcp',
                f'{self._prefix}flange',
                f'{self._prefix}robotiq_85_base_link',
                f'{self._prefix}robotiq_85_left_finger_tip_link',
                f'{self._prefix}robotiq_85_right_finger_tip_link',
                f'{self._prefix}robotiq_85_left_finger_link',
                f'{self._prefix}robotiq_85_right_finger_link',
                f'{self._prefix}robotiq_85_left_inner_knuckle_link',
                f'{self._prefix}robotiq_85_right_inner_knuckle_link',
                f'{self._prefix}robotiq_85_left_knuckle_link',
                f'{self._prefix}robotiq_85_right_knuckle_link',
            ],
        )

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
                q = tr.transform.rotation
                self._entity_poses[tr.child_frame_id] = (p.x, p.y, p.z)
                self._entity_full_poses[tr.child_frame_id] = (
                    p.x, p.y, p.z, (q.x, q.y, q.z, q.w)
                )

    def wait_ready(self, timeout_sec: float = 30.0) -> bool:
        self.get_logger().info(
            f'Waiting for {self._ns_prefix}/move_action and TF map->{self._base_frame}...'
        )
        if not self._move_client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().error('move_action server did not become available')
            return False

        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._joint_positions and self._tf_buffer.can_transform(
                self._base_frame, 'map', rclpy.time.Time()
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

    def map_to_base_link(self, wx: float, wy: float, wz: float) -> Tuple[float, float, float]:
        tf = self._tf_buffer.lookup_transform(
            self._base_frame, 'map', rclpy.time.Time(), timeout=Duration(seconds=2.0)
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
        rclpy.spin_until_future_complete(self, res_fut, timeout_sec=90.0)
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

    def move_joints(
        self, group: str, targets: Dict[str, float], label: str, tol: float = 0.01
    ) -> bool:
        req = MotionPlanRequest(
            group_name=group, num_planning_attempts=3, allowed_planning_time=5.0
        )
        req.max_velocity_scaling_factor = 0.40
        req.max_acceleration_scaling_factor = 0.40
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

    def _make_pose_stamped(
        self, bx: float, by: float, bz: float, pitch_down: float = 0.314
    ) -> PoseStamped:
        yaw = math.atan2(by, bx)
        qx, qy, qz, qw = z_approach_quat_tuple(yaw, pitch_down)
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
            ik_req.ik_request.group_name = 'bcr_arm'
            ik_req.ik_request.ik_link_name = self._tcp_frame
            ik_req.ik_request.pose_stamped = ps
            merged = dict(self._joint_positions)
            if 'joint1' not in merged:
                merged.update(self._srdf_states.get('bcr_arm', {}).get('pre_fold', {}))
            rs = RobotState()
            rs.joint_state.name = list(merged.keys())
            rs.joint_state.position = [merged[k] for k in rs.joint_state.name]
            ik_req.ik_request.robot_state = rs
            ik_req.ik_request.avoid_collisions = True
            fut = self._ik_client.call_async(ik_req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
            resp = fut.result()
            if resp and resp.error_code.val == 1:
                arm_names = [f'joint{i}' for i in range(1, 8)]
                sol = dict(zip(resp.solution.joint_state.name, resp.solution.joint_state.position))
                targets: Dict[str, float] = {}
                for k in arm_names:
                    if k not in sol:
                        continue
                    val = float(sol[k])
                    if k in ('joint1', 'joint3', 'joint5', 'joint7'):
                        cur = float(self._joint_positions.get(k, 0.0))
                        cands = [
                            val + d
                            for d in (-2.0 * math.pi, 0.0, 2.0 * math.pi)
                            if -6.28 <= val + d <= 6.28
                        ]
                        if cands:
                            val = min(cands, key=lambda c: abs(c - cur))
                    targets[k] = val
                return self.move_joints('bcr_arm', targets, label)

        req = MotionPlanRequest(
            group_name='bcr_arm', num_planning_attempts=5, allowed_planning_time=8.0
        )
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
        req.goal_constraints.append(
            Constraints(position_constraints=[pc], orientation_constraints=[oc])
        )
        return self._send_move_goal(req, label)

    def move_cartesian(self, ps: PoseStamped, label: str) -> bool:
        cart_ready = (
            self._cart_client.wait_for_service(timeout_sec=2.0)
            and self._exec_client.wait_for_server(timeout_sec=2.0)
        )
        if cart_ready:
            req = GetCartesianPath.Request()
            req.header = ps.header
            rs = RobotState()
            rs.joint_state.name = list(self._joint_positions.keys())
            rs.joint_state.position = [self._joint_positions[k] for k in rs.joint_state.name]
            req.start_state = rs
            req.group_name = 'bcr_arm'
            req.link_name = self._tcp_frame
            req.waypoints = [ps.pose]
            req.max_step = 0.01
            req.jump_threshold = 0.0
            req.prismatic_jump_threshold = 0.0
            req.revolute_jump_threshold = 1.5
            req.avoid_collisions = True
            req.max_velocity_scaling_factor = 0.25
            req.max_acceleration_scaling_factor = 0.25
            fut = self._cart_client.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=8.0)
            resp = fut.result()
            ok = (
                resp is not None
                and resp.error_code.val == 1
                and resp.fraction >= 0.90
                and bool(resp.solution.joint_trajectory.points)
            )
            if ok:
                pts = resp.solution.joint_trajectory.points
                last_ns = (
                    pts[-1].time_from_start.sec * 1_000_000_000
                    + pts[-1].time_from_start.nanosec
                )
                if last_ns == 0:
                    for idx, pt in enumerate(pts):
                        pt.time_from_start = Duration(seconds=0.1 * idx).to_msg()
                goal = ExecuteTrajectory.Goal(trajectory=resp.solution)
                gf = self._exec_client.send_goal_async(goal)
                rclpy.spin_until_future_complete(self, gf, timeout_sec=10.0)
                gh = gf.result()
                if gh and gh.accepted:
                    rf = gh.get_result_async()
                    rclpy.spin_until_future_complete(self, rf, timeout_sec=90.0)
                    wr = rf.result()
                    if wr and wr.result.error_code.val == 1:
                        self.get_logger().info(f'[{label}] Cartesian SUCCEEDED')
                        return True
        return self.move_to_pose(ps, label)

    def spin_pause(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def gz_attach(self, model_name: str) -> None:
        """Publish to Gazebo DynamicAttachSystem to weld `model_name` to the flange."""
        msg = GzStringMsg()
        msg.data = model_name
        self._gz_attach_pub.publish(msg)
        time.sleep(0.05)
        self._gz_attach_pub.publish(msg)

    def gz_detach(self) -> None:
        """Publish to Gazebo DynamicAttachSystem to remove the rigid grasp joint."""
        msg = GzEmpty()
        self._gz_detach_pub.publish(msg)
        time.sleep(0.05)
        self._gz_detach_pub.publish(msg)

    def load_nearby_world_obstacles(self) -> None:
        """Load nearby Gazebo SDF model colliders into MoveIt PlanningScene."""
        if not self._psm.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('apply_planning_scene not ready; skipping world obstacles')
            return
        try:
            tf = self._tf_buffer.lookup_transform(
                'map', self._base_frame, rclpy.time.Time(), timeout=Duration(seconds=2.0)
            )
            center_xy = (tf.transform.translation.x, tf.transform.translation.y)
        except Exception as exc:
            self.get_logger().warn(f'TF lookup map->{self._base_frame} failed: {exc}')
            return

        includes = {}
        try:
            sim_share = get_package_share_directory('warehouse_sim')
            includes = parse_world_sdf_includes(f'{sim_share}/worlds/warehouse.sdf')
        except Exception:
            includes = {}
        has_warehouse_model = any(k in self._entity_full_poses for k in includes)
        active_includes = includes if has_warehouse_model else None

        objs = build_world_collision_objects(
            self._entity_full_poses,
            center_xy=center_xy,
            radius_m=2.0,
            world_includes=active_includes,
            exclude_names=(self._box_name,),
            frame_id='map',
        )
        if objs and self._psm.add_collision_objects(objs):
            total_prims = sum(len(o.primitives) for o in objs)
            names = ', '.join(o.id for o in objs)
            self.get_logger().info(
                f'Loaded {len(objs)} world obstacle(s) ({total_prims} primitives): [{names}]'
            )

    def get_box_z(self) -> Optional[float]:
        rclpy.spin_once(self, timeout_sec=0.2)
        pose = self._entity_poses.get(self._box_name)
        return pose[2] if pose else None


def main() -> int:
    parser = argparse.ArgumentParser(
        description='Native MoveIt 2 pick-and-place verification for BCR arm'
    )
    parser.add_argument('--namespace', default='bcr_arm', help='Robot ROS namespace')
    parser.add_argument('--prefix', default='', help='TF link prefix (default <namespace>_)')
    parser.add_argument('--world-name', default='default', help='Gazebo world name')
    parser.add_argument('--box-name', default='graspable_box', help='Target Gazebo box model name')
    parser.add_argument('--box-x', type=float, default=None, help='Override box world X')
    parser.add_argument('--box-y', type=float, default=None, help='Override box world Y')
    parser.add_argument('--box-z', type=float, default=None, help='Override box world Z')
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    tester = BcrArmPickPlaceTester(args.namespace, args.prefix, args.box_name, args.world_name)
    try:
        if not tester.wait_ready(timeout_sec=30.0):
            return 1

        deadline = time.monotonic() + 3.0
        while args.box_name not in tester._entity_poses and time.monotonic() < deadline:
            rclpy.spin_once(tester, timeout_sec=0.1)
        tester.spin_pause(0.4)
        tester.load_nearby_world_obstacles()

        live = tester._entity_poses.get(args.box_name)
        wx = args.box_x if args.box_x is not None else (live[0] if live else 1.20)
        wy = args.box_y if args.box_y is not None else (live[1] if live else 13.08)
        wz = args.box_z if args.box_z is not None else (live[2] if live else 0.945)

        bx, by, bz = tester.map_to_base_link(wx, wy, wz)
        yaw = math.atan2(by, bx)
        pre_z = live[2] if live else wz
        tester.get_logger().info(
            f'Resolved target box in {tester._base_frame}: x={bx:.3f}, y={by:.3f}, z={bz:.3f}'
        )

        p_pre = tester._make_pose_stamped(
            bx - 0.08 * math.cos(yaw), by - 0.08 * math.sin(yaw), bz + 0.10
        )
        p_grasp = tester._make_pose_stamped(bx, by, bz + 0.02)
        p_lift = tester._make_pose_stamped(
            bx - 0.04 * math.cos(yaw), by - 0.04 * math.sin(yaw), bz + 0.20
        )
        p_pre_place = tester._make_pose_stamped(bx, by + 0.12, bz + 0.18)
        p_place = tester._make_pose_stamped(bx, by + 0.12, bz + 0.04)

        for fn in (
            lambda: tester.move_named_state('bcr_gripper', 'open', 'open_gripper', tol=0.05),
            lambda: tester.move_named_state('bcr_arm', 'home', 'arm_home'),
            lambda: tester.move_to_pose(p_pre, 'pre_grasp'),
            lambda: tester.move_cartesian(p_grasp, 'grasp_approach'),
            lambda: tester.move_named_state('bcr_gripper', 'attach', 'close_gripper', tol=0.05),
        ):
            if not fn():
                return 1
            tester.spin_pause(0.6)

        tester.gz_attach(args.box_name)
        tester._psm.add_box(
            args.box_name, (0.06, 0.06, 0.06), (0.0, 0.0, 0.0), frame_id=tester._tcp_frame
        )
        tester._psm.attach_object(args.box_name, link_name=tester._tcp_frame)
        tester.spin_pause(0.5)

        if not tester.move_cartesian(p_lift, 'lift_box'):
            return 1
        tester.spin_pause(1.0)

        post_z = tester.get_box_z()
        if post_z is not None and post_z < pre_z + 0.05:
            tester.get_logger().error(
                f'Box Z did not lift cleanly: {post_z:.3f} vs pre {pre_z:.3f}'
            )
            return 1

        if not tester.move_to_pose(p_pre_place, 'pre_place'):
            return 1
        if not tester.move_cartesian(p_place, 'place_descend'):
            return 1
        tester.spin_pause(0.4)

        tester.gz_detach()
        tester._psm.detach_object(args.box_name, link_name=tester._tcp_frame)
        tester._psm.remove_object(args.box_name)
        tester.spin_pause(0.3)

        for fn in (
            lambda: tester.move_named_state('bcr_gripper', 'open', 'release_box', tol=0.05),
            lambda: tester.move_cartesian(p_pre_place, 'retreat'),
            lambda: tester.move_named_state('bcr_arm', 'home', 'return_home'),
        ):
            if not fn():
                return 1
            tester.spin_pause(0.5)

        tester.get_logger().info('Native MoveIt 2 pick-and-place verification PASSED.')
        return 0
    finally:
        tester.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
