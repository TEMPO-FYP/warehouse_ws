#!/usr/bin/env python3
"""MoveIt 2 pick-and-place verification script for Unitree B1Z1."""

import argparse
import math
import time
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
)
from sensor_msgs.msg import JointState
import tf2_ros


def _matmul4(a, b):
    return [
        [sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)]
        for i in range(4)
    ]


def _rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return [[1.0, 0.0, 0.0, 0.0], [0.0, c, -s, 0.0], [0.0, s, c, 0.0], [0.0, 0.0, 0.0, 1.0]]


def _rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return [[c, 0.0, s, 0.0], [0.0, 1.0, 0.0, 0.0], [-s, 0.0, c, 0.0], [0.0, 0.0, 0.0, 1.0]]


def _rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return [[c, -s, 0.0, 0.0], [s, c, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]


def _trans(x, y, z):
    return [[1.0, 0.0, 0.0, x], [0.0, 1.0, 0.0, y], [0.0, 0.0, 1.0, z], [0.0, 0.0, 0.0, 1.0]]


def fk_tcp_base_link(j1: float, j2: float, j3: float, j4: float, j5: float = 0.0, j6: float = 1.57):
    """Forward kinematics from base_link to gripper_tcp matching b1z1.urdf.xacro + z1_arm.xacro."""
    t = _trans(0.20, 0.0, 0.08)
    t = _matmul4(t, _matmul4(_trans(0.0, 0.0, 0.0585), _rot_z(j1)))
    t = _matmul4(t, _matmul4(_trans(0.0, 0.0, 0.045), _rot_y(j2)))
    t = _matmul4(t, _matmul4(_trans(-0.35, 0.0, 0.0), _rot_y(j3)))
    t = _matmul4(t, _matmul4(_trans(0.218, 0.0, 0.057), _rot_y(j4)))
    t = _matmul4(t, _matmul4(_trans(0.07, 0.0, 0.0), _rot_z(j5)))
    t = _matmul4(t, _matmul4(_trans(0.0492, 0.0, 0.0), _rot_x(j6)))
    t = _matmul4(t, _trans(0.151, 0.0, 0.0))  # 0.051 stator + 0.10 tcp
    return (t[0][3], t[1][3], t[2][3])


def _solve_4x4(a, b):
    """Solve 4x4 linear system Ax = b via partial-pivot Gaussian elimination."""
    m = [list(a[i]) + [b[i]] for i in range(4)]
    for col in range(4):
        piv = max(range(col, 4), key=lambda r: abs(m[r][col]))
        m[col], m[piv] = m[piv], m[col]
        if abs(m[col][col]) < 1e-12:
            return [0.0, 0.0, 0.0, 0.0]
        div = m[col][col]
        for j in range(col, 5):
            m[col][j] /= div
        for r in range(4):
            if r != col:
                fac = m[r][col]
                for j in range(col, 5):
                    m[r][j] -= fac * m[col][j]
    return [m[i][4] for i in range(4)]


def solve_z1_ik(tx: float, ty: float, tz: float, j6: float = 1.57) -> dict:
    """Solve damped least-squares side-pinch IK for gripper_tcp in base_link within Z1 joint limits."""
    # Roll joint6=1.57 places the moving jaw off-center; offset Y by +0.025 m so jaw gap centers on target.
    aim_y = ty + 0.025
    q = [math.atan2(aim_y, max(tx - 0.20, 0.05)), 2.1, -0.95, -0.95]
    lo = [-2.50, 0.10, -2.80, -1.40]
    hi = [2.50, 2.92, -0.10, 1.40]

    def residual(x):
        px, py, pz = fk_tcp_base_link(x[0], x[1], x[2], x[3], 0.0, j6)
        return [
            (px - tx) * 30.0,
            (py - aim_y) * 30.0,
            (pz - tz) * 30.0,
            (x[2] - (-0.95)) * 0.5,
            (x[3] - (-1.00)) * 0.5,
        ]

    for _ in range(100):
        r = residual(q)
        pos_err = math.sqrt((r[0] ** 2 + r[1] ** 2 + r[2] ** 2)) / 30.0
        if pos_err < 1e-4:
            break
        eps = 1e-5
        cols = []
        for k in range(4):
            qk = list(q)
            qk[k] += eps
            rk = residual(qk)
            cols.append([(rk[i] - r[i]) / eps for i in range(5)])
        jtj = [[sum(cols[i][m] * cols[j][m] for m in range(5)) for j in range(4)] for i in range(4)]
        for i in range(4):
            jtj[i][i] += 1e-2
        jtr = [-sum(cols[i][m] * r[m] for m in range(5)) for i in range(4)]
        dq = _solve_4x4(jtj, jtr)
        for i in range(4):
            q[i] = max(lo[i], min(hi[i], q[i] + 0.5 * dq[i]))

    return {
        'joint1': q[0],
        'joint2': q[1],
        'joint3': q[2],
        'joint4': q[3],
        'joint5': 0.0,
        'joint6': j6,
    }


class B1Z1PickPlaceTester(Node):
    """Execute a full pre-grasp -> grasp -> lift -> place sequence via /move_action."""

    def __init__(self) -> None:
        super().__init__(
            'b1z1_pick_place_tester',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
        )
        self._client = ActionClient(self, MoveGroup, '/move_action')
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._joint_positions = {}
        self.create_subscription(JointState, '/joint_states', self._js_cb, 10)

    def _js_cb(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            self._joint_positions[name] = pos

    def wait_ready(self, timeout_sec: float = 30.0) -> bool:
        self.get_logger().info('Waiting for /move_action and TF odom->base_link...')
        if not self._client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().error('/move_action action server did not become available')
            return False

        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._joint_positions and self._tf_buffer.can_transform(
                'base_link', 'odom', rclpy.time.Time()
            ):
                return True
        self.get_logger().error('Timed out waiting for /joint_states or TF odom->base_link')
        return False

    def world_to_base_link(self, wx: float, wy: float, wz: float):
        """Transform an odom-frame coordinate into current base_link frame."""
        tf = self._tf_buffer.lookup_transform(
            'base_link', 'odom', rclpy.time.Time(), timeout=Duration(seconds=2.0)
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

    def _send_goal(self, req: MotionPlanRequest, label: str) -> bool:
        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 2

        self.get_logger().info(f'Planning + executing [{label}]...')
        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=15.0)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f'[{label}] goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)
        wrapped = result_future.result()
        if wrapped is None:
            self.get_logger().error(f'[{label}] timed out waiting for result')
            return False

        code = wrapped.result.error_code.val
        if code != 1:  # moveit_msgs/MoveItErrorCodes.SUCCESS == 1
            self.get_logger().error(f'[{label}] failed with MoveItErrorCode={code}')
            return False

        self.get_logger().info(f'[{label}] SUCCEEDED')
        return True

    def move_joints(self, group: str, targets: dict, label: str, tol: float = 0.005) -> bool:
        req = MotionPlanRequest()
        req.group_name = group
        req.num_planning_attempts = 3
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.5
        req.max_acceleration_scaling_factor = 0.5

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
        return self._send_goal(req, label)

    def spin_pause(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)


def main() -> int:
    parser = argparse.ArgumentParser(description='MoveIt 2 pick-and-place verification for B1Z1')
    parser.add_argument('--box-x', type=float, default=0.73, help='Box world X (default 0.73)')
    parser.add_argument('--box-y', type=float, default=0.00, help='Box world Y (default 0.00)')
    parser.add_argument('--box-z', type=float, default=0.731, help='Box world Z (default 0.731)')
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    tester = B1Z1PickPlaceTester()
    try:
        if not tester.wait_ready(timeout_sec=30.0):
            return 1

        bx, by, bz = tester.world_to_base_link(args.box_x, args.box_y, args.box_z)
        tester.get_logger().info(
            f'Resolved target box in base_link: x={bx:.3f}, y={by:.3f}, z={bz:.3f}'
        )

        q_pre_fold = {'joint1': 0.0, 'joint2': 0.8, 'joint3': -0.95, 'joint4': -0.95, 'joint5': 0.0, 'joint6': 1.57}
        q_pre_grasp = solve_z1_ik(bx - 0.030, by, bz + 0.15)
        q_grasp = solve_z1_ik(bx - 0.030, by, bz + 0.028)
        q_lift = solve_z1_ik(bx - 0.050, by, bz + 0.22)
        q_place = solve_z1_ik(bx - 0.050, by + 0.22, bz + 0.10)
        q_home = {'joint1': 0.0, 'joint2': 0.5, 'joint3': -0.5, 'joint4': 0.0, 'joint5': 0.0, 'joint6': 0.0}

        steps = [
            lambda: tester.move_joints('z1_gripper', {'jointGripper': -1.50}, 'open_gripper', tol=0.08),
            lambda: tester.move_joints('z1_arm', q_pre_fold, 'arm_pre_fold'),
            lambda: tester.move_joints('z1_arm', q_pre_grasp, 'pre_grasp'),
            lambda: tester.move_joints('z1_arm', q_grasp, 'grasp_approach'),
            lambda: tester.move_joints('z1_gripper', {'jointGripper': -0.05}, 'close_gripper', tol=0.02),
        ]

        for fn in steps:
            if not fn():
                return 1
            tester.spin_pause(0.8)

        grip_pos = tester._joint_positions.get('jointGripper', float('nan'))
        tester.get_logger().info(f'Gripper settled position after close: {grip_pos:.3f} rad')

        post_grasp_steps = [
            lambda: tester.move_joints('z1_arm', q_lift, 'lift_box'),
            lambda: tester.move_joints('z1_arm', q_place, 'place_pose'),
            lambda: tester.move_joints('z1_gripper', {'jointGripper': -1.50}, 'release_box', tol=0.08),
            lambda: tester.move_joints('z1_arm', q_home, 'return_home'),
        ]

        for fn in post_grasp_steps:
            if not fn():
                return 1
            tester.spin_pause(0.5)

        tester.get_logger().info('Full MoveIt 2 pick-and-place verification PASSED.')
        return 0
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
