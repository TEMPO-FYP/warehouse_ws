#!/usr/bin/env python3
"""
GripperInterface: ROS 2 client for Robotiq 2F-85 gripper control and stall detection.

Sends JointTrajectory commands to `/gripper_controller/joint_trajectory` and
monitors `/joint_states` to verify open/close completion and contact-based
grasp stalls on target objects.
"""

from collections import deque
import threading
import time
from typing import Deque, Optional, Tuple

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


GRIPPER_JOINT_NAME = "robotiq_85_left_knuckle_joint"
GRIPPER_OPEN_POSITION = 0.00
GRIPPER_CLOSE_POSITION = 0.80


class GripperInterface:
    """Controls the Robotiq 2F-85 gripper and detects contact stalls via /joint_states."""

    def __init__(
        self,
        node: Node,
        trajectory_topic: str = "/gripper_controller/joint_trajectory",
        action_name: str = "/gripper_controller/follow_joint_trajectory",
        joint_states_topic: str = "/joint_states",
        joint_name: str = GRIPPER_JOINT_NAME,
        spin_locally: bool = True,
    ) -> None:
        self.node = node
        self.logger = node.get_logger()
        self.joint_name = joint_name
        self.spin_locally = spin_locally

        self._lock = threading.Lock()
        self._current_position: Optional[float] = None
        self._current_velocity: Optional[float] = None
        self._history: Deque[Tuple[float, float]] = deque(maxlen=100)
        self._have_state = False

        self._traj_pub = self.node.create_publisher(
            JointTrajectory,
            trajectory_topic,
            10,
        )
        self._action_client = ActionClient(
            self.node,
            FollowJointTrajectory,
            action_name,
        )
        self._joint_state_sub = self.node.create_subscription(
            JointState,
            joint_states_topic,
            self._joint_state_callback,
            20,
        )

    def _sim_now_sec(self) -> float:
        """Return node clock timestamp in seconds (uses /clock when use_sim_time=true)."""
        return self.node.get_clock().now().nanoseconds / 1e9

    def _joint_state_callback(self, msg: JointState) -> None:
        if self.joint_name not in msg.name:
            return
        idx = msg.name.index(self.joint_name)
        if idx >= len(msg.position):
            return

        pos = float(msg.position[idx])
        vel = float(msg.velocity[idx]) if idx < len(msg.velocity) else 0.0
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp_sec <= 0.0:
            stamp_sec = time.monotonic()

        with self._lock:
            self._current_position = pos
            self._current_velocity = vel
            self._history.append((stamp_sec, pos))
            self._have_state = True

    def _step_wait(self, dt_sec: float = 0.02) -> None:
        if self.spin_locally:
            rclpy.spin_once(self.node, timeout_sec=dt_sec)
        else:
            time.sleep(dt_sec)

    def wait_for_joint_state(self, timeout_sec: float = 5.0) -> bool:
        """Wait until at least one /joint_states sample containing the gripper joint arrives."""
        wall_deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < wall_deadline:
            with self._lock:
                if self._have_state:
                    return True
            self._step_wait(0.02)
        self.logger.error(
            f"Timed out waiting for '{self.joint_name}' in /joint_states."
        )
        return False

    @property
    def position(self) -> Optional[float]:
        """Return the latest gripper knuckle joint position in radians."""
        with self._lock:
            return self._current_position

    @property
    def velocity(self) -> Optional[float]:
        """Return the latest gripper knuckle joint velocity in rad/s."""
        with self._lock:
            return self._current_velocity

    def get_effective_velocity(self, window_sec: float = 0.25) -> Optional[float]:
        """
        Estimate smoothed joint velocity over a recent time window to reject
        single-step simulation noise.
        """
        with self._lock:
            if not self._have_state or self._current_velocity is None:
                return None
            if len(self._history) < 2:
                return abs(self._current_velocity)
            t_latest, q_latest = self._history[-1]
            q_prev = q_latest
            t_prev = t_latest
            for t_sample, q_sample in reversed(self._history):
                if t_latest - t_sample >= window_sec:
                    t_prev = t_sample
                    q_prev = q_sample
                    break
                t_prev = t_sample
                q_prev = q_sample

            dt = t_latest - t_prev
            if dt <= 1e-4:
                return abs(self._current_velocity)
            return abs(q_latest - q_prev) / dt

    def send_position_command(
        self,
        target_position: float,
        duration_sec: float = 1.0,
    ) -> None:
        """Publish a single-point JointTrajectory command to /gripper_controller/joint_trajectory."""
        clamped = max(0.0, min(0.8, float(target_position)))
        msg = JointTrajectory()
        msg.joint_names = [self.joint_name]

        point = JointTrajectoryPoint()
        point.positions = [clamped]
        point.velocities = [0.0]
        sec = int(duration_sec)
        nanosec = int((duration_sec - sec) * 1e9)
        point.time_from_start = Duration(sec=sec, nanosec=nanosec)
        msg.points.append(point)

        self._traj_pub.publish(msg)
        self.logger.info(
            f"Published gripper trajectory target q={clamped:.3f} rad (duration={duration_sec:.2f}s)."
        )

    def _wait_elapsed(self, sim_duration: float, wall_timeout: float) -> None:
        """Wait for `sim_duration` seconds on the ROS clock, bounded by `wall_timeout`."""
        sim_start = self._sim_now_sec()
        wall_deadline = time.monotonic() + wall_timeout
        while rclpy.ok() and time.monotonic() < wall_deadline:
            self._step_wait(0.02)
            if sim_start > 0.0 and (self._sim_now_sec() - sim_start) >= sim_duration:
                return

    def open(
        self,
        position: float = GRIPPER_OPEN_POSITION,
        duration_sec: float = 1.0,
        wait: bool = True,
        tolerance: float = 0.06,
        timeout_sec: float = 4.0,
    ) -> bool:
        """
        Command the gripper open (q = 0.00 rad -> ~85 mm aperture).

        :return: True if the command was sent and (when wait=True) the gripper opened.
        """
        if not self.wait_for_joint_state(timeout_sec=3.0):
            return False

        self.send_position_command(position, duration_sec=duration_sec)
        if not wait:
            return True

        sim_start = self._sim_now_sec()
        wall_deadline = time.monotonic() + max(timeout_sec, duration_sec + 2.0)
        while rclpy.ok() and time.monotonic() < wall_deadline:
            self._step_wait(0.02)
            q_now = self.position
            v_eff = self.get_effective_velocity(window_sec=0.15)
            elapsed_sim = self._sim_now_sec() - sim_start if sim_start > 0.0 else 0.0
            if (
                q_now is not None
                and abs(q_now - position) <= tolerance
                and (v_eff is None or v_eff < 0.05)
                and elapsed_sim >= min(0.3, duration_sec * 0.5)
            ):
                self.logger.info(f"Gripper opened: q={q_now:.4f} rad.")
                return True

        q_final = self.position
        self.logger.warn(
            f"Gripper open timed out waiting for q~={position:.3f}; final q={q_final}."
        )
        return q_final is not None and abs(q_final - position) <= (tolerance + 0.05)

    def verify_grasp(
        self,
        min_pos: float = 0.30,
        max_pos: float = 0.68,
        nominal_min_pos: float = 0.35,
        nominal_max_pos: float = 0.65,
        vel_threshold: float = 0.01,
        settle_duration_sec: float = 0.25,
        timeout_sec: float = 6.0,
    ) -> bool:
        """
        Verify a physical object grasp using stall detection on /joint_states.

        A valid grasp on a 50 mm cube stalls before full closure (0.80 rad) with:
        1. |dq/dt| < vel_threshold (default 0.01 rad/s)
        2. min_pos <= q_final <= max_pos (nominal 0.35..0.65 rad)
        """
        if not self.wait_for_joint_state(timeout_sec=2.0):
            return False

        wall_deadline = time.monotonic() + timeout_sec
        stall_start_wall: Optional[float] = None

        while rclpy.ok() and time.monotonic() < wall_deadline:
            self._step_wait(0.02)
            q_now = self.position
            v_eff = self.get_effective_velocity(window_sec=0.20)
            if q_now is None or v_eff is None:
                continue

            if v_eff < vel_threshold and min_pos <= q_now <= max_pos:
                if stall_start_wall is None:
                    stall_start_wall = time.monotonic()
                elif (time.monotonic() - stall_start_wall) >= settle_duration_sec:
                    in_nominal = nominal_min_pos <= q_now <= nominal_max_pos
                    band_note = (
                        "within nominal [0.35, 0.65] rad"
                        if in_nominal
                        else f"within compliant [{min_pos:.2f}, {max_pos:.2f}] rad"
                    )
                    self.logger.info(
                        f"Grasp VERIFIED (stall detected): q_final={q_now:.4f} rad "
                        f"({band_note}), |dq/dt|={v_eff:.5f} rad/s < {vel_threshold} rad/s."
                    )
                    return True
            else:
                stall_start_wall = None

        q_final = self.position
        v_final = self.get_effective_velocity(window_sec=0.20)
        self.logger.error(
            f"Grasp verification FAILED: q_final={q_final}, |dq/dt|={v_final} "
            f"(expected {min_pos:.2f}..{max_pos:.2f} rad and |dq/dt| < {vel_threshold} rad/s)."
        )
        return False

    def close(
        self,
        position: float = GRIPPER_CLOSE_POSITION,
        duration_sec: float = 1.2,
        wait: bool = True,
        verify_stall: bool = True,
        min_pos: float = 0.30,
        max_pos: float = 0.68,
        vel_threshold: float = 0.01,
    ) -> bool:
        """
        Command the gripper closed (q_cmd = 0.80 rad) and optionally verify contact stall.

        :return: True if the command succeeded (and stall check passed when verify_stall=True).
        """
        if not self.wait_for_joint_state(timeout_sec=3.0):
            return False

        self.send_position_command(position, duration_sec=duration_sec)
        if not wait:
            return True

        # Allow trajectory duration to elapse before checking stall settlement
        self._wait_elapsed(sim_duration=max(0.6, duration_sec * 0.85), wall_timeout=duration_sec + 3.0)

        if verify_stall:
            return self.verify_grasp(
                min_pos=min_pos,
                max_pos=max_pos,
                vel_threshold=vel_threshold,
                timeout_sec=4.0,
            )

        wall_deadline = time.monotonic() + 3.0
        while rclpy.ok() and time.monotonic() < wall_deadline:
            self._step_wait(0.02)
            q_now = self.position
            v_eff = self.get_effective_velocity(window_sec=0.20)
            if q_now is not None and v_eff is not None and v_eff < vel_threshold:
                self.logger.info(f"Gripper closed to q={q_now:.4f} rad.")
                return True

        self.logger.warn(f"Gripper close settled at q={self.position}.")
        return True


def main(args=None) -> None:
    """Standalone CLI smoke check for GripperInterface open/close cycle."""
    rclpy.init(args=args)
    node = Node("gripper_interface_test")
    gripper = GripperInterface(node, spin_locally=True)

    try:
        if not gripper.wait_for_joint_state(timeout_sec=5.0):
            return

        node.get_logger().info(f"Initial gripper joint q={gripper.position:.4f} rad")
        node.get_logger().info("Testing open()...")
        gripper.open(duration_sec=1.0, wait=True)

        node.get_logger().info("Testing close(verify_stall=False)...")
        gripper.close(duration_sec=1.2, wait=True, verify_stall=False)
        node.get_logger().info(f"Closed empty gripper q={gripper.position:.4f} rad")

        node.get_logger().info("Re-opening gripper...")
        gripper.open(duration_sec=1.0, wait=True)
        node.get_logger().info("GripperInterface smoke test complete.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
