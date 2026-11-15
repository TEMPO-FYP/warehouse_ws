#!/usr/bin/env python3
"""Interactive Keyboard Teleoperation for BCR Arm and Robotiq Gripper in Gazebo."""

import math
import select
import sys
import termios
import threading
import time
import tty

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM_JOINTS = [
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
    "joint7",
]

GRIPPER_JOINT = "robotiq_85_left_knuckle_joint"

JOINT_LIMITS = {
    "joint1": (-6.28, 6.28),
    "joint2": (-2.0, 2.0),
    "joint3": (-6.28, 6.28),
    "joint4": (-2.0, 2.0),
    "joint5": (-6.28, 6.28),
    "joint6": (-2.0, 2.0),
    "joint7": (-6.28, 6.28),
    GRIPPER_JOINT: (0.0, 0.8),
}

DEFAULT_STEP = 0.05  # radians (~2.8 degrees)
LARGE_STEP = 0.15    # radians (~8.6 degrees)

HELP_MSG = """
======================================================
     BCR Arm + Gripper Keyboard Teleoperation
======================================================
  Select Joint:
    [1] to [7]  : Select joint 1 through 7
  
  Jog Selected Joint:
    w / s       : Jog + / - by 0.05 rad (~2.8 deg)
    W / S       : Jog + / - by 0.15 rad (~8.6 deg)
  
  Gripper Control:
    o           : Open gripper (0.0)
    c           : Close gripper (0.8)
    [ / ]       : Nudge gripper open / close by 0.05
  
  Presets:
    h           : Home pose (upright [0,0,0,0,0,0,0])
    p           : Pick-ready pose (bent forward to shelf)
  
  General:
    ?           : Redisplay this help screen
    q           : Quit
======================================================
"""


class ArmTeleopNode(Node):
    def __init__(self):
        super().__init__("teleop_arm")

        self.arm_pub = self.create_publisher(
            JointTrajectory,
            "/joint_trajectory_controller/joint_trajectory",
            10,
        )

        self.gripper_pub = self.create_publisher(
            JointTrajectory,
            "/gripper_controller/joint_trajectory",
            10,
        )

        self.joint_states_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self._joint_state_callback,
            10,
        )

        self.current_arm_positions = [0.0] * len(ARM_JOINTS)
        self.target_arm_positions = [0.0] * len(ARM_JOINTS)
        self.current_gripper_pos = 0.0
        self.target_gripper_pos = 0.0

        self.have_joint_states = False
        self.active_joint_idx = 1  # 0-indexed, default joint2 (index 1)
        self.lock = threading.Lock()

    def _joint_state_callback(self, msg: JointState):
        with self.lock:
            name_to_pos = dict(zip(msg.name, msg.position))

            all_arm_found = True
            for i, name in enumerate(ARM_JOINTS):
                if name in name_to_pos:
                    self.current_arm_positions[i] = name_to_pos[name]
                else:
                    all_arm_found = False

            if GRIPPER_JOINT in name_to_pos:
                self.current_gripper_pos = name_to_pos[GRIPPER_JOINT]

            if not self.have_joint_states and all_arm_found:
                self.target_arm_positions = list(self.current_arm_positions)
                self.target_gripper_pos = self.current_gripper_pos
                self.have_joint_states = True

    def send_arm_command(self, positions, duration_sec=0.5):
        msg = JointTrajectory()
        msg.joint_names = ARM_JOINTS

        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start = Duration(
            sec=int(duration_sec),
            nanosec=int((duration_sec % 1) * 1e9),
        )
        msg.points.append(point)
        self.arm_pub.publish(msg)

    def send_gripper_command(self, position, duration_sec=1.0):
        msg = JointTrajectory()
        msg.joint_names = [GRIPPER_JOINT]

        point = JointTrajectoryPoint()
        point.positions = [float(position)]
        point.time_from_start = Duration(
            sec=int(duration_sec),
            nanosec=int((duration_sec % 1) * 1e9),
        )
        msg.points.append(point)
        self.gripper_pub.publish(msg)

    def jog_active_joint(self, delta):
        with self.lock:
            name = ARM_JOINTS[self.active_joint_idx]
            min_val, max_val = JOINT_LIMITS[name]
            new_val = self.target_arm_positions[self.active_joint_idx] + delta
            new_val = max(min_val, min(max_val, new_val))
            self.target_arm_positions[self.active_joint_idx] = new_val
            positions_to_send = list(self.target_arm_positions)

        self.send_arm_command(positions_to_send, duration_sec=0.4)

    def nudge_gripper(self, delta):
        with self.lock:
            min_val, max_val = JOINT_LIMITS[GRIPPER_JOINT]
            new_val = self.target_gripper_pos + delta
            new_val = max(min_val, min(max_val, new_val))
            self.target_gripper_pos = new_val

        self.send_gripper_command(new_val, duration_sec=0.5)

    def set_gripper(self, target_val, duration_sec=1.2):
        with self.lock:
            self.target_gripper_pos = target_val
        self.send_gripper_command(target_val, duration_sec=duration_sec)

    def go_home(self):
        with self.lock:
            self.target_arm_positions = [0.0] * len(ARM_JOINTS)
            positions = list(self.target_arm_positions)
        self.send_arm_command(positions, duration_sec=2.5)

    def go_pick_ready(self):
        # Convenient pose leaning forward towards the shelf at x=0.55
        ready_pose = [0.0, 0.45, 0.0, -0.90, 0.0, 0.45, 0.0]
        with self.lock:
            self.target_arm_positions = list(ready_pose)
        self.send_arm_command(ready_pose, duration_sec=3.0)


def get_key(settings, timeout=0.1):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ""
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def print_status(node: ArmTeleopNode):
    with node.lock:
        active_name = ARM_JOINTS[node.active_joint_idx]
        angles_deg = [math.degrees(a) for a in node.target_arm_positions]
        gripper_val = node.target_gripper_pos

    status_lines = [
        f"\r[Active Joint: {active_name.upper()} (idx {node.active_joint_idx+1})]",
        f"  J1: {angles_deg[0]:+6.1f}° | J2: {angles_deg[1]:+6.1f}° | J3: {angles_deg[2]:+6.1f}° | J4: {angles_deg[3]:+6.1f}°",
        f"  J5: {angles_deg[4]:+6.1f}° | J6: {angles_deg[5]:+6.1f}° | J7: {angles_deg[6]:+6.1f}° | Gripper: {gripper_val:.2f}",
        "  Command (w/s=jog, o=open, c=close, h=home, p=ready, q=quit): ",
    ]
    sys.stdout.write("\033[2K" + "\n".join(status_lines))
    # Move cursor back up 3 lines for next update
    sys.stdout.write(f"\033[3A\r")
    sys.stdout.flush()


def main(args=None):
    rclpy.init(args=args)
    node = ArmTeleopNode()

    # Spin ROS 2 in a background thread so callbacks update smoothly
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    sys.stdout.write("Waiting for joint states from Gazebo...")
    sys.stdout.flush()
    while rclpy.ok() and not node.have_joint_states:
        time.sleep(0.1)
    sys.stdout.write(" Connected!\n")

    print(HELP_MSG)

    old_settings = termios.tcgetattr(sys.stdin)

    try:
        while rclpy.ok():
            print_status(node)
            key = get_key(old_settings, timeout=0.1)

            if not key:
                continue

            if key in ["1", "2", "3", "4", "5", "6", "7"]:
                node.active_joint_idx = int(key) - 1
            elif key == "w":
                node.jog_active_joint(+DEFAULT_STEP)
            elif key == "s":
                node.jog_active_joint(-DEFAULT_STEP)
            elif key == "W":
                node.jog_active_joint(+LARGE_STEP)
            elif key == "S":
                node.jog_active_joint(-LARGE_STEP)
            elif key == "o":
                node.set_gripper(0.0)
            elif key == "c":
                node.set_gripper(0.8)
            elif key == "[":
                node.nudge_gripper(-0.05)
            elif key == "]":
                node.nudge_gripper(+0.05)
            elif key == "h":
                node.go_home()
            elif key == "p":
                node.go_pick_ready()
            elif key == "?":
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
                print("\n" + HELP_MSG)
            elif key == "q" or key == "\x03":  # q or Ctrl+C
                break

    except Exception as e:
        print(f"\nError in teleop loop: {e}")
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        print("\nExiting teleoperation...")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
