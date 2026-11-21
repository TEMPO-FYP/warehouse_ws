"""
Wait until parts of the robot's stack are ready, then exit.

Exits 0 once, under the robot's namespace,
  * a message has arrived on every topic in `topics` (e.g. joint_states and
    the lidar scan, which AMCL and the costmaps need),
  * every controller in `controllers` is active on the controller manager,
  * every lifecycle node in `lifecycle_nodes` (e.g. amcl) is active, and
  * every file in `files` exists,
or exits 1 after `timeout_sec` (wall time). mobile_robot_bringup.launch.py
uses it to start the stack in stages (robot, localization, Nav2, MoveIt +
controller): when
everything starts at once, DDS discovery is flooded and lifecycle service
replies get lost, which leaves Nav2 half-configured.
"""

import os
import sys
import time

from controller_manager_msgs.srv import ListControllers
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message


class WaitForRobot(Node):
    """Track the readiness conditions listed in the parameters."""

    def __init__(self) -> None:
        super().__init__('wait_for_robot')
        self.declare_parameter('topics', [''])
        self.declare_parameter('topic_types', [''])
        self.declare_parameter('controllers', [''])
        self.declare_parameter('controller_manager', 'controller_manager')
        self.declare_parameter('files', [''])
        self.declare_parameter('lifecycle_nodes', [''])
        self.declare_parameter('timeout_sec', 900.0)

        topics = [t for t in self.get_parameter('topics').value if t]
        types = [t for t in self.get_parameter('topic_types').value if t]
        self.controllers = [c for c in self.get_parameter('controllers').value if c]
        self.files = [f for f in self.get_parameter('files').value if f]
        self.timeout = float(self.get_parameter('timeout_sec').value)

        self.pending_topics = set(topics)
        for topic, type_name in zip(topics, types):
            self.create_subscription(
                get_message(type_name), topic,
                lambda _msg, t=topic: self.pending_topics.discard(t),
                qos_profile_sensor_data)
        cm = self.get_parameter('controller_manager').value
        self.list_cli = self.create_client(ListControllers, f'{cm}/list_controllers')
        self.lifecycle_clis = {
            n: self.create_client(GetState, f'{n}/get_state')
            for n in self.get_parameter('lifecycle_nodes').value if n
        }
        self.pending_lifecycle = set(self.lifecycle_clis)

    def _lifecycle_active(self) -> bool:
        for name in sorted(self.pending_lifecycle):
            cli = self.lifecycle_clis[name]
            if not cli.service_is_ready():
                return False
            future = cli.call_async(GetState.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            if not future.done() or future.result() is None:
                return False
            if future.result().current_state.id != State.PRIMARY_STATE_ACTIVE:
                return False
            self.pending_lifecycle.discard(name)
        return True

    def _controllers_active(self) -> bool:
        if not self.controllers:
            return True
        if not self.list_cli.service_is_ready():
            return False
        future = self.list_cli.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done() or future.result() is None:
            return False
        active = {c.name for c in future.result().controller if c.state == 'active'}
        return all(c in active for c in self.controllers)

    def run(self) -> bool:
        deadline = time.monotonic() + self.timeout
        last_log = 0.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=1.0)
            missing_files = [f for f in self.files if not os.path.isfile(f)]
            if (not self.pending_topics and not missing_files
                    and self._controllers_active() and self._lifecycle_active()):
                self.get_logger().info('Ready.')
                return True
            if time.monotonic() - last_log > 15.0:
                last_log = time.monotonic()
                self.get_logger().info(
                    f'Waiting for robot: topics {sorted(self.pending_topics)}, '
                    f'files {missing_files}, controllers {self.controllers}, '
                    f'lifecycle nodes {sorted(self.pending_lifecycle)}')
        self.get_logger().error(
            f'Not ready after {self.timeout:.0f}s: topics {sorted(self.pending_topics)}, '
            f'lifecycle nodes {sorted(self.pending_lifecycle)}.')
        return False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WaitForRobot()
    ok = False
    try:
        ok = node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
