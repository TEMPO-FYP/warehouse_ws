"""
Make sure a robot's ros2_control controllers end up active.

Clearpath's controller spawners give up after 10 s per controller switch and
share a system-wide lock, so when the simulation runs well below real time
(Gazebo on CPU) they time out and leave the wheels and arm without
controllers. This node does not use that lock: it waits for the controller
manager, then loads, configures and activates whichever of the listed
controllers is not active yet, retrying until all are active or the timeout
(wall time) runs out. Controllers that Clearpath's spawners already brought up
are left alone.

ROS parameters
--------------
  controller_manager (str):  e.g. /a200_0000/controller_manager
  controllers (str[]):       in activation order
  timeout_sec (float):       give up after this long (wall time)
  switch_timeout_sec (float): per-activation timeout passed to the manager

"""

import sys
import time

from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import (
    ConfigureController,
    ListControllers,
    LoadController,
    SwitchController,
)
import rclpy
from rclpy.node import Node

SERVICE_CALL_TIMEOUT_SEC = 60.0
RETRY_PERIOD_SEC = 2.0


class EnsureControllers(Node):
    """Load, configure and activate the listed controllers until all are active."""

    def __init__(self) -> None:
        super().__init__('ensure_controllers')
        self.declare_parameter('controller_manager', '/controller_manager')
        self.declare_parameter('controllers', [''])
        self.declare_parameter('timeout_sec', 600.0)
        self.declare_parameter('switch_timeout_sec', 60.0)

        cm = self.get_parameter('controller_manager').value.rstrip('/')
        self.controllers = [c for c in self.get_parameter('controllers').value if c]
        self.timeout = float(self.get_parameter('timeout_sec').value)
        self.switch_timeout = float(self.get_parameter('switch_timeout_sec').value)

        self.list_cli = self.create_client(ListControllers, f'{cm}/list_controllers')
        self.load_cli = self.create_client(LoadController, f'{cm}/load_controller')
        self.configure_cli = self.create_client(
            ConfigureController, f'{cm}/configure_controller')
        self.switch_cli = self.create_client(SwitchController, f'{cm}/switch_controller')

    def _call(self, client, request):
        """Call a service and wait for the response; None on timeout."""
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=SERVICE_CALL_TIMEOUT_SEC)
        return future.result() if future.done() else None

    def _states(self):
        """Return {controller name: state} from the controller manager, or None."""
        response = self._call(self.list_cli, ListControllers.Request())
        if response is None:
            return None
        return {c.name: c.state for c in response.controller}

    def _advance(self, name: str, state) -> None:
        """Move one controller a step towards 'active'."""
        if state is None:
            ok = self._call(self.load_cli, LoadController.Request(name=name))
            self.get_logger().info(f'Load {name}: {ok.ok if ok else "timed out"}')
        elif state == 'unconfigured':
            ok = self._call(self.configure_cli, ConfigureController.Request(name=name))
            self.get_logger().info(f'Configure {name}: {ok.ok if ok else "timed out"}')
        elif state == 'inactive':
            request = SwitchController.Request(
                activate_controllers=[name],
                strictness=SwitchController.Request.BEST_EFFORT,
                activate_asap=True,
                timeout=Duration(sec=int(self.switch_timeout)),
            )
            ok = self._call(self.switch_cli, request)
            self.get_logger().info(f'Activate {name}: {ok.ok if ok else "timed out"}')

    def run(self) -> bool:
        """Retry until every listed controller is active; False on timeout."""
        deadline = time.monotonic() + self.timeout
        self.get_logger().info(f'Ensuring controllers are active: {self.controllers}')
        while time.monotonic() < deadline:
            if not self.list_cli.wait_for_service(timeout_sec=5.0):
                continue
            states = self._states()
            if states is None:
                continue
            pending = [c for c in self.controllers if states.get(c) != 'active']
            if not pending:
                self.get_logger().info(f'All controllers active: {self.controllers}')
                return True
            # One step at a time, in order, then re-read the states
            self._advance(pending[0], states.get(pending[0]))
            time.sleep(RETRY_PERIOD_SEC)
        self.get_logger().error(
            f'Timed out after {self.timeout:.0f}s; not active: '
            f'{[c for c in self.controllers if (self._states() or {}).get(c) != "active"]}'
        )
        return False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EnsureControllers()
    try:
        ok = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
