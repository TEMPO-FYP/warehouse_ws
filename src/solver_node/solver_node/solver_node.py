"""ROS 2 Node for the TEMPO Task Allocation Solver."""

import json
from typing import Any, Dict, List, Optional

import rclpy
from rclpy.node import Node
from solver_node.fleet_loader import get_default_fleet_robots
from solver_node.naive_solver import allocate_subtasks
from std_msgs.msg import String


class SolverNode(Node):
    """
    ROS 2 Node for TEMPO Task Allocation Solver.

    Bridges decomposed subtask DAG plans to execution dispatching by assigning
    capable robot resources to each subtask.
    """

    def __init__(self, fleet_robots: Optional[List[str]] = None):
        """Initialize SolverNode subscriptions, publishers, and fleet."""
        super().__init__('solver_node')

        self.fleet_robots = (
            fleet_robots
            if fleet_robots is not None
            else get_default_fleet_robots()
        )

        # Input topic: receives validated plan DAG and iTax subclass from LLM planner
        self.sub_plan = self.create_subscription(
            String,
            '/mission/plan',
            self.on_plan_received,
            10,
        )

        # Output topic: publishes assigned plan for Dispatcher node
        self.pub_assigned_plan = self.create_publisher(
            String,
            '/mission/assigned_plan',
            10,
        )

        self.get_logger().info(
            'TEMPO Solver Node initialized successfully.\n'
            "  -> Subscribing to: '/mission/plan'\n"
            "  -> Publishing to:  '/mission/assigned_plan'\n"
            f"  -> Fleet:          {', '.join(self.fleet_robots)}"
        )

    def on_plan_received(self, msg: String) -> None:
        """Process an unassigned plan published to '/mission/plan'."""
        raw = msg.data.strip()
        if not raw:
            return

        try:
            payload: Dict[str, Any] = json.loads(raw)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f'Failed to parse incoming plan JSON: {exc}')
            return

        status = payload.get('status')
        if status != 'ok':
            self.get_logger().info(
                f"Ignoring plan with status '{status}' (only 'ok' plans are allocated)."
            )
            return

        plan_dict = payload.get('plan')
        if not isinstance(plan_dict, dict):
            plan_dict = payload

        subtasks = plan_dict.get('subtasks', [])
        if not subtasks:
            self.get_logger().warn('Incoming plan has no subtasks; ignoring.')
            return

        plan_id = (
            plan_dict.get('plan_id')
            or payload.get('plan_id')
            or payload.get('id')
            or 'unknown_plan'
        )
        itax_subclass = payload.get('itax_subclass', 'ND[ST-SR-IA]')

        # Run allocation
        assigned_subtasks = allocate_subtasks(subtasks, self.fleet_robots)

        # Format assigned plan envelope for dispatcher_node
        assigned_plan_payload = {
            'id': plan_id,
            'plan_id': plan_id,
            'itax_subclass': itax_subclass,
            'output': {
                'status': 'ok',
                'solver_method': 'naive_chain_allocation',
                'subtasks': assigned_subtasks,
            },
        }

        out_msg = String()
        out_msg.data = json.dumps(assigned_plan_payload, indent=2)
        self.pub_assigned_plan.publish(out_msg)

        unique_robots = sorted(
            {t.get('robot') for t in assigned_subtasks if t.get('robot')}
        )
        self.get_logger().info(
            f"Allocated and published assigned plan '{plan_id}' to '/mission/assigned_plan':\n"
            f'  - Subtasks:    {len(assigned_subtasks)}\n'
            f'  - Subclass:    {itax_subclass}\n'
            f"  - Fleet used:  {', '.join(unique_robots)}"
        )


def main(args=None):
    """Run SolverNode entrypoint."""
    rclpy.init(args=args)
    node = SolverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Solver Node...')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
