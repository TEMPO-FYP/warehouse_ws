"""
planner_node.py

ROS 2 Node for the TEMPO LLM Task Decomposition and Taxonomy Classification pipeline.
Subscribes to '/mission/instruction' to receive natural language missions from the Instruction node,
performs Stage 1 task decomposition into an atomic DAG and Stage 2 iTax taxonomy classification,
and publishes the validated plan payload to '/mission/plan' for the downstream Solver node.
"""

import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from llm_planner.orchestrator import orchestrate_mission, MissionPlanResult


class LLMPlannerNode(Node):
    """
    ROS 2 Node for TEMPO Task Decomposition and Taxonomy Classification.
    Bridges natural language instructions to mathematical task allocation solvers.
    """

    def __init__(self):
        super().__init__("llm_planner_node")

        # Input topic: receives natural language missions from the Instruction node
        self.sub_instruction = self.create_subscription(
            String,
            "/mission/instruction",
            self.on_instruction_received,
            10,
        )

        # Output topic: publishes validated plan DAG and iTax subclass for the Solver node
        self.pub_plan = self.create_publisher(
            String,
            "/mission/plan",
            10,
        )

        self.get_logger().info(
            "TEMPO LLM Planner Node initialized successfully.\n"
            "  -> Subscribing to: '/mission/instruction'\n"
            "  -> Publishing to:  '/mission/plan'"
        )

    def on_instruction_received(self, msg: String):
        """Callback triggered when an instruction is published to '/mission/instruction'."""
        instruction = msg.data.strip()
        if not instruction:
            self.get_logger().warn("Received empty instruction on '/mission/instruction'. Ignoring.")
            return

        self.get_logger().info(f"Received mission instruction: '{instruction}'")

        # Execute Stage 1 (Decomposition) & Stage 2 (Taxonomy Classification)
        result: MissionPlanResult = orchestrate_mission(instruction)

        # Publish the plan to /mission/plan
        self._publish_plan(result)

    def _publish_plan(self, result: MissionPlanResult):
        """Serializes and publishes the plan result to '/mission/plan'."""
        payload = {
            "status": result.status,
            "itax_subclass": result.itax_subclass,
            "axes": result.axes,
            "taxonomy_cot": result.taxonomy_cot,
            "plan": result.plan,
            "errors": result.errors,
            "attempts": {
                "decomposition": result.decomposition_attempts,
                "taxonomy": result.taxonomy_attempts,
            },
        }

        msg = String()
        msg.data = json.dumps(payload, indent=2)
        self.pub_plan.publish(msg)

        self.get_logger().info(
            f"Published mission plan to '/mission/plan':\n"
            f"  - Status:        {result.status}\n"
            f"  - iTax Subclass: {result.itax_subclass}\n"
            f"  - Subtasks:      {len(result.plan.get('subtasks', [])) if result.plan else 0}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = LLMPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down LLM Planner Node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
