"""
ROS 2 Node for the TEMPO LLM Task Decomposition and Taxonomy Classification pipeline.

Hosts the 'plan_mission' Action Server to receive natural language missions,
performs Stage 1 task decomposition into an atomic DAG and Stage 2 iTax taxonomy
classification, and publishes the validated plan payload to '/mission/plan'.
"""

import json

from llm_planner.orchestrator import MissionPlanResult, orchestrate_mission
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from std_msgs.msg import String
from tempo_msgs.action import PlanMission


class LLMPlannerNode(Node):
    """
    ROS 2 Node for TEMPO Task Decomposition and Taxonomy Classification.

    Bridges natural language instructions to mathematical task allocation solvers.
    """

    def __init__(self):
        super().__init__('llm_planner_node')

        # Action server for interactive mission planning
        self._action_server = ActionServer(
            self,
            PlanMission,
            'plan_mission',
            self.execute_mission_goal,
        )

        # Output topic: publishes validated plan DAG and iTax subclass for the Solver node
        self.pub_plan = self.create_publisher(
            String,
            '/mission/plan',
            10,
        )

        self.get_logger().info(
            'TEMPO LLM Planner Node initialized successfully.\n'
            "  -> Action Server:  'plan_mission' (PlanMission)\n"
            "  -> Publishing to:  '/mission/plan'"
        )

    def execute_mission_goal(self, goal_handle):
        """Execute the 'plan_mission' action goal."""
        instruction = goal_handle.request.mission_instruction.strip()
        self.get_logger().info(f"Received mission instruction goal: '{instruction}'")

        result = PlanMission.Result()

        if not instruction:
            result.status = 'error'
            result.reason = 'Empty mission instruction received.'
            result.errors = ['Empty instruction string']
            goal_handle.abort()
            return result

        # Execute Stage 1 (Decomposition) & Stage 2 (Taxonomy Classification)
        try:
            mission_res: MissionPlanResult = orchestrate_mission(instruction)

            result.status = mission_res.status
            result.itax_subclass = mission_res.itax_subclass or ''
            result.errors = mission_res.errors or []

            if mission_res.plan and isinstance(mission_res.plan, dict):
                result.reason = mission_res.plan.get('reason', '')
                result.plan_json = json.dumps(mission_res.plan, indent=2)
            else:
                result.reason = ''
                result.plan_json = ''

            # Publish validated plan to /mission/plan for downstream solver if ok
            if mission_res.status == 'ok':
                self._publish_plan(mission_res)

            if mission_res.status.startswith('failed'):
                goal_handle.abort()
            else:
                goal_handle.succeed()

            return result
        except Exception as e:
            self.get_logger().error(f'Error executing mission goal: {e}')
            result.status = 'error'
            result.reason = str(e)
            result.errors = [str(e)]
            goal_handle.abort()
            return result

    def _publish_plan(self, result: MissionPlanResult):
        """Serialize and publish the plan result to '/mission/plan'."""
        payload = {
            'status': result.status,
            'itax_subclass': result.itax_subclass,
            'axes': result.axes,
            'taxonomy_cot': result.taxonomy_cot,
            'plan': result.plan,
            'errors': result.errors,
            'attempts': {
                'decomposition': result.decomposition_attempts,
                'taxonomy': result.taxonomy_attempts,
            },
        }

        msg = String()
        msg.data = json.dumps(payload, indent=2)
        self.pub_plan.publish(msg)

        self.get_logger().info(
            f"Published mission plan to '/mission/plan':\n"
            f'  - Status:        {result.status}\n'
            f'  - iTax Subclass: {result.itax_subclass}\n'
            f"  - Subtasks:      {len(result.plan.get('subtasks', [])) if result.plan else 0}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = LLMPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down LLM Planner Node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
