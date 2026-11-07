"""ROS 2 Node for TEMPO Mission Instruction dispatch via Action Client."""

import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import rclpy.utilities
from tempo_msgs.action import PlanMission


def send_mission_goal(node: Node, client: ActionClient, instruction_text: str):
    """
    Send a PlanMission goal and wait synchronously for the result.

    Returns the action result object, or None if rejected.
    """
    goal_msg = PlanMission.Goal()
    goal_msg.mission_instruction = instruction_text

    send_goal_future = client.send_goal_async(goal_msg)
    rclpy.spin_until_future_complete(node, send_goal_future)
    goal_handle = send_goal_future.result()

    if not goal_handle or not goal_handle.accepted:
        node.get_logger().error('Mission goal was rejected by planner node.')
        return None

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)
    action_result = result_future.result()
    return action_result.result if action_result else None


def main():
    """Read mission from terminal arguments, dispatch via Action, and handle response."""
    non_ros = rclpy.utilities.remove_ros_args(sys.argv)
    if len(non_ros) > 1:
        instruction = ' '.join(non_ros[1:]).strip()
    else:
        try:
            instruction = input('Enter mission instruction: ').strip()
        except (KeyboardInterrupt, EOFError):
            sys.exit(0)

    if not instruction or instruction.lower() in ('exit', 'quit'):
        sys.exit(0)

    rclpy.init()
    node = Node('instruction_node')
    client = ActionClient(node, PlanMission, 'plan_mission')

    print('Connecting to plan_mission action server...')
    if not client.wait_for_server(timeout_sec=5.0):
        print('Error: plan_mission action server not available. Is llm_planner_node running?')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    try:
        while instruction:
            result = send_mission_goal(node, client, instruction)
            if not result:
                print('Error: Failed to receive plan result from planner.')
                break

            if result.status == 'ok':
                print('\n==================================================')
                print('[TEMPO] Mission Plan Successful!')
                print(f'  iTax Subclass: {result.itax_subclass}')
                print('  Status:        Dispatched to /mission/plan')
                print('==================================================\n')
                break

            if result.status == 'clarification_needed':
                print(f'\n[Clarification Needed]: {result.reason}')
                try:
                    instruction = input('Enter clarified mission (or "exit" to quit): ').strip()
                except (KeyboardInterrupt, EOFError):
                    break
                if not instruction or instruction.lower() in ('exit', 'quit'):
                    break
                continue

            if result.status == 'infeasible':
                print(f'\n[Mission Infeasible]: {result.reason}')
                try:
                    instruction = input('Enter new mission (or "exit" to quit): ').strip()
                except (KeyboardInterrupt, EOFError):
                    break
                if not instruction or instruction.lower() in ('exit', 'quit'):
                    break
                continue

            # Fallback for unexpected error status
            print(f'\n[Mission Error]: Status {result.status}. Reason: {result.reason}')
            break
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
