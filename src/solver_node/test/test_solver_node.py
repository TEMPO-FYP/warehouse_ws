"""Unit tests for TEMPO Solver ROS 2 Node."""

import json
import unittest
from unittest.mock import MagicMock

import rclpy
from solver_node.solver_node import SolverNode
from std_msgs.msg import String


class TestSolverNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.node = SolverNode()
        self.node.pub_assigned_plan = MagicMock()

    def tearDown(self):
        self.node.destroy_node()

    def test_on_plan_received_valid(self):
        plan_input = {
            'status': 'ok',
            'itax_subclass': 'ND[ST-SR-IA]',
            'fleet': {
                'mobile_robot_1': {
                    'robot_type': 'mobile_robot',
                    'capabilities': ['navigation', 'manipulation'],
                    'skills': ['navigate_to', 'pick', 'place'],
                    'specs': {'payload_capacity_kg': 10.0, 'speed_m_s': 1.0},
                },
                'mobile_robot_2': {
                    'robot_type': 'mobile_robot',
                    'capabilities': ['navigation', 'manipulation'],
                    'skills': ['navigate_to', 'pick', 'place'],
                    'specs': {'payload_capacity_kg': 10.0, 'speed_m_s': 1.0},
                },
            },
            'plan': {
                'plan_id': 'plan_001',
                'status': 'ok',
                'subtasks': [
                    {
                        'id': 'T1',
                        'description': 'Navigate to shelf A.',
                        'required_skills': ['navigate_to'],
                        'args': {'location': 'shelf_A'},
                        'dependencies': [],
                    },
                    {
                        'id': 'T2',
                        'description': 'Pick up box at shelf A.',
                        'required_skills': ['pick'],
                        'args': {'object': 'box_A'},
                        'dependencies': ['T1'],
                    },
                    {
                        'id': 'T3',
                        'description': 'Navigate to shelf B.',
                        'required_skills': ['navigate_to'],
                        'args': {'location': 'shelf_B'},
                        'dependencies': [],
                    },
                ],
            },
        }

        msg = String()
        msg.data = json.dumps(plan_input)
        self.node.on_plan_received(msg)

        self.node.pub_assigned_plan.publish.assert_called_once()
        published_msg = self.node.pub_assigned_plan.publish.call_args[0][0]
        payload = json.loads(published_msg.data)

        # Verify envelope structure expected by dispatcher_node
        assert payload['id'] == 'plan_001'
        assert payload['itax_subclass'] == 'ND[ST-SR-IA]'
        assert payload['output']['status'] == 'ok'
        assert payload['output']['solver_method'] == 'naive_chain_allocation'

        subtasks = payload['output']['subtasks']
        assert len(subtasks) == 3
        # T1 and T2 are in Chain 1 -> mobile_robot_1
        assert subtasks[0]['robot'] == 'mobile_robot_1'
        assert subtasks[1]['robot'] == 'mobile_robot_1'
        # T3 is in Chain 2 -> mobile_robot_2
        assert subtasks[2]['robot'] == 'mobile_robot_2'

    def test_on_plan_received_non_ok_status(self):
        plan_input = {
            'status': 'clarification_needed',
            'reason': 'Unresolved location.',
        }

        msg = String()
        msg.data = json.dumps(plan_input)
        self.node.on_plan_received(msg)

        # Should not publish anything for non-ok status
        self.node.pub_assigned_plan.publish.assert_not_called()

    def test_on_plan_received_empty_string(self):
        msg = String()
        msg.data = ''
        self.node.on_plan_received(msg)
        self.node.pub_assigned_plan.publish.assert_not_called()

    def test_on_plan_received_invalid_json(self):
        msg = String()
        msg.data = '{invalid_json'
        self.node.on_plan_received(msg)
        self.node.pub_assigned_plan.publish.assert_not_called()
