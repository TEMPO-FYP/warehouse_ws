"""
test_orchestrator.py

Unit tests for the MissionPlanner task decomposition orchestrator.
Uses mocked LLM responses to test retry logic, DAG construction, and error
handling without making live API calls.
"""

import json
from unittest.mock import MagicMock, patch
import unittest

import networkx as nx

from llm_planner.orchestrator import (
    DecompositionResult,
    MissionPlanner,
    sanitize_json_response,
)


class TestOrchestrator(unittest.TestCase):
    def setUp(self):
        self.mock_skill_bank = [
            {
                "name": "navigate_to",
                "description": "Navigate to location.",
                "inputs": ["location"],
                "preconditions": [],
                "effects": [],
                "required_capabilities": ["navigation"],
            },
            {
                "name": "pick",
                "description": "Pick object.",
                "inputs": ["object"],
                "preconditions": [],
                "effects": [],
                "required_capabilities": ["manipulation"],
            },
            {
                "name": "place",
                "description": "Place object at location.",
                "inputs": ["object", "location"],
                "preconditions": [],
                "effects": [],
                "required_capabilities": ["manipulation"],
            },
        ]
        self.mock_skill_retriever = MagicMock()
        self.mock_skill_retriever.retrieve.return_value = [
            (self.mock_skill_bank[0], 0.9),
            (self.mock_skill_bank[1], 0.8),
            (self.mock_skill_bank[2], 0.7),
        ]

        self.mock_few_shot_retriever = MagicMock()
        self.mock_few_shot_retriever.retrieve.return_value = [
            (
                {
                    "id": "plan_001",
                    "instruction": "Test instruction",
                    "output": {"status": "ok", "subtasks": []},
                },
                0.95,
            )
        ]

        self.valid_plan = {
            "status": "ok",
            "plan_id": "plan_test_001",
            "original_instruction": "Retrieve box from shelf A to origin.",
            "subtasks": [
                {
                    "id": "T1",
                    "description": "Navigate to shelf A",
                    "required_skills": ["navigate_to"],
                    "args": {"location": "shelf_A"},
                    "dependencies": [],
                },
                {
                    "id": "T2",
                    "description": "Pick box",
                    "required_skills": ["pick"],
                    "args": {"object": "box"},
                    "dependencies": ["T1"],
                },
                {
                    "id": "T3",
                    "description": "Navigate to origin",
                    "required_skills": ["navigate_to"],
                    "args": {"location": "origin"},
                    "dependencies": ["T2"],
                },
                {
                    "id": "T4",
                    "description": "Place box at origin",
                    "required_skills": ["place"],
                    "args": {"object": "box", "location": "origin"},
                    "dependencies": ["T3"],
                },
            ],
        }

    def test_sanitize_json_response(self):
        # Plain json
        self.assertEqual(sanitize_json_response('{"a": 1}'), '{"a": 1}')
        # Markdown fenced json
        fenced = "```json\n{\"status\": \"ok\"}\n```"
        self.assertEqual(sanitize_json_response(fenced), '{"status": "ok"}')
        # Markdown fenced without 'json' tag
        fenced_no_tag = "```\n{\"status\": \"ok\"}\n```"
        self.assertEqual(sanitize_json_response(fenced_no_tag), '{"status": "ok"}')

    @patch("llm_planner.orchestrator.call_llm_chat")
    def test_plan_immediate_success(self, mock_call_llm):
        mock_call_llm.return_value = json.dumps(self.valid_plan)

        planner = MissionPlanner(
            skill_bank=self.mock_skill_bank,
            skill_retriever=self.mock_skill_retriever,
            few_shot_examples=[],
            few_shot_retriever=self.mock_few_shot_retriever,
            client=MagicMock(),
            max_retries=3,
        )

        result = planner.plan_mission("Retrieve box from shelf A to origin.")

        self.assertEqual(result.status, "ok")
        self.assertTrue(result.is_success)
        self.assertEqual(result.attempts, 1)
        self.assertEqual(len(result.errors), 0)
        self.assertIsNotNone(result.dag)
        self.assertTrue(nx.is_directed_acyclic_graph(result.dag))
        self.assertEqual(result.dag.number_of_nodes(), 4)
        self.assertEqual(result.dag.number_of_edges(), 3)

    @patch("llm_planner.orchestrator.call_llm_chat")
    def test_plan_clarification_needed(self, mock_call_llm):
        rejection_output = {
            "status": "clarification_needed",
            "reason": "Referent 'that shelf' is ambiguous.",
        }
        mock_call_llm.return_value = json.dumps(rejection_output)

        planner = MissionPlanner(
            skill_bank=self.mock_skill_bank,
            skill_retriever=self.mock_skill_retriever,
            few_shot_examples=[],
            few_shot_retriever=self.mock_few_shot_retriever,
            client=MagicMock(),
            max_retries=3,
        )

        result = planner.plan_mission("Go to that shelf.")
        self.assertEqual(result.status, "clarification_needed")
        self.assertTrue(result.is_success)
        self.assertIsNone(result.dag)
        self.assertEqual(result.attempts, 1)

    @patch("llm_planner.orchestrator.call_llm_chat")
    def test_plan_infeasible(self, mock_call_llm):
        infeasible_output = {
            "status": "infeasible",
            "reason": "Welding is outside robot fleet capabilities.",
        }
        mock_call_llm.return_value = json.dumps(infeasible_output)

        planner = MissionPlanner(
            skill_bank=self.mock_skill_bank,
            skill_retriever=self.mock_skill_retriever,
            few_shot_examples=[],
            few_shot_retriever=self.mock_few_shot_retriever,
            client=MagicMock(),
            max_retries=3,
        )

        result = planner.plan_mission("Weld the door.")
        self.assertEqual(result.status, "infeasible")
        self.assertTrue(result.is_success)
        self.assertIsNone(result.dag)

    @patch("llm_planner.orchestrator.call_llm_chat")
    def test_retry_on_json_syntax_error(self, mock_call_llm):
        # Attempt 1: Malformed JSON. Attempt 2: Correct JSON.
        mock_call_llm.side_effect = [
            "Malformed { JSON string",
            json.dumps(self.valid_plan),
        ]

        planner = MissionPlanner(
            skill_bank=self.mock_skill_bank,
            skill_retriever=self.mock_skill_retriever,
            few_shot_examples=[],
            few_shot_retriever=self.mock_few_shot_retriever,
            client=MagicMock(),
            max_retries=3,
        )

        result = planner.plan_mission("Retrieve box from shelf A to origin.")
        self.assertEqual(result.status, "ok")
        self.assertTrue(result.is_success)
        self.assertEqual(result.attempts, 2)
        self.assertEqual(mock_call_llm.call_count, 2)

    @patch("llm_planner.orchestrator.call_llm_chat")
    def test_retry_on_validation_failure(self, mock_call_llm):
        # Attempt 1: Plan with invalid skill and missing argument
        invalid_plan = {
            "status": "ok",
            "plan_id": "plan_001",
            "original_instruction": "Test",
            "subtasks": [
                {
                    "id": "T1",
                    "description": "Flying",
                    "required_skills": ["fly_drone"],  # Invalid skill
                    "args": {},
                    "dependencies": [],
                }
            ],
        }

        # Attempt 2: Valid plan
        mock_call_llm.side_effect = [
            json.dumps(invalid_plan),
            json.dumps(self.valid_plan),
        ]

        planner = MissionPlanner(
            skill_bank=self.mock_skill_bank,
            skill_retriever=self.mock_skill_retriever,
            few_shot_examples=[],
            few_shot_retriever=self.mock_few_shot_retriever,
            client=MagicMock(),
            max_retries=3,
        )

        result = planner.plan_mission("Retrieve box from shelf A to origin.")
        self.assertEqual(result.status, "ok")
        self.assertTrue(result.is_success)
        self.assertEqual(result.attempts, 2)
        self.assertEqual(mock_call_llm.call_count, 2)

    @patch("llm_planner.orchestrator.call_llm_chat")
    def test_max_retries_exhausted(self, mock_call_llm):
        # Persistent invalid skill error across all 3 attempts
        invalid_plan = {
            "status": "ok",
            "plan_id": "plan_001",
            "original_instruction": "Test",
            "subtasks": [
                {
                    "id": "T1",
                    "description": "Flying",
                    "required_skills": ["non_existent_skill"],
                    "args": {},
                    "dependencies": [],
                }
            ],
        }

        mock_call_llm.return_value = json.dumps(invalid_plan)

        planner = MissionPlanner(
            skill_bank=self.mock_skill_bank,
            skill_retriever=self.mock_skill_retriever,
            few_shot_examples=[],
            few_shot_retriever=self.mock_few_shot_retriever,
            client=MagicMock(),
            max_retries=3,
        )

        result = planner.plan_mission("Retrieve box from shelf A to origin.")
        self.assertEqual(result.status, "failed_validation")
        self.assertFalse(result.is_success)
        self.assertEqual(result.attempts, 3)
        self.assertGreater(len(result.errors), 0)
        self.assertTrue(any("INVALID_SKILL" in err for err in result.errors))


if __name__ == "__main__":
    unittest.main()
