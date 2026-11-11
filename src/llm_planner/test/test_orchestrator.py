"""
test_orchestrator.py

Unit tests for the MissionPlanner task decomposition orchestrator.
Tests Stage 0 grounder integration, retry logic on JSON and validation failures,
Check 6 grounded entity validation and self-correction, DAG construction,
and end-to-end plan and classify workflow using mocked LLM responses.
"""

import json
from unittest.mock import MagicMock, patch
import unittest

import networkx as nx

from llm_planner.orchestrator import (
    DecompositionResult,
    MissionPlanResult,
    MissionPlanner,
    sanitize_json_response,
)
from llm_planner.scene_graph_grounder import GroundedSceneResult, SceneGraphGrounder


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
                    "id": "plan_010",
                    "instruction": "Test instruction",
                    "output": {"status": "ok", "subtasks": []},
                },
                0.95,
            )
        ]

        self.mock_grounded_scene = GroundedSceneResult(
            environment_prompt="GROUNDED 3D SCENE GRAPH ENVIRONMENT:\nZones: picking_zone, packing_zone",
            valid_entities={
                "objects": {"small_box_001_1", "Pallet_Standard"},
                "locations": {
                    "picking_zone",
                    "packing_zone",
                    "Picking_Shelves.001",
                    "Packing_Station",
                    "small_box_001_1",
                },
            },
            objects_metadata={
                "small_box_001_1": {
                    "weight_kg": 2.0,
                    "pose": {"x": -7.26, "y": 1.95, "z": 0.2},
                    "zone_id": "picking_zone",
                    "parent_location": "Picking_Shelves.001",
                },
                "Pallet_Standard": {
                    "weight_kg": 25.0,
                    "pose": {"x": 16.35, "y": 18.82, "z": 0.0},
                    "zone_id": "staging_area",
                    "parent_location": None,
                },
            },
            expanded_nodes=["picking_zone", "packing_zone", "Picking_Shelves.001"],
            turns_used=3,
        )

        self.grounder_patcher = patch.object(
            SceneGraphGrounder, "ground_mission", return_value=self.mock_grounded_scene
        )
        self.mock_ground_mission = self.grounder_patcher.start()
        self.addCleanup(self.grounder_patcher.stop)

        self.valid_plan = {
            "status": "ok",
            "plan_id": "plan_test_001",
            "original_instruction": "Retrieve small box from shelf 001 to packing station.",
            "subtasks": [
                {
                    "id": "T1",
                    "description": "Navigate to shelf 001",
                    "required_skills": ["navigate_to"],
                    "args": {"location": "Picking_Shelves.001"},
                    "dependencies": [],
                },
                {
                    "id": "T2",
                    "description": "Pick small box",
                    "required_skills": ["pick"],
                    "args": {"object": "small_box_001_1"},
                    "dependencies": ["T1"],
                },
                {
                    "id": "T3",
                    "description": "Navigate to packing station",
                    "required_skills": ["navigate_to"],
                    "args": {"location": "Packing_Station"},
                    "dependencies": ["T2"],
                },
                {
                    "id": "T4",
                    "description": "Place small box at packing station",
                    "required_skills": ["place"],
                    "args": {
                        "object": "small_box_001_1",
                        "location": "Packing_Station",
                    },
                    "dependencies": ["T3"],
                },
            ],
        }

    def test_sanitize_json_response(self):
        # Plain json
        self.assertEqual(sanitize_json_response('{"a": 1}'), '{"a": 1}')
        # Markdown fenced json
        fenced = '```json\n{"status": "ok"}\n```'
        self.assertEqual(sanitize_json_response(fenced), '{"status": "ok"}')
        # Markdown fenced without 'json' tag
        fenced_no_tag = '```\n{"status": "ok"}\n```'
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

        result = planner.plan_mission(
            "Retrieve small box from shelf 001 to packing station."
        )

        self.assertEqual(result.status, "ok")
        self.assertTrue(result.is_success)
        self.assertEqual(result.attempts, 1)
        self.assertEqual(len(result.errors), 0)
        self.assertIsNotNone(result.dag)
        self.assertTrue(nx.is_directed_acyclic_graph(result.dag))
        self.assertEqual(result.dag.number_of_nodes(), 4)
        self.assertEqual(result.dag.number_of_edges(), 3)
        self.assertEqual(result.grounded_scene, self.mock_grounded_scene)

    @patch("llm_planner.orchestrator.call_llm_chat")
    def test_plan_clarification_needed(self, mock_call_llm):
        rejection_output = {
            "status": "clarification_needed",
            "reason": "Referent 'that shelf' is ambiguous and ungrounded.",
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

        result = planner.plan_mission("Retrieve box from shelf 001 to packing station.")
        self.assertEqual(result.status, "ok")
        self.assertTrue(result.is_success)
        self.assertEqual(result.attempts, 2)
        self.assertEqual(mock_call_llm.call_count, 2)

    @patch("llm_planner.orchestrator.call_llm_chat")
    def test_retry_on_check_6_ungrounded_entity_failure(self, mock_call_llm):
        """Tests that referencing a hallucinated entity triggers Check 6 retry and succeeds after self-correction."""
        # Attempt 1: Plan with hallucinated ungrounded object ID
        invalid_plan = {
            "status": "ok",
            "plan_id": "plan_001",
            "original_instruction": "Test",
            "subtasks": [
                {
                    "id": "T1",
                    "description": "Navigate",
                    "required_skills": ["navigate_to"],
                    "args": {"location": "Picking_Shelves.001"},
                    "dependencies": [],
                },
                {
                    "id": "T2",
                    "description": "Pick ungrounded box",
                    "required_skills": ["pick"],
                    "args": {"object": "hallucinated_red_box_999"},
                    "dependencies": ["T1"],
                },
            ],
        }

        # Attempt 2: Valid plan with small_box_001_1
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

        result = planner.plan_mission("Retrieve box from shelf 001 to packing station.")
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

        result = planner.plan_mission("Retrieve box from shelf 001 to packing station.")
        self.assertEqual(result.status, "failed_validation")
        self.assertFalse(result.is_success)
        self.assertEqual(result.attempts, 3)
        self.assertGreater(len(result.errors), 0)
        self.assertTrue(any("INVALID_SKILL" in err for err in result.errors))

    @patch("llm_planner.taxonomy_classifier.classify_taxonomy")
    @patch("llm_planner.orchestrator.call_llm_chat")
    def test_plan_and_classify_integration_with_grounded_weights(
        self, mock_call_llm, mock_classify
    ):
        """Verifies that Stage 0 grounder passes real object weights into Stage 2 classify_taxonomy."""
        mock_call_llm.return_value = json.dumps(self.valid_plan)

        mock_tax_result = MagicMock()
        mock_tax_result.is_success = True
        mock_tax_result.subclass = "ND[ST-SR-IA]"
        mock_tax_result.axes = {"robot_model": "ST", "task_model": "SR", "time_horizon": "IA", "dependency": "ND"}
        mock_tax_result.chain_of_thought = {"step_1_robot_model": "ST", "step_2_task_model": "SR"}
        mock_tax_result.attempts = 1
        mock_tax_result.errors = []
        mock_classify.return_value = mock_tax_result

        planner = MissionPlanner(
            skill_bank=self.mock_skill_bank,
            skill_retriever=self.mock_skill_retriever,
            few_shot_examples=[],
            few_shot_retriever=self.mock_few_shot_retriever,
            client=MagicMock(),
            max_retries=3,
        )

        mission_res = planner.plan_and_classify(
            "Retrieve small box from shelf 001 to packing station."
        )

        self.assertEqual(mission_res.status, "ok")
        self.assertTrue(mission_res.is_success)
        self.assertEqual(mission_res.itax_subclass, "ND[ST-SR-IA]")
        self.assertEqual(mission_res.grounding_attempts, 3)

        # Verify classify_taxonomy was called with exact weight 2.0 kg from scene graph
        self.assertTrue(mock_classify.called)
        _, kwargs = mock_classify.call_args
        self.assertIn("objects", kwargs)
        self.assertIn("small_box_001_1", kwargs["objects"])
        self.assertEqual(kwargs["objects"]["small_box_001_1"]["weight_kg"], 2.0)


if __name__ == "__main__":
    unittest.main()
