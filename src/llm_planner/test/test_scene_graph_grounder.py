"""
test_scene_graph_grounder.py

Unit tests for Stage 0 SayPlan 3D Scene Graph semantic grounding,
SceneGraphNavigator expand/collapse mechanics, Check 6 grounded entity validation,
and real object weight extraction for Stage 2.
"""

import json
from unittest.mock import MagicMock, patch
import unittest

import networkx as nx

from llm_planner.dag_builder import build_dag, validate_plan
from llm_planner.scene_graph_grounder import (
    GroundedSceneResult,
    SceneGraphGrounder,
    SceneGraphNavigator,
    extract_objects_dict,
    load_scene_graph,
)


class TestSceneGraphNavigator(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.scene_graph = load_scene_graph()

    def setUp(self):
        self.nav = SceneGraphNavigator(self.scene_graph)

    def test_initial_state_collapsed(self):
        """Initial state should have zero expanded nodes and compact view."""
        self.assertEqual(len(self.nav.expanded_zones), 0)
        self.assertEqual(len(self.nav.expanded_aisles), 0)
        self.assertEqual(len(self.nav.expanded_fixtures), 0)
        self.assertEqual(len(self.nav.memory), 0)

        view = self.nav.render_scene_view()
        self.assertIn("[COLLAPSED] Zone 'picking_zone'", view)
        self.assertIn("[COLLAPSED] Zone 'packing_zone'", view)
        self.assertIn("[COLLAPSED] Zone 'staging_area'", view)
        self.assertIn("[COLLAPSED] Zone 'hazmat_zone'", view)
        self.assertIn("[COLLAPSED] Zone 'maintenance_zone'", view)

    def test_expand_zone_and_aisle(self):
        """Expanding picking_zone and aisle_1 exposes the 14 shelves in aisle_1."""
        expanded = self.nav.expand(["picking_zone", "aisle_1"])
        self.assertIn("picking_zone", self.nav.expanded_zones)
        self.assertIn("aisle_1", self.nav.expanded_aisles)
        self.assertIn("picking_zone", self.nav.memory)
        self.assertIn("aisle_1", self.nav.memory)

        view = self.nav.render_scene_view()
        self.assertIn("[EXPANDED] Zone 'picking_zone'", view)
        self.assertIn("[EXPANDED] Aisle 'aisle_1'", view)
        self.assertIn("Picking_Shelves.001", view)
        # aisle_2 should remain collapsed
        self.assertIn("[COLLAPSED] Aisle 'aisle_2'", view)

    def test_expand_fixture_auto_expands_parents(self):
        """Expanding a fixture directly auto-expands its parent aisle and zone."""
        expanded = self.nav.expand(["Picking_Shelves.001"])
        self.assertIn("Picking_Shelves.001", self.nav.expanded_fixtures)
        self.assertIn("aisle_1", self.nav.expanded_aisles)
        self.assertIn("picking_zone", self.nav.expanded_zones)

        view = self.nav.render_scene_view()
        self.assertIn("[EXPANDED] Fixture 'Picking_Shelves.001'", view)
        self.assertIn("small_box_001_1", view)
        self.assertIn("small_box_001_6", view)

    def test_collapse_cascades_to_children(self):
        """Collapsing a zone folds its active aisles and fixtures."""
        self.nav.expand(["Picking_Shelves.001"])
        self.assertIn("Picking_Shelves.001", self.nav.expanded_fixtures)

        collapsed = self.nav.collapse(["picking_zone"])
        self.assertIn("picking_zone", collapsed)
        self.assertNotIn("picking_zone", self.nav.expanded_zones)
        self.assertNotIn("aisle_1", self.nav.expanded_aisles)
        self.assertNotIn("Picking_Shelves.001", self.nav.expanded_fixtures)

        # Memory retains that they were inspected earlier
        self.assertIn("Picking_Shelves.001", self.nav.memory)

    def test_memory_trace_rendering(self):
        """Memory trace records turn number, action, nodes, and reasoning."""
        self.nav.record_action(
            1, "expand", ["picking_zone"], "Looking for picking zone"
        )
        self.nav.record_action(
            2, "expand", ["aisle_1"], "Looking for shelf in aisle 1"
        )
        trace = self.nav.render_memory_trace()
        self.assertIn("Turn 1: action='expand'", trace)
        self.assertIn("Turn 2: action='expand'", trace)
        self.assertIn("Looking for shelf in aisle 1", trace)

    def test_get_grounded_entities_and_environment_prompt(self):
        """Valid entities include exposed objects, fixtures, and zone IDs."""
        self.nav.expand(["Picking_Shelves.001", "packing_zone"])
        valid_objs, valid_locs, objs_meta = self.nav.get_grounded_entities()

        # Children of Picking_Shelves.001
        self.assertIn("small_box_001_1", valid_objs)
        self.assertIn("small_box_001_1", valid_locs)
        self.assertIn("Picking_Shelves.001", valid_locs)
        self.assertIn("Packing_Station", valid_locs)
        self.assertIn("packing_zone", valid_locs)
        self.assertIn("picking_zone", valid_locs)

        # Object metadata includes exact weight and parent location
        meta_1 = objs_meta["small_box_001_1"]
        self.assertEqual(meta_1["weight_kg"], 2.0)
        self.assertEqual(meta_1["parent_location"], "Picking_Shelves.001")
        self.assertEqual(meta_1["shelf_tier"], 1)

        # Formatted environment prompt
        env_text = self.nav.format_grounded_environment()
        self.assertIn("GROUNDED 3D SCENE GRAPH ENVIRONMENT", env_text)
        self.assertIn("small_box_001_1", env_text)
        self.assertIn("Picking_Shelves.001", env_text)
        self.assertIn("Inter-Zone Distance Matrix", env_text)


class TestSceneGraphGrounder(unittest.TestCase):
    @patch("llm_planner.scene_graph_grounder.call_llm_chat")
    def test_ground_mission_multi_turn_to_done(self, mock_call_llm):
        """Simulates interactive SayPlan search loop to locate a box and destination."""
        turn_1_resp = json.dumps({
            "action": "expand",
            "nodes": ["picking_zone", "packing_zone"],
            "reasoning": "Need to check picking storage and packing stations",
        })
        turn_2_resp = json.dumps({
            "action": "expand",
            "nodes": ["aisle_1"],
            "reasoning": "Checking aisle 1 for shelves",
        })
        turn_3_resp = json.dumps({
            "action": "expand",
            "nodes": ["Picking_Shelves.001"],
            "reasoning": "Inspecting shelves for boxes",
        })
        turn_4_resp = json.dumps({
            "action": "done",
            "nodes": [],
            "reasoning": "Located small_box_001_1 and Packing_Station",
        })

        mock_call_llm.side_effect = [
            turn_1_resp,
            turn_2_resp,
            turn_3_resp,
            turn_4_resp,
        ]

        grounder = SceneGraphGrounder(client=MagicMock(), max_turns=6)
        result = grounder.ground_mission(
            "Move a small box from aisle 1 to a packing station."
        )

        self.assertEqual(result.turns_used, 4)
        self.assertIn("small_box_001_1", result.valid_entities["objects"])
        self.assertIn("Picking_Shelves.001", result.valid_entities["locations"])
        self.assertIn("Packing_Station", result.valid_entities["locations"])
        self.assertEqual(len(result.action_history), 4)

    def test_extract_objects_dict(self):
        """Tests extracting object weights from grounded scene for subtasks."""
        grounded_meta = {
            "small_box_001_1": {"weight_kg": 2.0},
            "Pallet_Standard": {"weight_kg": 25.0},
            "cordless_drill": {"weight_kg": 1.5},
        }
        mock_result = GroundedSceneResult(
            environment_prompt="",
            valid_entities={"objects": set(), "locations": set()},
            objects_metadata=grounded_meta,
            expanded_nodes=[],
            turns_used=1,
        )

        subtasks = [
            {"id": "T1", "args": {"object": "small_box_001_1"}},
            {"id": "T2", "args": {"object": "Pallet_Standard"}},
        ]

        objs = extract_objects_dict(mock_result, subtasks)
        self.assertEqual(objs["small_box_001_1"]["weight_kg"], 2.0)
        self.assertEqual(objs["Pallet_Standard"]["weight_kg"], 25.0)


class TestDAGValidatorCheck6(unittest.TestCase):
    def setUp(self):
        self.skill_bank = [
            {"name": "navigate_to", "inputs": ["location"]},
            {"name": "pick", "inputs": ["object"]},
            {"name": "place", "inputs": ["object", "location"]},
        ]
        self.grounded_entities = {
            "objects": {"small_box_001_1", "cordless_drill"},
            "locations": {
                "picking_zone",
                "packing_zone",
                "Picking_Shelves.001",
                "Packing_Station",
                "small_box_001_1",
            },
        }

    def test_valid_grounded_plan_passes(self):
        """Plan referencing grounded entity IDs passes Check 6 with zero errors."""
        plan = {
            "status": "ok",
            "subtasks": [
                {
                    "id": "T1",
                    "required_skills": ["navigate_to"],
                    "args": {"location": "Picking_Shelves.001"},
                    "dependencies": [],
                },
                {
                    "id": "T2",
                    "required_skills": ["pick"],
                    "args": {"object": "small_box_001_1"},
                    "dependencies": ["T1"],
                },
                {
                    "id": "T3",
                    "required_skills": ["navigate_to"],
                    "args": {"location": "Packing_Station"},
                    "dependencies": ["T2"],
                },
                {
                    "id": "T4",
                    "required_skills": ["place"],
                    "args": {"object": "small_box_001_1", "location": "Packing_Station"},
                    "dependencies": ["T3"],
                },
            ],
        }
        dag = build_dag(plan)
        errors = validate_plan(
            plan, dag, self.skill_bank, grounded_entities=self.grounded_entities
        )
        self.assertEqual(errors, [])

    def test_hallucinated_object_fails_check_6(self):
        """Plan referencing non-existent object fails Check 6 with UNGROUNDED_OBJECT."""
        plan = {
            "status": "ok",
            "subtasks": [
                {
                    "id": "T1",
                    "required_skills": ["pick"],
                    "args": {"object": "hallucinated_box_999"},
                    "dependencies": [],
                }
            ],
        }
        dag = build_dag(plan)
        errors = validate_plan(
            plan, dag, self.skill_bank, grounded_entities=self.grounded_entities
        )
        self.assertTrue(any("UNGROUNDED_OBJECT" in err for err in errors))
        self.assertTrue(any("hallucinated_box_999" in err for err in errors))

    def test_hallucinated_location_fails_check_6(self):
        """Plan referencing non-existent location fails Check 6 with UNGROUNDED_LOCATION."""
        plan = {
            "status": "ok",
            "subtasks": [
                {
                    "id": "T1",
                    "required_skills": ["navigate_to"],
                    "args": {"location": "non_existent_shelf_Z"},
                    "dependencies": [],
                }
            ],
        }
        dag = build_dag(plan)
        errors = validate_plan(
            plan, dag, self.skill_bank, grounded_entities=self.grounded_entities
        )
        self.assertTrue(any("UNGROUNDED_LOCATION" in err for err in errors))
        self.assertTrue(any("non_existent_shelf_Z" in err for err in errors))


if __name__ == "__main__":
    unittest.main()
