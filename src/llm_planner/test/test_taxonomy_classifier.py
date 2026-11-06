"""
test_taxonomy_classifier.py

Unit tests for the TaxonomyClassifier and taxonomy_validator.
Tests schema validation, CoT completeness checks, mathematical consistency,
and retry recovery logic using mocked LLM responses.
"""

import json
from unittest.mock import MagicMock, patch
import unittest

from llm_planner.taxonomy_classifier import TaxonomyClassifier
from llm_planner.taxonomy_prompt_builder import (
    build_axes_reference_block,
    build_exemplars_reference_block,
    build_taxonomy_prompt,
    load_taxonomy_axes,
    load_taxonomy_exemplars,
)
from llm_planner.taxonomy_validator import validate_taxonomy_classification


class TestTaxonomyValidator(unittest.TestCase):
    def setUp(self):
        self.valid_output = {
            "chain_of_thought": {
                "step_1_robot_model": "Robots are strictly Single-Task (ST).",
                "step_2_task_model": "Max object weight (2.0kg) <= 10.0kg payload -> SR.",
                "step_3_time_horizon": "2 qualified robots available for 2 tasks -> IA.",
                "step_4_dependency": "Disjoint subtask chains with no precedence -> ND.",
            },
            "output_classification": {
                "axes": {
                    "robot_model": "ST",
                    "task_model": "SR",
                    "time_horizon": "IA",
                    "dependency": "ND",
                },
                "subclass": "ND[ST-SR-IA]",
            },
        }

    def test_valid_classification_passes(self):
        errors = validate_taxonomy_classification(self.valid_output)
        self.assertEqual(len(errors), 0, f"Expected 0 errors, got: {errors}")

    def test_missing_top_level_keys(self):
        bad_output = {"chain_of_thought": {}}
        errors = validate_taxonomy_classification(bad_output)
        self.assertTrue(any("MISSING_OUTPUT_CLASSIFICATION" in e for e in errors))

    def test_missing_cot_step(self):
        bad_output = json.loads(json.dumps(self.valid_output))
        del bad_output["chain_of_thought"]["step_3_time_horizon"]
        errors = validate_taxonomy_classification(bad_output)
        self.assertTrue(any("MISSING_COT_STEP" in e for e in errors))

    def test_prohibited_solver_prediction(self):
        bad_output = json.loads(json.dumps(self.valid_output))
        bad_output["output_classification"]["matched_solver"] = "Hungarian"
        errors = validate_taxonomy_classification(bad_output)
        self.assertTrue(any("SEPARATION_OF_CONCERNS_VIOLATION" in e for e in errors))

    def test_invalid_axis_value(self):
        bad_output = json.loads(json.dumps(self.valid_output))
        bad_output["output_classification"]["axes"]["robot_model"] = "INVALID"
        errors = validate_taxonomy_classification(bad_output)
        self.assertTrue(any("INVALID_AXIS_VALUE" in e for e in errors))

    def test_mathematical_mismatch(self):
        # Axes say ND[ST-SR-IA], but subclass is written as XD[ST-MR-TA]
        bad_output = json.loads(json.dumps(self.valid_output))
        bad_output["output_classification"]["subclass"] = "XD[ST-MR-TA]"
        errors = validate_taxonomy_classification(bad_output)
        self.assertTrue(any("MATHEMATICAL_MISMATCH" in e for e in errors))

    def test_unsupported_subclass(self):
        bad_output = json.loads(json.dumps(self.valid_output))
        bad_output["output_classification"]["axes"] = {
            "robot_model": "MT",
            "task_model": "MR",
            "time_horizon": "TA",
            "dependency": "CD",
        }
        bad_output["output_classification"]["subclass"] = "CD[MT-MR-TA]"
        errors = validate_taxonomy_classification(bad_output)
        self.assertTrue(any("UNSUPPORTED_SUBCLASS" in e for e in errors))


class TestTaxonomyPromptBuilder(unittest.TestCase):
    def test_load_and_build_blocks(self):
        axes_data = load_taxonomy_axes()
        self.assertIn("axes", axes_data)
        self.assertIn("subclasses", axes_data)

        exemplars = load_taxonomy_exemplars()
        self.assertEqual(len(exemplars), 7)

        axes_block = build_axes_reference_block(axes_data)
        self.assertIn("AXES DEFINITIONS", axes_block)
        self.assertIn("ND[ST-SR-IA]", axes_block)

        exemplars_block = build_exemplars_reference_block(exemplars)
        self.assertIn("EXEMPLAR 1", exemplars_block)
        self.assertIn("EXEMPLAR 7", exemplars_block)

        prompt = build_taxonomy_prompt(
            subtasks=[{"id": "T1", "description": "test"}],
            fleet={"mobile_robot": {"count": 2}},
            objects={"box": {"weight_kg": 2.0}},
            axes_data=axes_data,
            exemplars=exemplars,
        )
        self.assertIn("=== CANDIDATE MISSION TO CLASSIFY ===", prompt)
        self.assertIn("=== OUTPUT SCHEMA ===", prompt)


class TestTaxonomyClassifier(unittest.TestCase):
    def setUp(self):
        self.valid_output = {
            "chain_of_thought": {
                "step_1_robot_model": "ST robots.",
                "step_2_task_model": "2kg <= 10kg -> SR.",
                "step_3_time_horizon": "2 tasks <= 2 robots -> IA.",
                "step_4_dependency": "Disjoint -> ND.",
            },
            "output_classification": {
                "axes": {
                    "robot_model": "ST",
                    "task_model": "SR",
                    "time_horizon": "IA",
                    "dependency": "ND",
                },
                "subclass": "ND[ST-SR-IA]",
            },
        }

    @patch("llm_planner.taxonomy_classifier.call_llm_chat")
    def test_classify_immediate_success(self, mock_call_llm):
        mock_call_llm.return_value = json.dumps(self.valid_output)

        classifier = TaxonomyClassifier(
            axes_data={"axes": {}, "subclasses": {}},
            exemplars=[],
            client=MagicMock(),
            max_retries=3,
        )

        result = classifier.classify(
            subtasks=[{"id": "T1"}],
            fleet={"mobile_robot": {"count": 2}},
            objects={"box": {"weight_kg": 2.0}},
        )

        self.assertEqual(result.subclass, "ND[ST-SR-IA]")
        self.assertTrue(result.is_success)
        self.assertEqual(result.attempts, 1)
        self.assertEqual(len(result.errors), 0)

    @patch("llm_planner.taxonomy_classifier.call_llm_chat")
    def test_classify_retry_on_mathematical_mismatch(self, mock_call_llm):
        # Attempt 1: Output with mismatch (subclass != axes combination)
        mismatched_output = json.loads(json.dumps(self.valid_output))
        mismatched_output["output_classification"]["subclass"] = "XD[ST-MR-TA]"

        # Attempt 2: Corrected output
        mock_call_llm.side_effect = [
            json.dumps(mismatched_output),
            json.dumps(self.valid_output),
        ]

        classifier = TaxonomyClassifier(
            axes_data={"axes": {}, "subclasses": {}},
            exemplars=[],
            client=MagicMock(),
            max_retries=3,
        )

        result = classifier.classify(
            subtasks=[{"id": "T1"}],
            fleet={"mobile_robot": {"count": 2}},
            objects={"box": {"weight_kg": 2.0}},
        )

        self.assertEqual(result.subclass, "ND[ST-SR-IA]")
        self.assertTrue(result.is_success)
        self.assertEqual(result.attempts, 2)
        self.assertEqual(mock_call_llm.call_count, 2)

    @patch("llm_planner.taxonomy_classifier.call_llm_chat")
    def test_classify_max_retries_exhausted(self, mock_call_llm):
        bad_output = {"invalid": True}
        mock_call_llm.return_value = json.dumps(bad_output)

        classifier = TaxonomyClassifier(
            axes_data={"axes": {}, "subclasses": {}},
            exemplars=[],
            client=MagicMock(),
            max_retries=3,
        )

        result = classifier.classify(
            subtasks=[{"id": "T1"}],
            fleet={"mobile_robot": {"count": 2}},
            objects={"box": {"weight_kg": 2.0}},
        )

        self.assertFalse(result.is_success)
        self.assertEqual(result.attempts, 3)
        self.assertGreater(len(result.errors), 0)


if __name__ == "__main__":
    unittest.main()
