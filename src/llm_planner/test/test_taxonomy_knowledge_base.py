import json
from pathlib import Path
import unittest
from llm_planner.config import get_knowledge_base_dir


class TestTaxonomyKnowledgeBase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.kb_dir = Path(get_knowledge_base_dir())
        cls.target_subclasses = [
            "ND[ST-SR-IA]",
            "ND[ST-SR-TA]",
            "ID[ST-SR-TA]",
            "XD[ST-SR-IA]",
            "XD[ST-SR-TA]",
            "XD[ST-MR-IA]",
            "XD[ST-MR-TA]",
        ]

    def test_korsah_taxonomy_axes_file(self):
        axes_file = self.kb_dir / "korsah_taxonomy_axes.json"
        self.assertTrue(axes_file.exists(), "korsah_taxonomy_axes.json does not exist")

        with open(axes_file, "r") as f:
            data = json.load(f)

        self.assertIn("axes", data)
        self.assertIn("subclasses", data)

        # Check axes definitions
        expected_axes = ["robot_model", "task_model", "time_horizon", "dependency"]
        for axis in expected_axes:
            self.assertIn(axis, data["axes"])
            axis_entry = data["axes"][axis]
            self.assertIn("description", axis_entry)
            self.assertIn("values", axis_entry)
            self.assertGreaterEqual(len(axis_entry["values"]), 2)
            for val_name, val_body in axis_entry["values"].items():
                self.assertIn("definition", val_body)
                self.assertIn("example", val_body)
                self.assertTrue(len(val_body["definition"].strip()) > 10)
                self.assertTrue(len(val_body["example"].strip()) > 10)

        # Check subclasses definitions
        for sc in self.target_subclasses:
            self.assertIn(sc, data["subclasses"])
            sc_entry = data["subclasses"][sc]
            required_keys = [
                "canonical_notation",
                "name",
                "complexity",
                "theoretical_formulation",
                "criteria",
                "optimization_target",
            ]
            for key in required_keys:
                self.assertIn(key, sc_entry)
                self.assertTrue(len(sc_entry[key].strip()) > 0)

        # Check CD definition conforms to Korsah et al. (2013)
        cd_val = data["axes"]["dependency"]["values"]["CD"]
        self.assertIn("Complex dependencies", cd_val["definition"])

        # Check ND[ST-SR-TA] is polynomial and LSAP reformulable
        nd_ta = data["subclasses"]["ND[ST-SR-TA]"]
        self.assertIn("Polynomial", nd_ta["complexity"])
        self.assertIn("Linear Sum Assignment Problem (LSAP)", nd_ta["theoretical_formulation"])

    def test_subtasks_match_stage1_few_shot_examples(self):
        """Verify that every exemplar's subtask decomposition strictly matches few_shot_examples.json 1:1."""
        stage1_file = self.kb_dir / "few_shot_examples.json"
        stage2_file = self.kb_dir / "few_shot_examples_taxonomy.json"
        self.assertTrue(stage1_file.exists())
        self.assertTrue(stage2_file.exists())

        with open(stage1_file, "r") as f:
            stage1_data = json.load(f)
        with open(stage2_file, "r") as f:
            stage2_data = json.load(f)

        # Map plan IDs to stage 1 subtasks
        stage1_subtasks_by_id = {}
        for ex in stage1_data["examples"]:
            if ex.get("output", {}).get("status") == "ok":
                stage1_subtasks_by_id[ex["id"]] = ex["output"]["subtasks"]

        subclass_to_stage1_id = {
            "ND[ST-SR-IA]": ["plan_001"],
            "ND[ST-SR-TA]": ["plan_002"],
            "ID[ST-SR-TA]": ["plan_003"],
            "XD[ST-SR-IA]": ["plan_004"],
            "XD[ST-SR-TA]": ["plan_005"],
            "XD[ST-MR-IA]": ["plan_006"],
            "XD[ST-MR-TA]": ["plan_007"],
        }

        for ex in stage2_data["examples"]:
            subclass = ex["output_classification"]["subclass"]
            candidate_ids = subclass_to_stage1_id[subclass]
            expected_subtasks = None
            matched_id = None
            for cid in candidate_ids:
                if cid in stage1_subtasks_by_id:
                    expected_subtasks = stage1_subtasks_by_id[cid]
                    matched_id = cid
                    break

            self.assertIsNotNone(
                expected_subtasks,
                f"Could not find stage 1 plan for {subclass} (looked for {candidate_ids})",
            )
            actual_subtasks = ex["input"]["subtasks"]

            self.assertEqual(
                len(actual_subtasks),
                len(expected_subtasks),
                f"Subtask count mismatch for {subclass} ({ex['id']})",
            )

            for act_t, exp_t in zip(actual_subtasks, expected_subtasks):
                self.assertEqual(act_t["id"], exp_t["id"])
                self.assertEqual(act_t["description"], exp_t["description"])
                self.assertEqual(act_t["required_skills"], exp_t["required_skills"])
                self.assertEqual(act_t["args"], exp_t["args"])
                self.assertEqual(act_t["dependencies"], exp_t["dependencies"])

    def test_few_shot_examples_taxonomy_file(self):
        examples_file = self.kb_dir / "few_shot_examples_taxonomy.json"
        self.assertTrue(
            examples_file.exists(), "few_shot_examples_taxonomy.json does not exist"
        )

        with open(examples_file, "r") as f:
            data = json.load(f)

        self.assertIn("examples", data)
        examples = data["examples"]
        self.assertEqual(len(examples), 7, "Must contain exactly 7 benchmark exemplars")

        covered_subclasses = set()
        for ex in examples:
            for top_key in [
                "id",
                "name",
                "input",
                "chain_of_thought",
                "output_classification",
            ]:
                self.assertIn(top_key, ex, f"Missing {top_key} in {ex.get('id')}")

            # Validate input schema
            inp = ex["input"]
            self.assertIn("subtasks", inp)
            self.assertIn("fleet", inp)
            self.assertIn("objects", inp)
            self.assertGreater(len(inp["subtasks"]), 0)

            for task in inp["subtasks"]:
                for tkey in [
                    "id",
                    "description",
                    "required_skills",
                    "args",
                    "dependencies",
                ]:
                    self.assertIn(
                        tkey, task, f"Task {task.get('id')} missing {tkey} in {ex['id']}"
                    )
                self.assertIsInstance(task["dependencies"], list)
                self.assertIsInstance(task["required_skills"], list)

            # Validate CoT schema
            cot = ex["chain_of_thought"]
            for step_key in [
                "step_1_robot_model",
                "step_2_task_model",
                "step_3_time_horizon",
                "step_4_dependency",
            ]:
                self.assertIn(step_key, cot, f"Missing {step_key} in {ex['id']}")
                self.assertTrue(len(cot[step_key].strip()) > 10)

            # Validate output classification schema
            out = ex["output_classification"]
            self.assertIn("axes", out)
            self.assertIn("subclass", out)
            self.assertNotIn(
                "matched_solver",
                out,
                f"matched_solver must NOT be predicted by LLM in {ex['id']}",
            )

            axes = out["axes"]
            self.assertIn(axes["robot_model"], {"ST", "MT"})
            self.assertIn(axes["task_model"], {"SR", "MR"})
            self.assertIn(axes["time_horizon"], {"IA", "TA"})
            self.assertIn(axes["dependency"], {"ND", "ID", "XD", "CD"})
            expected_subclass = (
                f"{axes['dependency']}[{axes['robot_model']}-"
                f"{axes['task_model']}-{axes['time_horizon']}]"
            )
            self.assertEqual(out["subclass"], expected_subclass)

            sc = out["subclass"]
            self.assertIn(sc, self.target_subclasses)
            covered_subclasses.add(sc)

        # Ensure all 7 subclasses are uniquely covered
        self.assertEqual(
            covered_subclasses,
            set(self.target_subclasses),
            "Exemplars must cover all 7 target subclasses",
        )

    def test_load_fleet_instances(self):
        from llm_planner.taxonomy_classifier import load_fleet_instances

        instances = load_fleet_instances()
        self.assertIn("mobile_robot_1", instances)
        self.assertIn("mobile_robot_2", instances)
        self.assertIn("robotic_arm_1", instances)

        mr1 = instances["mobile_robot_1"]
        self.assertEqual(mr1["robot_type"], "mobile_robot")
        self.assertIn("navigate_to", mr1["skills"])
        self.assertIn("navigation", mr1["capabilities"])
        self.assertEqual(mr1["specs"]["payload_capacity_kg"], 10.0)

        arm = instances["robotic_arm_1"]
        self.assertEqual(arm["robot_type"], "robotic_arm")
        self.assertIn("manipulation", arm["capabilities"])
        self.assertEqual(arm["specs"]["payload_capacity_kg"], 15.0)

    def test_load_fleet_instances_empty_files_log_errors(self):
        from unittest.mock import patch
        import tempfile
        from llm_planner.taxonomy_classifier import load_fleet_instances, logger

        with tempfile.TemporaryDirectory() as tmp_dir:
            with patch("llm_planner.taxonomy_classifier.get_knowledge_base_dir", return_value=tmp_dir):
                with patch.object(logger, "error") as mock_err:
                    instances = load_fleet_instances()
                    self.assertEqual(instances, {})
                    self.assertTrue(mock_err.called)
                    logged = [call[0][0] for call in mock_err.call_args_list]
                    self.assertTrue(any("Fleet file does not exist" in m or "empty" in m for m in logged))
                    self.assertTrue(any("Registry file does not exist" in m or "empty" in m for m in logged))

    def test_load_default_fleet_empty_files_log_errors(self):
        from unittest.mock import patch
        import tempfile
        from llm_planner.taxonomy_classifier import load_default_fleet, logger

        with tempfile.TemporaryDirectory() as tmp_dir:
            fleet_file = Path(tmp_dir) / "robot_fleet.json"
            registry_file = Path(tmp_dir) / "robot_registry.json"
            fleet_file.write_text(json.dumps({}))
            registry_file.write_text(json.dumps({}))

            with patch("llm_planner.taxonomy_classifier.get_knowledge_base_dir", return_value=tmp_dir):
                with patch.object(logger, "error") as mock_err:
                    fleet = load_default_fleet()
                    self.assertEqual(fleet, {})
                    self.assertTrue(mock_err.called)
                    logged = [call[0][0] for call in mock_err.call_args_list]
                    self.assertTrue(any("empty" in m for m in logged))


if __name__ == "__main__":
    unittest.main()
