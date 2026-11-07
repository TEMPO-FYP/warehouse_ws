"""
taxonomy_classifier.py

Stage 2: Taxonomy Classification Module for TEMPO.
Classifies a validated subtask DAG into one of the 7 target iTax subclasses
using full in-context taxonomy definitions and 7 benchmark CoT exemplars,
accompanied by automated consistency validation and retry handling.
"""

from dataclasses import dataclass, field
import json
import os
import sys
from typing import Any, Dict, List, Optional

from llm_planner.config import (
    GEMINI_MODEL,
    GROQ_MODEL,
    LLM_PROVIDER,
    MAX_RETRIES,
    get_knowledge_base_dir,
)
from llm_planner.llm_client import call_llm_chat, get_client
from llm_planner.orchestrator import sanitize_json_response

from llm_planner.taxonomy_prompt_builder import (
    TAXONOMY_SYSTEM_PROMPT,
    build_taxonomy_prompt,
    load_taxonomy_axes,
    load_taxonomy_exemplars,
)
from llm_planner.taxonomy_validator import validate_taxonomy_classification

DEFAULT_MODEL = GEMINI_MODEL if LLM_PROVIDER == "gemini" else GROQ_MODEL


def load_default_fleet() -> Dict[str, Any]:
    """
    Constructs a fleet specification dictionary by combining robot_fleet.json
    and robot_registry.json.
    """
    kb_dir = get_knowledge_base_dir()
    fleet_file = os.path.join(kb_dir, "robot_fleet.json")
    registry_file = os.path.join(kb_dir, "robot_registry.json")

    fleet_counts = {}
    if os.path.exists(fleet_file):
        with open(fleet_file, "r", encoding="utf-8") as f:
            fleet_counts = json.load(f).get("fleet", {})

    specs_by_type = {}
    if os.path.exists(registry_file):
        with open(registry_file, "r", encoding="utf-8") as f:
            robots_list = json.load(f).get("robots", [])
            for r in robots_list:
                rtype = r.get("robot_type")
                if rtype:
                    specs_by_type[rtype] = {
                        "payload_capacity_kg": r.get("specs", {}).get(
                            "payload_capacity_kg", 10.0
                        ),
                        "speed_m_s": r.get("specs", {}).get("speed_m_s", 1.0),
                        "capabilities": r.get("capabilities", []),
                    }

    combined_fleet = {}
    for rtype, count in fleet_counts.items():
        specs = specs_by_type.get(
            rtype,
            {
                "payload_capacity_kg": 10.0,
                "speed_m_s": 1.0,
                "capabilities": ["navigation", "manipulation"],
            },
        )
        combined_fleet[rtype] = {
            "count": count,
            "payload_capacity_kg": specs["payload_capacity_kg"],
            "speed_m_s": specs["speed_m_s"],
            "capabilities": specs["capabilities"],
        }

    return combined_fleet


def load_fleet_instances() -> Dict[str, Any]:
    """
    Deterministically expands robot_fleet.json counts and robot_registry.json
    specifications into concrete robot instance entries with skills and specs.
    """
    kb_dir = get_knowledge_base_dir()
    fleet_file = os.path.join(kb_dir, "robot_fleet.json")
    registry_file = os.path.join(kb_dir, "robot_registry.json")

    fleet_counts = {}
    if os.path.exists(fleet_file):
        with open(fleet_file, "r", encoding="utf-8") as f:
            fleet_counts = json.load(f).get("fleet", {})

    specs_by_type = {}
    if os.path.exists(registry_file):
        with open(registry_file, "r", encoding="utf-8") as f:
            robots_list = json.load(f).get("robots", [])
            for r in robots_list:
                rtype = r.get("robot_type")
                if rtype:
                    specs_by_type[rtype] = {
                        "capabilities": r.get("capabilities", []),
                        "skill_ids": r.get("skill_ids", []),
                        "specs": r.get("specs", {}),
                    }

    instances = {}
    for rtype, count in fleet_counts.items():
        specs = specs_by_type.get(
            rtype,
            {
                "capabilities": ["navigation", "manipulation"],
                "skill_ids": ["navigate_to", "pick", "place"],
                "specs": {"payload_capacity_kg": 10.0, "speed_m_s": 1.0},
            },
        )
        for i in range(1, count + 1):
            instance_id = f"{rtype}_{i}"
            instances[instance_id] = {
                "robot_type": rtype,
                "capabilities": list(specs["capabilities"]),
                "skills": list(specs["skill_ids"]),
                "specs": dict(specs["specs"]),
            }

    return instances


def infer_default_objects(subtasks: List[Dict[str, Any]]) -> Dict[str, Any]:
    """
    Infers an objects specification dictionary from subtask arguments.
    Standard objects default to 2.0 kg; objects with 'heavy' in their name default to 20.0 kg.
    """
    objects = {}
    for task in subtasks:
        args = task.get("args", {})
        obj_name = args.get("object")
        if obj_name and obj_name not in objects:
            weight = 20.0 if "heavy" in str(obj_name).lower() else 2.0
            objects[obj_name] = {"weight_kg": weight}
    return objects


@dataclass
class TaxonomyClassificationResult:
    """Encapsulates the result of a taxonomy classification attempt."""

    subclass: str = ""
    axes: Dict[str, str] = field(default_factory=dict)
    chain_of_thought: Dict[str, str] = field(default_factory=dict)
    attempts: int = 1
    errors: List[str] = field(default_factory=list)
    raw_response: str = ""

    @property
    def is_success(self) -> bool:
        """Returns True if the output strictly validated with zero errors."""
        return bool(self.subclass) and not self.errors


class TaxonomyClassifier:
    """
    Taxonomy classification orchestrator for TEMPO Stage 2.
    Preloads and caches formal axes definitions and the 7 canonical CoT exemplars.
    """

    def __init__(
        self,
        axes_data: Optional[Dict[str, Any]] = None,
        exemplars: Optional[List[Dict[str, Any]]] = None,
        client: Optional[Any] = None,
        model: Optional[str] = None,
        max_retries: int = MAX_RETRIES,
    ):
        self.axes_data = (
            axes_data if axes_data is not None else load_taxonomy_axes()
        )
        self.exemplars = (
            exemplars if exemplars is not None else load_taxonomy_exemplars()
        )
        self.client = client if client is not None else get_client()
        self.model = model or DEFAULT_MODEL
        self.max_retries = max_retries

    def classify(
        self,
        subtasks: List[Dict[str, Any]],
        fleet: Optional[Dict[str, Any]] = None,
        objects: Optional[Dict[str, Any]] = None,
    ) -> TaxonomyClassificationResult:
        """
        Classifies a candidate subtask DAG into an iTax subclass using full in-context
        taxonomy formalisms and exemplars, with automated validation and retry handling.
        """
        active_fleet = fleet if fleet is not None else load_default_fleet()
        active_objects = (
            objects if objects is not None else infer_default_objects(subtasks)
        )

        user_prompt = build_taxonomy_prompt(
            subtasks=subtasks,
            fleet=active_fleet,
            objects=active_objects,
            axes_data=self.axes_data,
            exemplars=self.exemplars,
        )

        messages: List[Dict[str, str]] = [
            {"role": "system", "content": TAXONOMY_SYSTEM_PROMPT},
            {"role": "user", "content": user_prompt},
        ]

        last_raw_response: str = ""
        last_errors: List[str] = []
        last_parsed: Optional[Dict[str, Any]] = None

        for attempt in range(1, self.max_retries + 1):
            print(f"[TaxonomyClassifier] Classification attempt {attempt}/{self.max_retries}...")
            raw_response = call_llm_chat(self.client, messages, model=self.model)
            last_raw_response = raw_response
            clean_json = sanitize_json_response(raw_response)

            current_errors: List[str] = []

            # 1. JSON parse
            try:
                parsed = json.loads(clean_json)
                last_parsed = parsed
            except json.JSONDecodeError as exc:
                current_errors.append(
                    f"JSON_DECODE_ERROR: Output was not valid JSON ({exc.msg} at line {exc.lineno}, col {exc.colno})."
                )
                last_errors = current_errors
                if attempt < self.max_retries:
                    self._append_retry_feedback(messages, raw_response, current_errors)
                continue

            # 2. Taxonomy output validation
            current_errors = validate_taxonomy_classification(parsed)

            if not current_errors:
                out_class = parsed.get("output_classification", {})
                subclass = out_class.get("subclass", "")
                axes = out_class.get("axes", {})
                cot = parsed.get("chain_of_thought", {})

                print(f"[TaxonomyClassifier] Classified mission as: {subclass}")
                return TaxonomyClassificationResult(
                    subclass=subclass,
                    axes=axes,
                    chain_of_thought=cot,
                    attempts=attempt,
                    errors=[],
                    raw_response=raw_response,
                )

            last_errors = current_errors
            print(f"[TaxonomyClassifier] Validation failed on attempt {attempt}: {current_errors}")
            if attempt < self.max_retries:
                self._append_retry_feedback(messages, raw_response, current_errors)

        # Retries exhausted
        print(f"[TaxonomyClassifier] Classification failed after {self.max_retries} attempts.")
        subclass = ""
        axes = {}
        cot = {}
        if last_parsed and isinstance(last_parsed, dict):
            out_class = last_parsed.get("output_classification", {})
            subclass = out_class.get("subclass", "")
            axes = out_class.get("axes", {})
            cot = last_parsed.get("chain_of_thought", {})

        return TaxonomyClassificationResult(
            subclass=subclass,
            axes=axes,
            chain_of_thought=cot,
            attempts=self.max_retries,
            errors=last_errors,
            raw_response=last_raw_response,
        )

    def _append_retry_feedback(
        self, messages: List[Dict[str, str]], raw_response: str, errors: List[str]
    ) -> None:
        """Appends the assistant response and validation error feedback for the next turn."""
        messages.append({"role": "assistant", "content": raw_response})
        error_items = "\n".join(f"- {err}" for err in errors)
        feedback = (
            f"Your taxonomy classification failed validation with the following errors:\n"
            f"{error_items}\n\n"
            f"Please carefully fix all listed errors and output ONLY the complete corrected JSON classification adhering strictly to the schema."
        )
        messages.append({"role": "user", "content": feedback})


# Global cached classifier instance
_default_taxonomy_classifier: Optional[TaxonomyClassifier] = None


def classify_taxonomy(
    subtasks: List[Dict[str, Any]],
    fleet: Optional[Dict[str, Any]] = None,
    objects: Optional[Dict[str, Any]] = None,
    client: Optional[Any] = None,
    model: Optional[str] = None,
    max_retries: Optional[int] = None,
) -> TaxonomyClassificationResult:
    """
    Classifies a subtask DAG into an iTax subclass using a configured or cached classifier.
    """
    if client is not None or model is not None or max_retries is not None:
        classifier = TaxonomyClassifier(
            client=client,
            model=model,
            max_retries=max_retries if max_retries is not None else MAX_RETRIES,
        )
        return classifier.classify(subtasks, fleet=fleet, objects=objects)

    global _default_taxonomy_classifier
    if _default_taxonomy_classifier is None:
        _default_taxonomy_classifier = TaxonomyClassifier()
    return _default_taxonomy_classifier.classify(subtasks, fleet=fleet, objects=objects)


if __name__ == "__main__":
    from llm_planner.orchestrator import decompose_mission

    mission_text = (
        sys.argv[1]
        if len(sys.argv) > 1
        else "Retrieve the red box from shelf A and deliver it to origin."
    )

    print(f"\n==========================================")
    print(f"TEMPO Taxonomy Classification (Stage 2)")
    print(f"Mission: {mission_text}")
    print(f"==========================================\n")

    # Step 1: Decompose
    print("Executing Stage 1: Task Decomposition...")
    decomp_result = decompose_mission(mission_text)

    if not decomp_result.is_success or decomp_result.status != "ok":
        print(f"Cannot classify mission with status '{decomp_result.status}'. Reason: {decomp_result.plan.get('reason')}")
        sys.exit(0)

    # Step 2: Classify
    print("\nExecuting Stage 2: Taxonomy Classification...")
    tax_result = classify_taxonomy(decomp_result.plan["subtasks"])

    print("\n---------------- TAXONOMY RESULT ----------------")
    print(f"Subclass: {tax_result.subclass}")
    print(f"Success:  {tax_result.is_success}")
    print(f"Attempts: {tax_result.attempts}")
    print("\nAxes Breakdown:")
    for axis, val in tax_result.axes.items():
        print(f"  {axis:15s}: {val}")

    print("\nChain of Thought:")
    for step, reasoning in tax_result.chain_of_thought.items():
        print(f"  {step}:")
        print(f"    {reasoning}")

    if tax_result.errors:
        print("\nErrors:")
        for err in tax_result.errors:
            print(f"  - {err}")
    print("=================================================\n")
