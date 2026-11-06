"""
taxonomy_prompt_builder.py

Builds the prompt and provides context for Stage 2: Taxonomy Classification.
Injects the complete Korsah taxonomy axes definitions, criteria for all 7 target
subclasses, and all 7 benchmark Chain-of-Thought exemplars into the LLM context.
"""

import json
import os
from typing import Any, Dict, List

from llm_planner.config import get_knowledge_base_dir

def get_axes_file() -> str:
    return os.path.join(get_knowledge_base_dir(), "korsah_taxonomy_axes.json")

def get_exemplars_file() -> str:
    return os.path.join(get_knowledge_base_dir(), "few_shot_examples_taxonomy.json")


def load_taxonomy_axes(filepath: str = None) -> Dict[str, Any]:
    """Loads formal taxonomy axes definitions and target subclass criteria."""
    if filepath is None:
        filepath = get_axes_file()
    with open(filepath, "r", encoding="utf-8") as f:
        return json.load(f)


def load_taxonomy_exemplars(filepath: str = None) -> List[Dict[str, Any]]:
    """Loads all benchmark taxonomy exemplars covering the 7 target subclasses."""
    if filepath is None:
        filepath = get_exemplars_file()
    with open(filepath, "r", encoding="utf-8") as f:
        data = json.load(f)
        return data.get("examples", [])


TAXONOMY_SYSTEM_PROMPT = """You are an expert Multi-Robot Task Allocation (MRTA) Taxonomy Classifier for the TEMPO framework.
Your task is to classify a candidate multi-robot mission into exactly one of the 7 target iTax subclasses based on the provided subtask DAG, robot fleet capabilities, and object specifications.

You must follow these strict rules:

1. Follow a 4-step Chain of Thought before writing the output classification:
   - step_1_robot_model: Determine if robots are Single-Task (ST) or Multi-Task (MT). By TEMPO system formulation, all robots in the fleet are ST (capable of executing at most one subtask at any instant).
   - step_2_task_model: Determine if subtasks are Single-Robot (SR) or Multi-Robot (MR). Compare the weight of each object against the individual payload capacities of the qualified robots in the fleet. If any object weight strictly exceeds the maximum individual payload capacity of any single robot, moving it strictly requires a synchronized coalition of >= 2 robots -> MR. If all objects can be handled by an individual robot -> SR.
   - step_3_time_horizon: Determine if allocation is Instantaneous Assignment (IA) or Time-Extended Assignment (TA). If all task chains can be dispatched concurrently right now by the available qualified robots without multi-trip backlogs (|tasks| <= |qualified robots|) -> IA. If the number of tasks exceeds available qualified robots (|tasks| > |qualified robots|) or capacity bottlenecks force robots to plan sequential trips over time -> TA.
   - step_4_dependency: Determine if task execution has No Dependencies (ND), In-Schedule Dependencies (ID), Cross-Schedule Dependencies (XD), or Complex Dependencies (CD). If independent subtask chains return to origin with zero cross-robot coupling -> ND. If a robot performs multi-stop routing tours where the visit order within that robot's schedule affects travel cost -> ID. If tasks are linked by inter-robot precedence constraints or multi-robot coalition synchronization -> XD.

2. The output subclass string must strictly be composed of the 4 axes values:
   subclass = "<dependency>[<robot_model>-<task_model>-<time_horizon>]"

3. The subclass must strictly be one of the 7 supported TEMPO target subclasses:
   - ND[ST-SR-IA]
   - ND[ST-SR-TA]
   - ID[ST-SR-TA]
   - XD[ST-SR-IA]
   - XD[ST-SR-TA]
   - XD[ST-MR-IA]
   - XD[ST-MR-TA]

4. Separation of Concerns: NEVER assign specific robots to tasks and NEVER predict or mention downstream mathematical solvers (e.g., Hungarian, OR-Tools, PuLP). Your sole job is taxonomy classification.

5. Output ONLY raw JSON. No explanation outside the JSON, no markdown formatting, no code fences."""

TAXONOMY_OUTPUT_SCHEMA = """{
  "chain_of_thought": {
    "step_1_robot_model": "<reasoning for ST or MT>",
    "step_2_task_model": "<numerical reasoning for SR or MR comparing object weights to robot payload capacities>",
    "step_3_time_horizon": "<temporal horizon reasoning comparing task count to available qualified robots>",
    "step_4_dependency": "<dependency reasoning analyzing disjointness, routing tours, or precedence/synchronization>"
  },
  "output_classification": {
    "axes": {
      "robot_model": "ST",
      "task_model": "SR" or "MR",
      "time_horizon": "IA" or "TA",
      "dependency": "ND" or "ID" or "XD"
    },
    "subclass": "<canonical notation: D[R-T-H]>"
  }
}"""


def build_axes_reference_block(axes_data: Dict[str, Any]) -> str:
    """Formats the formal taxonomy axes definitions and target subclass criteria."""
    lines = ["=== TAXONOMY FORMAL DEFINITIONS & CRITERIA ==="]

    lines.append("\n--- AXES DEFINITIONS ---")
    for axis_name, axis_info in axes_data.get("axes", {}).items():
        lines.append(f"\nAxis: {axis_name.upper()} ({axis_info.get('description', '')})")
        for val_name, val_info in axis_info.get("values", {}).items():
            lines.append(f"  [{val_name}]: {val_info.get('definition', '')}")
            lines.append(f"    Example: {val_info.get('example', '')}")

    lines.append("\n--- TARGET SUBCLASS CRITERIA ---")
    for sc_name, sc_info in axes_data.get("subclasses", {}).items():
        lines.append(f"\nSubclass: {sc_name}")
        lines.append(f"  Name: {sc_info.get('name', '')}")
        lines.append(f"  Complexity: {sc_info.get('complexity', '')}")
        lines.append(f"  Criteria: {sc_info.get('criteria', '')}")
        lines.append(f"  Optimization Target: {sc_info.get('optimization_target', '')}")

    return "\n".join(lines)


def build_exemplars_reference_block(exemplars: List[Dict[str, Any]]) -> str:
    """Formats all 7 benchmark exemplars showing the 4-step Chain of Thought."""
    lines = ["=== REFERENCE BENCHMARK EXEMPLARS (1 PER SUBCLASS) ==="]

    for i, ex in enumerate(exemplars, 1):
        lines.append(f"\n========== EXEMPLAR {i}: {ex.get('name', ex.get('id'))} ==========")
        lines.append(
            f"Input Mission:\n{json.dumps(ex.get('input', {}), indent=2)}"
        )
        lines.append(
            f"Chain of Thought:\n{json.dumps(ex.get('chain_of_thought', {}), indent=2)}"
        )
        lines.append(
            f"Output Classification:\n{json.dumps(ex.get('output_classification', {}), indent=2)}"
        )

    return "\n".join(lines)


def build_taxonomy_prompt(
    subtasks: List[Dict[str, Any]],
    fleet: Dict[str, Any],
    objects: Dict[str, Any],
    axes_data: Dict[str, Any] = None,
    exemplars: List[Dict[str, Any]] = None,
) -> str:
    """
    Assembles the complete taxonomy classification prompt including the full taxonomy
    definitions, all 7 benchmark exemplars, the output schema, and the candidate mission.
    """
    if axes_data is None:
        axes_data = load_taxonomy_axes()
    if exemplars is None:
        exemplars = load_taxonomy_exemplars()

    axes_block = build_axes_reference_block(axes_data)
    exemplars_block = build_exemplars_reference_block(exemplars)

    candidate_input = {
        "subtasks": subtasks,
        "fleet": fleet,
        "objects": objects,
    }

    return f"""{axes_block}

{exemplars_block}

=== OUTPUT SCHEMA ===
Output ONLY raw JSON matching this structure:
{TAXONOMY_OUTPUT_SCHEMA}

=== CANDIDATE MISSION TO CLASSIFY ===
Analyze the following subtasks, robot fleet specifications, and object weights:
{json.dumps(candidate_input, indent=2)}

Perform the 4-step Chain of Thought and output ONLY the raw JSON classification:
"""
