"""
prompt_builder.py

Contains the system prompt and the prompt assembly function for the
mission planner.

Follows DART-LLM's P = (I, E, S, F) prompt structure:
  I = instruction
  E = environment (TODO: replace placeholder once environment module is ready)
  S = skills (retrieved from skill_retriever)
  F = few-shot examples (retrieved from few_shot_retriever)
"""

import itertools

from llm_planner.rag.few_shot_retrieval import (
    CosineFewShotRetriever,
    format_examples_for_prompt,
)
from llm_planner.rag.skill_retrieval import build_skill_prompt_block
from llm_planner.config import FEW_SHOT_K


# Plan ID generator, increments on every decompose_instruction() call.
_plan_counter = itertools.count(1)

def _next_plan_id() -> str:
    "Returns the next unique plan ID"
    return f"plan_{next(_plan_counter):03d}"


# System prompt

SYSTEM_PROMPT = """You are a task decomposition planner for a multi-robot system.
Your job is to take a high-level natural language instruction and break it down
into atomic subtasks that the robot fleet can execute.

Rules you must follow:

1. BEFORE decomposing, classify the instruction into exactly one status:
   "ok", "clarification_needed", or "infeasible".

2. Use "ok" only when the instruction is clear, physically executable,
   and can be decomposed using only the provided skills.

3. Use "clarification_needed" ONLY when the instruction contains unresolvable
   deictic pronouns such as "it", "that thing", or "over there" where the
   object type or target zone itself is unknown, or when a requested object
   type does not exist anywhere in the grounded ENVIRONMENT.
   IMPORTANT: Indefinite phrases like "a small box from aisle 3", "a pallet
   from the staging area", or "a packing station" are NOT ambiguous! When the
   user asks for "a" or "any" item/station of a given type, deterministically
   select the first available grounded ID matching that type/aisle/zone from
   the ENVIRONMENT section (e.g., "small_box_027_1", "Packing_Station").

4. Use "infeasible" when the instruction is unambiguous but cannot be
   executed using the available skills, is physically impossible,
   or is self-contradictory.

5. For "clarification_needed" and "infeasible", return only status and
   reason. Do not produce a plan.

6. required_skills must only contain skill names from the provided
   skill list. No other values allowed.

7. Each subtask must have exactly one skill in required_skills.

8. Every subtask must have a unique id in the format T1, T2, T3 ...

9. dependencies must only reference ids of other subtasks in the
   same plan.

10. If a subtask has no prerequisites, set dependencies to an empty
    list [].

11. Think step by step about the physical actions needed and their
    order before writing JSON.

12. Output ONLY raw JSON. No explanation, no markdown, no code fences.

13. When an instruction acts on multiple distinct objects, each
    object's subtask chain is independent by default. Do NOT add
    a dependency from one object's chain to another unless the
    instruction explicitly requires ordering between them, or the
    physical action requires it.

14. Only add a dependency between two subtasks if one genuinely
    cannot start before the other completes. Unnecessary dependencies
    prevent parallel execution and reduce efficiency.

15. Never assign specific robots to subtasks. Your job is only to decompose
    the mission into subtasks and their dependencies.

16. The keys in a subtask's args must exactly match the input names
    listed for that skill in the skill list. For example, if a skill
    lists inputs [object, location], the args must be
    { "object": "...", "location": "..." } with no other key names.

17. A grounded 3D Scene Graph is provided in the ENVIRONMENT section.
    Every "object" and "location" value in args MUST use an exact ID
    listed in the ENVIRONMENT section (e.g., "small_box_001_1",
    "Picking_Shelves.001", "Packing_Station", "staging_area"). Never
    invent or guess entity IDs. If a requested object or location does
    not exist in the environment, return status "clarification_needed"
    or "infeasible"."""


# Output schema

_SCHEMA = """For status "ok", output this structure:
{
  "status": "ok",
  "plan_id": "<plan_id>",
  "original_instruction": "<copy the instruction exactly>",
  "subtasks": [
    {
      "id": "<T1, T2, T3 ...>",
      "description": "<what this subtask does in plain English>",
      "required_skills": ["<exactly one skill name from the skill list>"],
      "args": { "<input_name>": "<value>" },
      "dependencies": ["<ids of subtasks that must complete before this one, or empty list>"]
    }
  ]
}

For status "clarification_needed", output this structure and no plan:
{
  "status": "clarification_needed",
  "reason": "<short reason naming what is ambiguous>"
}

For status "infeasible", output this structure and no plan:
{
  "status": "infeasible",
  "reason": "<short reason naming why it cannot be executed>"
}"""


# Prompt assembly

def build_prompt(
    instruction: str,
    skill_block: str,
    plan_id: str,
    few_shot_block: str,
    environment: str,
) -> str:
    """
    Assembles the full prompt following P = (I, E, S, F).

    instruction    : the natural language mission to decompose.
    skill_block    : formatted skill list from build_skill_prompt_block().
    plan_id        : unique plan ID to embed in the schema.
    few_shot_block : formatted examples from format_examples_for_prompt().
    environment    : grounded 3D Scene Graph environment context.
    """
    env_section = environment.strip() if environment else "None"

    schema_with_id = _SCHEMA.replace("<plan_id>", plan_id)

    return f"""
=== SKILLS ===
{skill_block}

=== ENVIRONMENT ===
{env_section}

=== OUTPUT SCHEMA ===
First classify the instruction, then output exactly one of these JSON
structures. Use this plan_id for status "ok": {plan_id}
{schema_with_id}

=== FEW-SHOT EXAMPLES ===
{few_shot_block}

=== YOUR TASK ===
Decompose the following instruction:
Instruction: "{instruction}"

Output ONLY the raw JSON. Nothing else.
"""


def prepare_prompt(
    instruction: str,
    few_shot_retriever: CosineFewShotRetriever,
    skill_retriever,
    environment: str,
    few_shot_k: int = FEW_SHOT_K,
) -> tuple[str, str]:
    """
    Retrieves skills and few-shot examples for the instruction, then
    assembles and returns the full prompt and the plan_id as a tuple.

    Returns (user_prompt, plan_id) so the caller can:
      - pass user_prompt to call_llm()
      - pass plan_id to the plan dict after parsing
    """
    # Retrieve relevant skills for this instruction.
    retrieved_skills = skill_retriever.retrieve(instruction)
    skill_block = build_skill_prompt_block(retrieved_skills)
    print("[Planner] Retrieved skills: " + ", ".join(
        f"{s['name']}({score:.2f})" for s, score in retrieved_skills
    ))

    # Retrieve relevant few-shot examples for this instruction.
    retrieved_examples = few_shot_retriever.retrieve(instruction, k=few_shot_k)
    few_shot_block = format_examples_for_prompt(retrieved_examples)
    print("[Planner] Retrieved examples: " + ", ".join(
        f"{ex['id']}({score:.2f})" for ex, score in retrieved_examples
    ))

    plan_id = _next_plan_id()

    user_prompt = build_prompt(
        instruction=instruction,
        skill_block=skill_block,
        plan_id=plan_id,
        few_shot_block=few_shot_block,
        environment=environment,
    )

    return user_prompt, plan_id
