"""
dag_builder.py

Builds a NetworkX DAG from a validated plan and runs validation checks
on the plan structure.

Two responsibilities:
  1. build_dag(): converts a plan's subtask dependency lists into a
     NetworkX DiGraph where nodes are subtask ids and edges represent
     execution ordering constraints.
  2. validate_plan(): runs five checks on the plan and DAG to catch
     errors before the plan is dispatched to the solver.

Validation checks:
  1. No duplicate subtask ids.
  2. No cycles in the DAG.
  3. No dangling dependency references.
  4. All required_skills exist in the retrieved skill bank.
  5. Args keys match the skill's inputs exactly.
"""

import networkx as nx


# DAG builder

def build_dag(plan: dict) -> nx.DiGraph:
    """
    Builds a NetworkX DiGraph from the plan's subtask dependency lists.

    Node  = subtask id
    Edge A -> B = A must complete before B starts
    """
    G = nx.DiGraph()

    for task in plan["subtasks"]:
        G.add_node(task["id"], **task)

    for task in plan["subtasks"]:
        for dep_id in task["dependencies"]:
            G.add_edge(dep_id, task["id"])

    return G


# Validator

def validate_plan(plan: dict, G: nx.DiGraph,
                  skill_bank: list[dict]) -> list[str]:
    """
    Validates the plan against five checks. Returns a list of error
    strings. An empty list means the plan is valid.

    skill_bank: the list of skill entries from skill_retriever, used
                for checks 4 and 5.

    Check 1: No duplicate subtask ids.
             Every subtask in the plan must have a unique id.

    Check 2: No cycles in the DAG.

    Check 3: No dangling dependency references.
             Every id in a subtask's dependencies must refer to another
             subtask that actually exists in this plan.

    Check 4: All required_skills exist in the retrieved skill bank.
             The LLM must only use skills it was shown in the prompt.

    Check 5: Args keys match the skill's inputs exactly.
             The keys in a subtask's args must match the input names
             listed for that skill in the skill bank. 
    """
    errors = []

    # Build lookups from the skill bank.
    valid_skills   = {s["name"] for s in skill_bank}
    skill_inputs   = {s["name"]: set(s["inputs"]) for s in skill_bank}
    task_ids       = {t["id"] for t in plan["subtasks"]}

    # Check 1, duplicate task ids.
    seen_ids = set()
    duplicate_ids = set()
    for task in plan["subtasks"]:
        tid = task["id"]
        if tid in seen_ids:
            duplicate_ids.add(tid)
        seen_ids.add(tid)

    if duplicate_ids:
        errors.append(
            f"DUPLICATE_TASK_ID: duplicate subtask ids found: {sorted(duplicate_ids)}. "
            f"Every subtask must have a unique id."
        )

    # Check 2, cycle detection.
    if not nx.is_directed_acyclic_graph(G):
        try:
            cycle = nx.find_cycle(G)
            errors.append(
                f"CYCLE_DETECTED: dependency graph has a cycle: {cycle}. "
                f"No task may depend on its own successor."
            )
        except nx.NetworkXNoCycle:
            errors.append(
                "CYCLE_DETECTED: cycle found but could not be identified."
            )

    # Check 3, dangling dependency references.
    for task in plan["subtasks"]:
        for dep in task["dependencies"]:
            if dep not in task_ids:
                errors.append(
                    f"INVALID_DEPENDENCY: task '{task['id']}' depends on "
                    f"'{dep}' which does not exist in this plan. "
                    f"Valid ids: {sorted(task_ids)}"
                )

    # Check 4, skill validity.
    for task in plan["subtasks"]:
        for skill in task["required_skills"]:
            if skill not in valid_skills:
                errors.append(
                    f"INVALID_SKILL: task '{task['id']}' uses skill "
                    f"'{skill}' which was not in the retrieved skill bank. "
                    f"Valid skills: {sorted(valid_skills)}"
                )

    # Check 5, args keys match skill inputs.
    for task in plan["subtasks"]:
        skill = task["required_skills"][0] if task["required_skills"] else None
        if skill not in skill_inputs:
            continue

        expected_inputs = skill_inputs[skill]
        actual_args     = set(task.get("args", {}).keys())

        missing = expected_inputs - actual_args
        extra   = actual_args - expected_inputs

        if missing:
            errors.append(
                f"MISSING_ARGS: task '{task['id']}' uses skill '{skill}' "
                f"but is missing required args: {sorted(missing)}. "
                f"Expected inputs: {sorted(expected_inputs)}"
            )
        if extra:
            errors.append(
                f"EXTRA_ARGS: task '{task['id']}' uses skill '{skill}' "
                f"but has unexpected args: {sorted(extra)}. "
                f"Expected inputs: {sorted(expected_inputs)}"
            )

    return errors