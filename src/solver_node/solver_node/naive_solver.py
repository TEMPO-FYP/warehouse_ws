"""Core task allocation logic for the TEMPO Solver node."""

import copy
from typing import Any, Dict, List, Optional, Set

import networkx as nx


def _select_robot_for_chain(
    req_skills: Set[str],
    fleet_robots: List[str],
    fleet_specs: Dict[str, Dict[str, Any]],
    load_count: Dict[str, int],
    fallback_idx: int,
) -> str:
    """Select the most suitable robot matching required skills and balancing load."""
    eligible = [
        r for r in fleet_robots
        if req_skills.issubset(set(fleet_specs.get(r, {}).get('skills', [])))
    ]
    if not eligible:
        return fleet_robots[fallback_idx % len(fleet_robots)]

    order_idx = {r: i for i, r in enumerate(fleet_robots)}
    # Prefer balanced load first among specialists with minimal excess capabilities
    # so navigation-only chains route to quadruped_transport_robot before general manipulators.
    return min(
        eligible,
        key=lambda r: (
            len(set(fleet_specs.get(r, {}).get('skills', []))) - len(req_skills)
            if load_count[r] == 0
            else len(set(fleet_specs.get(r, {}).get('skills', []))) - len(req_skills) + 100,
            load_count[r],
            order_idx[r],
        ),
    )


def allocate_subtasks(
    subtasks: List[Dict[str, Any]],
    fleet_robots: Optional[List[str]] = None,
    fleet_specs: Optional[Dict[str, Dict[str, Any]]] = None,
) -> List[Dict[str, Any]]:
    """
    Allocate robot IDs to a list of subtasks.

    Identifies independent subtask chains using weakly connected components on the DAG.
    When fleet_specs are supplied, filters candidates by required_skills and prefers
    exact-capability specialists (e.g., quadruped_transport_robot for navigate_to chains); otherwise
    allocates each chain round-robin across fleet_robots.

    :param subtasks: List of subtask dictionaries from the LLM planner.
    :param fleet_robots: Optional list of available robot IDs.
    :param fleet_specs: Optional mapping of robot ID -> {'skills': [...], ...}.
    :return: A new list of subtask dictionaries with the 'robot' field populated.
    """
    if not subtasks:
        return []

    if not fleet_robots:
        fleet_robots = ['mobile_robot_1']

    # Reconstruct the dependency DAG
    G = nx.DiGraph()
    task_by_id: Dict[str, Dict[str, Any]] = {}
    for task in subtasks:
        G.add_node(task['id'])
        task_by_id[task['id']] = task

    for task in subtasks:
        for dep_id in task.get('dependencies', []):
            if dep_id in G:
                G.add_edge(dep_id, task['id'])

    # Extract independent components (chains)
    raw_components = list(nx.weakly_connected_components(G))

    # Deterministic sorting based on the order tasks appear in the plan
    task_order = {t['id']: i for i, t in enumerate(subtasks)}
    components = sorted(
        raw_components,
        key=lambda comp: min(task_order.get(tid, 0) for tid in comp),
    )

    # Map each component to a robot
    robot_map: Dict[str, str] = {}
    load_count: Dict[str, int] = {r: 0 for r in fleet_robots}
    use_skill_matching = bool(
        fleet_specs and any('skills' in (fleet_specs.get(r) or {}) for r in fleet_robots)
    )
    for idx, comp in enumerate(components):
        if use_skill_matching and fleet_specs is not None:
            req_skills: Set[str] = {
                s
                for tid in comp
                for s in task_by_id[tid].get('required_skills', [])
            }
            assigned_robot = _select_robot_for_chain(
                req_skills, fleet_robots, fleet_specs, load_count, idx
            )
        else:
            assigned_robot = fleet_robots[idx % len(fleet_robots)]
        load_count[assigned_robot] = load_count.get(assigned_robot, 0) + 1
        for tid in comp:
            robot_map[tid] = assigned_robot

    # Populate 'robot' in assigned subtasks
    assigned_subtasks: List[Dict[str, Any]] = []
    for task in subtasks:
        task_copy = copy.deepcopy(task)
        task_copy['robot'] = robot_map.get(task['id'], fleet_robots[0])
        assigned_subtasks.append(task_copy)

    return assigned_subtasks
