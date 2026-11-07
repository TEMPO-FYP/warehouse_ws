"""Core task allocation logic for the TEMPO Solver node."""

import copy
from typing import Any, Dict, List, Optional

import networkx as nx

from solver_node.fleet_loader import get_default_fleet_robots


def allocate_subtasks(
    subtasks: List[Dict[str, Any]],
    fleet_robots: Optional[List[str]] = None,
) -> List[Dict[str, Any]]:
    """
    Allocate robot IDs to a list of subtasks.

    Identifies independent subtask chains using weakly connected components on the DAG.
    Each independent chain is allocated to a single robot in round-robin fashion,
    ensuring all subtasks in a sequential chain execute on the same robot.

    :param subtasks: List of subtask dictionaries from the LLM planner.
    :param fleet_robots: Optional list of available robot IDs. Defaults to standard fleet.
    :return: A new list of subtask dictionaries with the 'robot' field populated.
    """
    if not subtasks:
        return []

    if fleet_robots is None or len(fleet_robots) == 0:
        fleet_robots = get_default_fleet_robots()

    # Reconstruct the dependency DAG
    G = nx.DiGraph()
    for task in subtasks:
        G.add_node(task['id'])

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
    for idx, comp in enumerate(components):
        assigned_robot = fleet_robots[idx % len(fleet_robots)]
        for tid in comp:
            robot_map[tid] = assigned_robot

    # Populate 'robot' in assigned subtasks
    assigned_subtasks: List[Dict[str, Any]] = []
    for task in subtasks:
        task_copy = copy.deepcopy(task)
        task_copy['robot'] = robot_map.get(task['id'], fleet_robots[0])
        assigned_subtasks.append(task_copy)

    return assigned_subtasks
