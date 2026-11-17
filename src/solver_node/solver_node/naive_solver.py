"""Core task allocation logic for the TEMPO Solver node."""

import copy
from typing import Any, Dict, List, Optional, Set

import networkx as nx


class NoCapableRobotError(ValueError):
    """Raised when no robot in the fleet has every skill a subtask chain needs."""


def allocate_subtasks(
    subtasks: List[Dict[str, Any]],
    fleet_robots: Optional[List[str]] = None,
    robot_skills: Optional[Dict[str, List[str]]] = None,
) -> List[Dict[str, Any]]:
    """
    Allocate robot IDs to a list of subtasks.

    Identifies independent subtask chains using weakly connected components on the DAG.
    Each chain is allocated to a single robot, ensuring all subtasks in a sequential
    chain execute on the same robot. Only robots whose skills cover every skill in the
    chain are candidates; among those, the robot with the fewest chains so far is chosen
    (ties broken by fleet order), which reduces to round-robin for a homogeneous fleet.

    :param subtasks: List of subtask dictionaries from the LLM planner.
    :param fleet_robots: Optional list of available robot IDs.
    :param robot_skills: Optional map of robot ID -> skill IDs it can execute.
        Robots absent from this map are assumed capable of any skill.
    :return: A new list of subtask dictionaries with the 'robot' field populated.
    :raises NoCapableRobotError: If no robot can execute some chain.
    """
    if not subtasks:
        return []

    if not fleet_robots:
        fleet_robots = ['mobile_robot_1']

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

    robot_skills = robot_skills or {}
    task_by_id = {t['id']: t for t in subtasks}

    def _can_execute(robot: str, needed: Set[str]) -> bool:
        if robot not in robot_skills:
            return True
        return needed.issubset(robot_skills[robot])

    # Map each component to the least-loaded capable robot
    load = {robot: 0 for robot in fleet_robots}
    robot_map: Dict[str, str] = {}
    for comp in components:
        needed = {
            skill
            for tid in comp
            for skill in task_by_id[tid].get('required_skills', [])
        }
        candidates = [r for r in fleet_robots if _can_execute(r, needed)]
        if not candidates:
            raise NoCapableRobotError(
                f'No robot in fleet {fleet_robots} can execute skills '
                f'{sorted(needed)} required by subtasks {sorted(comp)}.'
            )
        assigned_robot = min(candidates, key=lambda r: load[r])
        load[assigned_robot] += 1
        for tid in comp:
            robot_map[tid] = assigned_robot

    # Populate 'robot' in assigned subtasks
    assigned_subtasks: List[Dict[str, Any]] = []
    for task in subtasks:
        task_copy = copy.deepcopy(task)
        task_copy['robot'] = robot_map.get(task['id'], fleet_robots[0])
        assigned_subtasks.append(task_copy)

    return assigned_subtasks
