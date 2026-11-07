"""Unit tests for TEMPO naive connected-chain task allocation logic."""

import networkx as nx
from solver_node.naive_solver import allocate_subtasks


def _build_two_chain_subtasks():
    return [
        {
            'id': 'T1',
            'description': 'Navigate to shelf A.',
            'required_skills': ['navigate_to'],
            'args': {'location': 'shelf_A'},
            'dependencies': [],
        },
        {
            'id': 'T2',
            'description': 'Pick up box at shelf A.',
            'required_skills': ['pick'],
            'args': {'object': 'box_shelf_A'},
            'dependencies': ['T1'],
        },
        {
            'id': 'T3',
            'description': 'Navigate to origin with box A.',
            'required_skills': ['navigate_to'],
            'args': {'location': 'origin'},
            'dependencies': ['T2'],
        },
        {
            'id': 'T4',
            'description': 'Place box A at origin.',
            'required_skills': ['place'],
            'args': {'object': 'box_shelf_A', 'location': 'origin'},
            'dependencies': ['T3'],
        },
        {
            'id': 'T5',
            'description': 'Navigate to shelf B.',
            'required_skills': ['navigate_to'],
            'args': {'location': 'shelf_B'},
            'dependencies': [],
        },
        {
            'id': 'T6',
            'description': 'Pick up box at shelf B.',
            'required_skills': ['pick'],
            'args': {'object': 'box_shelf_B'},
            'dependencies': ['T5'],
        },
        {
            'id': 'T7',
            'description': 'Navigate to origin with box B.',
            'required_skills': ['navigate_to'],
            'args': {'location': 'origin'},
            'dependencies': ['T6'],
        },
        {
            'id': 'T8',
            'description': 'Place box B at origin.',
            'required_skills': ['place'],
            'args': {'object': 'box_shelf_B', 'location': 'origin'},
            'dependencies': ['T7'],
        },
    ]


def test_empty_subtasks():
    assert allocate_subtasks([]) == []


def test_single_chain_allocation():
    subtasks = [
        {'id': 'T1', 'dependencies': []},
        {'id': 'T2', 'dependencies': ['T1']},
        {'id': 'T3', 'dependencies': ['T2']},
    ]
    assigned = allocate_subtasks(subtasks)
    assert len(assigned) == 3
    # All tasks in the single chain should be allocated to the same robot
    assert assigned[0]['robot'] == 'mobile_robot_1'
    assert assigned[1]['robot'] == 'mobile_robot_1'
    assert assigned[2]['robot'] == 'mobile_robot_1'


def test_two_chain_allocation():
    subtasks = _build_two_chain_subtasks()
    assigned = allocate_subtasks(subtasks)
    assert len(assigned) == 8

    # Chain 1 (T1-T4) allocated to mobile_robot_1
    for task in assigned[:4]:
        assert task['robot'] == 'mobile_robot_1'

    # Chain 2 (T5-T8) allocated to mobile_robot_2
    for task in assigned[4:]:
        assert task['robot'] == 'mobile_robot_2'


def test_single_task_per_wave_guarantee():
    """Verify that no robot is ever assigned more than one subtask per wave."""
    subtasks = _build_two_chain_subtasks()
    assigned = allocate_subtasks(subtasks)

    # Reconstruct DAG as dispatcher_node does
    G = nx.DiGraph()
    task_map = {}
    for t in assigned:
        task_map[t['id']] = t
        G.add_node(t['id'])

    for t in assigned:
        for dep in t['dependencies']:
            G.add_edge(dep, t['id'])

    waves = [list(gen) for gen in nx.topological_generations(G)]
    assert len(waves) == 4

    for wave_idx, wave in enumerate(waves):
        robots_in_wave = [task_map[tid]['robot'] for tid in wave]
        # Ensure no robot appears multiple times in the same wave
        assert len(robots_in_wave) == len(set(robots_in_wave)), (
            f'Wave {wave_idx + 1} ({wave}) has duplicate robot assignment: {robots_in_wave}'
        )


def test_custom_fleet():
    subtasks = _build_two_chain_subtasks()
    custom_fleet = ['tb3_0', 'tb3_1']
    assigned = allocate_subtasks(subtasks, fleet_robots=custom_fleet)

    assert assigned[0]['robot'] == 'tb3_0'
    assert assigned[4]['robot'] == 'tb3_1'


def test_immutability_of_input():
    subtasks = [{'id': 'T1', 'dependencies': []}]
    assigned = allocate_subtasks(subtasks)
    assert 'robot' not in subtasks[0]
    assert assigned[0]['robot'] == 'mobile_robot_1'
