# Copyright 2026 TEMPO Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Unit tests for TransportRobotController configuration and dispatch logic."""

import json
import os

from ament_index_python.packages import get_package_share_directory
import rclpy
from robot_controllers.transport_robot_controller import TransportRobotController


def test_transport_robot_registered_in_kb():
    """Verify quadruped_transport_robot exists in llm_planner robot_registry.json."""
    pkg_dir = get_package_share_directory('llm_planner')
    reg_path = os.path.join(pkg_dir, 'knowledge_base', 'robot_registry.json')
    with open(reg_path, 'r', encoding='utf-8') as f:
        robots = {r['robot_type']: r for r in json.load(f)['robots']}
    assert 'quadruped_transport_robot' in robots
    tr = robots['quadruped_transport_robot']
    assert tr['skill_ids'] == ['navigate_to']
    assert 'cargo_transport' in tr['capabilities']
    assert tr['specs']['payload_capacity_kg'] == 15.0


def test_transport_controller_rejects_manipulation_and_resolves_zone():
    """Verify unsupported skills fail cleanly and named warehouse zones resolve free."""
    rclpy.init()
    node = TransportRobotController()
    published = []
    node._feedback_pub = type(
        '_PubStub', (), {'publish': lambda _self, msg: published.append(json.loads(msg.data))}
    )()
    try:
        # Manipulation skill must fail cleanly with unsupported_skill stage.
        node._handle_subtask({
            'plan_id': 'p1',
            'task_id': 't1',
            'robot': 'quadruped_transport_robot_1',
            'required_skills': ['pick'],
            'args': {'object': 'small_box'},
        })
        assert len(published) == 1
        assert published[0]['status'] == 'failed'
        assert published[0]['stage'] == 'unsupported_skill'

        # Unknown robot instance fails with config_error.
        node._handle_subtask({
            'plan_id': 'p1',
            'task_id': 't2',
            'robot': 'quadruped_transport_robot_99',
            'required_skills': ['navigate_to'],
            'args': {'location': 'staging_area'},
        })
        assert published[1]['status'] == 'failed'
        assert published[1]['stage'] == 'config_error'

        # Subtask targeted at mobile_robot must be ignored by transport controller.
        node._handle_subtask({
            'plan_id': 'p1',
            'task_id': 't3',
            'robot': 'mobile_robot_1',
            'required_skills': ['pick'],
            'args': {},
        })
        assert len(published) == 2

        # Named warehouse free zone resolves without collision.
        pose, is_named = node._resolve_target_pose('staging_area')
        assert is_named and pose is not None
        gx, gy, _ = node._compute_nav_goal(pose[0], pose[1], 0.0, node._standoff, is_named)
        assert abs(gx - 0.0) < 1e-6 and abs(gy - 16.0) < 1e-6
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
