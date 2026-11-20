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

"""Unit tests for multi-robot MobileRobotController, MoveIt 2 helpers, and PlanningSceneManager."""

import json
import math
import threading
import time
from unittest.mock import MagicMock

from moveit_msgs.msg import CollisionObject
import rclpy
from robot_controllers.mobile_robot_controller import _load_config, MobileRobotController
from robot_controllers.moveit_client import parse_srdf_group_states
from robot_controllers.planning_scene_manager import PlanningSceneManager
from std_msgs.msg import String


def _init_node_with_poses():
    node = MobileRobotController()
    node._pose_registry['b1z1'] = (1.0, 2.0, 0.56, (0.0, 0.0, 0.0, 1.0))
    node._joint_positions['b1z1_1'].update({
        'joint1': 0.0,
        'joint2': 1.8,
        'joint3': -0.95,
        'joint4': -0.85,
        'joint5': 0.0,
        'joint6': 1.57,
    })
    psm = MagicMock()
    psm.add_box.return_value = True
    psm.attach_object.return_value = True
    psm.detach_object.return_value = True
    psm.remove_object.return_value = True
    node._scene_managers['b1z1_1'] = psm
    node._gz_node = MagicMock()
    node._gz_node.request.return_value = (True, MagicMock(data=True))
    return node


def test_controller_config_exposes_b1z1_and_husky_profiles():
    """Verify multi-robot config supports both b1z1 and husky_ur5e without named_locations."""
    cfg = _load_config()
    assert 'named_locations' not in cfg.get('navigation', {})
    assert 'b1z1' in cfg['robot_profiles']
    assert 'husky_ur5e' in cfg['robot_profiles']
    assert cfg['robot_profiles']['b1z1']['namespace'] == '/b1z1'
    assert cfg['robot_profiles']['husky_ur5e']['namespace'] == '/a200_0000'
    assert cfg['robot_profiles']['husky_ur5e']['manipulation']['gripper_mode'] == 'gripper_command'


def test_parse_srdf_group_states():
    srdf = """
    <robot name="b1z1">
      <group_state name="pre_fold" group="z1_arm">
        <joint name="joint1" value="0.0"/>
        <joint name="joint6" value="1.57"/>
      </group_state>
      <group_state name="open" group="z1_gripper">
        <joint name="jointGripper" value="-1.50"/>
      </group_state>
    </robot>
    """
    parsed = parse_srdf_group_states(srdf)
    assert math.isclose(parsed['z1_arm']['pre_fold']['joint6'], 1.57)
    assert math.isclose(parsed['z1_gripper']['open']['jointGripper'], -1.50)


def test_kinematic_attach_uses_tf_pose_and_updates():
    """Verify proximity gating, TF2 TCP tracking, MoveIt attach, and Gazebo set_pose."""
    rclpy.init()
    node = _init_node_with_poses()
    try:
        pinch_quat = (math.sin(math.pi / 4.0), 0.0, 0.0, math.cos(math.pi / 4.0))
        tcp_pos = (1.65, 2.0, 0.72)
        node._lookup_frame_in_map = MagicMock(return_value=(*tcp_pos, pinch_quat))
        node._gz_attach_pubs['b1z1_1'] = MagicMock()
        node._gz_attach_pubs['b1z1_1'].publish.return_value = True
        node._gz_detach_pubs['b1z1_1'] = MagicMock()
        node._gz_detach_pubs['b1z1_1'].publish.return_value = True

        prof = node._profiles['b1z1_1']
        assert math.isclose(prof.grasp_pitch_rad, 0.26)
        assert math.isclose(prof.grasp_lateral_offset_m, 0.028)

        # Out-of-range object must be rejected before attaching.
        node._pose_registry['far_box'] = (
            tcp_pos[0] + 0.50, tcp_pos[1], tcp_pos[2], (0.0, 0.0, 0.0, 1.0)
        )
        err = node._attach_kinematic_object('b1z1_1', 'far_box')
        assert err is not None and 'too far' in err
        assert node._attached_objects['b1z1_1'] is None

        # Object within attach_max_distance_m attaches in MoveIt + Gazebo DynamicAttachSystem.
        node._pose_registry['small_crate'] = (
            tcp_pos[0] + 0.05, tcp_pos[1], tcp_pos[2], (0.0, 0.0, 0.0, 1.0)
        )
        err = node._attach_kinematic_object('b1z1_1', 'small_crate')
        assert err is None
        assert node._attached_objects['b1z1_1'] == 'small_crate'
        node._scene_managers['b1z1_1'].add_box.assert_called_once()
        node._scene_managers['b1z1_1'].attach_object.assert_called_once_with(
            'small_crate', link_name='b1z1_gripper_tcp'
        )
        attach_pub = node._gz_attach_pubs['b1z1_1']
        assert attach_pub.publish.called
        assert attach_pub.publish.call_args[0][0].data == 'small_crate'

        # Detaching publishes Gazebo detach, clears state, and removes from MoveIt scene.
        node._clear_attachment('b1z1_1', 'small_crate')
        assert node._attached_objects['b1z1_1'] is None
        assert node._gz_detach_pubs['b1z1_1'].publish.called
        node._scene_managers['b1z1_1'].detach_object.assert_called_once_with(
            'small_crate', link_name='b1z1_gripper_tcp'
        )
        node._scene_managers['b1z1_1'].remove_object.assert_called_once_with('small_crate')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_execute_place_verifies_held_object_even_when_args_omit_object():
    """Verify _execute_place verifies and clears _attached_objects even if args lacks 'object'."""
    rclpy.init()
    node = _init_node_with_poses()
    try:
        node._pose_registry['Packing_Station'] = (1.72, 2.0, 0.0, (0.0, 0.0, 0.0, 1.0))
        node._attached_objects['b1z1_1'] = 'graspable_box'
        node._ensure_aligned = MagicMock(return_value=(True, ''))
        node._move_arm_to_pose = MagicMock(return_value=True)
        node._open_gripper = MagicMock(return_value=True)
        node._move_arm_state = MagicMock(return_value=True)
        feedbacks = []
        node._publish_feedback = lambda *a, **kw: feedbacks.append((a, kw))

        node._execute_place('plan_1', 'T4', 'b1z1_1', {'location': 'Packing_Station'})
        assert node._attached_objects['b1z1_1'] is None
        assert feedbacks[-1][1]['status'] == 'success'
        assert 'graspable_box' in feedbacks[-1][1]['detail']
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_navigate_standoff_distinguishes_fixtures_from_objects():
    rclpy.init()
    node = _init_node_with_poses()
    try:
        assert node._is_large_fixture('Picking_Shelves.001')
        assert node._is_large_fixture('Packing_Station')
        assert node._is_large_fixture('Pallet_Standard_2')
        assert not node._is_large_fixture('arbitrary_cube_1')
        assert not node._is_large_fixture('graspable_box')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_confirm_object_z_waits_for_bridged_pose():
    rclpy.init()
    node = _init_node_with_poses()
    try:
        node._profiles['b1z1_1'].use_kinematic_attach = False
        node._pose_registry['crate'] = (1.0, 2.0, 0.70, (0.0, 0.0, 0.0, 1.0))

        z_stale = node._confirm_object_z('b1z1_1', 'crate', 0.90, timeout_sec=0.4)
        assert z_stale is not None and math.isclose(z_stale, 0.70, abs_tol=1e-9)

        node._pose_registry['crate'] = (1.0, 2.0, 0.95, (0.0, 0.0, 0.0, 1.0))
        z_ok = node._confirm_object_z('b1z1_1', 'crate', 0.90, timeout_sec=1.0)
        assert z_ok is not None and z_ok >= 0.90
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_planning_scene_manager_namespacing_and_attach_detach():
    """Verify PlanningSceneManager uses namespaced services and custom touch_links."""
    rclpy.init()
    node = rclpy.create_node('test_psm_node')
    try:
        custom_links = ['b1z1_gripper_tcp', 'b1z1_gripperStator']
        psm = PlanningSceneManager(
            node, spin_locally=False, namespace='b1z1', touch_links=custom_links
        )
        assert psm._apply_srv_name == '/b1z1/apply_planning_scene'
        assert psm._get_srv_name == '/b1z1/get_planning_scene'

        sent_scenes = []
        psm.apply_scene_diff = lambda scene: sent_scenes.append(scene) or True

        assert psm.add_box(
            'cargo_1', (0.1, 0.08, 0.06), (0.7, 0.0, 0.1), frame_id='b1z1_base_link'
        )
        assert psm.attach_object('cargo_1', link_name='b1z1_gripper_tcp')
        assert 'cargo_1' in psm._tracked_attached
        assert psm.detach_object('cargo_1')
        assert 'cargo_1' not in psm._tracked_attached
        assert psm.remove_object('cargo_1')
        assert 'cargo_1' not in psm._tracked_objects

        assert sent_scenes[0].world.collision_objects[0].header.frame_id == 'b1z1_base_link'
        attached = sent_scenes[1].robot_state.attached_collision_objects[0]
        assert attached.link_name == 'b1z1_gripper_tcp'
        assert attached.object.operation == CollisionObject.ADD
        assert list(attached.touch_links) == custom_links
        assert sent_scenes[2].robot_state.attached_collision_objects[0].object.operation == (
            CollisionObject.REMOVE
        )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_subtask_queue_serializes_wave_without_blocking_callback():
    """Verify _subtask_callback returns immediately and worker executes queued tasks in order."""
    rclpy.init()
    node = _init_node_with_poses()
    try:
        started_first = threading.Event()
        release_first = threading.Event()
        executed = []

        def fake_nav(plan_id, task_id, robot, args):
            executed.append(('start', task_id))
            if task_id == 'T1':
                started_first.set()
                release_first.wait(timeout=2.0)
            executed.append(('done', task_id))

        node._execute_navigate_to = fake_nav

        t0 = time.monotonic()
        for tid in ('T1', 'T2', 'T3'):
            msg = String()
            msg.data = json.dumps({
                'plan_id': 'p1',
                'task_id': tid,
                'robot': 'b1z1_1',
                'required_skills': ['navigate_to'],
                'args': {'location': 'Packing_Station'},
            })
            node._subtask_callback(msg)
        enqueue_elapsed = time.monotonic() - t0

        assert enqueue_elapsed < 0.2
        assert started_first.wait(timeout=2.0)
        # While T1 is blocked, T2 and T3 must still be queued, not started.
        time.sleep(0.05)
        assert executed == [('start', 'T1')]

        release_first.set()
        node._robot_queues['b1z1_1'].join()
        assert executed == [
            ('start', 'T1'), ('done', 'T1'),
            ('start', 'T2'), ('done', 'T2'),
            ('start', 'T3'), ('done', 'T3'),
        ]
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_ensure_aligned_enforces_min_reach_m():
    """Verify targets closer than prof.min_reach_m are rejected after alignment."""
    rclpy.init()
    node = _init_node_with_poses()
    try:
        node._profiles['b1z1_1'].fine_align_pulses = False
        node._send_nav_goal = MagicMock(return_value=(True, ''))
        # Robot is at (1.0, 2.0, yaw=0); place target only 0.28m ahead (< min_reach_m=0.40).
        ok, err = node._ensure_aligned('b1z1_1', 1.28, 2.0, 'T_close', 0.85, 0.73, tight=True)
        assert not ok
        assert 'Alignment outside manipulation window' in err
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_unmanaged_mobile_robot_prefix_ignored():
    """Verify mobile_robot_* subtasks are ignored unless configured in robot_instances."""
    rclpy.init()
    node = _init_node_with_poses()
    try:
        feedbacks = []
        node._publish_feedback = lambda *a, **kw: feedbacks.append((a, kw))
        msg = String()
        msg.data = json.dumps({
            'plan_id': 'p1',
            'task_id': 'T1',
            'robot': 'mobile_robot_1',
            'required_skills': ['navigate_to'],
            'args': {'location': 'Packing_Station'},
        })
        node._subtask_callback(msg)
        assert feedbacks == []
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
