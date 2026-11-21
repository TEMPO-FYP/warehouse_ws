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
    assert 'b2_cargo' in cfg['robot_profiles']
    assert cfg['robot_profiles']['b1z1']['namespace'] == '/b1z1'
    assert cfg['robot_profiles']['husky_ur5e']['namespace'] == '/a200_0000'
    assert cfg['robot_profiles']['husky_ur5e']['manipulation']['gripper_mode'] == 'gripper_command'
    assert cfg['robot_profiles']['b2_cargo']['namespace'] == '/b2_cargo'
    assert cfg['robot_profiles']['b2_cargo']['supports_manipulation'] is False
    assert cfg['robot_instances']['b2_cargo_1'] == 'b2_cargo'


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


def test_b2_cargo_nav_only_profile_rejects_manipulation():
    """Verify b2_cargo skips MoveIt/scene setup and fails pick/place as unsupported_skill."""
    rclpy.init()
    node = _init_node_with_poses()
    try:
        assert 'b2_cargo_1' in node._nav_clients
        assert 'b2_cargo_1' in node._cmd_vel_pubs
        assert 'b2_cargo_1' not in node._moveit_clients
        assert 'b2_cargo_1' not in node._scene_managers

        feedbacks = []
        node._publish_feedback = lambda *a, **kw: feedbacks.append((a, kw))
        for skill in ('pick', 'place'):
            node._handle_subtask({
                'plan_id': 'p1',
                'task_id': f'T_{skill}',
                'robot': 'b2_cargo_1',
                'required_skills': [skill],
                'args': {'object': 'small_box_1', 'location': 'Packing_Station'},
            })

        assert len(feedbacks) == 2
        for _, kw in feedbacks:
            assert kw['status'] == 'failed'
            assert kw['stage'] == 'unsupported_skill'
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_b2_cargo_subtask_queue_executes_navigate_to():
    """Verify navigate_to tasks for b2_cargo_1 flow through its dedicated worker queue."""
    rclpy.init()
    node = _init_node_with_poses()
    try:
        calls = []
        node._execute_navigate_to = (
            lambda plan_id, task_id, robot, args: calls.append((plan_id, task_id, robot, args))
        )
        msg = String()
        msg.data = json.dumps({
            'plan_id': 'p2',
            'task_id': 'T_nav',
            'robot': 'b2_cargo_1',
            'required_skills': ['navigate_to'],
            'args': {'location': 'Packing_Station'},
        })
        node._subtask_callback(msg)
        node._robot_queues['b2_cargo_1'].join()
        assert calls == [('p2', 'T_nav', 'b2_cargo_1', {'location': 'Packing_Station'})]
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_execute_place_into_b2_cargo_aims_at_crate_center():
    """Verify placing onto b2_cargo aims at cargo_rack center instead of clamping to 0.72."""
    rclpy.init()
    node = _init_node_with_poses()
    try:
        node._pose_registry['b2_cargo'] = (1.84, 2.0, 0.58, (0.0, 0.0, 0.0, 1.0))
        node._pose_registry['Packing_Station'] = (1.90, 2.0, 0.0, (0.0, 0.0, 0.0, 1.0))
        node._lookup_frame_in_map = MagicMock(
            return_value=(1.82, 2.0, 0.668, (0.0, 0.0, 0.0, 1.0))
        )
        align_calls = []
        node._ensure_aligned = (
            lambda robot, tx, ty, tid, max_d, pref_d, tight=True:
            align_calls.append((tid, pref_d, tight)) or (True, '')
        )
        recorded_poses = {}

        def fake_move_pose(robot, pose_stamped, label, **kwargs):
            recorded_poses[label] = pose_stamped
            return True

        node._move_arm_to_pose = fake_move_pose
        node._open_gripper = MagicMock(return_value=True)
        node._move_arm_state = MagicMock(return_value=True)
        feedbacks = []
        node._publish_feedback = lambda *a, **kw: feedbacks.append((a, kw))

        node._execute_place(
            'p1', 'T3', 'b1z1_1', {'object': 'tall_box', 'location': 'b2_cargo'}
        )

        assert align_calls == [('T3', 0.80, True)]
        place_pt = recorded_poses['T3_place_descend'].pose.position
        assert math.isclose(place_pt.x, 0.795, abs_tol=1e-6)
        assert math.isclose(place_pt.y, 0.028, abs_tol=1e-6)
        assert math.isclose(place_pt.z, 0.252, abs_tol=1e-6)
        assert feedbacks[-1][1]['status'] == 'success'

        # Large fixtures still clamp to place_drop_standoff (0.72 m).
        node._execute_place(
            'p1', 'T_pack', 'b1z1_1', {'object': 'tall_box', 'location': 'Packing_Station'}
        )
        pack_pt = recorded_poses['T_pack_place_descend'].pose.position
        assert math.isclose(pack_pt.x, 0.72, abs_tol=1e-6)

        assert node._is_inside_cargo_robot(1.82, 2.0) is True
        assert node._is_inside_cargo_robot(5.0, 5.0) is False
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_execute_pick_from_b2_cargo_raises_grasp_clearance_and_pitch():
    """Verify picking out of b2_cargo raises grasp Z clearance and wrist pitch."""
    rclpy.init()
    node = _init_node_with_poses()
    try:
        node._pose_registry['b2_cargo'] = (1.82, 2.0, 0.58, (0.0, 0.0, 0.0, 1.0))
        node._pose_registry['tall_box'] = (1.82, 2.0, 0.735, (0.0, 0.0, 0.0, 1.0))
        align_calls = []
        node._ensure_aligned = (
            lambda robot, tx, ty, tid, max_d, pref_d, tight=True:
            align_calls.append((tid, pref_d, tight)) or (True, '')
        )
        recorded_poses = {}

        def fake_move_pose(robot, pose_stamped, label, **kwargs):
            recorded_poses[label] = pose_stamped
            return True

        node._open_gripper = MagicMock(return_value=True)
        node._move_arm_state = MagicMock(return_value=True)
        node._move_arm_to_pose = fake_move_pose
        node._attach_kinematic_object = MagicMock(return_value=None)
        node._close_gripper = MagicMock(return_value=True)
        node._confirm_object_z = MagicMock(return_value=0.90)
        feedbacks = []
        node._publish_feedback = lambda *a, **kw: feedbacks.append((a, kw))

        node._execute_pick('p1', 'T5', 'b1z1_1', {'object': 'tall_box'})

        assert align_calls == [('T5', 0.80, True)]
        grasp_pt = recorded_poses['T5_grasp'].pose.position
        lift_pt = recorded_poses['T5_lift'].pose.position
        assert math.isclose(grasp_pt.z, 0.230, abs_tol=1e-6)
        assert math.isclose(lift_pt.z, 0.355, abs_tol=1e-6)
        assert math.isclose(lift_pt.x, 0.760, abs_tol=1e-6)
        assert feedbacks[-1][1]['status'] == 'success'
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_bcr_arm_profile_and_rejects_navigate_to():
    """Verify bcr_arm profile config and unsupported_skill rejection for navigate_to."""
    from robot_controllers.geometry_utils import z_approach_quat_tuple

    cfg = _load_config()
    assert 'bcr_arm' in cfg['robot_profiles']
    assert cfg['robot_instances']['bcr_arm_1'] == 'bcr_arm'
    assert cfg['robot_profiles']['bcr_arm']['supports_navigation'] is False
    assert cfg['robot_profiles']['bcr_arm']['supports_manipulation'] is True
    assert cfg['robot_profiles']['bcr_arm']['manipulation']['tcp_approach_axis'] == 'z'

    qx, qy, qz, qw = z_approach_quat_tuple(math.pi, 0.0)
    assert math.isclose(qx * qx + qy * qy + qz * qz + qw * qw, 1.0, abs_tol=1e-6)

    rclpy.init()
    node = _init_node_with_poses()
    try:
        assert 'bcr_arm_1' in node._moveit_clients
        assert 'bcr_arm_1' not in node._nav_clients
        assert 'bcr_arm_1' not in node._cmd_vel_pubs

        feedbacks = []
        node._publish_feedback = lambda *a, **kw: feedbacks.append((a, kw))
        node._handle_subtask({
            'plan_id': 'p_arm',
            'task_id': 'T_nav',
            'robot': 'bcr_arm_1',
            'required_skills': ['navigate_to'],
            'args': {'location': 'Packing_Station'},
        })
        assert len(feedbacks) == 1
        assert feedbacks[0][1]['status'] == 'failed'
        assert feedbacks[0][1]['stage'] == 'unsupported_skill'
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_bcr_arm_stationary_reachability_and_pick_place():
    """Verify 360-deg stationary reachability check and mocked pick/place on bcr_arm_1."""
    rclpy.init()
    node = _init_node_with_poses()
    try:
        psm = MagicMock()
        psm.add_box.return_value = True
        psm.attach_object.return_value = True
        psm.detach_object.return_value = True
        psm.remove_object.return_value = True
        node._scene_managers['bcr_arm_1'] = psm
        node._gz_attach_pubs['bcr_arm_1'] = MagicMock()
        node._gz_attach_pubs['bcr_arm_1'].publish.return_value = True
        node._gz_detach_pubs['bcr_arm_1'] = MagicMock()
        node._gz_detach_pubs['bcr_arm_1'].publish.return_value = True
        node._joint_positions['bcr_arm_1'].update({f'joint{i}': 0.0 for i in range(1, 8)})

        # Stationary base at (1.75, 13.08, 0.55) via TF2
        node._lookup_frame_in_map = MagicMock(
            return_value=(1.75, 13.08, 0.55, (0.0, 0.0, 0.0, 1.0))
        )

        # Target behind base (-X direction in base_link, 0.55 m away) must be reachable!
        ok_rear, err_rear = node._ensure_aligned('bcr_arm_1', 1.20, 13.08, 'T_pick', 0.92, 0.72)
        assert ok_rear, err_rear

        # Target 1.50 m away must be rejected as out of reach.
        ok_far, err_far = node._ensure_aligned('bcr_arm_1', 3.25, 13.08, 'T_far', 0.92, 0.72)
        assert not ok_far
        assert 'reachability window' in err_far

        # Mocked pick & place execution
        node._pose_registry['graspable_box'] = (1.20, 13.08, 0.945, (0.0, 0.0, 0.0, 1.0))
        node._pose_registry['Packing_Station'] = (0.934, 13.08, 0.0, (0.0, 0.0, 0.0, 1.0))
        node._open_gripper = MagicMock(return_value=True)
        node._close_gripper = MagicMock(return_value=True)
        node._move_arm_state = MagicMock(return_value=True)
        recorded = {}

        def fake_move_pose(robot, ps, label, **kwargs):
            recorded[label] = ps
            return True

        node._move_arm_to_pose = fake_move_pose
        node._attach_kinematic_object = MagicMock(return_value=None)
        node._confirm_object_z = MagicMock(return_value=1.10)
        feedbacks = []
        node._publish_feedback = lambda *a, **kw: feedbacks.append((a, kw))

        node._execute_pick('p1', 'T1', 'bcr_arm_1', {'object': 'graspable_box'})
        assert feedbacks[-1][1]['status'] == 'success'
        assert feedbacks[-1][1]['stage'] == 'complete'
        assert recorded['T1_grasp'].pose.position.x < 0.0

        node._attached_objects['bcr_arm_1'] = 'graspable_box'
        node._execute_place(
            'p1', 'T2', 'bcr_arm_1', {'object': 'graspable_box', 'location': 'Packing_Station'}
        )
        assert feedbacks[-1][1]['status'] == 'success'
        assert feedbacks[-1][1]['stage'] == 'complete'
        assert math.hypot(
            recorded['T2_place_descend'].pose.position.x,
            recorded['T2_place_descend'].pose.position.y,
        ) <= 0.721
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
