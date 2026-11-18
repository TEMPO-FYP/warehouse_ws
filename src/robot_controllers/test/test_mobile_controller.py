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

"""Unit tests for MobileRobotController kinematic attach and standoff selection."""

import math
from unittest.mock import MagicMock

from moveit_msgs.msg import CollisionObject
import rclpy
from robot_controllers.mobile_robot_controller import MobileRobotController
from robot_controllers.planning_scene_manager import (
    DEFAULT_GRIPPER_TOUCH_LINKS,
    PlanningSceneManager,
)
from robot_controllers.z1_ik import fk_tcp_base_link, transform_base_to_world


def _init_node_with_poses():
    node = MobileRobotController()
    node._stop_attach_thread = True
    node._attach_event.set()
    if node._attach_thread is not None and node._attach_thread.is_alive():
        node._attach_thread.join(timeout=1.0)
    node._pose_registry['b1z1'] = (1.0, 2.0, 0.56, (0.0, 0.0, 0.0, 1.0))
    node._joint_positions.update({
        'joint1': 0.0,
        'joint2': 1.8,
        'joint3': -0.95,
        'joint4': -0.85,
        'joint5': 0.0,
        'joint6': 1.57,
    })
    node._scene_manager = MagicMock()
    node._scene_manager.add_box.return_value = True
    node._scene_manager.attach_object.return_value = True
    node._scene_manager.detach_object.return_value = True
    node._scene_manager.remove_object.return_value = True
    node._gz_node = MagicMock()
    node._gz_node.request.return_value = (True, MagicMock(data=True))
    return node


def test_kinematic_attach_proximity_and_update():
    """Verify proximity gating, MoveIt scene attach, and Gazebo set_pose updates."""
    rclpy.init()
    node = _init_node_with_poses()
    try:
        bx, by, bz = fk_tcp_base_link(0.0, 1.8, -0.95, -0.85, 0.0, 1.57)
        wx, wy, wz = transform_base_to_world(
            bx, by, bz, (1.0, 2.0, 0.56), (0.0, 0.0, 0.0, 1.0)
        )

        assert math.isclose(node._gripper_attach, -1.10, abs_tol=1e-6)
        assert node._attach_tcp_offset == (0.015, 0.0, 0.0)

        # Out-of-range object must be rejected before attaching.
        node._pose_registry['far_box'] = (wx + 0.50, wy, wz, (0.0, 0.0, 0.0, 1.0))
        err = node._attach_kinematic_object('far_box')
        assert err is not None and 'too far' in err
        assert node._attached_object is None

        # Object within attach_max_distance attaches in MoveIt + Gazebo.
        node._pose_registry['small_crate'] = (wx + 0.05, wy, wz, (0.0, 0.0, 0.0, 1.0))
        err = node._attach_kinematic_object('small_crate')
        assert err is None
        assert node._attached_object == 'small_crate'
        node._scene_manager.add_box.assert_called_once()
        node._scene_manager.attach_object.assert_called_once_with(
            'small_crate', link_name='gripper_tcp'
        )
        assert node._gz_node.request.called
        service_name, req, _, _, _ = node._gz_node.request.call_args[0]
        assert service_name == '/world/default/set_pose'
        assert req.name == 'small_crate'
        expected_wx, expected_wy, expected_wz = transform_base_to_world(
            bx + node._attach_tcp_offset[0],
            by - 0.034 * math.sin(1.57) + node._attach_tcp_offset[1],
            bz + 0.034 * math.cos(1.57) + node._attach_tcp_offset[2],
            (1.0, 2.0, 0.56),
            (0.0, 0.0, 0.0, 1.0),
        )
        assert math.isclose(req.position.x, expected_wx, abs_tol=1e-5)
        assert math.isclose(req.position.y, expected_wy, abs_tol=1e-5)
        assert math.isclose(req.position.z, expected_wz, abs_tol=1e-5)

        # Unchanged pose skips redundant Gazebo set_pose calls.
        node._gz_node.request.reset_mock()
        node._update_attached_object_pose()
        assert not node._gz_node.request.called

        # Verify jaw-gap cradle offset shifts to +0.034 Z and rolls -pi/2 when j6 = 0.0.
        node._joint_positions['joint6'] = 0.0
        node._update_attached_object_pose()
        cbx, cby, cbz = fk_tcp_base_link(0.0, 1.8, -0.95, -0.85, 0.0, 0.0)
        cwx, cwy, cwz = transform_base_to_world(
            cbx + node._attach_tcp_offset[0],
            cby + node._attach_tcp_offset[1],
            cbz + 0.034 + node._attach_tcp_offset[2],
            (1.0, 2.0, 0.56),
            (0.0, 0.0, 0.0, 1.0),
        )
        _, creq, _, _, _ = node._gz_node.request.call_args[0]
        assert math.isclose(creq.position.x, cwx, abs_tol=1e-5)
        assert math.isclose(creq.position.y, cwy, abs_tol=1e-5)
        assert math.isclose(creq.position.z, cwz, abs_tol=1e-5)
        assert math.isclose(creq.orientation.x, -math.sin(math.pi / 4.0), abs_tol=1e-5)
        assert math.isclose(creq.orientation.w, math.cos(math.pi / 4.0), abs_tol=1e-5)

        # Detaching clears state and removes from MoveIt scene.
        node._clear_attachment('small_crate')
        assert node._attached_object is None
        node._scene_manager.detach_object.assert_called_once_with('small_crate')
        node._scene_manager.remove_object.assert_called_once_with('small_crate')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_navigate_standoff_distinguishes_fixtures_from_objects():
    """Verify large fixtures keep 0.90 m standoff while arbitrary objects use 0.73 m."""
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
    """Verify post-lift confirmation tolerates a briefly stale pose registry."""
    rclpy.init()
    node = _init_node_with_poses()
    try:
        node._use_kinematic_attach = False
        node._pose_registry['crate'] = (1.0, 2.0, 0.70, (0.0, 0.0, 0.0, 1.0))

        # Never reaches min_z: returns the stale value after the timeout.
        z_stale = node._confirm_object_z('crate', 0.90, timeout_sec=0.4)
        assert z_stale is not None and math.isclose(z_stale, 0.70, abs_tol=1e-9)

        # Registry catches up (as gz_pose_bridge would) while we are polling.
        node._pose_registry['crate'] = (1.0, 2.0, 0.95, (0.0, 0.0, 0.0, 1.0))
        z_ok = node._confirm_object_z('crate', 0.90, timeout_sec=1.0)
        assert z_ok is not None and z_ok >= 0.90
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_planning_scene_manager_attach_and_detach():
    """Verify PlanningSceneManager builds valid MoveIt add/attach/detach diffs."""
    rclpy.init()
    node = rclpy.create_node('test_psm_node')
    try:
        psm = PlanningSceneManager(node, spin_locally=False)
        sent_scenes = []

        def _fake_apply(scene):
            sent_scenes.append(scene)
            return True

        psm.apply_scene_diff = _fake_apply

        assert psm.add_box('cargo_1', (0.1, 0.08, 0.06), (0.7, 0.0, 0.1), frame_id='base_link')
        assert psm.attach_object('cargo_1', link_name='gripper_tcp')
        assert 'cargo_1' in psm._tracked_attached
        assert psm.detach_object('cargo_1')
        assert 'cargo_1' not in psm._tracked_attached
        assert psm.remove_object('cargo_1')
        assert 'cargo_1' not in psm._tracked_objects

        assert sent_scenes[0].world.collision_objects[0].header.frame_id == 'base_link'
        attached = sent_scenes[1].robot_state.attached_collision_objects[0]
        assert attached.link_name == 'gripper_tcp'
        assert attached.object.operation == CollisionObject.ADD
        assert list(attached.touch_links) == DEFAULT_GRIPPER_TOUCH_LINKS
        assert sent_scenes[2].robot_state.attached_collision_objects[0].object.operation == (
            CollisionObject.REMOVE
        )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
