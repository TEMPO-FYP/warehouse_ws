"""Tests for the TEMPO mobile robot controller (no simulation needed)."""

import math
import os
import time
import uuid

from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
import pytest
import rclpy
from robot_controllers.mobile_robot_controller import (
    _apply_standoff,
    _clamp,
    _horizontal_approach,
    _lookup_by_prefix,
    _model_prefix,
    _quaternion_to_yaw,
    _tool_orientation,
    _world_to_robot,
    _wrap_angle,
    _yaw_to_quaternion,
    MobileRobotController,
)

CONFIG = {
    'robot_instances': {'mobile_robot_1': '/test_husky'},
    'navigation': {
        'standoff_distance': 1.0,
        'standoff_overrides': {'Picking_Shelves': 0.85},
        'approach_axes': {'Picking_Shelves': 'local_y'},
        'default_approach_axis': 'radial',
        'nav_timeout_sec': 2.0,
    },
    'manipulation': {
        'move_group_name': 'arm_0',
        'end_effector_link': 'arm_0_tool0',
        'tcp_offset': 0.13,
        'arm_base_xyz': [0.08, 0.0, 0.414],
        'max_reach': 0.85,
        'grasp_offsets': {'small_box': 0.12},
        'place_heights': {'Picking_Shelves': [0.78, 0.47], 'place_spot': [0.0]},
        'place_offsets': {'Packing_Station': 0.2},
        'object_sizes': {'small_box': [0.3, 0.2, 0.1875]},
        'fixture_sizes': {'Picking_Shelves': [0.9, 0.44, 1.9]},
        'fixture_radius': 2.0,
        'touch_links': ['arm_0_tool0'],
        'gripper_open_position': 0.0,
        'gripper_closed_position': 0.8,
        'moveit_timeout_sec': 2.0,
        'gripper_max_effort': 50.0,
        'gripper_timeout_sec': 2.0,
        'stow_on_startup': False,
        'attach': {'enabled': True},
    },
    'world': {'name': 'test_world', 'pose_update_period_sec': 0.05},
}


# Pure helpers

def test_standoff_picking_shelf_from_aisle_side_nearest_robot():
    """Picking_Shelves.002 (yaw 90 deg), robot to its +X side."""
    gx, gy, heading = _apply_standoff(-7.2612, 2.9492, math.pi / 2, 1.0, (0.0, -3.0))
    assert (gx, gy) == pytest.approx((-6.2612, 2.9492))
    assert heading == pytest.approx(math.pi)


def test_standoff_picks_other_side_when_robot_is_there():
    gx, gy, heading = _apply_standoff(-7.2612, 2.9492, math.pi / 2, 1.0, (-15.0, 3.0))
    assert (gx, gy) == pytest.approx((-8.2612, 2.9492))
    assert heading == pytest.approx(0.0)


def test_standoff_skips_side_blocked_by_back_to_back_twin():
    """Picking_Shelves.027 has its twin .020 0.5 m behind it (world -X)."""
    shelf, twin = (-0.26, -0.05), (-0.76, -0.05)
    # Robot on the blocked side: the nearer side is skipped anyway
    gx, gy, heading = _apply_standoff(
        *shelf, math.pi / 2, 0.85, (-3.0, 0.0), 'local_y',
        blockers=[twin], clearance=0.75 * 0.85)
    assert (gx, gy) == pytest.approx((0.59, -0.05))
    assert heading == pytest.approx(math.pi)


def test_standoff_radial_faces_target_from_robot_side():
    gx, gy, heading = _apply_standoff(3.0, 2.0, 0.0, 0.8, (0.0, 2.0), 'radial')
    assert (gx, gy) == pytest.approx((2.2, 2.0))
    assert heading == pytest.approx(0.0)


@pytest.mark.parametrize('yaw', [0.0, 0.7, -2.1, math.pi - 0.01])
def test_yaw_quaternion_round_trip(yaw):
    q = _yaw_to_quaternion(yaw)
    assert _quaternion_to_yaw(q.x, q.y, q.z, q.w) == pytest.approx(yaw)


@pytest.mark.parametrize('name, prefix', [
    ('Picking_Shelves.027', 'Picking_Shelves'),
    ('small_box.027_2', 'small_box'),
    ('small_box_000_1', 'small_box'),
    ('Packing_Station', 'Packing_Station'),
    ('test_box', 'test_box'),
])
def test_model_prefix(name, prefix):
    assert _model_prefix(name) == prefix


def test_lookup_by_prefix_prefers_longest_key():
    table = {'small': 1, 'small_box': 2}
    assert _lookup_by_prefix(table, 'small_box.001', 0) == 2
    assert _lookup_by_prefix(table, 'Picking_Shelves', 0) == 0


def test_world_to_robot():
    # Robot at (1, 2) facing +Y: a point 1 m further along +Y is 1 m ahead
    assert _world_to_robot((1.0, 2.0, 0.1, math.pi / 2), (1.0, 3.0, 0.6)) == \
        pytest.approx((1.0, 0.0, 0.5))


@pytest.mark.parametrize('angle', [0.0, 0.5, -1.2, math.pi])
def test_tool_orientation_points_along_approach_with_y_up(angle):
    ax, ay = math.cos(angle), math.sin(angle)
    q = _tool_orientation(ax, ay)
    x, y, z, w = q.x, q.y, q.z, q.w
    assert x * x + y * y + z * z + w * w == pytest.approx(1.0)
    # Third and second columns of the rotation matrix
    tool_z = (2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y))
    tool_y = (2 * (x * y - w * z), 1 - 2 * (x * x + z * z), 2 * (y * z + w * x))
    assert tool_z == pytest.approx((ax, ay, 0.0), abs=1e-9)
    assert tool_y == pytest.approx((0.0, 0.0, 1.0), abs=1e-9)


def test_wrap_and_clamp():
    assert _wrap_angle(3 * math.pi / 2) == pytest.approx(-math.pi / 2)
    assert _wrap_angle(-0.3) == pytest.approx(-0.3)
    assert _clamp(0.5, 0.1) == 0.1
    assert _clamp(-0.5, 0.1) == -0.1
    assert _clamp(0.05, 0.1) == 0.05


def test_horizontal_approach():
    assert _horizontal_approach((0.0, 0.0, 0.4), (2.0, 0.0, 0.0)) == pytest.approx((1.0, 0.0))
    assert _horizontal_approach((0.0, 0.0, 0.0), (0.0, 0.0, 1.0)) == (1.0, 0.0)


# Node-level tests (no Gazebo or ROS servers needed)

@pytest.fixture
def controller():
    # Isolate gz-transport discovery from any running simulation
    os.environ['GZ_PARTITION'] = f'mobile_test_{uuid.uuid4().hex}'
    rclpy.init()
    node = MobileRobotController(config=CONFIG)
    yield node
    node.destroy_node()
    rclpy.shutdown()


def _add_pose(msg, name, x, y, z, yaw, entity_id=0):
    pose = msg.pose.add()
    pose.name = name
    pose.id = entity_id
    pose.position.x, pose.position.y, pose.position.z = x, y, z
    q = _yaw_to_quaternion(yaw)
    pose.orientation.z = q.z
    pose.orientation.w = q.w


def test_pose_registry_keeps_gazebo_entity_names_and_ids(controller):
    msg = Pose_V()
    _add_pose(msg, 'Picking_Shelves.002', -7.2612, 2.9492, 0.0, math.pi / 2, 7)
    _add_pose(msg, 'test_husky/robot', 0.0, -3.0, 0.13, 0.0, 49)

    gz_node = GzNode()
    pub = gz_node.advertise('/world/test_world/pose/info', Pose_V)
    deadline = time.time() + 5.0
    while controller._get_pose('Picking_Shelves.002') is None:
        assert time.time() < deadline, 'pose never reached the registry'
        pub.publish(msg)
        time.sleep(0.1)

    x, y, z, yaw = controller._get_pose('Picking_Shelves.002')
    assert (x, y, z) == pytest.approx((-7.2612, 2.9492, 0.0))
    assert yaw == pytest.approx(math.pi / 2)
    # The robot's own pose is found under its Gazebo model name, with its id
    model = controller._robot_model_name('mobile_robot_1')
    assert model == 'test_husky/robot'
    assert controller._get_pose(model) is not None
    assert controller._entity_ids[model] == 49


def test_reach_check(controller):
    approach = (1.0, 0.0)
    assert controller._reach_error((0.9, 0.0, 0.45), approach) == ''
    assert 'navigate closer' in controller._reach_error((1.3, 0.0, 0.45), approach)


def test_release_point_skips_occupied_tier(controller):
    shelf = (0.0, 0.0, 0.0, math.pi / 2)
    robot = (0.85, 0.0, 0.13, math.pi)
    controller._pose_registry.update({
        'small_box.001_3': (0.0, 0.0, 0.78, 0.0),   # tier 3 is taken
        'small_box.027_2': (5.0, 5.0, 0.47, 0.0),   # held object, elsewhere
    })
    release = controller._choose_release_point(
        'Picking_Shelves.001', shelf, 'small_box.027_2', robot)
    assert release == pytest.approx((0.0, 0.0, 0.47 + controller._place_clearance))

    controller._pose_registry['small_box.001_2'] = (0.0, 0.0, 0.47, 0.0)
    assert controller._choose_release_point(
        'Picking_Shelves.001', shelf, 'small_box.027_2', robot) is None


def test_release_point_offset_towards_robot(controller):
    station = (0.0, 13.0, 0.0, 0.0)
    robot = (1.0, 13.0, 0.13, math.pi)
    controller._place_heights['Packing_Station'] = [1.01]
    release = controller._choose_release_point(
        'Packing_Station.002', station, 'small_box.027_2', robot)
    assert release[:2] == pytest.approx((0.2, 13.0))


class _GoalCapture:
    """Stands in for an ActionClient / service client and records the request."""

    def __init__(self):
        self.request = None

    def wait_for_server(self, timeout_sec=None):
        return True

    def wait_for_service(self, timeout_sec=None):
        return True

    def send_goal_async(self, goal):
        self.request = goal
        raise RuntimeError('captured')

    def call_async(self, request):
        self.request = request
        raise RuntimeError('captured')


def test_moveit_goals_are_well_formed(controller):
    """Build real MoveGroup / Cartesian messages (catches field type errors)."""
    controller._sync_scene = lambda robot, **kwargs: (True, '')
    capture = _GoalCapture()
    controller._moveit_clients['mobile_robot_1'] = capture
    with pytest.raises(RuntimeError, match='captured'):
        controller._move_tcp('mobile_robot_1', (0.9, 0.0, 0.45), (1.0, 0.0))
    request = capture.request.request
    position = request.goal_constraints[0].position_constraints[0]
    tool = position.constraint_region.primitive_poses[0].position
    assert (tool.x, tool.y, tool.z) == pytest.approx((0.77, 0.0, 0.45))
    assert request.goal_constraints[0].orientation_constraints[0].link_name == 'arm_0_tool0'

    capture = _GoalCapture()
    controller._cartesian_clients['mobile_robot_1'] = capture
    with pytest.raises(RuntimeError, match='captured'):
        controller._move_cartesian('mobile_robot_1', [(0.9, 0.0, 0.45)], (1.0, 0.0))
    assert capture.request.waypoints[0].position.x == pytest.approx(0.77)

    controller._arm_joints = ['j1', 'j2']
    controller._stow_positions = [0.0, 1.0]
    capture = _GoalCapture()
    controller._moveit_clients['mobile_robot_1'] = capture
    with pytest.raises(RuntimeError, match='captured'):
        controller._stow('mobile_robot_1')
    joints = capture.request.request.goal_constraints[0].joint_constraints
    assert [j.joint_name for j in joints] == ['j1', 'j2']


def test_planning_scene_has_floor_fixtures_and_held_object(controller):
    controller._pose_registry.update({
        'test_husky/robot': (0.0, 0.0, 0.13, 0.0),
        'Picking_Shelves.027': (1.0, 0.0, 0.0, math.pi / 2),
        'Picking_Shelves.099': (30.0, 0.0, 0.0, 0.0),   # too far
        'link': (1.0, 0.0, 0.0, 0.0),                    # a nested link name
    })
    controller._held['mobile_robot_1'] = {
        'name': 'small_box.027_2', 'size': [0.3, 0.2, 0.1875], 'grasp_offset': 0.12}
    capture = _GoalCapture()
    controller._scene_clients['mobile_robot_1'] = capture
    with pytest.raises(RuntimeError, match='captured'):
        controller._sync_scene('mobile_robot_1')
    scene = capture.request.scene
    ids = [o.id for o in scene.world.collision_objects]
    assert ids == ['ground', 'fixture/Picking_Shelves.027']
    shelf = scene.world.collision_objects[1].primitive_poses[0].position
    assert (shelf.x, shelf.y, shelf.z) == pytest.approx((1.0, 0.0, 0.95 - 0.13))
    held = scene.robot_state.attached_collision_objects[0]
    assert held.link_name == 'arm_0_tool0'
    assert list(held.object.primitives[0].dimensions) == pytest.approx([0.3, 0.1875, 0.2])
    assert held.object.primitive_poses[0].position.z == pytest.approx(0.13 + 0.12)


def test_allowed_collision_matrix_adds_pairs(controller):
    from moveit_msgs.msg import AllowedCollisionEntry
    from moveit_msgs.srv import GetPlanningScene
    from rclpy.task import Future

    response = GetPlanningScene.Response()
    acm = response.scene.allowed_collision_matrix
    acm.entry_names = ['base_link', 'top_plate_link']
    acm.entry_values = [AllowedCollisionEntry(enabled=[False, True]),
                        AllowedCollisionEntry(enabled=[True, False])]

    class _Client:
        def wait_for_service(self, timeout_sec=None):
            return True

        def call_async(self, request):
            future = Future()
            future.set_result(response)
            return future

    controller._get_scene_clients['mobile_robot_1'] = _Client()
    controller._allowed_pairs = [['lidar2d_0_link', 'top_plate_link']]
    merged = controller._allowed_collision_matrix('mobile_robot_1')
    names = list(merged.entry_names)
    assert names == ['base_link', 'top_plate_link', 'lidar2d_0_link']
    rows = [list(e.enabled) for e in merged.entry_values]
    assert rows[0] == [False, True, False]          # existing entries kept
    assert rows[1][2] and rows[2][1]                # new pair allowed both ways
    assert all(len(r) == 3 for r in rows)


def test_gripper_closes_to_object_width(controller):
    # 60 mm cube: fingers stop just around it
    assert controller._gripper_position_for(0.06) == pytest.approx(0.8 * (1 - 0.064 / 0.085))
    # Wider than the 85 mm opening: close fully (held by the attachment)
    assert controller._gripper_position_for(0.3) == pytest.approx(0.8)


def test_place_requires_holding_the_object(controller):
    feedback = []
    controller._publish_feedback = lambda *a, **kw: feedback.append(kw)
    controller._execute_place('p', 'T1', 'mobile_robot_1',
                              {'object': 'test_box', 'location': 'place_spot'})
    assert feedback[-1]['status'] == 'failed'
    assert 'Not holding' in feedback[-1]['detail']


def test_robot_to_world_inverts_world_to_robot():
    from robot_controllers.mobile_robot_controller import _robot_to_world
    robot = (1.0, 2.0, 0.13, 0.9)
    point = (0.4, -0.2, 0.6)
    assert _world_to_robot(robot, _robot_to_world(robot, point)) == pytest.approx(point)


def _fake_arm(controller, tool_position, calls):
    """Robot at the origin facing +x, tool0 pointing along +x at tool_position."""
    controller._pose_registry['test_husky/robot'] = (0.0, 0.0, 0.0, 0.0)
    controller._tool_in_planning_frame = lambda robot: (tool_position, (1.0, 0.0, 0.0))
    controller._set_object_pose = lambda name, xyz, yaw: calls.append(('snap', xyz)) or True
    controller._attach = lambda robot, name: calls.append(('attach', name)) or (True, '')
    controller._detach = lambda robot, name: calls.append(('detach', name)) or (True, '')
    controller._settle_time = 0.0


def test_expected_held_position(controller):
    _fake_arm(controller, (0.6, 0.1, 0.5), [])
    # Object centre: TCP (0.13 past tool0) plus the grasp offset
    assert controller._expected_held_position('mobile_robot_1', 0.12) == \
        pytest.approx((0.6 + 0.13 + 0.12, 0.1, 0.5))


def test_secure_object_attaches_in_place_when_close(controller):
    calls = []
    _fake_arm(controller, (0.6, 0.0, 0.5), calls)
    controller._pose_registry['box'] = (0.74, 0.0, 0.5, 0.3)   # 1 cm off
    assert controller._secure_object('mobile_robot_1', 'box', 0.0, resting=True)[0]
    assert calls == [('attach', 'box')]


def test_secure_object_snaps_into_gripper_never_down_into_support(controller):
    calls = []
    _fake_arm(controller, (0.6, 0.0, 0.47), calls)   # arm stopped short and low
    controller._pose_registry['box'] = (0.85, 0.05, 0.5, 0.3)
    assert controller._secure_object('mobile_robot_1', 'box', 0.0, resting=True)[0]
    assert calls[0][0] == 'snap'
    assert calls[0][1] == pytest.approx((0.73, 0.0, 0.5))   # kept at its own height
    assert calls[1] == ('attach', 'box')


def test_secure_object_refuses_when_arm_is_far(controller):
    calls = []
    _fake_arm(controller, (0.2, 0.0, 0.9), calls)
    controller._pose_registry['box'] = (1.0, 0.0, 0.5, 0.0)
    ok, err = controller._secure_object('mobile_robot_1', 'box', 0.0, resting=True)
    assert not ok and 'did not get there' in err
    assert calls == []


def test_lost_hold_is_resnapped_and_reattached(controller):
    calls = []
    _fake_arm(controller, (0.6, 0.0, 0.8), calls)
    controller._wait_pose_updates = lambda count, timeout_sec=5.0: None
    controller._pose_registry['box'] = (0.73, 0.0, 0.2, 0.0)   # fell out
    original_attach = controller._attach

    def attach_and_follow(robot, name):
        controller._pose_registry['box'] = (0.73, 0.0, 0.8, 0.0)
        return original_attach(robot, name)

    controller._attach = attach_and_follow
    assert controller._ensure_held('mobile_robot_1', 'box', 0.0) == (True, '')
    assert [c[0] for c in calls] == ['detach', 'snap', 'attach']
    assert calls[1][1] == pytest.approx((0.73, 0.0, 0.8))


def test_put_down_detaches_then_sets_object_on_release_point(controller):
    calls = []
    _fake_arm(controller, (0.6, 0.0, 0.8), calls)
    controller._held['mobile_robot_1'] = {'name': 'box', 'size': [0.1] * 3,
                                          'grasp_offset': 0.0}
    assert controller._put_down('mobile_robot_1', 'box', (1.0, 2.0, 0.6), 1.57)[0]
    assert calls == [('detach', 'box'), ('snap', (1.0, 2.0, 0.6))]
    assert controller._held['mobile_robot_1'] is None


def test_misplaced_object_is_put_back(controller):
    calls = []
    _fake_arm(controller, (0.6, 0.0, 0.8), calls)
    controller._wait_pose_updates = lambda count, timeout_sec=5.0: None
    controller._pose_registry['box'] = (0.9, 0.3, 0.8, 0.0)    # followed the arm

    def set_pose(name, xyz, yaw):
        calls.append(('snap', xyz))
        controller._pose_registry['box'] = (*xyz, yaw)
        return True

    controller._set_object_pose = set_pose
    assert controller._ensure_placed('mobile_robot_1', 'box', (1.0, 2.0, 0.6), 0.0)[0]
    assert ('snap', (1.0, 2.0, 0.6)) in calls
