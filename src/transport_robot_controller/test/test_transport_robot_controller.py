"""Tests for the TEMPO transport robot controller, using a mock Nav2 server."""

import json
import math
import os
import threading
import time
import uuid

from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
from nav2_msgs.action import NavigateToPose
import pytest
import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from transport_robot_controller.transport_robot_controller import (
    approach_pose,
    quaternion_to_yaw,
    TransportRobotController,
    yaw_to_quaternion,
)

NAMESPACE = '/test_husky'

CONFIG = {
    'robot_instances': {'transport_robot_1': NAMESPACE},
    'navigation': {
        'frame_id': 'map',
        'standoff_distance': 1.0,
        'server_wait_sec': 2.0,
        'nav_timeout_sec': 2.0,
    },
    'named_locations': {'origin': {'x': 0.0, 'y': -5.0, 'yaw': 0.0}},
    'world': {'name': 'test_world', 'pose_update_period_sec': 0.05},
}


# Pure helpers

def test_approach_picking_shelf_from_aisle_side_nearest_robot():
    """Taha's case: Picking_Shelves.002 (yaw 90 deg), robot at spawn (0, -5)."""
    gx, gy, heading = approach_pose(
        -7.2612, 2.9492, math.pi / 2, 1.0, axis='y', robot_xy=(0.0, -5.0))
    # Local +/-Y of a 90 deg shelf is world -/+X: goal is in the aisle on the
    # +X side (nearer the robot), not at y=1.9492 inside Picking_Shelves.001.
    assert (gx, gy) == pytest.approx((-6.2612, 2.9492))
    assert heading == pytest.approx(math.pi)  # facing -X, at the shelf


def test_approach_picks_other_side_when_robot_is_there():
    gx, gy, heading = approach_pose(
        -7.2612, 2.9492, math.pi / 2, 1.0, axis='y', robot_xy=(-15.0, 3.0))
    assert (gx, gy) == pytest.approx((-8.2612, 2.9492))
    assert heading == pytest.approx(0.0)


def test_approach_without_robot_pose_uses_positive_side():
    gx, gy, heading = approach_pose(4.0, 3.0, 0.0, 1.0, axis='y')
    assert (gx, gy) == pytest.approx((4.0, 4.0))
    assert heading == pytest.approx(-math.pi / 2)


def test_approach_along_local_x():
    gx, gy, _ = approach_pose(4.0, 3.0, 0.0, 1.0, axis='x', robot_xy=(0.0, 3.0))
    assert (gx, gy) == pytest.approx((3.0, 3.0))


def test_approach_rejects_bad_axis():
    with pytest.raises(ValueError):
        approach_pose(0.0, 0.0, 0.0, 1.0, axis='z')


@pytest.mark.parametrize('yaw', [0.0, 0.7, -2.1, math.pi - 0.01])
def test_yaw_quaternion_round_trip(yaw):
    q = yaw_to_quaternion(yaw)
    assert quaternion_to_yaw(q.x, q.y, q.z, q.w) == pytest.approx(yaw)


def test_yaw_of_tilted_entity():
    """Roll and pitch together: the 2*atan2(qz, qw) shortcut is wrong here."""
    roll, pitch, yaw = 0.4, 0.3, 0.5
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    assert quaternion_to_yaw(qx, qy, qz, qw) == pytest.approx(yaw)
    assert 2.0 * math.atan2(qz, qw) != pytest.approx(yaw, abs=0.01)


# Node-level tests with a mock Nav2 server

class MockNav2(Node):
    """Minimal NavigateToPose server whose outcome is controlled by the test."""

    def __init__(self):
        super().__init__('mock_nav2')
        self.outcome = 'succeed'  # 'succeed' | 'abort' | 'hang'
        self.goals = []
        self.active = 0
        self.max_active = 0
        self._lock = threading.Lock()
        self._server = ActionServer(
            self, NavigateToPose, f'{NAMESPACE}/navigate_to_pose',
            self._execute, callback_group=ReentrantCallbackGroup(),
        )

    def _execute(self, goal_handle):
        with self._lock:
            self.goals.append(goal_handle.request.pose)
            self.active += 1
            self.max_active = max(self.max_active, self.active)
        try:
            if self.outcome == 'hang':
                deadline = time.time() + 5.0
                while not goal_handle.is_cancel_requested and time.time() < deadline:
                    time.sleep(0.05)
                goal_handle.canceled()
            else:
                time.sleep(0.2)
                if self.outcome == 'succeed':
                    goal_handle.succeed()
                else:
                    goal_handle.abort()
        finally:
            with self._lock:
                self.active -= 1
        return NavigateToPose.Result()


@pytest.fixture
def system():
    # Isolate gz-transport discovery from any running simulation
    os.environ['GZ_PARTITION'] = f'transport_test_{uuid.uuid4().hex}'
    rclpy.init()
    controller = TransportRobotController(config=CONFIG)
    nav2 = MockNav2()
    probe = Node('feedback_probe')
    feedback = []
    probe.create_subscription(
        String, '/mission/feedback', lambda m: feedback.append(json.loads(m.data)), 10,
    )
    gz_node = GzNode()
    pose_pub = gz_node.advertise('/world/test_world/pose/info', Pose_V)

    executor = MultiThreadedExecutor(num_threads=6)
    for n in (controller, nav2, probe):
        executor.add_node(n)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    yield controller, nav2, feedback, pose_pub

    controller.stop_workers()
    executor.shutdown()
    for n in (controller, nav2, probe):
        n.destroy_node()
    rclpy.shutdown()


def _subtask(task_id, location, robot='transport_robot_1', skill='navigate_to'):
    return {
        'plan_id': 'plan_test',
        'task_id': task_id,
        'robot': robot,
        'required_skills': [skill],
        'args': {'location': location},
    }


def _wait_terminal(feedback, task_ids, timeout=8.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        done = {
            f['task_id'] for f in feedback
            if f['stage'] == 'complete' or f['status'] == 'failed'
        }
        if set(task_ids) <= done:
            return
        time.sleep(0.05)
    raise AssertionError(f'No terminal feedback for {task_ids}; got {feedback}')


def _terminal(feedback, task_id):
    return [
        f for f in feedback
        if f['task_id'] == task_id and (f['stage'] == 'complete' or f['status'] == 'failed')
    ]


def test_named_location_navigation_succeeds(system):
    controller, nav2, feedback, _ = system
    controller.handle_subtask(_subtask('T1', 'origin'))
    _wait_terminal(feedback, ['T1'])

    (final,) = _terminal(feedback, 'T1')
    assert final['status'] == 'success'
    assert any(f['status'] == 'in_progress' for f in feedback if f['task_id'] == 'T1')
    goal = nav2.goals[0]
    assert goal.header.frame_id == 'map'
    assert (goal.pose.position.x, goal.pose.position.y) == pytest.approx((0.0, -5.0))


def _add_pose(msg, name, x, y, yaw):
    pose = msg.pose.add()
    pose.name = name
    pose.position.x = x
    pose.position.y = y
    q = yaw_to_quaternion(yaw)
    pose.orientation.z = q.z
    pose.orientation.w = q.w


def test_gazebo_entity_goal_uses_robot_side_and_faces_entity(system):
    controller, nav2, feedback, pose_pub = system
    # Shelf at (4, 3) with yaw 0; robot model 'test_husky' (its namespace) at (4, 0).
    msg = Pose_V()
    _add_pose(msg, 'Picking_Shelves.001', 4.0, 3.0, 0.0)
    _add_pose(msg, 'test_husky', 4.0, 0.0, 0.0)
    deadline = time.time() + 3.0
    while controller.resolve_goal('Picking_Shelves.001') is None:
        assert time.time() < deadline, 'pose never reached the registry'
        pose_pub.publish(msg)
        time.sleep(0.1)

    controller.handle_subtask(_subtask('T1', 'Picking_Shelves.001'))
    _wait_terminal(feedback, ['T1'])
    goal = nav2.goals[0]
    assert (goal.pose.position.x, goal.pose.position.y) == pytest.approx((4.0, 2.0))
    q = goal.pose.orientation
    assert quaternion_to_yaw(q.x, q.y, q.z, q.w) == pytest.approx(math.pi / 2)


def test_subtasks_for_same_robot_run_sequentially(system):
    controller, nav2, feedback, _ = system
    controller.handle_subtask(_subtask('T1', 'origin'))
    controller.handle_subtask(_subtask('T2', 'origin'))
    _wait_terminal(feedback, ['T1', 'T2'])
    assert len(nav2.goals) == 2
    assert nav2.max_active == 1


def test_nav2_abort_reports_failure(system):
    controller, nav2, feedback, _ = system
    nav2.outcome = 'abort'
    controller.handle_subtask(_subtask('T1', 'origin'))
    _wait_terminal(feedback, ['T1'])
    (final,) = _terminal(feedback, 'T1')
    assert final['status'] == 'failed'


def test_nav2_timeout_cancels_and_fails(system):
    controller, nav2, feedback, _ = system
    nav2.outcome = 'hang'
    controller.handle_subtask(_subtask('T1', 'origin'))
    _wait_terminal(feedback, ['T1'])
    (final,) = _terminal(feedback, 'T1')
    assert final['status'] == 'failed'
    assert 'timed out' in final['detail']


def test_unsupported_skill_fails_without_nav2(system):
    controller, nav2, feedback, _ = system
    controller.handle_subtask(_subtask('T1', 'origin', skill='pick'))
    _wait_terminal(feedback, ['T1'])
    (final,) = _terminal(feedback, 'T1')
    assert final['status'] == 'failed'
    assert nav2.goals == []


def test_unknown_location_fails(system):
    controller, nav2, feedback, _ = system
    controller.handle_subtask(_subtask('T1', 'nowhere'))
    _wait_terminal(feedback, ['T1'])
    (final,) = _terminal(feedback, 'T1')
    assert final['status'] == 'failed'
    assert nav2.goals == []


def test_other_robot_types_are_ignored(system):
    controller, nav2, feedback, _ = system
    controller.handle_subtask(_subtask('T1', 'origin', robot='mobile_robot_1'))
    time.sleep(0.5)
    assert feedback == []
    assert nav2.goals == []
