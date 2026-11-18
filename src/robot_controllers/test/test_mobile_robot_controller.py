"""Tests for the TEMPO mobile robot controller."""

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
    _quaternion_to_yaw,
    _yaw_to_quaternion,
    MobileRobotController,
)

CONFIG = {
    'robot_instances': {'mobile_robot_1': '/test_husky'},
    'navigation': {'standoff_distance': 1.0, 'nav_timeout_sec': 2.0},
    'manipulation': {
        'move_group_name': 'arm_0',
        'gripper_open_position': 0.0,
        'gripper_closed_position': 0.8,
        'moveit_timeout_sec': 2.0,
        'end_effector_link': 'arm_0_tool0',
        'gripper_max_effort': 50.0,
        'gripper_timeout_sec': 2.0,
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


@pytest.mark.parametrize('yaw', [0.0, 0.7, -2.1, math.pi - 0.01])
def test_yaw_quaternion_round_trip(yaw):
    q = _yaw_to_quaternion(yaw)
    assert _quaternion_to_yaw(q.x, q.y, q.z, q.w) == pytest.approx(yaw)


# Pose registry, fed over gz-transport like the real /world/<world>/pose/info

@pytest.fixture
def controller():
    # Isolate gz-transport discovery from any running simulation
    os.environ['GZ_PARTITION'] = f'mobile_test_{uuid.uuid4().hex}'
    rclpy.init()
    node = MobileRobotController(config=CONFIG)
    yield node
    node.destroy_node()
    rclpy.shutdown()


def _add_pose(msg, name, x, y, z, yaw):
    pose = msg.pose.add()
    pose.name = name
    pose.position.x, pose.position.y, pose.position.z = x, y, z
    q = _yaw_to_quaternion(yaw)
    pose.orientation.z = q.z
    pose.orientation.w = q.w


def test_pose_registry_keeps_gazebo_entity_names(controller):
    msg = Pose_V()
    _add_pose(msg, 'Picking_Shelves.002', -7.2612, 2.9492, 0.0, math.pi / 2)
    _add_pose(msg, 'test_husky/robot', 0.0, -3.0, 0.13, 0.0)

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
    # The robot's own pose is found under its Gazebo model name
    assert controller._get_pose(controller._robot_model_name('mobile_robot_1')) is not None
