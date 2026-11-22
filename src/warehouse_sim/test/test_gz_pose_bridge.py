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

"""Unit tests for gz_pose_bridge child-link filtering."""

from warehouse_sim.gz_pose_bridge import _is_model_pose_name


def test_is_model_pose_name_accepts_top_level_models() -> None:
    """Verify uniquely named top-level robot and fixture models pass filtering."""
    assert _is_model_pose_name('a200_0000', 1) is True
    assert _is_model_pose_name('a200_0001', 1) is True
    assert _is_model_pose_name('b1z1', 1) is True
    assert _is_model_pose_name('b2_cargo', 1) is True
    assert _is_model_pose_name('Picking_Shelves.001', 1) is True
    assert _is_model_pose_name('a200_0001', 2) is False
    assert _is_model_pose_name('a200_0001::base_link', 1) is False


def test_is_model_pose_name_rejects_husky_and_robot_child_links() -> None:
    """Verify Clearpath Husky and existing robot child links are filtered out."""
    child_names = [
        'chassis_link',
        'top_plate_link',
        'top_plate_front_mount',
        'front_left_wheel_link',
        'rear_right_wheel_link',
        'front_bumper_link',
        'rear_bumper_mount',
        'user_rail_link',
        'lidar2d_0_laser',
        'imu_0_link',
        'arm_0_base_link',
        'arm_0_base_link_inertia',
        'arm_0_shoulder_link',
        'arm_0_upper_arm_link',
        'arm_0_forearm_link',
        'arm_0_wrist_1_link',
        'arm_0_wrist_3_link',
        'arm_0_tool0',
        'arm_0_gripper_robotiq_85_base_link',
        'arm_0_gripper_grasp_tcp',
        'b1z1_base_link',
        'b1z1_FR_hip',
    ]
    for name in child_names:
        assert _is_model_pose_name(name, 1) is False, f'Expected {name} to be filtered'
