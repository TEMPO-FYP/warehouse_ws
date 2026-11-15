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

"""Unit tests for SDF occupancy standoff resolution and Z1 IK transforms."""

import math
import os

from ament_index_python.packages import get_package_share_directory
from robot_controllers.map_standoff import OccupancyMap
from robot_controllers.z1_ik import (
    fk_tcp_base_link,
    solve_z1_ik,
    transform_world_to_base,
)


def _get_warehouse_map() -> OccupancyMap:
    pkg_dir = get_package_share_directory('b1z1_control')
    return OccupancyMap(os.path.join(pkg_dir, 'maps', 'warehouse_map.yaml'))


def test_staging_and_aisle_free():
    occ = _get_warehouse_map()
    assert occ.is_world_free(0.0, 16.0, clearance_radius=0.38)
    assert occ.is_world_free(-5.76, 1.95, clearance_radius=0.38)
    assert not occ.is_world_free(-7.2612, 1.9492, clearance_radius=0.36)


def test_standoff_for_picking_shelf_enters_aisle():
    occ = _get_warehouse_map()
    gx, gy, yaw = occ.find_standoff_pose(
        tx=-7.2612,
        ty=1.9492,
        target_yaw=1.5708,
        robot_x=0.0,
        robot_y=16.0,
        preferred_dist=0.90,
        clearance_radius=0.36,
    )
    assert occ.is_world_free(gx, gy, clearance_radius=0.36)
    # Must approach from aisle_1 (x > -7.26) and face west toward the shelf.
    assert gx > -7.0
    expected_yaw = math.atan2(1.9492 - gy, -7.2612 - gx)
    assert abs(yaw - expected_yaw) < 1e-6


def test_standoff_for_packing_station():
    occ = _get_warehouse_map()
    gx, gy, yaw = occ.find_standoff_pose(
        tx=0.9342,
        ty=13.0841,
        target_yaw=0.0,
        robot_x=0.0,
        robot_y=16.0,
        preferred_dist=0.90,
        clearance_radius=0.36,
    )
    assert occ.is_world_free(gx, gy, clearance_radius=0.36)
    expected_yaw = math.atan2(13.0841 - gy, 0.9342 - gx)
    assert abs(yaw - expected_yaw) < 1e-6


def test_z1_ik_and_world_to_base():
    bx, by, bz = transform_world_to_base(
        wx=0.0,
        wy=16.73,
        wz=0.73,
        base_pos=(0.0, 16.0, 0.56),
        base_quat=(0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0)),
    )
    assert math.isclose(bx, 0.73, abs_tol=1e-4)
    assert math.isclose(by, 0.0, abs_tol=1e-4)
    assert math.isclose(bz, 0.17, abs_tol=1e-4)

    q = solve_z1_ik(bx, by, bz)
    px, py, pz = fk_tcp_base_link(
        q['joint1'], q['joint2'], q['joint3'], q['joint4'], q['joint5'], q['joint6']
    )
    err = math.dist((px, py, pz), (bx, by + 0.025, bz))
    assert err < 0.005
