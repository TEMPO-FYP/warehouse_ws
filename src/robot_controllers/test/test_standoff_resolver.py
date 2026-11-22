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

"""Unit tests for SDF occupancy standoff resolution, line-of-sight, and geometry utilities."""

import math
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import numpy as np
from robot_controllers.geometry_utils import (
    transform_base_to_world,
    transform_world_to_base,
)
from robot_controllers.map_standoff import OccupancyMap


def _get_warehouse_map() -> OccupancyMap:
    pkg_dir = get_package_share_directory('warehouse_sim')
    return OccupancyMap(os.path.join(pkg_dir, 'maps', 'warehouse_map.yaml'))


def _make_synthetic_map(tmp_path: Path, occ_mask: np.ndarray, res: float = 0.1) -> OccupancyMap:
    h, w = occ_mask.shape
    pgm_path = tmp_path / 'synthetic.pgm'
    yaml_path = tmp_path / 'synthetic.yaml'
    img = np.where(np.flipud(occ_mask), 0, 254).astype(np.uint8)
    with open(pgm_path, 'wb') as f:
        f.write(f'P5\n{w} {h}\n255\n'.encode('ascii'))
        f.write(img.tobytes())
    yaml_path.write_text(
        f'image: {pgm_path.name}\nmode: trinary\nresolution: {res}\norigin: [0.0, 0.0, 0.0]\n'
        'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n',
        encoding='utf-8',
    )
    return OccupancyMap(str(yaml_path))


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


def test_line_of_sight_rejects_wall_between_standoff_and_target(tmp_path: Path):
    """Standoffs must not be chosen on the opposite side of an intervening wall."""
    occ_mask = np.zeros((60, 60), dtype=bool)
    # Place a thin horizontal wall at y in [2.6, 2.8] (rows 26..27) blocking approach from y < 2.6.
    occ_mask[26:28, 10:50] = True
    occ = _make_synthetic_map(tmp_path, occ_mask, res=0.1)

    assert not occ.has_line_of_sight(3.0, 2.0, 3.0, 3.0)
    assert occ.has_line_of_sight(3.0, 3.9, 3.0, 3.0)

    gx, gy, _ = occ.find_standoff_pose(
        tx=3.0,
        ty=3.0,
        target_yaw=0.0,
        robot_x=3.0,
        robot_y=1.0,
        preferred_dist=0.90,
        clearance_radius=0.30,
        max_standoff_dist=0.95,
    )
    assert gy > 2.8
    assert math.hypot(gx - 3.0, gy - 3.0) <= 0.95 + 1e-6


def test_world_to_base_and_round_trip():
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

    base_pos = (1.25, -3.40, 0.56)
    base_quat = (0.05, -0.03, math.sin(0.65), math.cos(0.65))
    norm = math.sqrt(sum(c * c for c in base_quat))
    base_quat = tuple(c / norm for c in base_quat)

    world_pt = (2.10, -2.75, 0.82)
    bx2, by2, bz2 = transform_world_to_base(*world_pt, base_pos=base_pos, base_quat=base_quat)
    rx, ry, rz = transform_base_to_world(bx2, by2, bz2, base_pos=base_pos, base_quat=base_quat)

    assert math.isclose(rx, world_pt[0], abs_tol=1e-6)
    assert math.isclose(ry, world_pt[1], abs_tol=1e-6)
    assert math.isclose(rz, world_pt[2], abs_tol=1e-6)


def test_preferred_approach_angles_selects_broadside_over_bumper(tmp_path: Path):
    """Verify preferred broadside angles override straight-line bumper approach."""
    occ_mask = np.zeros((80, 80), dtype=bool)
    occ = _make_synthetic_map(tmp_path, occ_mask, res=0.1)

    # Robot starts straight behind target (angle pi); prefer broadsides ±pi/2.
    gx, gy, yaw = occ.find_standoff_pose(
        tx=4.0,
        ty=4.0,
        target_yaw=0.0,
        robot_x=2.0,
        robot_y=4.0,
        preferred_dist=0.80,
        clearance_radius=0.35,
        preferred_approach_angles=[math.pi / 2.0, -math.pi / 2.0],
    )
    assert math.isclose(gx, 4.0, abs_tol=1e-6)
    assert math.isclose(gy, 4.80, abs_tol=1e-6)
    assert math.isclose(yaw, -math.pi / 2.0, abs_tol=1e-6)

    # Block the +pi/2 broadside with a wall at y in [4.5, 5.5] -> falls back to -pi/2.
    occ_mask[45:56, 30:50] = True
    occ_blocked = _make_synthetic_map(tmp_path, occ_mask, res=0.1)
    gx2, gy2, yaw2 = occ_blocked.find_standoff_pose(
        tx=4.0,
        ty=4.0,
        target_yaw=0.0,
        robot_x=2.0,
        robot_y=4.0,
        preferred_dist=0.80,
        clearance_radius=0.35,
        preferred_approach_angles=[math.pi / 2.0, -math.pi / 2.0],
    )
    assert math.isclose(gx2, 4.0, abs_tol=1e-6)
    assert math.isclose(gy2, 3.20, abs_tol=1e-6)
    assert math.isclose(yaw2, math.pi / 2.0, abs_tol=1e-6)
