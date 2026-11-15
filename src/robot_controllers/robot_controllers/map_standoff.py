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

"""2D occupancy map loader and collision-free standoff pose resolver."""

import math
import os
from typing import List, Optional, Tuple

import numpy as np
import yaml


def _read_pgm(pgm_path: str) -> np.ndarray:
    """Read a binary P5 or ASCII P2 PGM image into a uint8 (H, W) array."""
    with open(pgm_path, 'rb') as f:
        tokens: List[bytes] = []
        while len(tokens) < 4:
            line = f.readline()
            if not line:
                raise ValueError(f'Unexpected EOF reading PGM header: {pgm_path}')
            stripped = line.split(b'#', 1)[0].strip()
            if stripped:
                tokens.extend(stripped.split())
        magic = tokens[0]
        width = int(tokens[1])
        height = int(tokens[2])
        if magic == b'P5':
            raw = f.read(width * height)
            return np.frombuffer(raw, dtype=np.uint8).reshape((height, width))
        if magic == b'P2':
            vals = [int(v) for v in f.read().split()]
            return np.array(vals[: width * height], dtype=np.uint8).reshape((height, width))
        raise ValueError(f'Unsupported PGM format {magic!r} in {pgm_path}')


class OccupancyMap:
    """Queries ROS 2 occupancy grid free space and standoff approaches."""

    def __init__(self, yaml_path: str) -> None:
        """Load map metadata YAML and corresponding PGM raster image."""
        with open(yaml_path, 'r', encoding='utf-8') as f:
            meta = yaml.safe_load(f)

        img_rel = meta['image']
        img_path = (
            img_rel
            if os.path.isabs(img_rel)
            else os.path.join(os.path.dirname(os.path.abspath(yaml_path)), img_rel)
        )
        raster = _read_pgm(img_path)
        self.height, self.width = raster.shape
        self.resolution = float(meta['resolution'])
        origin = meta['origin']
        self.origin_x = float(origin[0])
        self.origin_y = float(origin[1])

        negate = bool(meta.get('negate', 0))
        occ_thresh = float(meta.get('occupied_thresh', 0.65))
        # Flip vertically so row 0 corresponds to origin_y (bottom of map).
        img_bottom_up = np.flipud(raster).astype(np.float32) / 255.0
        occ_prob = img_bottom_up if negate else (1.0 - img_bottom_up)
        self._occupied = occ_prob >= occ_thresh

    def is_world_free(
        self,
        wx: float,
        wy: float,
        clearance_radius: float = 0.35,
    ) -> bool:
        """Return True iff every cell within clearance_radius of (wx, wy) is free."""
        c0 = int(math.floor((wx - clearance_radius - self.origin_x) / self.resolution))
        c1 = int(math.ceil((wx + clearance_radius - self.origin_x) / self.resolution))
        r0 = int(math.floor((wy - clearance_radius - self.origin_y) / self.resolution))
        r1 = int(math.ceil((wy + clearance_radius - self.origin_y) / self.resolution))

        if c0 < 0 or r0 < 0 or c1 >= self.width or r1 >= self.height:
            return False

        cols = np.arange(c0, c1 + 1, dtype=np.float64)
        rows = np.arange(r0, r1 + 1, dtype=np.float64)
        xs = self.origin_x + (cols + 0.5) * self.resolution
        ys = self.origin_y + (rows + 0.5) * self.resolution
        gx, gy = np.meshgrid(xs, ys)
        mask = ((gx - wx) ** 2 + (gy - wy) ** 2) <= (clearance_radius * clearance_radius)
        return not bool(np.any(self._occupied[r0:r1 + 1, c0:c1 + 1][mask]))

    def find_standoff_pose(
        self,
        tx: float,
        ty: float,
        target_yaw: float,
        robot_x: Optional[float] = None,
        robot_y: Optional[float] = None,
        preferred_dist: float = 0.90,
        clearance_radius: float = 0.36,
    ) -> Tuple[float, float, float]:
        """Select a collision-free standoff (x, y, yaw) facing (tx, ty)."""
        approach_angle = (
            math.atan2(robot_y - ty, robot_x - tx)
            if robot_x is not None and robot_y is not None
            else (target_yaw + math.pi)
        )

        # Sample angles biased toward the robot's current approach direction.
        angle_offsets = [0.0]
        for step in range(1, 9):
            delta = step * (math.pi / 8.0)
            angle_offsets.append(delta)
            if step < 8:
                angle_offsets.append(-delta)

        radii = [
            preferred_dist,
            preferred_dist + 0.25,
            preferred_dist + 0.50,
            preferred_dist + 0.75,
            preferred_dist + 1.05,
            max(0.55, preferred_dist - 0.15),
        ]

        for radius in radii:
            for dtheta in angle_offsets:
                theta = approach_angle + dtheta
                cand_x = tx + radius * math.cos(theta)
                cand_y = ty + radius * math.sin(theta)
                if self.is_world_free(cand_x, cand_y, clearance_radius=clearance_radius):
                    facing_yaw = math.atan2(ty - cand_y, tx - cand_x)
                    return cand_x, cand_y, facing_yaw

        # Fallback along approach direction if map bounds/radii fail.
        cand_x = tx + preferred_dist * math.cos(approach_angle)
        cand_y = ty + preferred_dist * math.sin(approach_angle)
        return cand_x, cand_y, math.atan2(ty - cand_y, tx - cand_x)
