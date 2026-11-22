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
from typing import List, Optional, Sequence, Tuple

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

    def has_line_of_sight(
        self,
        x0: float,
        y0: float,
        x1: float,
        y1: float,
    ) -> bool:
        """
        Return True iff no separate obstacle or wall lies between (x0, y0) and target (x1, y1).

        Allows the ray to terminate inside the target fixture's own occupied footprint,
        while rejecting candidates that cross occupied cells and re-enter free space
        before (x1, y1).
        """
        dist = math.hypot(x1 - x0, y1 - y0)
        if dist < 1e-6:
            return True
        step = max(0.02, 0.5 * self.resolution)
        n_steps = max(1, int(math.ceil(dist / step)))
        saw_occupied = False
        for i in range(n_steps + 1):
            t = i / n_steps
            wx = x0 + t * (x1 - x0)
            wy = y0 + t * (y1 - y0)
            col = int(math.floor((wx - self.origin_x) / self.resolution))
            row = int(math.floor((wy - self.origin_y) / self.resolution))
            if col < 0 or row < 0 or col >= self.width or row >= self.height:
                return False
            occupied = bool(self._occupied[row, col])
            if occupied:
                saw_occupied = True
            elif saw_occupied:
                return False
        return True

    def find_standoff_pose(
        self,
        tx: float,
        ty: float,
        target_yaw: float,
        robot_x: Optional[float] = None,
        robot_y: Optional[float] = None,
        preferred_dist: float = 0.90,
        clearance_radius: float = 0.36,
        max_standoff_dist: Optional[float] = None,
        preferred_approach_angles: Optional[Sequence[float]] = None,
    ) -> Tuple[float, float, float]:
        """Select a collision-free standoff (x, y, yaw) facing (tx, ty) with line of sight."""
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

        candidate_angles: List[float] = []
        if preferred_approach_angles:
            raw_angles: List[float] = list(preferred_approach_angles)
            for pref in preferred_approach_angles:
                for fan in (math.pi / 12.0, -math.pi / 12.0, math.pi / 6.0, -math.pi / 6.0):
                    raw_angles.append(pref + fan)
            for dtheta in angle_offsets:
                raw_angles.append(approach_angle + dtheta)

            for ang in raw_angles:
                norm_ang = math.atan2(math.sin(ang), math.cos(ang))
                if not any(
                    abs(math.atan2(math.sin(norm_ang - seen), math.cos(norm_ang - seen))) < 0.05
                    for seen in candidate_angles
                ):
                    candidate_angles.append(norm_ang)
            fallback_angle = preferred_approach_angles[0]
        else:
            candidate_angles = [approach_angle + dtheta for dtheta in angle_offsets]
            fallback_angle = approach_angle

        radii = [
            preferred_dist,
            preferred_dist + 0.25,
            preferred_dist + 0.50,
            preferred_dist + 0.75,
            preferred_dist + 1.05,
            max(0.55, preferred_dist - 0.15),
        ]
        if max_standoff_dist is not None:
            radii = [r for r in radii if r <= max_standoff_dist + 1e-6]
            if not radii:
                radii = [min(preferred_dist, max_standoff_dist)]

        for radius in radii:
            for theta in candidate_angles:
                cand_x = tx + radius * math.cos(theta)
                cand_y = ty + radius * math.sin(theta)
                if self.is_world_free(
                    cand_x, cand_y, clearance_radius=clearance_radius
                ) and self.has_line_of_sight(cand_x, cand_y, tx, ty):
                    facing_yaw = math.atan2(ty - cand_y, tx - cand_x)
                    return cand_x, cand_y, facing_yaw

        # Fallback along primary approach direction if map bounds/radii fail.
        fallback_dist = (
            min(preferred_dist, max_standoff_dist)
            if max_standoff_dist is not None
            else preferred_dist
        )
        cand_x = tx + fallback_dist * math.cos(fallback_angle)
        cand_y = ty + fallback_dist * math.sin(fallback_angle)
        return cand_x, cand_y, math.atan2(ty - cand_y, tx - cand_x)
