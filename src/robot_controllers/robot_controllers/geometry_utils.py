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

"""Robot-agnostic quaternion, orientation, and 3D rigid frame utilities."""

import math
from typing import Tuple

from geometry_msgs.msg import Quaternion


def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Extract yaw angle (rad) from a quaternion (x, y, z, w)."""
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Create a geometry_msgs/Quaternion for a pure Z rotation by yaw (rad)."""
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def yaw_quat_tuple(yaw: float) -> Tuple[float, float, float, float]:
    """Return a unit quaternion tuple (x, y, z, w) for pure yaw rotation."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def rpy_to_quat_tuple(
    roll: float,
    pitch: float,
    yaw: float,
) -> Tuple[float, float, float, float]:
    """Convert Euler (roll, pitch, yaw) in XYZ fixed order to (x, y, z, w)."""
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    return (qx, qy, qz, qw)


def multiply_quats(
    q1: Tuple[float, float, float, float],
    q2: Tuple[float, float, float, float],
) -> Tuple[float, float, float, float]:
    """Multiply two quaternions (x, y, z, w): returns q1 * q2."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def yaw_roll_quat_tuple(yaw: float, roll: float) -> Tuple[float, float, float, float]:
    """Return unit quaternion (x, y, z, w) for yaw around Z followed by roll around local X."""
    return rpy_to_quat_tuple(roll, 0.0, yaw)


def transform_world_to_base(
    wx: float,
    wy: float,
    wz: float,
    base_pos: Tuple[float, float, float],
    base_quat: Tuple[float, float, float, float],
) -> Tuple[float, float, float]:
    """Transform a world point (wx, wy, wz) into a base frame at (base_pos, base_quat)."""
    dx = wx - base_pos[0]
    dy = wy - base_pos[1]
    dz = wz - base_pos[2]
    qx, qy, qz, qw = base_quat
    iqx, iqy, iqz = -qx, -qy, -qz
    tx = 2.0 * (iqy * dz - iqz * dy)
    ty = 2.0 * (iqz * dx - iqx * dz)
    tz = 2.0 * (iqx * dy - iqy * dx)
    bx = dx + qw * tx + (iqy * tz - iqz * ty)
    by = dy + qw * ty + (iqz * tx - iqx * tz)
    bz = dz + qw * tz + (iqx * ty - iqy * tx)
    return bx, by, bz


def transform_base_to_world(
    bx: float,
    by: float,
    bz: float,
    base_pos: Tuple[float, float, float],
    base_quat: Tuple[float, float, float, float],
) -> Tuple[float, float, float]:
    """Transform a base_link point (bx, by, bz) into world coordinates."""
    qx, qy, qz, qw = base_quat
    tx = 2.0 * (qy * bz - qz * by)
    ty = 2.0 * (qz * bx - qx * bz)
    tz = 2.0 * (qx * by - qy * bx)
    wx = base_pos[0] + bx + qw * tx + (qy * tz - qz * ty)
    wy = base_pos[1] + by + qw * ty + (qz * tx - qx * tz)
    wz = base_pos[2] + bz + qw * tz + (qx * ty - qy * tx)
    return wx, wy, wz
