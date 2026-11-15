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

"""Analytical FK, damped-least-squares IK, and frame transforms for Unitree Z1."""

import math
from typing import Dict, List, Tuple


def _matmul4(a: List[List[float]], b: List[List[float]]) -> List[List[float]]:
    return [
        [sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)]
        for i in range(4)
    ]


def _rot_x(angle: float) -> List[List[float]]:
    c, s = math.cos(angle), math.sin(angle)
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, c, -s, 0.0],
        [0.0, s, c, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _rot_y(angle: float) -> List[List[float]]:
    c, s = math.cos(angle), math.sin(angle)
    return [
        [c, 0.0, s, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [-s, 0.0, c, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _rot_z(angle: float) -> List[List[float]]:
    c, s = math.cos(angle), math.sin(angle)
    return [
        [c, -s, 0.0, 0.0],
        [s, c, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _trans(x: float, y: float, z: float) -> List[List[float]]:
    return [
        [1.0, 0.0, 0.0, x],
        [0.0, 1.0, 0.0, y],
        [0.0, 0.0, 1.0, z],
        [0.0, 0.0, 0.0, 1.0],
    ]


def fk_tcp_base_link(
    j1: float,
    j2: float,
    j3: float,
    j4: float,
    j5: float = 0.0,
    j6: float = 1.57,
) -> Tuple[float, float, float]:
    """Compute forward kinematics from b1z1 base_link to gripper_tcp."""
    t = _trans(0.20, 0.0, 0.08)
    t = _matmul4(t, _matmul4(_trans(0.0, 0.0, 0.0585), _rot_z(j1)))
    t = _matmul4(t, _matmul4(_trans(0.0, 0.0, 0.045), _rot_y(j2)))
    t = _matmul4(t, _matmul4(_trans(-0.35, 0.0, 0.0), _rot_y(j3)))
    t = _matmul4(t, _matmul4(_trans(0.218, 0.0, 0.057), _rot_y(j4)))
    t = _matmul4(t, _matmul4(_trans(0.07, 0.0, 0.0), _rot_z(j5)))
    t = _matmul4(t, _matmul4(_trans(0.0492, 0.0, 0.0), _rot_x(j6)))
    t = _matmul4(t, _trans(0.151, 0.0, 0.0))
    return (t[0][3], t[1][3], t[2][3])


def _solve_4x4(a: List[List[float]], b: List[float]) -> List[float]:
    """Solve a 4x4 linear system via partial-pivot Gaussian elimination."""
    mat = [list(a[i]) + [b[i]] for i in range(4)]
    for col in range(4):
        piv = max(range(col, 4), key=lambda r: abs(mat[r][col]))
        mat[col], mat[piv] = mat[piv], mat[col]
        if abs(mat[col][col]) < 1e-12:
            return [0.0, 0.0, 0.0, 0.0]
        div = mat[col][col]
        for j in range(col, 5):
            mat[col][j] /= div
        for r in range(4):
            if r != col:
                fac = mat[r][col]
                for j in range(col, 5):
                    mat[r][j] -= fac * mat[col][j]
    return [mat[i][4] for i in range(4)]


def solve_z1_ik(tx: float, ty: float, tz: float, j6: float = 1.57) -> Dict[str, float]:
    """Solve damped least-squares side-pinch IK for gripper_tcp in base_link."""
    # Roll joint6=1.57 places the moving jaw off-center; +0.025 m centers the jaw gap.
    aim_y = ty + (0.025 if abs(j6) > 0.5 else 0.0)
    q = [math.atan2(aim_y, max(tx - 0.20, 0.05)), 2.1, -0.95, -0.95]
    lo = [-2.50, 0.10, -2.80, -1.40]
    hi = [2.50, 2.92, -0.10, 1.40]

    def residual(x: List[float]) -> List[float]:
        px, py, pz = fk_tcp_base_link(x[0], x[1], x[2], x[3], 0.0, j6)
        return [
            (px - tx) * 30.0,
            (py - aim_y) * 30.0,
            (pz - tz) * 30.0,
            (x[2] - (-0.95)) * 0.5,
            (x[3] - (-1.00)) * 0.5,
        ]

    for _ in range(100):
        r = residual(q)
        pos_err = math.sqrt(r[0] ** 2 + r[1] ** 2 + r[2] ** 2) / 30.0
        if pos_err < 1e-4:
            break
        eps = 1e-5
        cols = []
        for k in range(4):
            qk = list(q)
            qk[k] += eps
            rk = residual(qk)
            cols.append([(rk[i] - r[i]) / eps for i in range(5)])
        jtj = [
            [sum(cols[i][m] * cols[j][m] for m in range(5)) for j in range(4)]
            for i in range(4)
        ]
        for i in range(4):
            jtj[i][i] += 1e-2
        jtr = [-sum(cols[i][m] * r[m] for m in range(5)) for i in range(4)]
        dq = _solve_4x4(jtj, jtr)
        for i in range(4):
            q[i] = max(lo[i], min(hi[i], q[i] + 0.5 * dq[i]))

    return {
        'joint1': q[0],
        'joint2': q[1],
        'joint3': q[2],
        'joint4': q[3],
        'joint5': 0.0,
        'joint6': j6,
    }


def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Extract yaw angle (rad) from a quaternion (x, y, z, w)."""
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


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
    # Apply inverse rotation (conjugate unit quaternion: -qx, -qy, -qz, qw).
    iqx, iqy, iqz = -qx, -qy, -qz
    tx = 2.0 * (iqy * dz - iqz * dy)
    ty = 2.0 * (iqz * dx - iqx * dz)
    tz = 2.0 * (iqx * dy - iqy * dx)
    bx = dx + qw * tx + (iqy * tz - iqz * ty)
    by = dy + qw * ty + (iqz * tx - iqx * tz)
    bz = dz + qw * tz + (iqx * ty - iqy * tx)
    return bx, by, bz
