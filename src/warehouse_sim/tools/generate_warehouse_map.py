#!/usr/bin/env python3
"""Generate a ROS 2 2D occupancy grid map from Gazebo SDF <collision> geometry."""

import argparse
import math
import os
from typing import Any, Dict, List, Optional, Set, Tuple
import xml.etree.ElementTree as ET

import numpy as np


DEFAULT_EXCLUDED_MODELS = {
    'small_box',
    'cordless_drill_default_inertia',
    'cordless_drill_auto_inertia',
    'graspable_box',
}


def parse_pose(text: Optional[str]) -> Tuple[float, float, float, float, float, float]:
    """Parse an SDF '<pose>x y z r p y</pose>' string into 6 floats."""
    if not text:
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    vals = [float(v) for v in text.strip().split()]
    while len(vals) < 6:
        vals.append(0.0)
    return (vals[0], vals[1], vals[2], vals[3], vals[4], vals[5])


def pose_to_matrix(pose: Tuple[float, float, float, float, float, float]) -> np.ndarray:
    """Convert (x, y, z, roll, pitch, yaw) into a 4x4 homogeneous transform."""
    x, y, z, roll, pitch, yaw = pose
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    rot_x = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=np.float64)
    rot_y = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=np.float64)
    rot_z = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=np.float64)

    mat = np.eye(4, dtype=np.float64)
    mat[:3, :3] = rot_z @ rot_y @ rot_x
    mat[:3, 3] = [x, y, z]
    return mat


def load_model_collisions(
    models_dir: str,
) -> Dict[str, List[Tuple[np.ndarray, str, Tuple[Any, ...]]]]:
    """Parse all <collision> primitives for each model under models_dir."""
    cache: Dict[str, List[Tuple[np.ndarray, str, Tuple[Any, ...]]]] = {}
    for model_name in sorted(os.listdir(models_dir)):
        sdf_path = os.path.join(models_dir, model_name, 'model.sdf')
        if not os.path.isfile(sdf_path):
            continue
        root = ET.parse(sdf_path).getroot()
        primitives: List[Tuple[np.ndarray, str, Tuple[Any, ...]]] = []
        for link in root.findall('.//link'):
            t_link = pose_to_matrix(parse_pose(link.findtext('pose')))
            for col in link.findall('collision'):
                t_col = pose_to_matrix(parse_pose(col.findtext('pose')))
                t_local = t_link @ t_col
                geom = col.find('geometry')
                if geom is None:
                    continue
                box = geom.find('box')
                if box is not None:
                    sz = tuple(float(v) for v in (box.findtext('size') or '').split()[:3])
                    if len(sz) == 3:
                        primitives.append((t_local, 'box', sz))
                    continue
                cyl = geom.find('cylinder')
                if cyl is not None:
                    radius = float(cyl.findtext('radius') or '0.0')
                    length = float(cyl.findtext('length') or '0.0')
                    primitives.append((t_local, 'cylinder', (radius, length)))
                    continue
                sphere = geom.find('sphere')
                if sphere is not None:
                    radius = float(sphere.findtext('radius') or '0.0')
                    primitives.append((t_local, 'sphere', (radius,)))
                    continue
                if geom.find('mesh') is not None:
                    primitives.append((t_local, 'mesh', (model_name,)))
        cache[model_name] = primitives
    return cache


def _rasterize_box(
    grid: np.ndarray,
    t_world: np.ndarray,
    sx: float,
    sy: float,
    sz: float,
    x_min: float,
    y_min: float,
    resolution: float,
    z_low: float,
    z_high: float,
    floor_cutoff: float,
) -> None:
    """Mark grid cells occupied by an oriented 3D box within the target Z slice."""
    hx, hy, hz = 0.5 * sx, 0.5 * sy, 0.5 * sz
    corners_local = np.array(
        [
            [dx, dy, dz, 1.0]
            for dx in (-hx, hx)
            for dy in (-hy, hy)
            for dz in (-hz, hz)
        ],
        dtype=np.float64,
    ).T
    corners_world = (t_world @ corners_local)[:3, :]
    z_min_w = float(corners_world[2, :].min())
    z_max_w = float(corners_world[2, :].max())

    if z_max_w <= floor_cutoff:
        return
    if z_max_w < z_low or z_min_w > z_high:
        return

    height, width = grid.shape
    wx_lo = float(corners_world[0, :].min())
    wx_hi = float(corners_world[0, :].max())
    wy_lo = float(corners_world[1, :].min())
    wy_hi = float(corners_world[1, :].max())

    c0 = max(0, int(math.floor((wx_lo - x_min) / resolution)))
    c1 = min(width - 1, int(math.ceil((wx_hi - x_min) / resolution)))
    r0 = max(0, int(math.floor((wy_lo - y_min) / resolution)))
    r1 = min(height - 1, int(math.ceil((wy_hi - y_min) / resolution)))
    if c1 < c0 or r1 < r0:
        return

    t_inv = np.linalg.inv(t_world)
    xs = x_min + (np.arange(c0, c1 + 1, dtype=np.float64) + 0.5) * resolution
    ys = y_min + (np.arange(r0, r1 + 1, dtype=np.float64) + 0.5) * resolution
    gx, gy = np.meshgrid(xs, ys)

    z_band_lo = max(z_low, z_min_w)
    z_band_hi = min(z_high, z_max_w)
    # Sample vertically only when box has non-trivial tilt relative to Z.
    tilted = abs(float(t_world[2, 2])) < 0.995
    z_samples = (
        np.linspace(z_band_lo, z_band_hi, 5)
        if tilted and (z_band_hi - z_band_lo) > 1e-3
        else np.array([0.5 * (z_band_lo + z_band_hi)])
    )

    hit = np.zeros(gx.shape, dtype=bool)
    rot_inv = t_inv[:3, :3]
    tr_inv = t_inv[:3, 3]
    for zs in z_samples:
        lx = rot_inv[0, 0] * gx + rot_inv[0, 1] * gy + rot_inv[0, 2] * zs + tr_inv[0]
        ly = rot_inv[1, 0] * gx + rot_inv[1, 1] * gy + rot_inv[1, 2] * zs + tr_inv[1]
        if tilted:
            lz = rot_inv[2, 0] * gx + rot_inv[2, 1] * gy + rot_inv[2, 2] * zs + tr_inv[2]
            in_z = np.abs(lz) <= (hz + 1e-4)
        else:
            in_z = True
        hit |= (np.abs(lx) <= (hx + 1e-4)) & (np.abs(ly) <= (hy + 1e-4)) & in_z

    grid[r0:r1 + 1, c0:c1 + 1][hit] = True


def _rasterize_cylinder(
    grid: np.ndarray,
    t_world: np.ndarray,
    radius: float,
    length: float,
    x_min: float,
    y_min: float,
    resolution: float,
    z_low: float,
    z_high: float,
    floor_cutoff: float,
) -> None:
    """Mark grid cells occupied by a cylinder collider within the target Z slice."""
    hl = 0.5 * length
    axis_z = abs(float(t_world[2, 2]))
    radial_z = radius * math.sqrt(max(0.0, 1.0 - axis_z * axis_z))
    cz = float(t_world[2, 3])
    z_min_w = cz - hl * axis_z - radial_z
    z_max_w = cz + hl * axis_z + radial_z

    if z_max_w <= floor_cutoff or z_max_w < z_low or z_min_w > z_high:
        return

    # Horizontal cylinders (axis orthogonal to Z) fall back to tight bounding box.
    if axis_z < 0.85:
        _rasterize_box(
            grid,
            t_world,
            2.0 * radius,
            2.0 * radius,
            length,
            x_min,
            y_min,
            resolution,
            z_low,
            z_high,
            floor_cutoff,
        )
        return

    height, width = grid.shape
    cx, cy = float(t_world[0, 3]), float(t_world[1, 3])
    c0 = max(0, int(math.floor((cx - radius - x_min) / resolution)))
    c1 = min(width - 1, int(math.ceil((cx + radius - x_min) / resolution)))
    r0 = max(0, int(math.floor((cy - radius - y_min) / resolution)))
    r1 = min(height - 1, int(math.ceil((cy + radius - y_min) / resolution)))
    if c1 < c0 or r1 < r0:
        return

    xs = x_min + (np.arange(c0, c1 + 1, dtype=np.float64) + 0.5) * resolution
    ys = y_min + (np.arange(r0, r1 + 1, dtype=np.float64) + 0.5) * resolution
    gx, gy = np.meshgrid(xs, ys)
    hit = ((gx - cx) ** 2 + (gy - cy) ** 2) <= (radius * radius)
    grid[r0:r1 + 1, c0:c1 + 1][hit] = True


def build_occupancy_grid(
    world_sdf: str,
    models_dir: str,
    x_min: float = -32.5,
    x_max: float = 32.5,
    y_min: float = -9.0,
    y_max: float = 44.0,
    resolution: float = 0.05,
    z_low: float = 0.10,
    z_high: float = 1.00,
    floor_cutoff: float = 0.095,
    excluded_models: Optional[Set[str]] = None,
) -> Tuple[np.ndarray, int]:
    """Rasterize world SDF <include> collisions into a boolean occupancy grid."""
    if excluded_models is None:
        excluded_models = set(DEFAULT_EXCLUDED_MODELS)

    model_collisions = load_model_collisions(models_dir)
    root = ET.parse(world_sdf).getroot()
    world = root.find('world')
    if world is None:
        raise RuntimeError(f'No <world> element found in {world_sdf}')

    width = int(round((x_max - x_min) / resolution))
    height = int(round((y_max - y_min) / resolution))
    grid = np.zeros((height, width), dtype=bool)

    processed_includes = 0
    for inc in world.findall('include'):
        uri = (inc.findtext('uri') or '').replace('model://', '').strip()
        if not uri or uri in excluded_models:
            continue
        primitives = model_collisions.get(uri)
        if not primitives:
            continue
        t_inc = pose_to_matrix(parse_pose(inc.findtext('pose')))
        processed_includes += 1
        for t_local, gtype, params in primitives:
            t_world = t_inc @ t_local
            if gtype == 'box':
                _rasterize_box(
                    grid, t_world, params[0], params[1], params[2],
                    x_min, y_min, resolution, z_low, z_high, floor_cutoff,
                )
            elif gtype == 'cylinder':
                _rasterize_cylinder(
                    grid, t_world, params[0], params[1],
                    x_min, y_min, resolution, z_low, z_high, floor_cutoff,
                )
            elif gtype == 'sphere':
                r = params[0]
                _rasterize_box(
                    grid, t_world, 2.0 * r, 2.0 * r, 2.0 * r,
                    x_min, y_min, resolution, z_low, z_high, floor_cutoff,
                )
    return grid, processed_includes


def write_ros_map(
    grid: np.ndarray,
    output_dir: str,
    map_name: str,
    x_min: float,
    y_min: float,
    resolution: float,
) -> Tuple[str, str]:
    """Write ROS 2 map_server PGM (P5 binary) + YAML metadata files."""
    os.makedirs(output_dir, exist_ok=True)
    pgm_name = f'{map_name}.pgm'
    pgm_path = os.path.join(output_dir, pgm_name)
    yaml_path = os.path.join(output_dir, f'{map_name}.yaml')

    height, width = grid.shape
    # ROS 2 map_server flips Y when loading images: row 0 in PGM corresponds to y_max.
    img = np.where(np.flipud(grid), 0, 254).astype(np.uint8)
    header = f'P5\n{width} {height}\n255\n'.encode('ascii')
    with open(pgm_path, 'wb') as f:
        f.write(header)
        f.write(img.tobytes())

    yaml_content = (
        f'image: {pgm_name}\n'
        'mode: trinary\n'
        f'resolution: {resolution}\n'
        f'origin: [{x_min}, {y_min}, 0.0]\n'
        'negate: 0\n'
        'occupied_thresh: 0.65\n'
        'free_thresh: 0.25\n'
    )
    with open(yaml_path, 'w', encoding='utf-8') as f:
        f.write(yaml_content)

    return pgm_path, yaml_path


def main() -> int:
    parser = argparse.ArgumentParser(
        description='Generate ROS 2 2D occupancy map from warehouse.sdf collisions.'
    )
    default_ws_src = os.path.abspath(
        os.path.join(os.path.dirname(__file__), '..', '..')
    )
    parser.add_argument(
        '--world-sdf',
        default=os.path.join(default_ws_src, 'warehouse_sim', 'worlds', 'warehouse.sdf'),
    )
    parser.add_argument(
        '--models-dir',
        default=os.path.join(default_ws_src, 'warehouse_sim', 'models'),
    )
    parser.add_argument(
        '--output-dir',
        default=os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'maps')),
    )
    parser.add_argument('--map-name', default='warehouse_map')
    parser.add_argument('--resolution', type=float, default=0.05)
    parser.add_argument('--x-min', type=float, default=-32.5)
    parser.add_argument('--x-max', type=float, default=32.5)
    parser.add_argument('--y-min', type=float, default=-9.0)
    parser.add_argument('--y-max', type=float, default=44.0)
    args = parser.parse_args()

    grid, count = build_occupancy_grid(
        world_sdf=args.world_sdf,
        models_dir=args.models_dir,
        x_min=args.x_min,
        x_max=args.x_max,
        y_min=args.y_min,
        y_max=args.y_max,
        resolution=args.resolution,
    )
    pgm_path, yaml_path = write_ros_map(
        grid=grid,
        output_dir=args.output_dir,
        map_name=args.map_name,
        x_min=args.x_min,
        y_min=args.y_min,
        resolution=args.resolution,
    )
    occ_pct = 100.0 * float(grid.sum()) / float(grid.size)
    print(
        f'Generated {pgm_path} & {yaml_path} ({grid.shape[1]}x{grid.shape[0]} px, '
        f'{count} static models, {occ_pct:.2f}% occupied)'
    )
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
