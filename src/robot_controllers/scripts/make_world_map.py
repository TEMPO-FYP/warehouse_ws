#!/usr/bin/env python3
"""
Rasterise the static box models of a simple SDF world into a Nav2 map.

  python3 make_world_map.py ../../warehouse_sim/worlds/empty.sdf ../maps/empty_map

Writes <out>.pgm and <out>.yaml in Gazebo world coordinates (map frame =
world frame). Every <static> model with a <box> collision whose height range
overlaps [min_z, max_z] (the band the 2D lidar and the costmaps see) is drawn
as occupied; cells outside the bounding box of the walls are unknown. Only
box collisions and yaw rotations are handled, which covers empty.sdf.
"""

import argparse
import math
import os
import xml.etree.ElementTree as ET


def static_boxes(sdf_path, min_z, max_z):
    """Yield (cx, cy, sx, sy, yaw) of static box collisions in the height band."""
    world = ET.parse(sdf_path).getroot().find('world')
    for model in world.findall('model'):
        if (model.findtext('static') or '').strip() != 'true':
            continue
        mx, my, mz, _, _, myaw = [float(v) for v in
                                  (model.findtext('pose') or '0 0 0 0 0 0').split()]
        for coll in model.iter('collision'):
            box = coll.find('geometry/box/size')
            if box is None:
                continue
            sx, sy, sz = [float(v) for v in box.text.split()]
            cx, cy, cz, _, _, cyaw = [float(v) for v in
                                      (coll.findtext('pose') or '0 0 0 0 0 0').split()]
            z0, z1 = mz + cz - sz / 2, mz + cz + sz / 2
            if z1 < min_z or z0 > max_z:
                continue
            wx = mx + cx * math.cos(myaw) - cy * math.sin(myaw)
            wy = my + cx * math.sin(myaw) + cy * math.cos(myaw)
            yield wx, wy, sx, sy, myaw + cyaw


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('sdf')
    parser.add_argument('out', help='output path without extension')
    parser.add_argument('--resolution', type=float, default=0.05)
    parser.add_argument('--margin', type=float, default=0.8,
                        help='unknown border around the boxes (m)')
    parser.add_argument('--min-z', type=float, default=0.05)
    parser.add_argument('--max-z', type=float, default=2.0)
    args = parser.parse_args()

    boxes = list(static_boxes(args.sdf, args.min_z, args.max_z))
    if not boxes:
        raise SystemExit('No static boxes found.')
    corners = []
    for cx, cy, sx, sy, yaw in boxes:
        for dx in (-sx / 2, sx / 2):
            for dy in (-sy / 2, sy / 2):
                corners.append((cx + dx * math.cos(yaw) - dy * math.sin(yaw),
                                cy + dx * math.sin(yaw) + dy * math.cos(yaw)))
    res = args.resolution
    xs, ys = [c[0] for c in corners], [c[1] for c in corners]
    inner = (min(xs), max(xs), min(ys), max(ys))
    x0 = math.floor((inner[0] - args.margin) / res) * res
    y0 = math.floor((inner[2] - args.margin) / res) * res
    width = int(math.ceil((inner[1] + args.margin - x0) / res))
    height = int(math.ceil((inner[3] + args.margin - y0) / res))

    pixels = bytearray()
    for row in range(height):
        y = y0 + (height - 1 - row + 0.5) * res  # row 0 is the top (max y)
        for col in range(width):
            x = x0 + (col + 0.5) * res
            value = 254  # free
            if not (inner[0] <= x <= inner[1] and inner[2] <= y <= inner[3]):
                value = 205  # unknown
            for cx, cy, sx, sy, yaw in boxes:
                lx = (x - cx) * math.cos(yaw) + (y - cy) * math.sin(yaw)
                ly = -(x - cx) * math.sin(yaw) + (y - cy) * math.cos(yaw)
                if abs(lx) <= sx / 2 + res / 2 and abs(ly) <= sy / 2 + res / 2:
                    value = 0  # occupied
                    break
            pixels.append(value)

    with open(args.out + '.pgm', 'wb') as f:
        f.write(f'P5\n{width} {height}\n255\n'.encode())
        f.write(bytes(pixels))
    with open(args.out + '.yaml', 'w', encoding='utf-8') as f:
        f.write(
            f'# Map of {os.path.basename(args.sdf)} in Gazebo world coordinates, made by\n'
            f'# robot_controllers/scripts/make_world_map.py; regenerate it if the\n'
            f'# static models change.\n'
            f'image: {os.path.basename(args.out)}.pgm\n'
            'mode: trinary\n'
            f'resolution: {res}\n'
            f'origin: [{x0:.3f}, {y0:.3f}, 0.0]\n'
            'negate: 0\n'
            'occupied_thresh: 0.65\n'
            'free_thresh: 0.25\n')
    print(f'{len(boxes)} boxes -> {width}x{height} map, origin ({x0:.2f}, {y0:.2f})')


if __name__ == '__main__':
    main()
