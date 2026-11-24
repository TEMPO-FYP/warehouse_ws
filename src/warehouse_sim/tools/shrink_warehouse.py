#!/usr/bin/env python3
"""Shrink the Distribution_Warehouse model along +X by cutting out vertical slabs.

For every slab [a, b] (world X, metres):
  * mesh triangles / collision boxes lying entirely inside the slab are removed,
  * every vertex with x > a is shifted left by the part of the slab it lies past,
    so geometry spanning the slab (floor, ceiling, long walls) is shortened and
    everything to the right of the slab (right wall, door bays, ...) slides left.

Slabs must be placed in empty gaps between features (e.g. between door bays)
so nothing visible gets squashed.

Usage:
  python3 tools/shrink_warehouse.py SRC_MODEL_DIR DST_MODEL_DIR [--slab A B ...]
SRC and DST may be the same directory (files are overwritten in place).
"""

import argparse
import json
import os
import re
import struct
import xml.etree.ElementTree as ET

import numpy as np

DEFAULT_SLABS = [(-1.9, 2.4), (7.5, 26.5)]  # keep only the container bay at x~4.9

COMP_DTYPE = {5121: np.uint8, 5123: np.uint16, 5125: np.uint32, 5126: np.float32}


def shift_of(x, slabs):
    """Amount each world-x value moves left."""
    x = np.asarray(x, dtype=np.float64)
    s = np.zeros_like(x)
    for a, b in slabs:
        s += np.clip(x, a, b) - a
    return s


def map_x(x, slabs):
    return np.asarray(x, dtype=np.float64) - shift_of(x, slabs)


def inside_any(lo, hi, slabs, eps=1e-6):
    return any(lo >= a - eps and hi <= b + eps for a, b in slabs)


# --------------------------------------------------------------------------- GLB
def read_glb(path):
    with open(path, 'rb') as f:
        data = f.read()
    magic, version, _ = struct.unpack_from('<III', data, 0)
    assert magic == 0x46546C67 and version == 2, 'not a glTF 2.0 binary'
    off = 12
    js = binbuf = None
    while off < len(data):
        clen, ctype = struct.unpack_from('<II', data, off)
        chunk = data[off + 8: off + 8 + clen]
        if ctype == 0x4E4F534A:
            js = json.loads(chunk.decode('utf-8'))
        elif ctype == 0x004E4942:
            binbuf = bytearray(chunk)
        off += 8 + clen
    return js, binbuf


def write_glb(path, js, binbuf):
    jbytes = json.dumps(js, separators=(',', ':')).encode('utf-8')
    jbytes += b' ' * (-len(jbytes) % 4)
    binbuf = bytes(binbuf) + b'\0' * (-len(binbuf) % 4)
    js_len = 8 + len(jbytes)
    total = 12 + js_len + 8 + len(binbuf)
    with open(path, 'wb') as f:
        f.write(struct.pack('<III', 0x46546C67, 2, total))
        f.write(struct.pack('<II', len(jbytes), 0x4E4F534A))
        f.write(jbytes)
        f.write(struct.pack('<II', len(binbuf), 0x004E4942))
        f.write(binbuf)


def accessor_view(js, binbuf, idx):
    acc = js['accessors'][idx]
    bv = js['bufferViews'][acc['bufferView']]
    assert 'byteStride' not in bv, 'interleaved buffers not supported'
    ncomp = {'SCALAR': 1, 'VEC2': 2, 'VEC3': 3, 'VEC4': 4}[acc['type']]
    dt = COMP_DTYPE[acc['componentType']]
    start = bv.get('byteOffset', 0) + acc.get('byteOffset', 0)
    arr = np.frombuffer(binbuf, dtype=dt, count=acc['count'] * ncomp, offset=start)
    return arr.reshape(-1, ncomp) if ncomp > 1 else arr, start, dt


def shrink_glb(src, dst, slabs):
    js, binbuf = read_glb(src)
    pos_users = {}
    for m in js['meshes']:
        for p in m['primitives']:
            pos_users[p['attributes']['POSITION']] = pos_users.get(p['attributes']['POSITION'], 0) + 1
    assert all(v == 1 for v in pos_users.values()), 'shared POSITION accessors not supported'

    removed_tris = kept_tris = dropped_nodes = 0
    for node in js['nodes']:
        if 'mesh' not in node:
            continue
        mesh = js['meshes'][node['mesh']]
        tx = node.get('translation', [0, 0, 0])[0]
        sx = node.get('scale', [1, 1, 1])[0]
        if 'rotation' in node or 'matrix' in node or abs(sx - 1) > 1e-9:
            # Only handled when the node is completely left of every slab.
            for p in mesh['primitives']:
                acc = js['accessors'][p['attributes']['POSITION']]
                r = max(abs(acc['min'][0]), abs(acc['max'][0]), abs(acc['min'][1]), abs(acc['max'][1]))
                assert tx + r * max(1, abs(sx)) < min(a for a, _ in slabs), node['name']
            continue

        new_prims = []
        for p in mesh['primitives']:
            assert p.get('mode', 4) == 4 and 'indices' in p, 'only indexed triangles supported'
            pos, pstart, _ = accessor_view(js, binbuf, p['attributes']['POSITION'])
            pos = pos.astype(np.float64)
            wx = pos[:, 0] + tx

            idx, istart, idt = accessor_view(js, binbuf, p['indices'])
            tris = idx.reshape(-1, 3)
            tx3 = wx[tris]
            dead = np.zeros(len(tris), dtype=bool)
            for a, b in slabs:
                dead |= np.all((tx3 >= a - 1e-6) & (tx3 <= b + 1e-6), axis=1)
            removed_tris += int(dead.sum())
            alive = tris[~dead]
            kept_tris += len(alive)
            if len(alive) == 0:
                continue

            # rewrite indices in place (fewer of them)
            flat = alive.reshape(-1).astype(idt)
            binbuf[istart: istart + flat.nbytes] = flat.tobytes()
            iacc = js['accessors'][p['indices']]
            iacc['count'] = int(flat.size)
            if 'min' in iacc:
                iacc['min'] = [int(flat.min())]
                iacc['max'] = [int(flat.max())]

            # shift vertices
            pos[:, 0] = map_x(wx, slabs) - tx
            p32 = pos.astype(np.float32)
            binbuf[pstart: pstart + p32.nbytes] = p32.tobytes()
            used = p32[np.unique(flat)]
            pacc = js['accessors'][p['attributes']['POSITION']]
            pacc['min'] = [float(v) for v in used.min(axis=0)]
            pacc['max'] = [float(v) for v in used.max(axis=0)]
            new_prims.append(p)

        if new_prims:
            mesh['primitives'] = new_prims
        else:
            del node['mesh']
            dropped_nodes += 1

    write_glb(dst, js, binbuf)
    print(f'glb: removed {removed_tris} triangles, kept {kept_tris}, dropped {dropped_nodes} mesh nodes')


# --------------------------------------------------------------------------- SDF
COLLISION_RE = re.compile(r'[ \t]*<collision name="[^"]*">.*?</collision>[ \t]*\r?\n', re.S)


def shrink_sdf(src, dst, slabs):
    """Edit <collision> box blocks textually so the rest of the file keeps its formatting."""
    text = open(src, newline='').read()
    stats = {'removed': 0, 'changed': 0}

    def fix(m):
        block = m.group(0)
        col = ET.fromstring(block.strip())
        pose_el = col.find('pose')
        size_el = col.find('geometry/box/size')
        if pose_el is None or size_el is None:
            return block
        pose = [float(v) for v in pose_el.text.split()]
        size = [float(v) for v in size_el.text.split()]
        # roll-only rotations (ramps/stairs) do not change the X extent
        assert abs(pose[4]) < 1e-6 and abs(pose[5]) < 1e-6, col.get('name')
        lo, hi = pose[0] - size[0] / 2, pose[0] + size[0] / 2
        if inside_any(lo, hi, slabs):
            stats['removed'] += 1
            return ''
        nlo, nhi = map_x([lo, hi], slabs)
        if abs(nlo - lo) < 1e-9 and abs(nhi - hi) < 1e-9:
            return block
        stats['changed'] += 1
        new_pose = [f'{(nlo + nhi) / 2:.6f}'] + pose_el.text.split()[1:]
        new_size = [f'{nhi - nlo:.6f}'] + size_el.text.split()[1:]
        block = block.replace(f'<pose>{pose_el.text}</pose>', f'<pose>{" ".join(new_pose)}</pose>', 1)
        block = block.replace(f'<size>{size_el.text}</size>', f'<size>{" ".join(new_size)}</size>', 1)
        return block

    text = COLLISION_RE.sub(fix, text)
    with open(dst, 'w', newline='') as f:
        f.write(text)
    print(f'sdf: removed {stats["removed"]} colliders, moved/resized {stats["changed"]}')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('src')
    ap.add_argument('dst')
    ap.add_argument('--slab', nargs=2, type=float, action='append', metavar=('A', 'B'))
    args = ap.parse_args()
    slabs = sorted(tuple(s) for s in (args.slab or DEFAULT_SLABS))
    os.makedirs(args.dst, exist_ok=True)
    print('slabs:', slabs, ' total width removed:', sum(b - a for a, b in slabs))
    shrink_glb(os.path.join(args.src, 'base_visual.glb'), os.path.join(args.dst, 'base_visual.glb'), slabs)
    shrink_sdf(os.path.join(args.src, 'model.sdf'), os.path.join(args.dst, 'model.sdf'), slabs)


if __name__ == '__main__':
    main()
