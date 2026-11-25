#!/usr/bin/env python3
"""Make the Distribution_Warehouse visual mesh cheaper to render.

Run AFTER tools/shrink_warehouse.py (it relies on the original node names):

  python3 tools/shrink_warehouse.py   models/Distribution_Warehouse_backup models/Distribution_Warehouse
  python3 tools/simplify_warehouse.py models/Distribution_Warehouse

What it does to base_visual.glb (model.sdf colliders are only read, never changed):
  * Container  : the 69k-triangle container shell is replaced by plain boxes (sides, roof,
                 back) built from the container's own <collision> boxes in model.sdf
                 (so visuals match physics exactly).
  * Rafters    : only the roof trusses closest to KEEP_RAFTER_X are kept.
  * Draw calls : every remaining mesh is baked to world space and merged into one
                 primitive per material (hundreds of nodes -> ~20 draw calls).
  * Textures   : every image is downscaled to at most MAX_TEX px and re-encoded as JPEG.
"""

import argparse
import io
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from shrink_warehouse import accessor_view, read_glb, write_glb  # noqa: E402

KEEP_RAFTER_X = [-24.38, -15.24, -6.10, 1.80]
MAX_TEX = 1024
JPEG_QUALITY = 85


# ----------------------------------------------------------------- math helpers
def quat_to_mat(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def node_matrix(node):
    m = np.eye(4)
    if 'matrix' in node:
        return np.array(node['matrix'], dtype=np.float64).reshape(4, 4).T
    r = quat_to_mat(node.get('rotation', [0, 0, 0, 1]))
    s = np.diag(node.get('scale', [1, 1, 1]))
    m[:3, :3] = r @ s
    m[:3, 3] = node.get('translation', [0, 0, 0])
    return m


# ----------------------------------------------------------------- box geometry
def box_geometry(center, size):
    """24 vertices (4 per face, own normals) + 36 indices for an axis-aligned box."""
    c, h = np.asarray(center, float), np.asarray(size, float) / 2
    pos, nrm, idx = [], [], []
    for axis in range(3):
        for sign in (-1, 1):
            n = np.zeros(3)
            n[axis] = sign
            u, v = [a for a in range(3) if a != axis]
            corners = []
            for du, dv in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
                p = c.copy()
                p[axis] += sign * h[axis]
                p[u] += du * h[u]
                p[v] += dv * h[v]
                corners.append(p)
            base = len(pos)
            pos += corners
            nrm += [n] * 4
            # winding so the face points along +n
            tri = [0, 1, 2, 0, 2, 3]
            if np.dot(np.cross(corners[1] - corners[0], corners[2] - corners[0]), n) < 0:
                tri = [0, 2, 1, 0, 3, 2]
            idx += [base + t for t in tri]
    return np.array(pos), np.array(nrm), np.array(idx)


def container_boxes(sdf_path):
    boxes = []
    for col in ET.parse(sdf_path).getroot().iter('collision'):
        if not col.get('name', '').startswith('container'):
            continue
        pose = [float(v) for v in col.find('pose').text.split()]
        size = [float(v) for v in col.find('geometry/box/size').text.split()]
        boxes.append((pose[:3], size))
    return boxes


def mean_color(img_bytes):
    im = Image.open(io.BytesIO(img_bytes)).convert('RGB').resize((64, 64))
    c = np.asarray(im, dtype=np.float64).reshape(-1, 3).mean(axis=0) / 255.0
    return [float(v) ** 2.2 for v in c] + [1.0]  # sRGB -> linear baseColorFactor


# ----------------------------------------------------------------- main
def simplify(model_dir):
    glb_path = os.path.join(model_dir, 'base_visual.glb')
    js, binbuf = read_glb(glb_path)
    names = [n.get('name', '') for n in js['nodes']]
    assert any(n.startswith('Container') for n in names), \
        'no Container node found - run simplify on the output of shrink_warehouse.py, not twice'

    def image_bytes(i):
        bv = js['bufferViews'][js['images'][i]['bufferView']]
        o = bv.get('byteOffset', 0)
        return bytes(binbuf[o: o + bv['byteLength']])

    def image_index(name_part):
        return next(i for i, im in enumerate(js['images']) if name_part in im.get('name', ''))

    # pick rafters to keep
    rafter_nodes = []
    for i, n in enumerate(js['nodes']):
        if n.get('name', '').startswith('Rafters') and 'mesh' in n:
            acc = js['accessors'][js['meshes'][n['mesh']]['primitives'][0]['attributes']['POSITION']]
            x = (acc['min'][0] + acc['max'][0]) / 2 + n.get('translation', [0, 0, 0])[0]
            rafter_nodes.append((i, x))
    keep_rafters = {min(rafter_nodes, key=lambda t: abs(t[1] - kx))[0] for kx in KEEP_RAFTER_X}

    # gather geometry, grouped by (material, has TEXCOORD_1)
    groups = {}
    dropped = {'container': 0, 'rafters': 0}
    for ni, node in enumerate(js['nodes']):
        if 'mesh' not in node:
            continue
        name = node.get('name', '')
        mesh = js['meshes'][node['mesh']]
        if name.startswith('Container'):
            dropped['container'] += sum(js['accessors'][p['indices']]['count'] // 3 for p in mesh['primitives'])
            continue
        if name.startswith('Rafters') and ni not in keep_rafters:
            dropped['rafters'] += sum(js['accessors'][p['indices']]['count'] // 3 for p in mesh['primitives'])
            continue
        m = node_matrix(node)
        nm = np.linalg.inv(m[:3, :3]).T
        for p in mesh['primitives']:
            attrs = p['attributes']
            pos = accessor_view(js, binbuf, attrs['POSITION'])[0].astype(np.float64)
            nrm = accessor_view(js, binbuf, attrs['NORMAL'])[0].astype(np.float64)
            uv0 = accessor_view(js, binbuf, attrs['TEXCOORD_0'])[0].astype(np.float32)
            uv1 = (accessor_view(js, binbuf, attrs['TEXCOORD_1'])[0].astype(np.float32)
                   if 'TEXCOORD_1' in attrs else None)
            idx = accessor_view(js, binbuf, p['indices'])[0].astype(np.uint32)
            pos = pos @ m[:3, :3].T + m[:3, 3]
            nrm = nrm @ nm.T
            nrm /= np.maximum(np.linalg.norm(nrm, axis=1, keepdims=True), 1e-12)
            if np.linalg.det(m[:3, :3]) < 0:  # mirrored node -> flip winding
                idx = idx.reshape(-1, 3)[:, ::-1].reshape(-1)
            key = (p.get('material'), uv1 is not None)
            groups.setdefault(key, []).append((pos, nrm, uv0, uv1, idx))

    # container replacement: open box assembled from its colliders
    materials = js['materials']
    cont_mat = len(materials)
    materials.append({
        'name': 'ContainerSimple',
        'doubleSided': True,
        'pbrMetallicRoughness': {
            'baseColorFactor': mean_color(image_bytes(image_index('Container_Albedo'))),
            'metallicFactor': 0.3, 'roughnessFactor': 0.7},
    })
    for center, size in container_boxes(os.path.join(model_dir, 'model.sdf')):
        if size[2] < 0.2 and center[2] < 0.5:
            continue  # floor: the original textured ContainerFloor quad is kept
        pos, nrm, idx = box_geometry(center, size)
        groups.setdefault((cont_mat, False), []).append(
            (pos, nrm, np.zeros((len(pos), 2), np.float32), None, idx.astype(np.uint32)))

    # ------------------------------------------------------------- write new glb
    out = bytearray()
    new_js = {k: js[k] for k in ('asset', 'scene', 'samplers', 'textures') if k in js}
    for k in ('extensionsUsed', 'extensionsRequired'):
        if k in js:
            new_js[k] = js[k]
    new_js['materials'] = materials
    accessors, views = [], []

    def add_view(arr, target=None):
        while len(out) % 4:
            out.append(0)
        data = np.ascontiguousarray(arr).tobytes()
        v = {'buffer': 0, 'byteOffset': len(out), 'byteLength': len(data)}
        if target:
            v['target'] = target
        out.extend(data)
        views.append(v)
        return len(views) - 1

    def add_accessor(arr, ctype, typ, target, minmax=False):
        a = {'bufferView': add_view(arr, target), 'componentType': ctype,
             'count': int(arr.shape[0]), 'type': typ}
        if minmax:
            a['min'] = [float(v) for v in arr.min(axis=0)]
            a['max'] = [float(v) for v in arr.max(axis=0)]
        accessors.append(a)
        return len(accessors) - 1

    meshes, nodes = [], []
    total_tris = 0
    for (mat, has_uv1), parts in sorted(groups.items(), key=lambda kv: (kv[0][0] is None, kv[0])):
        offs, pos_l, nrm_l, uv0_l, uv1_l, idx_l = 0, [], [], [], [], []
        for pos, nrm, uv0, uv1, idx in parts:
            pos_l.append(pos)
            nrm_l.append(nrm)
            uv0_l.append(uv0)
            if has_uv1:
                uv1_l.append(uv1)
            idx_l.append(idx + offs)
            offs += len(pos)
        idx = np.concatenate(idx_l).astype(np.uint32)
        total_tris += len(idx) // 3
        attrs = {
            'POSITION': add_accessor(np.concatenate(pos_l).astype(np.float32), 5126, 'VEC3', 34962, True),
            'NORMAL': add_accessor(np.concatenate(nrm_l).astype(np.float32), 5126, 'VEC3', 34962),
            'TEXCOORD_0': add_accessor(np.concatenate(uv0_l).astype(np.float32), 5126, 'VEC2', 34962),
        }
        if has_uv1:
            attrs['TEXCOORD_1'] = add_accessor(np.concatenate(uv1_l).astype(np.float32), 5126, 'VEC2', 34962)
        prim = {'attributes': attrs, 'indices': add_accessor(idx, 5125, 'SCALAR', 34963), 'mode': 4}
        if mat is not None:
            prim['material'] = mat
        mname = materials[mat]['name'] if mat is not None else 'default'
        meshes.append({'name': f'merged_{mname}', 'primitives': [prim]})
        nodes.append({'name': f'merged_{mname}', 'mesh': len(meshes) - 1})

    # textures: downscale + re-encode
    images = []
    tex_before = tex_after = 0
    for i, im in enumerate(js['images']):
        raw = image_bytes(i)
        tex_before += len(raw)
        pil = Image.open(io.BytesIO(raw))
        if max(pil.size) > MAX_TEX:
            scale = MAX_TEX / max(pil.size)
            pil = pil.resize((max(1, round(pil.width * scale)), max(1, round(pil.height * scale))),
                             Image.LANCZOS)
        buf = io.BytesIO()
        pil.convert('RGB').save(buf, format='JPEG', quality=JPEG_QUALITY, optimize=True)
        data = buf.getvalue()
        if len(data) > len(raw) and max(Image.open(io.BytesIO(raw)).size) <= MAX_TEX:
            data = raw  # small image that only got bigger: keep original
        tex_after += len(data)
        images.append({'name': im.get('name', f'img{i}'), 'mimeType': 'image/jpeg',
                       'bufferView': add_view(np.frombuffer(data, np.uint8))})

    new_js.update({
        'scenes': [{'name': js['scenes'][0].get('name', 'scene'), 'nodes': list(range(len(nodes)))}],
        'scene': 0, 'nodes': nodes, 'meshes': meshes, 'images': images,
        'accessors': accessors, 'bufferViews': views, 'buffers': [{'byteLength': len(out)}],
    })
    while len(out) % 4:
        out.append(0)
    new_js['buffers'][0]['byteLength'] = len(out)
    write_glb(glb_path, new_js, out)

    print(f'dropped container: {dropped["container"]} tris, rafters: {dropped["rafters"]} tris '
          f'(kept {len(keep_rafters)} of {len(rafter_nodes)} rafters)')
    print(f'result: {len(nodes)} draw calls, {total_tris} triangles, '
          f'textures {tex_before / 1e6:.1f} MB -> {tex_after / 1e6:.1f} MB, '
          f'file {os.path.getsize(glb_path) / 1e6:.1f} MB')


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('model_dir')
    simplify(ap.parse_args().model_dir)
