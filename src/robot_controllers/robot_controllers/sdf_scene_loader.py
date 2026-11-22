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

"""Parse Gazebo SDF models/worlds and build MoveIt 2 PlanningScene CollisionObjects."""

from functools import lru_cache
import math
import os
import re
from typing import Dict, Iterable, List, Optional, Tuple
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_msgs.msg import CollisionObject
from robot_controllers.geometry_utils import (
    multiply_quats,
    rpy_to_quat_tuple,
    transform_base_to_world,
)
from shape_msgs.msg import SolidPrimitive

PosTuple = Tuple[float, float, float]
QuatTuple = Tuple[float, float, float, float]
CollisionPrimitiveSpec = Tuple[int, Tuple[float, ...], PosTuple, QuatTuple]
WorldIncludeMap = Dict[str, Tuple[str, PosTuple, QuatTuple]]

_IGNORED_MODELS = frozenset({'ground_plane', 'sun'})
_DEFAULT_ROBOT_PREFIXES = ('b1z1', 'b2_cargo', 'bcr_arm', 'a200', 'husky')


def _parse_pose_text(
    text: Optional[str],
) -> Tuple[PosTuple, QuatTuple]:
    """Parse SDF <pose>x y z r p y</pose> text into (position, quaternion_xyzw)."""
    if not text:
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)
    vals = [float(v) for v in text.split()]
    while len(vals) < 6:
        vals.append(0.0)
    pos = (vals[0], vals[1], vals[2])
    quat = rpy_to_quat_tuple(vals[3], vals[4], vals[5])
    return pos, quat


def compose_pose(
    parent_pos: PosTuple,
    parent_quat: QuatTuple,
    child_pos: PosTuple,
    child_quat: QuatTuple,
) -> Tuple[PosTuple, QuatTuple]:
    """Compose child pose expressed in parent frame into the parent's reference frame."""
    world_pos = transform_base_to_world(
        child_pos[0], child_pos[1], child_pos[2], parent_pos, parent_quat
    )
    world_quat = multiply_quats(parent_quat, child_quat)
    return world_pos, world_quat


@lru_cache(maxsize=64)
def parse_model_sdf_collisions(model_sdf_path: str) -> Tuple[CollisionPrimitiveSpec, ...]:
    """Parse supported (<box>, <cylinder>) collision primitives from a model.sdf file."""
    if not os.path.isfile(model_sdf_path):
        return ()
    tree = ET.parse(model_sdf_path)
    root = tree.getroot()
    model_el = root.find('model')
    if model_el is None:
        return ()

    specs: List[CollisionPrimitiveSpec] = []
    for link_el in model_el.findall('link'):
        link_pos, link_quat = _parse_pose_text(link_el.findtext('pose'))
        for col_el in link_el.findall('collision'):
            col_pos, col_quat = _parse_pose_text(col_el.findtext('pose'))
            local_pos, local_quat = compose_pose(link_pos, link_quat, col_pos, col_quat)
            geom = col_el.find('geometry')
            if geom is None:
                continue
            box_el = geom.find('box')
            if box_el is not None:
                size_txt = box_el.findtext('size')
                if size_txt:
                    dims = tuple(float(v) for v in size_txt.split()[:3])
                    if len(dims) == 3:
                        specs.append((SolidPrimitive.BOX, dims, local_pos, local_quat))
                continue
            cyl_el = geom.find('cylinder')
            if cyl_el is not None:
                r_txt = cyl_el.findtext('radius')
                l_txt = cyl_el.findtext('length')
                if r_txt is not None and l_txt is not None:
                    dims = (float(l_txt), float(r_txt))
                    specs.append((SolidPrimitive.CYLINDER, dims, local_pos, local_quat))
    return tuple(specs)


@lru_cache(maxsize=8)
def parse_world_sdf_includes(world_sdf_path: str) -> WorldIncludeMap:
    """Parse <include> models from a Gazebo world SDF into name -> (uri, pos, quat)."""
    if not os.path.isfile(world_sdf_path):
        return {}
    tree = ET.parse(world_sdf_path)
    root = tree.getroot()
    includes: WorldIncludeMap = {}
    for inc in root.findall('.//include'):
        name = (inc.findtext('name') or '').strip()
        uri = (inc.findtext('uri') or '').strip()
        if not name or not uri:
            continue
        if uri.startswith('model://'):
            uri = uri[len('model://'):]
        pos, quat = _parse_pose_text(inc.findtext('pose'))
        includes[name] = (uri, pos, quat)
    return includes


def _default_models_dir() -> Optional[str]:
    try:
        pkg = get_package_share_directory('warehouse_sim')
        candidate = os.path.join(pkg, 'models')
        if os.path.isdir(candidate):
            return candidate
    except Exception:
        pass
    return None


def resolve_model_sdf_path(
    entity_name: str,
    uri_hint: Optional[str] = None,
    models_dir: Optional[str] = None,
) -> Optional[str]:
    """Resolve the model.sdf path for a Gazebo entity or model URI inside models_dir."""
    base_dir = models_dir or _default_models_dir()
    if not base_dir or not os.path.isdir(base_dir):
        return None

    candidates: List[str] = []
    if uri_hint:
        candidates.append(uri_hint.strip())
    if entity_name:
        candidates.append(entity_name.strip())
        stripped_dot = re.sub(r'\.\d+$', '', entity_name.strip())
        stripped_under = re.sub(r'_\d+$', '', entity_name.strip())
        candidates.extend([stripped_dot, stripped_under])

    seen = set()
    for cand in candidates:
        if not cand or cand in seen:
            continue
        seen.add(cand)
        sdf_path = os.path.join(base_dir, cand, 'model.sdf')
        if os.path.isfile(sdf_path):
            return sdf_path
    return None


def build_collision_object_from_sdf(
    scene_id: str,
    model_sdf_path: str,
    world_pos: PosTuple,
    world_quat: QuatTuple,
    frame_id: str = 'map',
) -> Optional[CollisionObject]:
    """Build a multi-primitive MoveIt CollisionObject from a model.sdf placed at world pose."""
    specs = parse_model_sdf_collisions(model_sdf_path)
    if not specs:
        return None

    co = CollisionObject()
    co.header.frame_id = frame_id
    co.id = scene_id
    co.operation = CollisionObject.ADD

    for prim_type, dims, local_pos, local_quat in specs:
        prim = SolidPrimitive()
        prim.type = prim_type
        prim.dimensions = [float(d) for d in dims]

        abs_pos, abs_quat = compose_pose(world_pos, world_quat, local_pos, local_quat)
        pose = Pose()
        pose.position = Point(x=abs_pos[0], y=abs_pos[1], z=abs_pos[2])
        pose.orientation = Quaternion(
            x=abs_quat[0], y=abs_quat[1], z=abs_quat[2], w=abs_quat[3]
        )
        co.primitives.append(prim)
        co.primitive_poses.append(pose)

    return co


def build_world_collision_objects(
    entity_poses: Dict[str, Tuple[float, float, float, QuatTuple]],
    center_xy: Tuple[float, float],
    radius_m: float = 2.0,
    world_includes: Optional[WorldIncludeMap] = None,
    models_dir: Optional[str] = None,
    exclude_names: Iterable[str] = (),
    frame_id: str = 'map',
) -> List[CollisionObject]:
    """Build MoveIt CollisionObjects for all resolvable world models near `center_xy`."""
    excluded = set(exclude_names) | _IGNORED_MODELS
    merged: Dict[str, Tuple[Optional[str], PosTuple, QuatTuple]] = {}

    if world_includes:
        for name, (uri, pos, quat) in world_includes.items():
            merged[name] = (uri, pos, quat)

    for name, pose in entity_poses.items():
        if not name or '::' in name:
            continue
        pos = (float(pose[0]), float(pose[1]), float(pose[2]))
        quat = (float(pose[3][0]), float(pose[3][1]), float(pose[3][2]), float(pose[3][3]))
        prev_uri = merged[name][0] if name in merged else None
        merged[name] = (prev_uri, pos, quat)

    cx, cy = float(center_xy[0]), float(center_xy[1])
    objects: List[CollisionObject] = []
    for name, (uri, pos, quat) in merged.items():
        if name in excluded:
            continue
        if any(name == p or name.startswith(f'{p}_') for p in _DEFAULT_ROBOT_PREFIXES):
            continue
        if math.hypot(pos[0] - cx, pos[1] - cy) > radius_m:
            continue
        sdf_path = resolve_model_sdf_path(name, uri_hint=uri, models_dir=models_dir)
        if not sdf_path:
            continue
        co = build_collision_object_from_sdf(
            f'world_{name}', sdf_path, pos, quat, frame_id=frame_id
        )
        if co is not None:
            objects.append(co)

    return objects
