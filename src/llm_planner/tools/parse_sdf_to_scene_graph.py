#!/usr/bin/env python3
"""
parse_sdf_to_scene_graph.py

Offline CLI tool for TEMPO.
Parses a Gazebo world SDF (e.g., warehouse.sdf) and maps models into
a multi-layer hierarchical 3D Scene Graph (Warehouse -> Zone -> Aisle/Fixture -> Object)
using zone/aisle bounding boxes, SDF joint plugins, and a model metadata registry.

Outputs: warehouse_scene_graph.json
"""

import json
import math
import os
import xml.etree.ElementTree as ET
from collections import Counter
from typing import Any, Dict, List, Optional, Tuple


def parse_pose(pose_str: Optional[str]) -> Dict[str, float]:
    """Parse space-separated pose string [x y z roll pitch yaw] into a dictionary."""
    if not pose_str or not pose_str.strip():
        return {"x": 0.0, "y": 0.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}
    parts = [float(p) for p in pose_str.strip().split()]
    while len(parts) < 6:
        parts.append(0.0)
    return {
        "x": round(parts[0], 4),
        "y": round(parts[1], 4),
        "z": round(parts[2], 4),
        "roll": round(parts[3], 4),
        "pitch": round(parts[4], 4),
        "yaw": round(parts[5], 4),
    }


def in_bounds(x: float, y: float, bounds: Optional[Dict[str, float]]) -> bool:
    """Return True if (x, y) lies within the given bounding box."""
    if not bounds:
        return False
    return (
        bounds["x_min"] <= x <= bounds["x_max"]
        and bounds["y_min"] <= y <= bounds["y_max"]
    )


def find_zone_for_model(x: float, y: float, zones_cfg: Dict[str, Any]) -> str:
    """Find the zone matching the given (x, y) coordinates based on bounding boxes."""
    catchall_zone = None
    for zone_id, zone_data in zones_cfg.items():
        if zone_data.get("is_catchall"):
            catchall_zone = zone_id
            continue
        if in_bounds(x, y, zone_data.get("bounds")):
            return zone_id

    return catchall_zone or "maintenance_zone"


def find_aisle_for_fixture(
    x: float, y: float, aisles_cfg: Dict[str, Any]
) -> Optional[str]:
    """Find the aisle within a zone matching the fixture's (x, y) coordinates."""
    for aisle_id, aisle_data in aisles_cfg.items():
        if in_bounds(x, y, aisle_data.get("bounds")):
            return aisle_id
    return None


def compute_inter_zone_distances(zones_cfg: Dict[str, Any]) -> Dict[str, Dict[str, float]]:
    """Compute pairwise 2D Euclidean distances (m) between zone centroids."""
    distances: Dict[str, Dict[str, float]] = {}
    zone_ids = list(zones_cfg.keys())
    for z1 in zone_ids:
        c1 = zones_cfg[z1].get("centroid", [0.0, 0.0])
        distances[z1] = {}
        for z2 in zone_ids:
            if z1 == z2:
                distances[z1][z2] = 0.0
            else:
                c2 = zones_cfg[z2].get("centroid", [0.0, 0.0])
                distances[z1][z2] = round(math.hypot(c1[0] - c2[0], c1[1] - c2[1]), 2)
    return distances


def extract_detachable_joints(inc_el: ET.Element, parent_name: str) -> List[Tuple[str, str, Optional[str]]]:
    """
    Extract (parent_model, child_model, detach_topic) from gz::sim::systems::DetachableJoint
    plugins embedded inside an <include> element in the SDF.
    """
    joints: List[Tuple[str, str, Optional[str]]] = []
    for plugin in inc_el.findall("plugin"):
        plugin_name = plugin.attrib.get("name", "")
        plugin_file = plugin.attrib.get("filename", "")
        if "DetachableJoint" in plugin_name or "detachable-joint" in plugin_file:
            child_el = plugin.find("child_model")
            topic_el = plugin.find("detach_topic")
            if child_el is not None and child_el.text:
                child_model = child_el.text.strip()
                detach_topic = topic_el.text.strip() if topic_el is not None and topic_el.text else None
                joints.append((parent_name, child_model, detach_topic))
    return joints


def detect_vertical_stacks(
    free_objects: List[Dict[str, Any]],
    xy_tolerance: float = 0.20,
    min_z_diff: float = 0.05,
) -> List[Dict[str, Any]]:
    """
    Detect vertical stacks among free objects of the same model_type at the same (x, y)
    (e.g., stacked pallets) and annotate stack_level / stacked_on + return STACKED_ON edges.
    """
    edges: List[Dict[str, Any]] = []
    visited = set()

    for i, obj in enumerate(free_objects):
        if obj["id"] in visited:
            continue
        cluster = [obj]
        ox, oy = obj["pose"]["x"], obj["pose"]["y"]
        for j, other in enumerate(free_objects):
            if i == j or other["id"] in visited:
                continue
            if other["model_type"] == obj["model_type"]:
                if math.hypot(ox - other["pose"]["x"], oy - other["pose"]["y"]) <= xy_tolerance:
                    cluster.append(other)

        if len(cluster) > 1:
            cluster.sort(key=lambda item: item["pose"]["z"])
            if cluster[-1]["pose"]["z"] - cluster[0]["pose"]["z"] >= min_z_diff:
                for idx, stacked_item in enumerate(cluster):
                    visited.add(stacked_item["id"])
                    stacked_item["stack_level"] = idx + 1
                    if idx > 0:
                        below_id = cluster[idx - 1]["id"]
                        stacked_item["stacked_on"] = below_id
                        edges.append({
                            "source": below_id,
                            "target": stacked_item["id"],
                            "relation": "SUPPORTS",
                            "metadata": {"stack_level": idx + 1},
                        })
                    else:
                        stacked_item["stacked_on"] = None

    return edges


def build_scene_graph(
    sdf_path: str,
    zones_path: str,
    weights_path: str,
) -> Dict[str, Any]:
    """Parse SDF and combine with zone/aisle and weight configurations into a 3D Scene Graph."""
    if not os.path.isfile(sdf_path):
        raise FileNotFoundError(f"SDF file not found: {sdf_path}")
    if not os.path.isfile(zones_path):
        raise FileNotFoundError(f"Zones configuration not found: {zones_path}")
    if not os.path.isfile(weights_path):
        raise FileNotFoundError(f"Model weights configuration not found: {weights_path}")

    with open(zones_path, "r", encoding="utf-8") as f:
        zones_cfg = json.load(f).get("zones", {})

    with open(weights_path, "r", encoding="utf-8") as f:
        model_meta = json.load(f).get("models", {})

    tree = ET.parse(sdf_path)
    root = tree.getroot()
    world = root.find("world")
    if world is None:
        raise ValueError("No <world> tag found in SDF.")

    includes = world.findall("include")

    # Initialize hierarchical zone containers
    zones_output: Dict[str, Dict[str, Any]] = {}
    for zone_id, zone_data in zones_cfg.items():
        aisles_cfg = zone_data.get("aisles", {})
        aisles_dict: Dict[str, Dict[str, Any]] = {}
        for aisle_id, aisle_data in aisles_cfg.items():
            aisles_dict[aisle_id] = {
                "name": aisle_data.get("name", aisle_id),
                "description": aisle_data.get("description", ""),
                "tags": aisle_data.get("tags", []),
                "bounds": aisle_data.get("bounds"),
                "centroid": aisle_data.get("centroid"),
                "summary": {},
                "fixtures": {},
            }

        zones_output[zone_id] = {
            "name": zone_data.get("name", zone_id),
            "description": zone_data.get("description", ""),
            "bounds": zone_data.get("bounds"),
            "centroid": zone_data.get("centroid"),
            "summary": {},
            "aisles": aisles_dict if aisles_dict else None,
            "fixtures": {},
            "free_objects": [],
        }
        if zones_output[zone_id]["aisles"] is None:
            del zones_output[zone_id]["aisles"]

    # Pass 1: Parse all <include> elements and SDF DetachableJoint plugins
    all_fixtures: Dict[str, Tuple[str, Optional[str], Dict[str, Any]]] = {}  # id -> (zone_id, aisle_id, fixture_dict)
    raw_objects: List[Tuple[str, Dict[str, Any]]] = []  # (zone_id, obj_dict)
    sdf_joints: Dict[str, Tuple[str, Optional[str]]] = {}  # child_id -> (parent_id, detach_topic)

    total_parsed = 0
    skipped_infrastructure = 0

    for inc in includes:
        name_el = inc.find("name")
        uri_el = inc.find("uri")
        pose_el = inc.find("pose")
        static_el = inc.find("static")

        if name_el is None or not name_el.text:
            continue

        model_name = name_el.text.strip()
        model_uri = uri_el.text.strip() if uri_el is not None and uri_el.text else model_name
        pose_str = pose_el.text.strip() if pose_el is not None and pose_el.text else None
        sdf_static = (
            static_el.text.strip().lower() == "true"
            if static_el is not None and static_el.text
            else False
        )

        # Record any DetachableJoint plugins on this model
        for parent_id, child_id, detach_topic in extract_detachable_joints(inc, model_name):
            sdf_joints[child_id] = (parent_id, detach_topic)

        meta = model_meta.get(model_uri, {})
        role = meta.get("role", "object")
        weight_kg = meta.get("weight_kg")
        category = meta.get("category", "generic")
        description = meta.get("description", "")

        if role == "static_infrastructure":
            skipped_infrastructure += 1
            continue

        pose = parse_pose(pose_str)
        zone_id = find_zone_for_model(pose["x"], pose["y"], zones_cfg)
        total_parsed += 1

        if role == "location":
            aisles_cfg = zones_cfg.get(zone_id, {}).get("aisles", {})
            aisle_id = (
                find_aisle_for_fixture(pose["x"], pose["y"], aisles_cfg)
                if aisles_cfg
                else None
            )
            fixture_entry: Dict[str, Any] = {
                "id": model_name,
                "model_type": model_uri,
                "role": "location",
                "category": category,
                "description": description,
                "pose": pose,
                "sdf_static": sdf_static,
                "zone_id": zone_id,
                "aisle_id": aisle_id,
                "children_summary": {},
                "children": [],
            }
            if aisle_id and "aisles" in zones_output[zone_id]:
                zones_output[zone_id]["aisles"][aisle_id]["fixtures"][model_name] = fixture_entry
            else:
                zones_output[zone_id]["fixtures"][model_name] = fixture_entry
            all_fixtures[model_name] = (zone_id, aisle_id, fixture_entry)
        else:
            obj_entry: Dict[str, Any] = {
                "id": model_name,
                "model_type": model_uri,
                "role": "object",
                "category": category,
                "description": description,
                "pose": pose,
                "sdf_static": sdf_static,
                "weight_kg": weight_kg,
                "zone_id": zone_id,
                "parent_location": None,
            }
            raw_objects.append((zone_id, obj_entry))

    # Pass 2: Establish parent-child containment edges and place objects into fixtures or free_objects
    edges: List[Dict[str, Any]] = []

    for zone_id, item in raw_objects:
        ix, iy, iz = item["pose"]["x"], item["pose"]["y"], item["pose"]["z"]
        matched_parent: Optional[Dict[str, Any]] = None
        edge_relation = "CONTAINS"
        edge_meta: Dict[str, Any] = {}

        # 1. Check explicit SDF DetachableJoint plugins first
        if item["id"] in sdf_joints:
            parent_id, detach_topic = sdf_joints[item["id"]]
            if parent_id in all_fixtures:
                _, _, matched_parent = all_fixtures[parent_id]
                edge_relation = "MOUNTED_ON"
                if detach_topic:
                    item["detach_topic"] = detach_topic
                    edge_meta["detach_topic"] = detach_topic

        # 2. Geometric containment for small_box on Picking_Shelves
        elif item["model_type"] == "small_box":
            for _, _, loc in all_fixtures.values():
                if loc["model_type"] == "Picking_Shelves":
                    lx, ly = loc["pose"]["x"], loc["pose"]["y"]
                    if math.hypot(ix - lx, iy - ly) < 0.05:
                        matched_parent = loc
                        tier = max(1, min(6, round((iz - 0.2) / 0.3) + 1))
                        item["shelf_tier"] = tier
                        edge_meta["shelf_tier"] = tier
                        break

        # 3. Fallback proximity matching for drills if not in SDF joints
        elif "cordless_drill" in item["model_type"]:
            for _, _, loc in all_fixtures.values():
                if loc["model_type"] == "wall_mount_bracket":
                    lx, ly = loc["pose"]["x"], loc["pose"]["y"]
                    if math.hypot(ix - lx, iy - ly) <= 1.0:
                        matched_parent = loc
                        edge_relation = "MOUNTED_ON"
                        break

        if matched_parent is not None:
            item["parent_location"] = matched_parent["id"]
            if matched_parent.get("aisle_id"):
                item["aisle_id"] = matched_parent["aisle_id"]
            matched_parent["children"].append(item)
            edges.append({
                "source": matched_parent["id"],
                "target": item["id"],
                "relation": edge_relation,
                "metadata": edge_meta,
            })
        else:
            zones_output[zone_id]["free_objects"].append(item)

    # Sort children inside each fixture and compute children_summary
    for _, _, fixture in all_fixtures.values():
        fixture["children"].sort(
            key=lambda c: (c.get("shelf_tier", 99), c["id"])
        )
        counts = Counter(c["model_type"] for c in fixture["children"])
        fixture["children_summary"] = dict(counts)

    # Detect vertical stacks among free_objects and compute summaries per aisle and zone
    total_fixtures_all = 0
    total_objects_all = 0

    for zone_id, zdata in zones_output.items():
        stack_edges = detect_vertical_stacks(zdata["free_objects"])
        edges.extend(stack_edges)

        zone_fixture_counts: Counter = Counter()
        zone_object_counts: Counter = Counter()

        # Summarize aisles if present
        if "aisles" in zdata:
            for aisle_id, adata in zdata["aisles"].items():
                aisle_fix_counts = Counter(
                    f["model_type"] for f in adata["fixtures"].values()
                )
                aisle_obj_counts: Counter = Counter()
                for f in adata["fixtures"].values():
                    for c in f["children"]:
                        aisle_obj_counts[c["model_type"]] += 1

                adata["summary"] = {
                    "total_fixtures": sum(aisle_fix_counts.values()),
                    "total_objects": sum(aisle_obj_counts.values()),
                    "fixture_counts": dict(aisle_fix_counts),
                    "object_counts": dict(aisle_obj_counts),
                }
                zone_fixture_counts.update(aisle_fix_counts)
                zone_object_counts.update(aisle_obj_counts)

        # Summarize zone-level fixtures
        for f in zdata["fixtures"].values():
            zone_fixture_counts[f["model_type"]] += 1
            for c in f["children"]:
                zone_object_counts[c["model_type"]] += 1

        # Summarize zone-level free objects
        for o in zdata["free_objects"]:
            zone_object_counts[o["model_type"]] += 1

        total_zone_fixtures = sum(zone_fixture_counts.values())
        total_zone_objects = sum(zone_object_counts.values())
        total_fixtures_all += total_zone_fixtures
        total_objects_all += total_zone_objects

        zdata["summary"] = {
            "total_fixtures": total_zone_fixtures,
            "total_objects": total_zone_objects,
            "aisle_count": len(zdata.get("aisles", {})),
            "fixture_counts": dict(zone_fixture_counts),
            "object_counts": dict(zone_object_counts),
        }

    scene_graph = {
        "warehouse": {
            "name": "TEMPO Distribution Warehouse",
            "world_file": os.path.basename(sdf_path),
            "total_models": total_parsed,
            "summary": {
                "total_zones": len(zones_output),
                "total_fixtures": total_fixtures_all,
                "total_objects": total_objects_all,
                "skipped_infrastructure": skipped_infrastructure,
            },
            "inter_zone_distances_m": compute_inter_zone_distances(zones_cfg),
            "zones": zones_output,
            "edges": edges,
        }
    }

    return scene_graph


def main():
    sdf_path = "src/warehouse_sim/worlds/warehouse.sdf"
    zones_path = "src/llm_planner/tools/zone_config.json"
    weights_path = "src/llm_planner/tools/model_weights.json"
    output_path = "src/llm_planner/knowledge_base/warehouse_scene_graph.json"

    print(f"Parsing SDF:   {sdf_path}")
    print(f"Using zones:   {zones_path}")
    print(f"Using weights: {weights_path}")

    sg = build_scene_graph(sdf_path, zones_path, weights_path)

    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(sg, f, indent=2)

    wh = sg["warehouse"]
    print(f"\nSuccessfully generated: {output_path}")
    print(
        f"Total mapped models: {wh['total_models']} "
        f"({wh['summary']['total_fixtures']} fixtures, "
        f"{wh['summary']['total_objects']} objects; "
        f"skipped {wh['summary']['skipped_infrastructure']} static infrastructure)"
    )
    print(f"Total relational edges: {len(wh['edges'])}")
    print("\n--- Zone Breakdown ---")
    for zone_id, zdata in wh["zones"].items():
        s = zdata["summary"]
        fix_str = ", ".join(f"{cnt}x {t}" for t, cnt in s["fixture_counts"].items()) or "none"
        obj_str = ", ".join(f"{cnt}x {t}" for t, cnt in s["object_counts"].items()) or "none"
        aisle_info = f" | {s['aisle_count']} aisles" if s.get("aisle_count") else ""
        print(
            f"  [{zone_id:18s}] Centroid: {str(zdata['centroid']):12s}{aisle_info} | "
            f"Fixtures ({s['total_fixtures']}): {fix_str} | "
            f"Objects ({s['total_objects']}): {obj_str}"
        )


if __name__ == "__main__":
    main()
