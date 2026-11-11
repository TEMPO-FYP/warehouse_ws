#!/usr/bin/env python3
"""
parse_sdf_to_scene_graph.py

Offline CLI tool for TEMPO.
Parses a Gazebo world SDF (e.g., warehouse.sdf) and maps models into
a structured 3D Scene Graph using zone bounding boxes and a model metadata registry.

Outputs: warehouse_scene_graph.json
"""

import argparse
import json
import os
import sys
import xml.etree.ElementTree as ET
from typing import Any, Dict, List, Optional


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


def find_zone_for_model(
    x: float, y: float, zones_cfg: Dict[str, Any]
) -> str:
    """Find the zone matching the given (x, y) coordinates based on bounding boxes."""
    catchall_zone = None
    for zone_id, zone_data in zones_cfg.items():
        if zone_data.get("is_catchall"):
            catchall_zone = zone_id
            continue

        bounds = zone_data.get("bounds")
        if bounds:
            if (
                bounds["x_min"] <= x <= bounds["x_max"]
                and bounds["y_min"] <= y <= bounds["y_max"]
            ):
                return zone_id

    return catchall_zone or "maintenance_zone"


def build_scene_graph(
    sdf_path: str,
    zones_path: str,
    weights_path: str,
) -> Dict[str, Any]:
    """Parse SDF and combine with zone and weight configurations."""
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

    # Initialize zone containers
    zones_output: Dict[str, Dict[str, Any]] = {}
    for zone_id, zone_data in zones_cfg.items():
        zones_output[zone_id] = {
            "name": zone_data.get("name", zone_id),
            "description": zone_data.get("description", ""),
            "bounds": zone_data.get("bounds"),
            "centroid": zone_data.get("centroid"),
            "objects": [],
        }

    total_parsed = 0
    skipped_infrastructure = 0

    for inc in includes:
        name_el = inc.find("name")
        uri_el = inc.find("uri")
        pose_el = inc.find("pose")

        if name_el is None or not name_el.text:
            continue

        model_name = name_el.text.strip()
        model_uri = uri_el.text.strip() if uri_el is not None and uri_el.text else model_name
        pose_str = pose_el.text.strip() if pose_el is not None and pose_el.text else None

        # Look up metadata
        meta = model_meta.get(model_uri, {})
        role = meta.get("role", "object")
        weight_kg = meta.get("weight_kg")
        category = meta.get("category", "generic")
        description = meta.get("description", "")

        # Skip entire building shell if needed, but include in static_infrastructure
        if role == "static_infrastructure":
            skipped_infrastructure += 1
            continue

        pose = parse_pose(pose_str)
        zone_id = find_zone_for_model(pose["x"], pose["y"], zones_cfg)

        obj_entry = {
            "id": model_name,
            "model_type": model_uri,
            "role": role,
            "category": category,
            "description": description,
            "pose": pose,
            "weight_kg": weight_kg,
            "parent_location": None,
        }

        zones_output[zone_id]["objects"].append(obj_entry)
        total_parsed += 1

    # Second pass: establish parent-child containment edges (e.g., box on shelf, drill on bracket)
    all_locations = []
    for zdata in zones_output.values():
        for item in zdata["objects"]:
            if item["role"] == "location":
                all_locations.append(item)

    import math
    for zdata in zones_output.values():
        for item in zdata["objects"]:
            if item["role"] != "object":
                continue
            ix, iy, iz = item["pose"]["x"], item["pose"]["y"], item["pose"]["z"]

            if item["model_type"] == "small_box":
                # Find matching Picking_Shelves at same (x, y)
                for loc in all_locations:
                    if loc["model_type"] == "Picking_Shelves":
                        lx, ly = loc["pose"]["x"], loc["pose"]["y"]
                        if math.hypot(ix - lx, iy - ly) < 0.05:
                            item["parent_location"] = loc["id"]
                            item["shelf_tier"] = max(1, min(6, round((iz - 0.2) / 0.3) + 1))
                            break
            elif "cordless_drill" in item["model_type"]:
                # Find wall_mount_bracket within 1.0m
                for loc in all_locations:
                    if loc["model_type"] == "wall_mount_bracket":
                        lx, ly = loc["pose"]["x"], loc["pose"]["y"]
                        if math.hypot(ix - lx, iy - ly) <= 1.0:
                            item["parent_location"] = loc["id"]
                            break

    # Add summary counts
    for zone_id, zdata in zones_output.items():
        zdata["item_count"] = len(zdata["objects"])

    scene_graph = {
        "warehouse": {
          "name": "TEMPO Distribution Warehouse",
          "world_file": os.path.basename(sdf_path),
          "total_models": total_parsed,
          "zones": zones_output,
        }
    }

    return scene_graph


def main():
    parser = argparse.ArgumentParser(description="Parse warehouse SDF to 3D Scene Graph JSON")
    parser.add_argument("--sdf", default="warehouse_ws/src/warehouse_sim/worlds/warehouse.sdf", help="Path to warehouse.sdf")
    parser.add_argument("--zones", default="warehouse_ws/src/llm_planner/knowledge_base/zone_config.json", help="Path to zone_config.json")
    parser.add_argument("--weights", default="warehouse_ws/src/llm_planner/knowledge_base/model_weights.json", help="Path to model_weights.json")
    parser.add_argument("--output", default="warehouse_ws/src/llm_planner/knowledge_base/warehouse_scene_graph.json", help="Path to output warehouse_scene_graph.json")

    args = parser.parse_args()

    print(f"Parsing SDF: {args.sdf}")
    print(f"Using zones: {args.zones}")
    print(f"Using weights: {args.weights}")

    sg = build_scene_graph(args.sdf, args.zones, args.weights)

    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    with open(args.output, "w", encoding="utf-8") as f:
        json.dump(sg, f, indent=2)

    print(f"\nSuccessfully generated: {args.output}")
    print(f"Total non-infrastructure models mapped: {sg['warehouse']['total_models']}")
    print("\n--- Zone Breakdown ---")
    for zone_id, zdata in sg["warehouse"]["zones"].items():
        # Count types
        types = {}
        for o in zdata["objects"]:
            types[o["model_type"]] = types.get(o["model_type"], 0) + 1
        summary_str = ", ".join(f"{cnt}x {t}" for t, cnt in sorted(types.items(), key=lambda x: -x[1])[:4])
        if len(types) > 4:
            summary_str += f", +{len(types)-4} more types"
        print(f"  [{zone_id:18s}] {zdata['item_count']:3d} items | Centroid: {zdata['centroid']} | {summary_str}")


if __name__ == "__main__":
    main()
