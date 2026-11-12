"""
scene_graph_grounder.py

Stage 0 Semantic Grounding Module for TEMPO.
Implements the SayPlan interactive 3D Scene Graph grounding paradigm:
- Represents warehouse as a multi-layer hierarchical 3DSG (Warehouse -> Zone -> Aisle / Fixture -> Object).
- Starts with collapsed zones to minimize token consumption.
- Runs an interactive expand/collapse search loop with full memory of previous actions and inspected nodes.
- Produces a grounded environment context, valid entity registry for DAG validation (Check 6),
  and ground-truth object weights for Stage 2 taxonomy classification.
"""

from dataclasses import dataclass, field
import json
import logging
import math
import os
import re
from typing import Any, Dict, List, Optional, Set, Tuple

from llm_planner.config import (
    GEMINI_MODEL,
    GROQ_MODEL,
    LLM_PROVIDER,
    get_knowledge_base_dir,
)
from llm_planner.llm_client import call_llm_chat, get_client

logger = logging.getLogger(__name__)
DEFAULT_MODEL = GEMINI_MODEL if LLM_PROVIDER == "gemini" else GROQ_MODEL

_CACHED_SCENE_GRAPH: Optional[Dict[str, Any]] = None


def load_scene_graph(path: Optional[str] = None) -> Dict[str, Any]:
    """Loads and caches the hierarchical warehouse_scene_graph.json."""
    global _CACHED_SCENE_GRAPH
    if _CACHED_SCENE_GRAPH is not None and path is None:
        return _CACHED_SCENE_GRAPH

    target_path = path or os.path.join(
        get_knowledge_base_dir(), "warehouse_scene_graph.json"
    )
    if not os.path.isfile(target_path):
        raise FileNotFoundError(f"Scene graph file not found at: {target_path}")

    with open(target_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    if path is None:
        _CACHED_SCENE_GRAPH = data
    return data


@dataclass
class GroundedSceneResult:
    """Encapsulates the output of Stage 0 3D Scene Graph semantic grounding."""

    environment_prompt: str
    valid_entities: Dict[str, Set[str]]  # {"objects": Set[str], "locations": Set[str]}
    objects_metadata: Dict[str, Dict[str, Any]]  # id -> {weight_kg, pose, zone_id, parent_location, model_type}
    expanded_nodes: List[str]
    turns_used: int
    action_history: List[Dict[str, Any]] = field(default_factory=list)


class SceneGraphNavigator:
    """
    Stateful view navigator over the 4-layer 3D Scene Graph.
    Tracks expanded zones, aisles, and fixtures, manages memory of inspected nodes,
    and dynamically renders compact textual views of the graph state.
    """

    def __init__(self, scene_graph: Dict[str, Any]):
        self.raw_data = scene_graph
        self.wh_data = scene_graph.get("warehouse", {})
        self.zones_dict: Dict[str, Dict[str, Any]] = self.wh_data.get("zones", {})
        self.inter_zone_distances: Dict[str, Dict[str, float]] = self.wh_data.get(
            "inter_zone_distances_m", {}
        )

        self.expanded_zones: Set[str] = set()
        self.expanded_aisles: Set[str] = set()
        self.expanded_fixtures: Set[str] = set()
        self.memory: List[str] = []
        self.action_history: List[Dict[str, Any]] = []

        # Index fixtures, aisles, and objects for fast lookup
        self.aisle_to_zone: Dict[str, str] = {}
        self.fixture_lookup: Dict[str, Dict[str, Any]] = {}
        self.object_lookup: Dict[str, Dict[str, Any]] = {}

        self._build_indexes()

    def _build_indexes(self) -> None:
        for zone_id, zdata in self.zones_dict.items():
            if "aisles" in zdata:
                for aisle_id, adata in zdata["aisles"].items():
                    self.aisle_to_zone[aisle_id] = zone_id
                    for fix_id, fentry in adata.get("fixtures", {}).items():
                        self.fixture_lookup[fix_id] = fentry
                        for child in fentry.get("children", []):
                            self.object_lookup[child["id"]] = child

            for fix_id, fentry in zdata.get("fixtures", {}).items():
                self.fixture_lookup[fix_id] = fentry
                for child in fentry.get("children", []):
                    self.object_lookup[child["id"]] = child

            for free_obj in zdata.get("free_objects", []):
                self.object_lookup[free_obj["id"]] = free_obj

    def expand(self, node_ids: List[str]) -> List[str]:
        """
        Expands specified node IDs (zones, aisles, or fixtures).
        Auto-expands ancestor containers if a child node is expanded directly.
        """
        expanded: List[str] = []
        for nid in node_ids:
            clean_id = nid.strip()
            if not clean_id:
                continue

            if clean_id in self.zones_dict:
                self.expanded_zones.add(clean_id)
                expanded.append(clean_id)
            elif clean_id in self.aisle_to_zone:
                parent_zone = self.aisle_to_zone[clean_id]
                self.expanded_zones.add(parent_zone)
                self.expanded_aisles.add(clean_id)
                expanded.append(clean_id)
            elif clean_id in self.fixture_lookup:
                fix_data = self.fixture_lookup[clean_id]
                p_zone = fix_data.get("zone_id")
                p_aisle = fix_data.get("aisle_id")
                if p_zone:
                    self.expanded_zones.add(p_zone)
                if p_aisle:
                    self.expanded_aisles.add(p_aisle)
                self.expanded_fixtures.add(clean_id)
                expanded.append(clean_id)

            if clean_id not in self.memory:
                self.memory.append(clean_id)

        return expanded

    def collapse(self, node_ids: List[str]) -> List[str]:
        """
        Collapses specified node IDs and any of their active child nodes.
        """
        collapsed: List[str] = []
        for nid in node_ids:
            clean_id = nid.strip()
            if clean_id in self.expanded_zones:
                self.expanded_zones.remove(clean_id)
                collapsed.append(clean_id)
                # Collapse all aisles and fixtures within this zone
                zdata = self.zones_dict.get(clean_id, {})
                if "aisles" in zdata:
                    for aid, adata in zdata["aisles"].items():
                        self.expanded_aisles.discard(aid)
                        for fid in adata.get("fixtures", {}):
                            self.expanded_fixtures.discard(fid)
                for fid in zdata.get("fixtures", {}):
                    self.expanded_fixtures.discard(fid)
            elif clean_id in self.expanded_aisles:
                self.expanded_aisles.remove(clean_id)
                collapsed.append(clean_id)
                # Collapse fixtures in this aisle
                p_zone = self.aisle_to_zone.get(clean_id)
                if p_zone:
                    adata = self.zones_dict[p_zone].get("aisles", {}).get(clean_id, {})
                    for fid in adata.get("fixtures", {}):
                        self.expanded_fixtures.discard(fid)
            elif clean_id in self.expanded_fixtures:
                self.expanded_fixtures.remove(clean_id)
                collapsed.append(clean_id)

        return collapsed

    def record_action(
        self, turn: int, action: str, nodes: List[str], reasoning: str
    ) -> None:
        self.action_history.append({
            "turn": turn,
            "action": action,
            "nodes": nodes,
            "reasoning": reasoning,
        })

    def render_memory_trace(self) -> str:
        """Renders the action history and inspected node memory for the LLM prompt."""
        if not self.action_history:
            return "No previous actions taken yet. This is Turn 1."

        lines = ["=== PREVIOUS SEARCH ACTIONS (MEMORY) ==="]
        for item in self.action_history:
            lines.append(
                f"- Turn {item['turn']}: action='{item['action']}', nodes={item['nodes']}, "
                f"reasoning=\"{item['reasoning']}\""
            )
        lines.append(f"Already Inspected Nodes: {self.memory}")
        return "\n".join(lines)

    def render_scene_view(self) -> str:
        """
        Renders the hierarchical 3D Scene Graph in its current expansion state.
        Unexpanded nodes are rendered as 1-line aggregate summaries.
        """
        lines = [
            f"=== 3D SCENE GRAPH: {self.wh_data.get('name', 'Warehouse')} ===",
            "Zones in Warehouse:",
        ]

        for zone_id, zdata in self.zones_dict.items():
            zname = zdata.get("name", zone_id)
            centroid = zdata.get("centroid", [0.0, 0.0])
            summary = zdata.get("summary", {})

            if zone_id not in self.expanded_zones:
                fix_str = (
                    ", ".join(
                        f"{c}x {t}" for t, c in summary.get("fixture_counts", {}).items()
                    )
                    or "none"
                )
                obj_str = (
                    ", ".join(
                        f"{c}x {t}" for t, c in summary.get("object_counts", {}).items()
                    )
                    or "none"
                )
                aisle_note = (
                    f" ({summary.get('aisle_count', 0)} aisles)"
                    if summary.get("aisle_count")
                    else ""
                )
                lines.append(
                    f"  [COLLAPSED] Zone '{zone_id}': \"{zname}\" at centroid {centroid}{aisle_note}. "
                    f"{zdata.get('description', '')} | Fixtures: {fix_str} | Objects: {obj_str}"
                )
                continue

            # Zone is EXPANDED
            lines.append(
                f"  [EXPANDED] Zone '{zone_id}': \"{zname}\" at centroid {centroid}. {zdata.get('description', '')}"
            )

            # 1. Aisles if present
            if "aisles" in zdata:
                lines.append(f"    Aisles in {zone_id}:")
                for aisle_id, adata in zdata["aisles"].items():
                    aname = adata.get("name", aisle_id)
                    acentroid = adata.get("centroid", [0.0, 0.0])
                    asummary = adata.get("summary", {})

                    if aisle_id not in self.expanded_aisles:
                        lines.append(
                            f"      [COLLAPSED] Aisle '{aisle_id}': \"{aname}\" at centroid {acentroid}. "
                            f"{adata.get('description', '')} (Tags: {adata.get('tags', [])}) | "
                            f"Contains {asummary.get('total_fixtures', 0)}x Picking_Shelves "
                            f"({asummary.get('total_objects', 0)}x small_box)"
                        )
                    else:
                        lines.append(
                            f"      [EXPANDED] Aisle '{aisle_id}': \"{aname}\" at centroid {acentroid}:"
                        )
                        for fix_id, fentry in adata.get("fixtures", {}).items():
                            fpose = [
                                fentry["pose"]["x"],
                                fentry["pose"]["y"],
                                fentry["pose"]["z"],
                            ]
                            if fix_id not in self.expanded_fixtures:
                                lines.append(
                                    f"        [COLLAPSED] Fixture '{fix_id}' (Picking_Shelves) at pose {fpose}. "
                                    f"Contains: {fentry.get('children_summary', {})}"
                                )
                            else:
                                lines.append(
                                    f"        [EXPANDED] Fixture '{fix_id}' at pose {fpose}:"
                                )
                                for child in fentry.get("children", []):
                                    lines.append(
                                        f"          - Object '{child['id']}' ({child['model_type']}), "
                                        f"tier {child.get('shelf_tier', 1)}, weight: {child.get('weight_kg')} kg"
                                    )

            # 2. Zone-level fixtures (e.g. Packing_Station, wall_mount_bracket, Dumpster_Large)
            if zdata.get("fixtures"):
                lines.append(f"    Fixtures in {zone_id}:")
                for fix_id, fentry in zdata["fixtures"].items():
                    fpose = [
                        fentry["pose"]["x"],
                        fentry["pose"]["y"],
                        fentry["pose"]["z"],
                    ]
                    if fix_id not in self.expanded_fixtures:
                        c_sum = fentry.get("children_summary", {})
                        c_note = f" (contains: {c_sum})" if c_sum else ""
                        lines.append(
                            f"      [COLLAPSED] Fixture '{fix_id}' ({fentry['model_type']}) at pose {fpose}{c_note}"
                        )
                    else:
                        lines.append(
                            f"      [EXPANDED] Fixture '{fix_id}' ({fentry['model_type']}) at pose {fpose}:"
                        )
                        for child in fentry.get("children", []):
                            detach_str = (
                                f", detach: {child['detach_topic']}"
                                if "detach_topic" in child
                                else ""
                            )
                            lines.append(
                                f"        - Object '{child['id']}' ({child['model_type']}), "
                                f"weight: {child.get('weight_kg')} kg{detach_str}"
                            )

            # 3. Zone-level free objects
            if zdata.get("free_objects"):
                lines.append(f"    Free Objects on floor in {zone_id}:")
                for fobj in zdata["free_objects"]:
                    opose = [
                        fobj["pose"]["x"],
                        fobj["pose"]["y"],
                        fobj["pose"]["z"],
                    ]
                    stack_str = (
                        f", stack_level: {fobj['stack_level']}"
                        if "stack_level" in fobj
                        else ""
                    )
                    lines.append(
                        f"      - Object '{fobj['id']}' ({fobj['model_type']}) at pose {opose}, "
                        f"weight: {fobj.get('weight_kg')} kg{stack_str}"
                    )

        return "\n".join(lines)

    def get_grounded_entities(
        self,
    ) -> Tuple[Set[str], Set[str], Dict[str, Dict[str, Any]]]:
        """
        Extracts valid entity sets (objects, locations) and objects metadata
        from the currently expanded graph view.
        """
        valid_objects: Set[str] = set()
        valid_locations: Set[str] = set()
        objects_metadata: Dict[str, Dict[str, Any]] = {}

        # Only active zones and expanded aisles are valid location referents
        active_zones = (
            self.expanded_zones if self.expanded_zones else set(self.zones_dict.keys())
        )
        for zid in active_zones:
            valid_locations.add(zid)
        for aid in self.expanded_aisles:
            valid_locations.add(aid)

        # Fixtures in expanded zones (zone-level fixtures) or expanded aisles (aisle-level fixtures)
        for fix_id, fentry in self.fixture_lookup.items():
            z_id = fentry.get("zone_id")
            a_id = fentry.get("aisle_id")
            if a_id:
                if a_id in self.expanded_aisles:
                    valid_locations.add(fix_id)
            elif z_id in self.expanded_zones:
                valid_locations.add(fix_id)

        # Objects exposed (free objects in expanded zones + children of expanded fixtures)
        for zid in self.expanded_zones:
            zdata = self.zones_dict.get(zid, {})
            for fobj in zdata.get("free_objects", []):
                oid = fobj["id"]
                valid_objects.add(oid)
                valid_locations.add(oid)  # May navigate directly to an object's pose
                objects_metadata[oid] = {
                    "weight_kg": fobj["weight_kg"],
                    "pose": fobj.get("pose"),
                    "zone_id": zid,
                    "parent_location": None,
                    "model_type": fobj.get("model_type"),
                    "category": fobj.get("category"),
                }

        for fix_id in self.expanded_fixtures:
            fentry = self.fixture_lookup.get(fix_id)
            if not fentry:
                continue
            for child in fentry.get("children", []):
                cid = child["id"]
                valid_objects.add(cid)
                valid_locations.add(cid)
                objects_metadata[cid] = {
                    "weight_kg": child["weight_kg"],
                    "pose": child.get("pose"),
                    "zone_id": child.get("zone_id"),
                    "parent_location": fix_id,
                    "shelf_tier": child.get("shelf_tier"),
                    "model_type": child.get("model_type"),
                    "category": child.get("category"),
                }

        return valid_objects, valid_locations, objects_metadata

    def format_grounded_environment(self) -> str:
        """
        Formats the grounded 3DSG subgraph into the text block for Stage 1 prompt.
        """
        valid_objs, valid_locs, objs_meta = self.get_grounded_entities()

        lines = [
            "GROUNDED 3D SCENE GRAPH ENVIRONMENT:",
            "The following zones, fixtures, and objects were grounded from the warehouse map:",
            "",
            "1. Grounded Zones & Centroids:",
        ]

        active_zones = (
            self.expanded_zones if self.expanded_zones else set(self.zones_dict.keys())
        )
        for zid in sorted(active_zones):
            zdata = self.zones_dict[zid]
            lines.append(
                f"   - Zone '{zid}' (Centroid: {zdata.get('centroid')}, Description: {zdata.get('description')})"
            )

        if self.expanded_aisles:
            lines.append("")
            lines.append("1b. Grounded Aisles:")
            for aid in sorted(self.expanded_aisles):
                p_zone = self.aisle_to_zone.get(aid, "picking_zone")
                adata = self.zones_dict.get(p_zone, {}).get("aisles", {}).get(aid, {})
                lines.append(
                    f"   - Aisle '{aid}' (Zone: '{p_zone}', Centroid: {adata.get('centroid')}, Description: {adata.get('description')})"
                )

        lines.append("")
        lines.append("2. Grounded Fixtures (Locations):")
        grounded_fixtures = [
            fid
            for fid in sorted(valid_locs)
            if fid in self.fixture_lookup
        ]
        if grounded_fixtures:
            for fid in grounded_fixtures:
                fentry = self.fixture_lookup[fid]
                aisle_str = (
                    f", aisle: '{fentry['aisle_id']}'"
                    if fentry.get("aisle_id")
                    else ""
                )
                lines.append(
                    f"   - Fixture '{fid}' ({fentry['model_type']}) at pose [{fentry['pose']['x']}, {fentry['pose']['y']}], zone: '{fentry['zone_id']}'{aisle_str}"
                )
        else:
            lines.append("   - (No individual fixtures expanded)")

        lines.append("")
        lines.append("3. Grounded Objects (Movable items with exact IDs and weights):")
        if objs_meta:
            for oid, meta in sorted(objs_meta.items()):
                if meta.get("parent_location"):
                    parent_fix = self.fixture_lookup.get(meta["parent_location"], {})
                    aisle_note = (
                        f" in aisle '{parent_fix['aisle_id']}'"
                        if parent_fix.get("aisle_id")
                        else ""
                    )
                    p_loc = f"on fixture '{meta['parent_location']}'{aisle_note} (zone '{meta['zone_id']}')"
                else:
                    p_loc = f"on floor in zone '{meta['zone_id']}'"
                tier_str = (
                    f", tier: {meta['shelf_tier']}" if meta.get("shelf_tier") else ""
                )
                lines.append(
                    f"   - Object '{oid}' ({meta['model_type']}): weight = {meta['weight_kg']} kg, {p_loc}{tier_str}, pose = [{meta['pose']['x']}, {meta['pose']['y']}]"
                )
        else:
            lines.append("   - (No individual objects expanded)")

        lines.append("")
        lines.append("4. Inter-Zone Distance Matrix (meters):")
        for z1 in sorted(active_zones):
            row_items = []
            for z2 in sorted(active_zones):
                dist = self.inter_zone_distances.get(z1, {}).get(z2, 0.0)
                row_items.append(f"{z2}: {dist:.1f}m")
            lines.append(f"   - {z1} -> {', '.join(row_items)}")

        return "\n".join(lines)


STAGE0_SYSTEM_PROMPT = """You are a 3D Scene Graph semantic grounding agent for a warehouse robot fleet.
Your job is to explore the warehouse 3D Scene Graph to locate the exact objects and locations required by a mission instruction.

The warehouse graph has 4 levels:
Warehouse -> Zone -> Aisle / Fixture -> Object

To explore, output JSON with one of three actions:
1. "expand": Unfolds one or more collapsed nodes to inspect their contents.
   {"action": "expand", "nodes": ["picking_zone", "aisle_1"], "reasoning": "Need to find boxes in picking zone aisle 1"}
2. "collapse": Folds previously expanded nodes that are no longer needed to save tokens.
   {"action": "collapse", "nodes": ["aisle_2"], "reasoning": "Not relevant to mission"}
3. "done": Call when all target objects and destination fixtures are fully expanded and visible.
   {"action": "done", "reasoning": "Located required target objects and destination fixtures"}

Rules:
- Output ONLY a JSON object with keys "action", "nodes" (list of node IDs), and "reasoning".
- For "done", "nodes" can be empty [].
- Never guess entity IDs; only expand nodes visible in the current scene graph.
- Termination Condition: Nodes in [COLLAPSED] state show only aggregate summaries, not concrete object IDs. You must call "done" ONLY when all target objects and destination fixtures are fully expanded and explicitly visible as leaf nodes in the scene view. If a target object is stored inside a collapsed container or fixture, you must expand that fixture first to expose its object IDs before terminating.
"""


def sanitize_grounder_json(raw_text: str) -> str:
    cleaned = raw_text.strip()
    match = re.search(r"^```(?:json)?\s*\n(.*?)\n```$", cleaned, re.DOTALL)
    if match:
        return match.group(1).strip()
    return cleaned


class SceneGraphGrounder:
    """
    Stage 0 Semantic Grounding Agent.
    Manages the multi-turn interactive dialogue with the LLM to ground mission referents
    against the hierarchical 3D Scene Graph before task decomposition.
    """

    def __init__(
        self,
        client: Optional[Any] = None,
        model: Optional[str] = None,
        scene_graph_path: Optional[str] = None,
        max_turns: int = 6,
    ):
        self.client = client if client is not None else get_client()
        self.model = model or DEFAULT_MODEL
        self.scene_graph = load_scene_graph(scene_graph_path)
        self.max_turns = max_turns

    def ground_mission(self, instruction: str) -> GroundedSceneResult:
        """
        Executes the interactive SayPlan semantic search loop for the given instruction.
        Returns a GroundedSceneResult with the grounded environment and valid entity sets.
        """
        navigator = SceneGraphNavigator(self.scene_graph)

        print(f"[Stage 0 Grounder] Starting semantic grounding for instruction: '{instruction}'")
        retry_feedback: Optional[str] = None

        turn = 0
        while turn < self.max_turns:
            turn += 1
            scene_view = navigator.render_scene_view()
            memory_trace = navigator.render_memory_trace()

            feedback_block = ""
            if retry_feedback:
                feedback_block = (
                    f"\n\nPREVIOUS ATTEMPT FAILED WITH ERROR:\n{retry_feedback}\n"
                    "Please correct your action and output strictly valid JSON adhering to the schema."
                )

            user_prompt = f"""Instruction to ground:
"{instruction}"

{memory_trace}

Current 3D Scene Graph State:
{scene_view}{feedback_block}

Choose your next action ("expand", "collapse", or "done"). Output ONLY raw JSON:"""

            messages = [
                {"role": "system", "content": STAGE0_SYSTEM_PROMPT},
                {"role": "user", "content": user_prompt},
            ]

            print(f"[Stage 0 Grounder] Turn {turn}/{self.max_turns} calling LLM...")
            raw_response = call_llm_chat(self.client, messages, model=self.model)
            clean_json = sanitize_grounder_json(raw_response)

            try:
                action_data = json.loads(clean_json)
            except json.JSONDecodeError as exc:
                err_msg = f"JSON_DECODE_ERROR: Model output was not valid JSON ({exc.msg} at line {exc.lineno}, col {exc.colno})."
                logger.warning(f"[Stage 0 Grounder] {err_msg}")
                retry_feedback = err_msg
                continue

            if not isinstance(action_data, dict):
                err_msg = f"INVALID_ROOT_TYPE: Expected JSON object at root with keys 'action', 'nodes', 'reasoning', got {type(action_data).__name__}."
                logger.warning(f"[Stage 0 Grounder] {err_msg}")
                retry_feedback = err_msg
                continue

            raw_action = action_data.get("action")
            if not isinstance(raw_action, str):
                err_msg = f"INVALID_ACTION: 'action' field must be a string ('expand', 'collapse', or 'done'), got {raw_action}."
                logger.warning(f"[Stage 0 Grounder] {err_msg}")
                retry_feedback = err_msg
                continue

            action = raw_action.lower().strip()
            reasoning = str(action_data.get("reasoning") or "")

            raw_nodes = action_data.get("nodes", [])
            if "node" in action_data and not raw_nodes:
                raw_nodes = [action_data["node"]]
            if isinstance(raw_nodes, str):
                raw_nodes = [raw_nodes]
            elif raw_nodes is None:
                raw_nodes = []
            elif not isinstance(raw_nodes, list):
                raw_nodes = []

            nodes = [str(n).strip() for n in raw_nodes if str(n).strip()]

            if action not in ("expand", "collapse", "done"):
                err_msg = f"UNRECOGNIZED_ACTION: 'action' must be 'expand', 'collapse', or 'done', got '{action}'."
                logger.warning(f"[Stage 0 Grounder] {err_msg}")
                retry_feedback = err_msg
                continue

            if action in ("expand", "collapse") and not nodes:
                err_msg = f"MISSING_NODES: Action '{action}' requires a non-empty list of node IDs in 'nodes'."
                logger.warning(f"[Stage 0 Grounder] {err_msg}")
                retry_feedback = err_msg
                continue

            retry_feedback = None
            print(
                f"[Stage 0 Grounder] Turn {turn}: action='{action}', nodes={nodes}, reasoning='{reasoning}'"
            )
            navigator.record_action(turn, action, nodes, reasoning)

            if action == "done":
                # Check for premature termination: did the LLM open aisles but expand zero fixtures across them?
                all_aisle_fixtures = [
                    fid
                    for exp_aisle in navigator.expanded_aisles
                    for fid in navigator.zones_dict.get(
                        navigator.aisle_to_zone.get(exp_aisle, ""), {}
                    )
                    .get("aisles", {})
                    .get(exp_aisle, {})
                    .get("fixtures", {})
                ]
                has_expanded_any_aisle_fixture = any(
                    fid in navigator.expanded_fixtures for fid in all_aisle_fixtures
                )

                if (
                    navigator.expanded_aisles
                    and not has_expanded_any_aisle_fixture
                    and turn < self.max_turns
                ):
                    warn_msg = (
                        f"[Stage 0 Grounder] ⚠️ PREMATURE TERMINATION DETECTED on Turn {turn}: "
                        f"LLM called 'done' after expanding aisle(s) {sorted(navigator.expanded_aisles)}, "
                        f"but did not expand any shelf fixture inside them. "
                        f"Rejecting 'done' and prompting LLM to expand a specific fixture."
                    )
                    print(warn_msg)
                    retry_feedback = (
                        f"TERMINATION_REJECTED: You called 'done', but you have not expanded any specific shelf "
                        f"fixture in the expanded aisle(s) ({sorted(navigator.expanded_aisles)}). Collapsed fixtures "
                        f"only show aggregate counts, not concrete object IDs. You must expand at least one specific "
                        f"fixture to expose concrete object IDs before calling 'done'."
                    )
                    continue

                print(f"[Stage 0 Grounder] Semantic grounding completed in {turn} turns.")
                break
            elif action == "expand":
                expanded = navigator.expand(nodes)
                print(f"[Stage 0 Grounder] Expanded: {expanded}")
            elif action == "collapse":
                collapsed = navigator.collapse(nodes)
                print(f"[Stage 0 Grounder] Collapsed: {collapsed}")

        valid_objs, valid_locs, objs_meta = navigator.get_grounded_entities()
        env_prompt = navigator.format_grounded_environment()

        print(
            f"[Stage 0 Grounder] Final Grounded View: {len(valid_objs)} objects, "
            f"{len(valid_locs)} locations. Expanded nodes: {navigator.memory}"
        )

        return GroundedSceneResult(
            environment_prompt=env_prompt,
            valid_entities={"objects": valid_objs, "locations": valid_locs},
            objects_metadata=objs_meta,
            expanded_nodes=list(navigator.memory),
            turns_used=turn,
            action_history=navigator.action_history,
        )


def extract_objects_dict(
    grounded: GroundedSceneResult, subtasks: List[Dict[str, Any]]
) -> Dict[str, Any]:
    """
    Extracts an objects dictionary with exact weight_kg values from the grounded 3DSG
    for all objects referenced in the validated subtask list.
    """
    objects: Dict[str, Any] = {}
    for task in subtasks:
        args = task.get("args", {})
        obj_name = args.get("object")
        if not obj_name:
            continue
        if obj_name not in grounded.objects_metadata:
            raise KeyError(
                f"Object '{obj_name}' referenced in subtask '{task.get('id')}' "
                f"is missing from grounded objects_metadata."
            )
        meta = grounded.objects_metadata[obj_name]
        objects[obj_name] = {"weight_kg": meta["weight_kg"]}
    return objects
