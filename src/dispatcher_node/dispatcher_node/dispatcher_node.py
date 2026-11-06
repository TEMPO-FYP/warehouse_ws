#!/usr/bin/env python3
"""
dispatcher_node.py

ROS 2 Dispatcher Node for TEMPO.

Subscribes to /mission/assigned_plan (std_msgs/String, JSON), reconstructs
the dependency DAG from the subtask list, sequences subtasks into execution
waves using topological ordering, and publishes one wave at a time to
/mission/subtask (std_msgs/String, JSON).

A new wave is released only after every subtask in the current wave reports
a terminal feedback message (stage == "complete" or status == "failed") on
/mission/feedback. A single failure in any wave aborts the entire plan.

Topics
------
Subscribed:
  /mission/assigned_plan  (std_msgs/String)  Full assigned plan JSON.
  /mission/feedback       (std_msgs/String)  Per-subtask feedback JSON.

Published:
  /mission/subtask        (std_msgs/String)  One subtask at a time.
"""

import json
import threading
from typing import Any, Dict, List, Optional, Set

import networkx as nx
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String


# DAG helpers

def _build_dag(subtasks: List[Dict[str, Any]]) -> nx.DiGraph:
    """
    Reconstructs a NetworkX DiGraph from the subtask dependency lists.

    Node  = subtask id
    Edge A -> B = A must complete before B starts.

    The plan has already been validated upstream (orchestrator + dag_builder);
    hence no re-validation is performed.
    """
    G = nx.DiGraph()

    for task in subtasks:
        G.add_node(task["id"], **task)

    for task in subtasks:
        for dep_id in task["dependencies"]:
            G.add_edge(dep_id, task["id"])

    return G


def _build_waves(G: nx.DiGraph) -> List[List[str]]:
    """
    Derives ordered execution waves from the DAG using topological generation.

    Tasks in the same wave have all their dependencies satisfied by earlier
    waves and may execute in parallel. The wave list is returned as a list
    of sorted task-id lists so the dispatch order is deterministic.
    """
    return [
        sorted(generation)
        for generation in nx.topological_generations(G)
    ]


# Feedback parsing

def _parse_feedback(
    raw: str,
    task_map: Dict[str, Any],
) -> tuple[Optional[str], Optional[str], Optional[str], str]:
    """
    Parses a raw feedback message into (plan_id, task_id, status, stage).

    Accepts two formats:
      - JSON object with keys plan_id, task_id, status, stage.
      - Plain task-id string (manual testing convenience).

    stage defaults to "complete" so that plain-string feedback from manual
    ros2 topic pub commands is treated as a terminal signal.

    Returns (plan_id, task_id, status, stage). Any field that cannot be
    resolved is returned as None except stage, which always has a value.
    """
    plan_id: Optional[str] = None
    task_id: Optional[str] = None
    status:  Optional[str] = None
    stage:   str            = "complete"

    try:
        parsed = json.loads(raw)
    except json.JSONDecodeError:
        parsed = raw.strip()

    if isinstance(parsed, dict):
        raw_plan_id = parsed.get("plan_id")
        raw_task_id = (
            parsed.get("task_id")
            or parsed.get("subtask_id")
            or parsed.get("id")
        )
        raw_status = parsed.get("status")
        raw_stage  = parsed.get("stage")

        if raw_plan_id is not None:
            plan_id = str(raw_plan_id)
        if raw_task_id is not None:
            task_id = str(raw_task_id)
        if raw_status is not None:
            status = str(raw_status).strip().lower()
        if raw_stage is not None:
            stage = str(raw_stage).strip().lower()

    elif isinstance(parsed, str) and parsed in task_map:
        task_id = parsed

    return plan_id, task_id, status, stage


# Dispatcher node

class DispatcherNode(Node):
    """
    Sequences an assigned mission plan into per-wave subtask publications.

    Lifecycle
    ---------
    1. /mission/assigned_plan arrives  -> _plan_callback
    2. DAG reconstructed, waves built  -> _start_plan
    3. Wave published task by task     -> _publish_current_wave
    4. Feedback arrives per subtask    -> _feedback_callback -> _handle_feedback
    5. Wave complete -> next wave, or plan complete / aborted -> _clear_plan
    """

    def __init__(self) -> None:
        super().__init__("dispatcher_node")

        self._callback_group = ReentrantCallbackGroup()

        # Subscriptions
        self.create_subscription(
            String,
            "/mission/assigned_plan",
            self._plan_callback,
            10,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            String,
            "/mission/feedback",
            self._feedback_callback,
            10,
            callback_group=self._callback_group,
        )

        # Publisher
        self._subtask_pub = self.create_publisher(
            String,
            "/mission/subtask",
            10,
        )

        # Internal state
        self._lock = threading.Lock()

        self._active_plan_id:    Optional[str]              = None
        self._task_map:          Dict[str, Dict[str, Any]]  = {}
        self._waves:             List[List[str]]             = []
        self._current_wave_idx:  int                        = 0
        self._pending_feedback:  Set[str]                   = set()
        self._completed_tasks:   Set[str]                   = set()

        self.get_logger().info(
            "DispatcherNode ready. "
            "Subscribing to /mission/assigned_plan and /mission/feedback. "
            "Publishing to /mission/subtask."
        )

    # Plan ingestion

    def _plan_callback(self, msg: String) -> None:
        """
        Receives and parses an assigned plan from /mission/assigned_plan.

        Rejects the incoming plan if another plan is still active, since the
        dispatcher processes one mission at a time.
        """
        raw = msg.data.strip()
        if not raw:
            return

        try:
            envelope = json.loads(raw)
        except json.JSONDecodeError as exc:
            self.get_logger().error(
                f"[plan_callback] Failed to parse assigned plan JSON: {exc}"
            )
            return

        with self._lock:
            if self._active_plan_id is not None:
                self.get_logger().warn(
                    f"[plan_callback] Plan {self._active_plan_id} is still active. "
                    "Ignoring incoming plan."
                )
                return

        plan_id  = envelope.get("id", "unknown")
        output   = envelope.get("output", {})
        status   = output.get("status")
        subtasks = output.get("subtasks", [])

        if status != "ok":
            self.get_logger().warn(
                f"[plan_callback] Plan {plan_id} has status '{status}'; "
                "only 'ok' plans are dispatched."
            )
            return

        if not subtasks:
            self.get_logger().warn(
                f"[plan_callback] Plan {plan_id} has no subtasks; nothing to dispatch."
            )
            return

        self.get_logger().info(
            f"[plan_callback] Received plan {plan_id} "
            f"with {len(subtasks)} subtask(s)."
        )
        self._start_plan(plan_id, subtasks)

    def _start_plan(
        self,
        plan_id: str,
        subtasks: List[Dict[str, Any]],
    ) -> None:
        """
        Initialises dispatcher state for a new plan and triggers the first wave.

        Builds the DAG from the subtask dependency lists and derives execution
        waves via topological ordering. All state mutations are performed under
        the lock before _publish_current_wave is called.
        """
        dag   = _build_dag(subtasks)
        waves = _build_waves(dag)

        self.get_logger().info(
            f"[start_plan] Plan {plan_id}: "
            f"{len(subtasks)} subtask(s) across {len(waves)} wave(s)."
        )

        with self._lock:
            self._active_plan_id   = plan_id
            self._task_map         = {t["id"]: t for t in subtasks}
            self._waves            = waves
            self._current_wave_idx = 0
            self._pending_feedback = set()
            self._completed_tasks  = set()

        self._publish_current_wave()

    # Wave dispatch

    def _publish_current_wave(self) -> None:
        """
        Publishes every subtask in the current wave to /mission/subtask.

        Each message carries plan_id, task_id, robot, required_skills, and
        args so that robot controller nodes have everything they need.

        Advances automatically if the wave is empty (should not occur with a
        valid DAG, but handled defensively).
        """
        with self._lock:
            if self._active_plan_id is None:
                return

            if self._current_wave_idx >= len(self._waves):
                self.get_logger().info(
                    f"[publish_wave] Plan {self._active_plan_id} complete. "
                    "All waves dispatched and acknowledged."
                )
                self._clear_plan()
                return

            wave       = self._waves[self._current_wave_idx]
            wave_num   = self._current_wave_idx + 1
            total      = len(self._waves)
            plan_id    = self._active_plan_id

            if not wave:
                # Empty wave
                self._current_wave_idx += 1

        if not wave:
            self._publish_current_wave()
            return

        self.get_logger().info(
            f"[publish_wave] Dispatching wave {wave_num}/{total}: {wave}"
        )

        with self._lock:
            self._pending_feedback = set(wave)

        for task_id in wave:
            task = self._task_map[task_id]
            payload = json.dumps({
                "plan_id":         plan_id,
                "task_id":         task["id"],
                "robot":           task.get("robot", ""),
                "required_skills": task.get("required_skills", []),
                "args":            task.get("args", {}),
            })
            out = String()
            out.data = payload
            self._subtask_pub.publish(out)
            self.get_logger().info(
                f"[publish_wave]   -> subtask {task_id} "
                f"(robot={task.get('robot')}, "
                f"skills={task.get('required_skills')})"
            )

    # Feedback handling

    def _feedback_callback(self, msg: String) -> None:
        """
        Receives feedback from /mission/feedback and delegates to _handle_feedback.

        Exceptions are caught here so a malformed feedback message never
        crashes the node or wedges the dispatcher indefinitely.
        """
        try:
            self._handle_feedback(msg.data.strip())
        except Exception as exc:
            self.get_logger().error(
                f"[feedback_callback] Unhandled exception: {exc}"
            )

    def _handle_feedback(self, raw: str) -> None:
        """
        Processes one feedback message.

        Rules
        -----
        - Feedback for a different plan_id is ignored.
        - Intermediate stages (stage != "complete") are logged but do not
          advance the wave.
        - A "failed" status aborts the entire plan immediately.
        - Once all tasks in the current wave report "complete", the wave
          index is incremented and the next wave is published.
        """
        if not raw:
            return

        with self._lock:
            task_map   = self._task_map
            plan_id    = self._active_plan_id

        if plan_id is None:
            self.get_logger().warn(
                "[handle_feedback] Feedback received but no plan is active."
            )
            return

        fb_plan_id, task_id, status, stage = _parse_feedback(raw, task_map)

        # Reject feedback for a different plan
        if fb_plan_id is not None and fb_plan_id != plan_id:
            self.get_logger().warn(
                f"[handle_feedback] Ignoring feedback for plan {fb_plan_id}; "
                f"active plan is {plan_id}."
            )
            return

        # Resolve task_id from pending set if the message omitted it
        with self._lock:
            pending = self._pending_feedback

        if task_id is None:
            if len(pending) == 1:
                task_id = next(iter(pending))
                self.get_logger().warn(
                    f"[handle_feedback] Feedback had no task_id; "
                    f"inferred {task_id} from pending set."
                )
            else:
                self.get_logger().warn(
                    "[handle_feedback] Could not resolve task_id from feedback. "
                    f"Pending: {sorted(pending)}"
                )
                return

        # Failure aborts the plan
        if status in {"failed", "failure", "error", "rejected"}:
            self.get_logger().error(
                f"[handle_feedback] Task {task_id} reported '{status}'. "
                f"Aborting plan {plan_id}."
            )
            with self._lock:
                self._clear_plan()
            return

        # Intermediate stage, log and wait
        if stage != "complete":
            self.get_logger().info(
                f"[handle_feedback] Task {task_id}: stage={stage}, status={status} "
                "(intermediate — waiting for 'complete')."
            )
            return

        # Duplicate feedback guard
        with self._lock:
            if task_id in self._completed_tasks:
                self.get_logger().info(
                    f"[handle_feedback] Duplicate feedback for {task_id}; ignoring."
                )
                return

            if task_id not in self._pending_feedback:
                self.get_logger().warn(
                    f"[handle_feedback] Feedback for {task_id} not in current wave. "
                    f"Pending: {sorted(self._pending_feedback)}"
                )
                return

            self._pending_feedback.discard(task_id)
            self._completed_tasks.add(task_id)
            remaining = set(self._pending_feedback)

        self.get_logger().info(
            f"[handle_feedback] Task {task_id} complete. "
            f"Remaining in wave: {sorted(remaining)}"
        )

        # Advance to next wave when current wave is fully acknowledged
        if not remaining:
            with self._lock:
                self._current_wave_idx += 1
            self._publish_current_wave()

    # State reset

    def _clear_plan(self) -> None:
        """
        Resets all dispatcher state after a plan completes or is aborted.
        """
        self._active_plan_id   = None
        self._task_map         = {}
        self._waves            = []
        self._current_wave_idx = 0
        self._pending_feedback = set()
        self._completed_tasks  = set()
        self.get_logger().info("[clear_plan] Dispatcher state reset.")


# main

def main(args=None) -> None:
    rclpy.init(args=args)
    node = DispatcherNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()