"""
orchestrator.py

End-to-end task decomposition orchestrator for TEMPO.
Coordinates RAG skill and few-shot retrieval, prompt assembly, deterministic
LLM invocation, JSON validation, NetworkX DAG construction, and a feedback-driven
retry loop for multi-robot mission planning.
"""

from dataclasses import dataclass, field
import json
import re
import sys
from typing import Any, Dict, List, Optional

import networkx as nx

from llm_planner.rag.few_shot_retrieval import CosineFewShotRetriever, load_examples
from llm_planner.rag.skill_retrieval import CosineSkillRetriever, load_skill_bank
from llm_planner.config import GEMINI_MODEL, GROQ_MODEL, LLM_PROVIDER, MAX_RETRIES
from llm_planner.dag_builder import build_dag, validate_plan
from llm_planner.llm_client import call_llm_chat, get_client
from llm_planner.prompt_builder import SYSTEM_PROMPT, prepare_prompt

DEFAULT_MODEL = GEMINI_MODEL if LLM_PROVIDER == "gemini" else GROQ_MODEL


@dataclass
class DecompositionResult:
    """Encapsulates the result of a mission decomposition attempt."""

    status: str
    plan: Optional[Dict[str, Any]] = None
    dag: Optional[nx.DiGraph] = None
    errors: List[str] = field(default_factory=list)
    attempts: int = 1
    raw_response: str = ""
    plan_id: str = ""

    @property
    def is_success(self) -> bool:
        """Returns True if the mission was successfully planned or validly rejected."""
        return self.status in ("ok", "clarification_needed", "infeasible") and not self.errors


@dataclass
class MissionPlanResult:
    """Encapsulates the unified end-to-end mission planning and taxonomy classification result."""

    status: str
    plan: Optional[Dict[str, Any]] = None
    dag: Optional[nx.DiGraph] = None
    itax_subclass: Optional[str] = None
    axes: Optional[Dict[str, str]] = None
    taxonomy_cot: Optional[Dict[str, str]] = None
    fleet: Optional[Dict[str, Any]] = None
    decomposition_attempts: int = 1
    taxonomy_attempts: int = 1
    errors: List[str] = field(default_factory=list)

    @property
    def is_success(self) -> bool:
        if self.status != "ok":
            return self.status in ("clarification_needed", "infeasible") and not self.errors
        return bool(self.itax_subclass) and not self.errors


def sanitize_json_response(raw_text: str) -> str:
    """
    Cleans raw LLM response text, stripping markdown code block fences if present.
    """
    cleaned = raw_text.strip()
    # Match ```json ... ``` or ``` ... ```
    match = re.search(r"^```(?:json)?\s*\n(.*?)\n```$", cleaned, re.DOTALL)
    if match:
        return match.group(1).strip()
    return cleaned


class MissionPlanner:
    """
    Stateful mission planner orchestrator.
    Caches the skill bank, few-shot examples, and RAG retrievers to avoid
    re-embedding across repeated planning queries.
    """

    def __init__(
        self,
        skill_bank: Optional[List[Dict[str, Any]]] = None,
        skill_retriever: Optional[CosineSkillRetriever] = None,
        few_shot_examples: Optional[List[Dict[str, Any]]] = None,
        few_shot_retriever: Optional[CosineFewShotRetriever] = None,
        client: Optional[Any] = None,
        model: Optional[str] = None,
        max_retries: int = MAX_RETRIES,
    ):
        self.skill_bank = skill_bank if skill_bank is not None else load_skill_bank()
        self.skill_retriever = (
            skill_retriever
            if skill_retriever is not None
            else CosineSkillRetriever(self.skill_bank)
        )
        self.few_shot_examples = (
            few_shot_examples if few_shot_examples is not None else load_examples()
        )
        self.few_shot_retriever = (
            few_shot_retriever
            if few_shot_retriever is not None
            else CosineFewShotRetriever(self.few_shot_examples)
        )
        self.client = client if client is not None else get_client()
        self.model = model or DEFAULT_MODEL
        self.max_retries = max_retries

    def plan_mission(
        self, instruction: str, environment: Optional[str] = None
    ) -> DecompositionResult:
        """
        Decomposes a natural language mission instruction into an atomic task DAG
        with automated validation and error-correcting retries.
        """
        # Step 1: RAG context retrieval and initial prompt assembly
        user_prompt, plan_id = prepare_prompt(
            instruction=instruction,
            few_shot_retriever=self.few_shot_retriever,
            skill_retriever=self.skill_retriever,
            environment=environment,
        )

        messages: List[Dict[str, str]] = [
            {"role": "system", "content": SYSTEM_PROMPT},
            {"role": "user", "content": user_prompt},
        ]

        last_plan: Optional[Dict[str, Any]] = None
        last_dag: Optional[nx.DiGraph] = None
        last_raw_response: str = ""
        last_errors: List[str] = []

        # Step 2: Inference & validation retry loop
        for attempt in range(1, self.max_retries + 1):
            print(f"[Orchestrator] Planning attempt {attempt}/{self.max_retries}...")
            raw_response = call_llm_chat(self.client, messages, model=self.model)
            last_raw_response = raw_response
            clean_json = sanitize_json_response(raw_response)

            current_errors: List[str] = []

            # 2a. JSON Syntax Validation
            try:
                plan = json.loads(clean_json)
                last_plan = plan
            except json.JSONDecodeError as exc:
                current_errors.append(
                    f"JSON_DECODE_ERROR: Model output was not valid JSON ({exc.msg} at line {exc.lineno}, col {exc.colno})."
                )
                last_errors = current_errors
                if attempt < self.max_retries:
                    self._append_retry_feedback(messages, raw_response, current_errors)
                continue

            if not isinstance(plan, dict):
                current_errors.append(
                    f"INVALID_ROOT_TYPE: Expected JSON object at root, got {type(plan).__name__}."
                )
                last_errors = current_errors
                if attempt < self.max_retries:
                    self._append_retry_feedback(messages, raw_response, current_errors)
                continue

            status = plan.get("status")

            # 2b. Status Classification Validation
            if status not in ("ok", "clarification_needed", "infeasible"):
                current_errors.append(
                    f"INVALID_STATUS: 'status' must be one of 'ok', 'clarification_needed', or 'infeasible', got '{status}'."
                )
                last_errors = current_errors
                if attempt < self.max_retries:
                    self._append_retry_feedback(messages, raw_response, current_errors)
                continue

            # 2c. Handling Rejection Statuses
            if status in ("clarification_needed", "infeasible"):
                if not plan.get("reason") or not str(plan["reason"]).strip():
                    current_errors.append(
                        f"MISSING_REASON: Status '{status}' requires a non-empty 'reason' field explaining the issue."
                    )
                    last_errors = current_errors
                    if attempt < self.max_retries:
                        self._append_retry_feedback(messages, raw_response, current_errors)
                    continue

                print(f"[Orchestrator] Mission categorized as '{status}': {plan.get('reason')}")
                return DecompositionResult(
                    status=status,
                    plan=plan,
                    dag=None,
                    errors=[],
                    attempts=attempt,
                    raw_response=raw_response,
                    plan_id=plan_id,
                )

            # 2d. Handling "ok" Task Decomposition Status
            subtasks = plan.get("subtasks")
            if not isinstance(subtasks, list) or len(subtasks) == 0:
                current_errors.append(
                    "MISSING_SUBTASKS: Status 'ok' requires a non-empty 'subtasks' array."
                )
                last_errors = current_errors
                if attempt < self.max_retries:
                    self._append_retry_feedback(messages, raw_response, current_errors)
                continue

            # 2e. NetworkX DAG Construction & Semantic Validation
            try:
                dag = build_dag(plan)
                last_dag = dag
            except Exception as dag_err:
                current_errors.append(f"DAG_BUILD_ERROR: Failed to construct DAG: {dag_err}")
                dag = None

            if dag is not None:
                validation_errors = validate_plan(plan, dag, self.skill_bank)
                current_errors.extend(validation_errors)

            if not current_errors:
                print(
                    f"[Orchestrator] Successfully decomposed into {len(subtasks)} subtasks (DAG nodes: {dag.number_of_nodes()}, edges: {dag.number_of_edges()})."
                )
                return DecompositionResult(
                    status="ok",
                    plan=plan,
                    dag=dag,
                    errors=[],
                    attempts=attempt,
                    raw_response=raw_response,
                    plan_id=plan.get("plan_id", plan_id),
                )

            # 2f. Record errors and prepare feedback prompt for retry
            last_errors = current_errors
            print(f"[Orchestrator] Validation failed on attempt {attempt}: {current_errors}")
            if attempt < self.max_retries:
                self._append_retry_feedback(messages, raw_response, current_errors)

        # Step 3: All retries exhausted
        print(f"[Orchestrator] Planning failed after {self.max_retries} attempts.")
        return DecompositionResult(
            status="failed_validation",
            plan=last_plan,
            dag=last_dag,
            errors=last_errors,
            attempts=self.max_retries,
            raw_response=last_raw_response,
            plan_id=plan_id,
        )

    def _append_retry_feedback(
        self, messages: List[Dict[str, str]], raw_response: str, errors: List[str]
    ) -> None:
        """Appends the assistant response and validation error feedback for the next turn."""
        messages.append({"role": "assistant", "content": raw_response})
        error_items = "\n".join(f"- {err}" for err in errors)
        feedback = (
            f"Your previous plan failed validation with the following errors:\n"
            f"{error_items}\n\n"
            f"Please carefully fix all listed errors and output ONLY the complete corrected JSON plan adhering strictly to the schema."
        )
        messages.append({"role": "user", "content": feedback})

    def plan_and_classify(
        self,
        instruction: str,
        environment: Optional[str] = None,
        fleet: Optional[Dict[str, Any]] = None,
        objects: Optional[Dict[str, Any]] = None,
    ) -> MissionPlanResult:
        """
        Executes unified end-to-end mission planning:
        1. Stage 1: Decomposes natural language instruction into a validated subtask DAG.
        2. Stage 2: Classifies the validated mission into an iTax subclass using full context knowledge.
        """
        decomp = self.plan_mission(instruction, environment=environment)
        if not decomp.is_success or decomp.status != "ok":
            return MissionPlanResult(
                status=decomp.status,
                plan=decomp.plan,
                dag=None,
                itax_subclass=None,
                axes=None,
                taxonomy_cot=None,
                decomposition_attempts=decomp.attempts,
                taxonomy_attempts=0,
                errors=decomp.errors,
            )

        from llm_planner.taxonomy_classifier import (
            classify_taxonomy,
            load_fleet_instances,
        )

        fleet_instances = load_fleet_instances()

        subtasks = decomp.plan.get("subtasks", [])
        tax_result = classify_taxonomy(
            subtasks,
            fleet=fleet,
            objects=objects,
            client=self.client,
            model=self.model,
            max_retries=self.max_retries,
        )

        combined_errors = list(decomp.errors) + list(tax_result.errors)
        status = "ok" if tax_result.is_success else "failed_taxonomy_classification"

        return MissionPlanResult(
            status=status,
            plan=decomp.plan,
            dag=decomp.dag,
            itax_subclass=tax_result.subclass,
            axes=tax_result.axes,
            taxonomy_cot=tax_result.chain_of_thought,
            fleet=fleet_instances,
            decomposition_attempts=decomp.attempts,
            taxonomy_attempts=tax_result.attempts,
            errors=combined_errors,
        )


# Global cached planner instance for one-line convenience imports
_default_planner: Optional[MissionPlanner] = None


def decompose_mission(
    instruction: str, environment: Optional[str] = None
) -> DecompositionResult:
    """
    Stage 1: Decomposes a mission instruction into an atomic DAG.
    """
    global _default_planner
    if _default_planner is None:
        _default_planner = MissionPlanner()
    return _default_planner.plan_mission(instruction, environment=environment)


def orchestrate_mission(
    instruction: str,
    environment: Optional[str] = None,
    fleet: Optional[Dict[str, Any]] = None,
    objects: Optional[Dict[str, Any]] = None,
) -> MissionPlanResult:
    """
    Unified Pipeline (Stage 1 + Stage 2):
    Decomposes the mission into an atomic subtask DAG and classifies it into an iTax subclass.
    """
    global _default_planner
    if _default_planner is None:
        _default_planner = MissionPlanner()
    return _default_planner.plan_and_classify(
        instruction, environment=environment, fleet=fleet, objects=objects
    )


if __name__ == "__main__":
    query_mission = (
        sys.argv[1]
        if len(sys.argv) > 1
        else "Retrieve the red box from shelf A and deliver it to origin."
    )

    print(f"\n==========================================")
    print(f"TEMPO End-to-End Mission Orchestrator")
    print(f"Mission: {query_mission}")
    print(f"==========================================\n")

    result = orchestrate_mission(query_mission)

    print("\n---------------- FINAL RESULT ----------------")
    print(f"Overall Status:     {result.status}")
    print(f"Success:            {result.is_success}")
    print(f"Decomp Attempts:    {result.decomposition_attempts}")
    print(f"Taxonomy Attempts:  {result.taxonomy_attempts}")
    print(f"iTax Subclass:      {result.itax_subclass}")

    if result.axes:
        print("\nTaxonomy Axes Breakdown:")
        for axis, val in result.axes.items():
            print(f"  {axis:15s}: {val}")

    if result.taxonomy_cot:
        print("\nTaxonomy Chain of Thought:")
        for step, reasoning in result.taxonomy_cot.items():
            print(f"  {step}: {reasoning}")

    if result.errors:
        print("\nErrors:")
        for err in result.errors:
            print(f"  - {err}")

    if result.plan:
        print("\nParsed Plan:")
        print(json.dumps(result.plan, indent=2))

    if result.dag:
        print("\nDAG Structure:")
        print(f"  Nodes: {list(result.dag.nodes)}")
        print(f"  Edges: {list(result.dag.edges)}")
        if nx.is_directed_acyclic_graph(result.dag):
            print(f"  Topological Sort Order: {list(nx.topological_sort(result.dag))}")
    print("==============================================\n")
    print("==========================================\n")
