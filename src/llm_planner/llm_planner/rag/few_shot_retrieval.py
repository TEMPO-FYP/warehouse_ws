"""
few_shot_retriever.py

Loads the few-shot example bank from few_shot_examples.json and retrieves
the top-k most relevant examples for a given instruction.

Two responsibilities:
  1. Loading: reads few_shot_examples.json and returns a clean list of
     example dicts.
  2. Retrieval: scores all examples against the query instruction using a
     cross-encoder at startup, then returns the top-k most relevant ones.

On top of pure top-k ranking, retrieve() applies two coverage guarantees:
  1. At least one REJECTION example (status != "ok") is always present in
     the retrieved set, so the LLM always sees how to handle instructions
     it should be refusing.
  2. At least one OK example (status == "ok") is always present in the
     retrieved set, so the LLM always sees what a valid plan looks like.

Retrieval method:
  A cross-encoder scores every (query, example) pair jointly rather than
  independently, catching semantic matches that keyword overlap would miss.
"""

import json
import os

from sentence_transformers import CrossEncoder

from llm_planner.config import get_knowledge_base_dir

# File path
def get_examples_file() -> str:
    return os.path.join(get_knowledge_base_dir(), "few_shot_examples.json")

# Cross-encoder model for reranking. Lightweight and runs on CPU.
RERANKER_MODEL = "cross-encoder/ms-marco-MiniLM-L-6-v2"


# Loading

def load_examples(filepath: str = None) -> list[dict]:
    """
    Load the few-shot bank from JSON. Returns a list of example dicts.
    """
    if filepath is None:
        filepath = get_examples_file()
    with open(filepath, "r", encoding="utf-8") as f:
        raw = json.load(f)

    examples = []
    for entry in raw["examples"]:
        examples.append({
            "id":          entry["id"],
            "instruction": entry["instruction"],
            "output":      entry["output"],
        })
    return examples


def bank_summary(examples: list[dict]) -> str:
    "One-line breakdown of the bank, for start-up logging."
    by_status = {}
    for ex in examples:
        status = ex["output"]["status"]
        by_status[status] = by_status.get(status, 0) + 1
    status_str = ", ".join(f"{n} {s}" for s, n in sorted(by_status.items()))
    return f"{len(examples)} examples ({status_str})"


# Retrieval

class CosineFewShotRetriever:
    def __init__(self, examples: list[dict], reranker_model: str = RERANKER_MODEL):
        """
        examples: list of dicts from load_examples().

        At startup, loads the cross-encoder model once. Reused on every
        retrieve() call so the model is not reloaded per instruction.
        """
        if not examples:
            raise ValueError("Few-shot bank is empty, nothing to retrieve from.")

        self.examples = examples

        # Cross-encoder loaded once at startup.
        self.reranker = CrossEncoder(reranker_model)


    def _score(self, query_instruction: str) -> list[tuple[dict, float]]:
        """
        Scores all examples against the query using the cross-encoder.

        The cross-encoder reads each (query, example) pair jointly rather
        than independently, so it catches semantic matches, for example "move the item" matching
        "transport the box to the dropoff point."

        Returns (example, score) pairs, highest score first.
        """
        pairs = [
            (query_instruction, ex["instruction"])
            for ex in self.examples
        ]
        scores = self.reranker.predict(pairs)
        ranked = sorted(
            zip(self.examples, scores),
            key=lambda pair: -pair[1],
        )
        return [(ex, float(score)) for ex, score in ranked]

    def retrieve(self, query_instruction: str, k: int = 4,
             guarantee_rejection: bool = True,
             guarantee_ok: bool = True) -> list[tuple[dict, float]]:
        """
        Returns up to k (example, score) tuples, highest score first.

        Scores all examples against the query using the cross-encoder and
        takes the top-k.

        Two coverage guarantees:
        1. guarantee_rejection, if the final top-k has no rejection example
            (status != "ok"), the weakest pick is swapped out for the best
            scoring rejection example from the full ranked list, so the LLM
            always sees at least one example of an instruction it should refuse.
        2. guarantee_ok, if the final top-k has no ok example (status == "ok"),
            the weakest pick is swapped out for the best scoring ok example
            from the full ranked list, so the LLM always sees what a valid
            plan looks like.
        """
        # Score all examples against the query.
        all_ranked = self._score(query_instruction)

        # Take top-k from ranked results.
        selected = all_ranked[:k]
        selected_ids = {ex["id"] for ex, _ in selected}
        guaranteed_ids = set()

        def swap_in_best(predicate) -> None:
            """
            Replace the weakest non-guaranteed pick with the best example
            from the full ranked list that satisfies predicate and is not
            already selected.
            """
            for ex, score in all_ranked:
                if ex["id"] in selected_ids:
                    continue
                if not predicate(ex):
                    continue
                # Found a matching example not yet selected.
                # Find weakest displaceable slot.
                displaceable = sorted(
                    (pair for pair in selected
                    if pair[0]["id"] not in guaranteed_ids),
                    key=lambda pair: pair[1],
                )
                if not displaceable:
                    return
                removed = displaceable[0]
                selected.remove(removed)
                selected_ids.discard(removed[0]["id"])
                selected.append((ex, score))
                selected_ids.add(ex["id"])
                guaranteed_ids.add(ex["id"])
                return

        def ensure_covered(predicate) -> None:
            """
            Make sure at least one selected example satisfies predicate.
            If one already does, protect it so nothing can displace it.
            If none does, swap in the best matching one from the full ranked list.
            """
            already_covered = next(
                (ex["id"] for ex, _ in selected if predicate(ex)), None
            )
            if already_covered is not None:
                guaranteed_ids.add(already_covered)
                return
            swap_in_best(predicate)

        if guarantee_rejection:
            ensure_covered(lambda ex: ex["output"]["status"] != "ok")

        if guarantee_ok:
            ensure_covered(lambda ex: ex["output"]["status"] == "ok")

        selected.sort(key=lambda pair: -pair[1])
        return selected


# Prompt formatting

def format_examples_for_prompt(retrieved: list[tuple[dict, float]],
                                indent: int = 2) -> str:
    """
    Renders retrieved examples into a prompt-ready text block.

    Synthesises plan_id and original_instruction on ok plans so every
    rendered example is a valid instance of the output schema the LLM
    is being asked to follow.
    """
    blocks = []
    for i, (ex, score) in enumerate(retrieved, 1):
        output = dict(ex["output"])
        if output["status"] == "ok":
            output = {
                "status":               "ok",
                "plan_id":              ex["id"],
                "original_instruction": ex["instruction"],
                "subtasks":             output.get("subtasks", []),
            }
        blocks.append(
            f"========== EXAMPLE {i} - {output['status']} ==========\n"
            f"Instruction: \"{ex['instruction']}\"\n"
            f"Output:\n{json.dumps(output, indent=indent)}\n"
        )
    return "\n".join(blocks)