"""
test_few_shot_retrieval.py

Basic smoke tests for few_shot_retriever.py.
Run from repo root: python -m tests.test_few_shot_retrieval
"""

import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from llm_planner.rag.few_shot_retrieval import (
    load_examples,
    bank_summary,
    CosineFewShotRetriever,
    format_examples_for_prompt,
)


def test_loading():
    print("\n--- Test 1: Loading ---")
    examples = load_examples()
    assert len(examples) > 0, "No examples loaded."
    for ex in examples:
        assert "id"          in ex, f"Missing id in {ex}"
        assert "instruction" in ex, f"Missing instruction in {ex}"
        assert "output"      in ex, f"Missing output in {ex}"
        assert "status"      in ex["output"], f"Missing status in output of {ex['id']}"
    print(f"Loaded {len(examples)} examples. OK")


def test_bank_summary():
    print("\n--- Test 2: Bank Summary ---")
    examples = load_examples()
    summary = bank_summary(examples)
    print(f"Summary: {summary}")
    assert isinstance(summary, str), "Summary should be a string."
    print("OK")


def test_retriever_init():
    print("\n--- Test 3: Retriever Initialization ---")
    examples = load_examples()
    retriever = CosineFewShotRetriever(examples)
    print("Cross-encoder reranker loaded. OK")


def test_retrieve():
    print("\n--- Test 4: Retrieval ---")
    examples = load_examples()
    retriever = CosineFewShotRetriever(examples)

    query = "Go to zone B and pick up the crate."
    retrieved = retriever.retrieve(query, k=4)

    print(f"Query: {query}")
    print(f"Retrieved {len(retrieved)} examples:")
    for ex, score in retrieved:
        print(f"  {ex['id']} ({ex['output']['status']}) score={score:.4f} | {ex['instruction']}")

    assert len(retrieved) > 0, "No examples retrieved."
    print("OK")


def test_guarantee_rejection():
    print("\n--- Test 5: Rejection Guarantee ---")
    examples = load_examples()
    retriever = CosineFewShotRetriever(examples)

    # Use a very specific query that would naturally retrieve only ok examples
    query = "Navigate to zone A and pick up the package."
    retrieved = retriever.retrieve(query, k=4, guarantee_rejection=True)

    statuses = [ex["output"]["status"] for ex, _ in retrieved]
    print(f"Retrieved statuses: {statuses}")
    assert any(s != "ok" for s in statuses), "No rejection example in retrieved set."
    print("Rejection guarantee working. OK")


def test_guarantee_ok():
    print("\n--- Test 6: OK Guarantee ---")
    examples = load_examples()
    retriever = CosineFewShotRetriever(examples)

    # Use a query that sounds like a rejection
    query = "Move it to the other place."
    retrieved = retriever.retrieve(query, k=4, guarantee_ok=True)

    statuses = [ex["output"]["status"] for ex, _ in retrieved]
    print(f"Retrieved statuses: {statuses}")
    assert any(s == "ok" for s in statuses), "No ok example in retrieved set."
    print("OK guarantee working. OK")


def test_format_for_prompt():
    print("\n--- Test 7: Format for Prompt ---")
    examples = load_examples()
    retriever = CosineFewShotRetriever(examples)

    query = "Fly the drone to the warehouse and inspect it."
    retrieved = retriever.retrieve(query, k=2)
    prompt_block = format_examples_for_prompt(retrieved)

    print(prompt_block)
    assert len(prompt_block) > 0, "Prompt block is empty."
    print("OK")


if __name__ == "__main__":
    test_loading()
    test_bank_summary()
    test_retriever_init()
    test_retrieve()
    test_guarantee_rejection()
    test_guarantee_ok()
    test_format_for_prompt()
    print("\n All tests passed.")