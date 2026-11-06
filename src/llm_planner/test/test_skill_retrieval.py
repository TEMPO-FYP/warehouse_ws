"""
test_skill_retrieval.py

Basic smoke tests for skill_retriever.py.
Run from repo root: python -m tests.test_skill_retrieval
"""

import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from llm_planner.rag.skill_retrieval import (
    load_skill_bank,
    bank_summary,
    CosineSkillRetriever,
    build_skill_prompt_block,
)


def test_loading():
    print("\n--- Test 1: Loading ---")
    bank = load_skill_bank()
    assert len(bank) > 0, "No skills loaded."
    for skill in bank:
        assert "name"                  in skill, f"Missing name in {skill}"
        assert "description"           in skill, f"Missing description in {skill}"
        assert "inputs"                in skill, f"Missing inputs in {skill}"
        assert "preconditions"         in skill, f"Missing preconditions in {skill}"
        assert "effects"               in skill, f"Missing effects in {skill}"
        assert "required_capabilities" in skill, f"Missing required_capabilities in {skill}"
        assert "text"                  in skill, f"Missing text in {skill}"
    print(f"Loaded {len(bank)} skills. OK")


def test_bank_summary():
    print("\n--- Test 2: Bank Summary ---")
    bank = load_skill_bank()
    summary = bank_summary(bank)
    print(f"Summary: {summary}")
    assert isinstance(summary, str), "Summary should be a string."
    print("OK")


def test_retriever_init():
    print("\n--- Test 3: Retriever Initialization ---")
    bank = load_skill_bank()
    retriever = CosineSkillRetriever(bank)
    print(f"Embeddings shape: {retriever.embeddings.shape}")
    assert retriever.embeddings.shape[0] == len(bank), (
        "Number of embeddings should match number of skills."
    )
    print("OK")


def test_retrieve():
    print("\n--- Test 4: Retrieval ---")
    bank = load_skill_bank()
    retriever = CosineSkillRetriever(bank)

    query = "Go to zone A and pick up the package."
    retrieved = retriever.retrieve(query, k=3)

    print(f"Query: {query}")
    print(f"Retrieved {len(retrieved)} skills:")
    for skill, score in retrieved:
        print(f"  {skill['name']} score={score:.4f} | {skill['description']}")

    assert len(retrieved) > 0, "No skills retrieved."
    assert len(retrieved) <= 3, "Retrieved more than k skills."
    print("OK")


def test_retrieve_always_returns_one():
    print("\n--- Test 5: Always Returns At Least One ---")
    bank = load_skill_bank()
    retriever = CosineSkillRetriever(bank)

    # Very unrelated query, should still return at least one skill.
    query = "bake a chocolate cake in the kitchen."
    retrieved = retriever.retrieve(query, min_score=0.99)

    print(f"Query: {query}")
    print(f"Retrieved {len(retrieved)} skills:")
    for skill, score in retrieved:
        print(f"  {skill['name']} score={score:.4f}")

    assert len(retrieved) >= 1, "Should always return at least one skill."
    print("OK")


def test_required_capabilities_not_in_prompt():
    print("\n--- Test 6: Required Capabilities Not Shown to LLM ---")
    bank = load_skill_bank()
    retriever = CosineSkillRetriever(bank)

    query = "Navigate to zone B and inspect the area."
    retrieved = retriever.retrieve(query, k=3)
    prompt_block = build_skill_prompt_block(retrieved)

    assert "required_capabilities" not in prompt_block, (
        "required_capabilities should not appear in the prompt block."
    )
    print("required_capabilities correctly hidden from LLM prompt. OK")


def test_required_capabilities_in_bank():
    print("\n--- Test 7: Required Capabilities Passed Through in Bank ---")
    bank = load_skill_bank()
    for skill in bank:
        assert isinstance(skill["required_capabilities"], list), (
            f"required_capabilities should be a list for skill {skill['name']}"
        )
        print(f"  {skill['name']}: {skill['required_capabilities']}")
    print("All skills have required_capabilities passed through. OK")


def test_prompt_block():
    print("\n--- Test 8: Prompt Block ---")
    bank = load_skill_bank()
    retriever = CosineSkillRetriever(bank)

    query = "Fly the drone to the warehouse and inspect it."
    retrieved = retriever.retrieve(query, k=3)
    prompt_block = build_skill_prompt_block(retrieved)

    print(prompt_block)
    assert len(prompt_block) > 0, "Prompt block is empty."
    assert "name" in prompt_block, "Prompt block should contain skill name."
    assert "description" in prompt_block, "Prompt block should contain description."
    assert "inputs" in prompt_block, "Prompt block should contain inputs."
    assert "effects" in prompt_block, "Prompt block should contain effects."
    print("OK")


if __name__ == "__main__":
    test_loading()
    test_bank_summary()
    test_retriever_init()
    test_retrieve()
    test_retrieve_always_returns_one()
    test_required_capabilities_not_in_prompt()
    test_required_capabilities_in_bank()
    test_prompt_block()
    print("\n All tests passed.")