"""
skill_retriever.py

Loads the skill bank from master_skills.json and retrieves the top-k most
relevant skills for a given instruction using sentence embeddings and cosine
similarity.

Two responsibilities:
  1. Loading: reads master_skills.json and builds a bank of skill entries,
     each enriched with a retrieval text field.
  2. Retrieval: embeds all skill texts at startup using a sentence transformer,
     then ranks skills by cosine similarity to a query instruction and returns
     the top-k most relevant ones.

Design decision:
  - Sentence embeddings are used instead of TF-IDF because the skill corpus
    is small (20-30 skills) and instructions rarely share exact words with
    skill descriptions. Embeddings catch paraphrases that TF-IDF would miss,
    for example "go to zone A" matching "navigate to a named location."
"""

import json
import os

import numpy as np
from sentence_transformers import SentenceTransformer
from sklearn.metrics.pairwise import cosine_similarity

from llm_planner.config import get_knowledge_base_dir

# File path
def get_master_skills_file() -> str:
    return os.path.join(get_knowledge_base_dir(), "master_skills.json")

# Sentence embedding model.
EMBEDDING_MODEL = "all-mpnet-base-v2"


# Loading

def _build_skill_text(skill: dict) -> str:
    """
    Builds a retrieval text string for one skill, combining its name,
    description, inputs, and effects into one readable block.

    This is what gets embedded for cosine similarity retrieval, so it
    should capture everything about the skill that an instruction might
    implicitly reference.

    Preconditions are excluded intentionally: they describe constraints
    on when a skill can run, not what it does, so they add noise to the
    retrieval signal rather than helping match instructions to skills.
    """
    parts = [
        skill["skill_id"].replace("_", " "),
        skill["description"],
    ]

    if skill.get("inputs"):
        parts.append("inputs " + " ".join(skill["inputs"]))

    if skill.get("effects"):
        parts.append("effects " + " ".join(skill["effects"]))

    return ". ".join(parts)


def load_skill_bank(filepath: str = None) -> list[dict]:
    """
    Loads all skills from master_skills.json and returns a list of skill
    bank entries, one per skill.
    """
    if filepath is None:
        filepath = get_master_skills_file()
    with open(filepath, "r", encoding="utf-8") as f:
        raw = json.load(f)

    bank = []
    for skill in raw["skills"]:
        bank.append({
            "name":          skill["skill_id"],
            "description":   skill["description"],
            "inputs":        skill.get("inputs", []),
            "preconditions": skill.get("preconditions", []),
            "effects":       skill.get("effects", []),
            "required_capabilities": skill.get("required_capabilities", []),
            "text":          _build_skill_text(skill),
        })

    if not bank:
        raise ValueError(
            "No skills found in master_skills.json, "
            "check the file is correctly formatted."
        )

    return bank


def bank_summary(bank: list[dict]) -> str:
    "One-line breakdown of the skill bank, for start-up logging."
    return f"{len(bank)} skills loaded"


# Retrieval

class CosineSkillRetriever:
    def __init__(self, bank: list[dict], model_name: str = EMBEDDING_MODEL):
        """
        bank: list of skill entries from load_skill_bank().

        At startup, embeds all skill texts using the sentence transformer.
        Done once and reused on every retrieve() call.
        """
        if not bank:
            raise ValueError("Skill bank is empty, nothing to retrieve from.")

        self.bank = bank
        self.model = SentenceTransformer(model_name)

        # Embed all skill texts once at startup.
        # normalize_embeddings=True makes cosine similarity a simple dot
        # product, which is faster and numerically stable.
        self.embeddings = self.model.encode(
            [entry["text"] for entry in bank],
            convert_to_numpy=True,
            normalize_embeddings=True,
        )

    def retrieve(self, instruction: str, k: int = None,
                 min_score: float = 0.0) -> list[tuple[dict, float]]:
        """
        Returns (skill_entry, score) tuples for skills scoring at or above
        min_score, ranked highest first, capped at k if k is given.

        Always returns at least one entry even if every score is below
        min_score.
        """
        query_emb = self.model.encode(
            [instruction],
            convert_to_numpy=True,
            normalize_embeddings=True,
        )
        scores = cosine_similarity(query_emb, self.embeddings)[0]
        ranked = sorted(range(len(self.bank)), key=lambda i: -scores[i])

        selected = [
            (self.bank[i], float(scores[i]))
            for i in ranked
            if scores[i] >= min_score
        ]

        # Always return at least one skill
        if not selected:
            top = ranked[0]
            selected = [(self.bank[top], float(scores[top]))]

        if k is not None:
            selected = selected[:k]

        return selected


# Prompt formatting

def build_skill_prompt_block(retrieved: list[tuple[dict, float]]) -> str:
    """
    Converts retrieved skills into a readable block for injection into
    the LLM prompt.

    Each skill's inputs, preconditions, and effects are shown so the LLM
    understands not just what the skill is called but what it needs and
    what it produces, which is what it needs to decompose a mission
    correctly and set dependencies between subtasks.
    """
    lines = ["Available Skills:"]
    for skill, _ in retrieved:
        inputs_str = ", ".join(skill["inputs"]) if skill["inputs"] else "none"
        pre_str    = "; ".join(skill["preconditions"]) if skill["preconditions"] else "none"
        eff_str    = "; ".join(skill["effects"]) if skill["effects"] else "none"
        lines.append(
            f"\n  name         : {skill['name']}"
            f"\n  description  : {skill['description']}"
            f"\n  inputs       : [{inputs_str}]"
            f"\n  preconditions: {pre_str}"
            f"\n  effects      : {eff_str}"
        )
    return "\n".join(lines)