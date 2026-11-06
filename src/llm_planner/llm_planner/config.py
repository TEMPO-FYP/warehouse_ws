"""
config.py

Central configuration for the mission planner. All constants, API keys,
path resolution, and environment loading live here.
"""

import os
from pathlib import Path


def _load_env_file(env_path: str) -> None:
    """
    Loads key=value pairs from a .env file into os.environ.
    """
    if not os.path.exists(env_path):
        return

    with open(env_path, "r", encoding="utf-8") as f:
        for raw_line in f:
            line = raw_line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, value = line.split("=", 1)
            key = key.strip()
            value = value.strip().strip('"').strip("'")
            if key and key not in os.environ:
                os.environ[key] = value


# Load .env: uses ament_index inside ROS 2, falling back to package root for standalone tests
_PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
try:
    from ament_index_python.packages import get_package_share_directory
    _env_file = os.path.join(get_package_share_directory("llm_planner"), ".env")
except Exception:
    _env_file = os.path.join(_PACKAGE_ROOT, ".env")

_load_env_file(_env_file)


def get_knowledge_base_dir() -> str:
    """
    Resolves the path to the knowledge_base directory.
    Uses ament_index when running inside ROS 2, falling back to package root for tests.
    """
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory("llm_planner"), "knowledge_base")
    except Exception:
        return os.path.join(_PACKAGE_ROOT, "knowledge_base")


# LLM configuration
LLM_PROVIDER = os.environ.get("LLM_PROVIDER", "gemini")

GEMINI_API_KEY = os.environ.get("GEMINI_API_KEY")
GEMINI_MODEL = os.environ.get("GEMINI_MODEL", "gemini-3.1-flash-lite")

GROQ_API_KEY = os.environ.get("GROQ_API_KEY")
GROQ_MODEL = os.environ.get("GROQ_MODEL", "openai/gpt-oss-120b")

# Retry configuration
MAX_RETRIES = 3

# Few-shot retrieval configuration
FEW_SHOT_K = 4
