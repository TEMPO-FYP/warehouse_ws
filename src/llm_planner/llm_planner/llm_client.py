"""
llm_client.py

Unified LLM API client wrapper supporting Google Gemini and Groq.
Provides deterministic structured JSON generation.
"""

from typing import Any, List, Optional
from llm_planner.config import (
    GEMINI_API_KEY,
    GEMINI_MODEL,
    GROQ_API_KEY,
    GROQ_MODEL,
    LLM_PROVIDER,
)

try:
    from google import genai
    from google.genai import types
except ImportError:
    genai = None
    types = None

try:
    from groq import Groq
except ImportError:
    Groq = None


# Client initialization

def get_client(provider: Optional[str] = None) -> Any:
    """
    Returns an initialized LLM client for the chosen provider (defaults to LLM_PROVIDER).
    """
    selected_provider = (provider or LLM_PROVIDER).lower()

    if selected_provider == "gemini":
        if genai is None:
            raise ImportError(
                "google-genai package is not installed. Run 'pip install google-genai'."
            )
        return genai.Client(api_key=GEMINI_API_KEY)
    elif selected_provider == "groq":
        if Groq is None:
            raise ImportError("groq package is not installed. Run 'pip install groq'.")
        return Groq(api_key=GROQ_API_KEY)
    else:
        raise ValueError(
            f"Unsupported LLM_PROVIDER '{selected_provider}'. Must be 'gemini' or 'groq'."
        )


# LLM call

def call_llm_chat(
    client: Any, messages: List[dict], model: Optional[str] = None
) -> str:
    """
    Sends a list of chat messages to the LLM and returns the response text.
    Deterministic (temperature=0) with JSON object output format.
    Automatically detects whether client is google.genai.Client or Groq.
    """
    # Check if Google GenAI client
    if genai is not None and isinstance(client, genai.Client):
        selected_model = model or GEMINI_MODEL
        # Extract system prompt if present in messages
        system_instruction = None
        user_model_messages = []
        for m in messages:
            if m.get("role") == "system":
                system_instruction = m.get("content", "")
            else:
                user_model_messages.append(m)

        contents = [
            types.Content(
                role=("model" if m["role"] == "assistant" else "user"),
                parts=[types.Part.from_text(text=m["content"])],
            )
            for m in user_model_messages
        ]

        config = types.GenerateContentConfig(
            system_instruction=system_instruction,
            response_mime_type="application/json",
            temperature=0.0,
        )

        response = client.models.generate_content(
            model=selected_model,
            contents=contents,
            config=config,
        )
        return response.text

    # Groq client fallback
    selected_model = model or GROQ_MODEL
    completion = client.chat.completions.create(
        model=selected_model,
        messages=messages,
        temperature=0,
        response_format={"type": "json_object"},
    )
    return completion.choices[0].message.content


def call_llm(client: Any, system_prompt: str, user_prompt: str) -> str:
    """
    Sends a system + user prompt to the configured LLM and returns
    the raw response text.
    """
    messages = [
        {"role": "system", "content": system_prompt},
        {"role": "user",   "content": user_prompt},
    ]
    return call_llm_chat(client, messages)
