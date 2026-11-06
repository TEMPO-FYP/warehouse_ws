"""
taxonomy_validator.py

Validates the output of the taxonomy classification LLM against formal structural,
semantic, and mathematical consistency rules.
"""

from typing import Any, Dict, List

# The 7 canonical target subclasses supported in TEMPO
TARGET_SUBCLASSES = {
    "ND[ST-SR-IA]",
    "ND[ST-SR-TA]",
    "ID[ST-SR-TA]",
    "XD[ST-SR-IA]",
    "XD[ST-SR-TA]",
    "XD[ST-MR-IA]",
    "XD[ST-MR-TA]",
}

VALID_ROBOT_MODELS = {"ST", "MT"}
VALID_TASK_MODELS = {"SR", "MR"}
VALID_TIME_HORIZONS = {"IA", "TA"}
VALID_DEPENDENCIES = {"ND", "ID", "XD", "CD"}

REQUIRED_COT_STEPS = [
    "step_1_robot_model",
    "step_2_task_model",
    "step_3_time_horizon",
    "step_4_dependency",
]


def validate_taxonomy_classification(output: Dict[str, Any]) -> List[str]:
    """
    Validates the taxonomy classification output. Returns a list of error strings.
    An empty list indicates the classification is strictly valid.
    """
    errors: List[str] = []

    if not isinstance(output, dict):
        return [f"INVALID_ROOT_TYPE: Expected JSON object at root, got {type(output).__name__}."]

    # Check 1: Top-level keys
    if "chain_of_thought" not in output:
        errors.append("MISSING_COT: Output is missing the 'chain_of_thought' object.")
    if "output_classification" not in output:
        errors.append(
            "MISSING_OUTPUT_CLASSIFICATION: Output is missing the 'output_classification' object."
        )

    if errors:
        return errors

    cot = output["chain_of_thought"]
    out_class = output["output_classification"]

    # Check 2: Separation of concerns (solver prediction prohibited)
    if "matched_solver" in out_class or "matched_solver" in output:
        errors.append(
            "SEPARATION_OF_CONCERNS_VIOLATION: 'matched_solver' must NOT be predicted by the LLM. "
            "Downstream solvers are determined by the taxonomy routing layer."
        )

    # Check 3: Chain of Thought completeness
    if not isinstance(cot, dict):
        errors.append(
            f"INVALID_COT_TYPE: 'chain_of_thought' must be an object, got {type(cot).__name__}."
        )
    else:
        for step in REQUIRED_COT_STEPS:
            if step not in cot:
                errors.append(f"MISSING_COT_STEP: 'chain_of_thought' is missing step '{step}'.")
            elif not isinstance(cot[step], str) or not cot[step].strip():
                errors.append(f"EMPTY_COT_STEP: 'chain_of_thought.{step}' must contain non-empty reasoning text.")

    # Check 4: Output classification structure & keys
    if not isinstance(out_class, dict):
        errors.append(
            f"INVALID_OUTPUT_TYPE: 'output_classification' must be an object, got {type(out_class).__name__}."
        )
        return errors

    if "axes" not in out_class:
        errors.append("MISSING_AXES: 'output_classification' must contain an 'axes' object.")
        return errors

    if "subclass" not in out_class:
        errors.append("MISSING_SUBCLASS: 'output_classification' must contain a 'subclass' string.")
        return errors

    axes = out_class["axes"]
    subclass = out_class["subclass"]

    if not isinstance(axes, dict):
        errors.append(f"INVALID_AXES_TYPE: 'axes' must be an object, got {type(axes).__name__}.")
        return errors

    # Check 5: Axis keys & domain values
    rm = axes.get("robot_model")
    tm = axes.get("task_model")
    th = axes.get("time_horizon")
    dep = axes.get("dependency")

    if rm not in VALID_ROBOT_MODELS:
        errors.append(
            f"INVALID_AXIS_VALUE: 'robot_model' must be one of {sorted(VALID_ROBOT_MODELS)}, got '{rm}'."
        )
    if tm not in VALID_TASK_MODELS:
        errors.append(
            f"INVALID_AXIS_VALUE: 'task_model' must be one of {sorted(VALID_TASK_MODELS)}, got '{tm}'."
        )
    if th not in VALID_TIME_HORIZONS:
        errors.append(
            f"INVALID_AXIS_VALUE: 'time_horizon' must be one of {sorted(VALID_TIME_HORIZONS)}, got '{th}'."
        )
    if dep not in VALID_DEPENDENCIES:
        errors.append(
            f"INVALID_AXIS_VALUE: 'dependency' must be one of {sorted(VALID_DEPENDENCIES)}, got '{dep}'."
        )

    # Check 6: Mathematical consistency between axes and subclass
    if rm in VALID_ROBOT_MODELS and tm in VALID_TASK_MODELS and th in VALID_TIME_HORIZONS and dep in VALID_DEPENDENCIES:
        expected_subclass = f"{dep}[{rm}-{tm}-{th}]"
        if subclass != expected_subclass:
            errors.append(
                f"MATHEMATICAL_MISMATCH: 'subclass' '{subclass}' does not match the canonical combination "
                f"of the 4 axes '{expected_subclass}' (D[R-T-H])."
            )

    # Check 7: Supported subclass whitelist
    if subclass not in TARGET_SUBCLASSES:
        errors.append(
            f"UNSUPPORTED_SUBCLASS: '{subclass}' is not one of the 7 supported TEMPO target subclasses: "
            f"{sorted(TARGET_SUBCLASSES)}."
        )

    return errors
