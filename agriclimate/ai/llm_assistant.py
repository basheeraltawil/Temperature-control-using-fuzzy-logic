"""LLM operator assistant (optional, advisory only).

Two jobs where a language model genuinely helps a grower / technician:

1. **Recipe authoring** - turn a natural-language request ("cherry tomato
   transplants, 3 weeks, then hardening; glass house in Magdeburg in March")
   into a structured, staged climate recipe.
2. **Shift reports** - explain what happened in a run (alarms, KPIs, energy)
   in plain language with concrete maintenance suggestions.

Design rules for industrial use:
* The LLM is **never in the control loop**. It proposes; deterministic code
  (``validate_recipe``) checks every number against a hard agronomic/safety
  envelope, and a human approves before the recipe is deployed.
* Structured outputs (Pydantic schema) - no free-text parsing.
* Works offline without the ``anthropic`` package: everything else in
  ``agriclimate`` runs without it.

Requires ``pip install anthropic`` and an ``ANTHROPIC_API_KEY`` (or
``ant auth login``).
"""
from __future__ import annotations

import json
from typing import Dict, List, Optional, Tuple

from pydantic import BaseModel

MODEL = "claude-opus-5"
FALLBACK_BETA = "server-side-fallback-2026-07-01"

# Hard envelope checked in code - independent of whatever the model says.
ENVELOPE = {"min_c": -2.0, "max_c": 40.0, "max_dif_k": 12.0, "max_step_k_per_h": 3.0,
            "rh_min": 30.0, "rh_max": 98.0, "max_stage_days": 365.0}


class RecipeStage(BaseModel):
    name: str
    duration_days: float
    day_temp_c: float
    night_temp_c: float
    day_start_h: float
    day_end_h: float
    rh_min_pct: float
    rh_max_pct: float
    notes: str


class ClimateRecipe(BaseModel):
    crop: str
    facility: str
    stages: List[RecipeStage]
    absolute_min_c: float
    absolute_max_c: float
    rationale: str
    assumptions: List[str]


class ShiftReport(BaseModel):
    summary: str
    incidents: List[str]
    root_causes: List[str]
    maintenance_actions: List[str]
    crop_risk: str


SYSTEM_RECIPE = """You are an agronomist and climate-control engineer helping growers configure \
climate recipes for greenhouses, growth chambers, mushroom rooms, livestock houses and cold stores.
Produce a staged temperature/humidity recipe for the request. Use well-established horticultural or \
animal-husbandry guidance, state your assumptions explicitly, and prefer conservative values when \
unsure. Hours are local clock hours (0-24). absolute_min_c/absolute_max_c are crop-damage limits \
used for alarms, not setpoints. A human reviews and a deterministic validator checks every value."""

SYSTEM_REPORT = """You are a climate-control service engineer writing a concise shift report for a \
grower and a maintenance technician. Base every statement on the supplied KPIs and alarm log; if the \
data does not support a conclusion, say so. Suggest concrete checks (sensor calibration, wiring, \
burner service ...) where alarms point to hardware."""


def _client():
    try:
        import anthropic
    except ImportError as exc:
        raise RuntimeError("The LLM assistant needs the 'anthropic' package: pip install anthropic") from exc
    return anthropic.Anthropic()


def _parse(system: str, prompt: str, schema):
    client = _client()
    response = client.beta.messages.parse(
        model=MODEL,
        max_tokens=16000,
        system=system,
        thinking={"type": "adaptive"},
        messages=[{"role": "user", "content": prompt}],
        output_format=schema,
        betas=[FALLBACK_BETA],
        fallbacks="default",          # re-run on a fallback model if a safety classifier declines
    )
    if response.stop_reason == "refusal":
        raise RuntimeError("The model declined this request; please rephrase it.")
    if response.stop_reason == "max_tokens" or response.parsed_output is None:
        raise RuntimeError(f"No structured output returned (stop_reason={response.stop_reason})")
    return response.parsed_output


# --------------------------------------------------------------------- recipes
def generate_recipe(request: str, facility: str = "greenhouse") -> ClimateRecipe:
    prompt = f"Facility type: {facility}\nGrower request:\n{request}"
    return _parse(SYSTEM_RECIPE, prompt, ClimateRecipe)


def validate_recipe(recipe: ClimateRecipe, envelope: Dict[str, float] = ENVELOPE) -> List[str]:
    """Deterministic guardrail. Returns a list of violations (empty = acceptable)."""
    issues = []
    if not recipe.stages:
        issues.append("recipe has no stages")
    for i, st in enumerate(recipe.stages):
        tag = f"stage {i + 1} '{st.name}'"
        for label, v in (("day", st.day_temp_c), ("night", st.night_temp_c)):
            if not envelope["min_c"] <= v <= envelope["max_c"]:
                issues.append(f"{tag}: {label} temperature {v} degC outside [{envelope['min_c']}, {envelope['max_c']}]")
            if not recipe.absolute_min_c <= v <= recipe.absolute_max_c:
                issues.append(f"{tag}: {label} setpoint {v} degC outside the recipe's own crop limits")
        if abs(st.day_temp_c - st.night_temp_c) > envelope["max_dif_k"]:
            issues.append(f"{tag}: day/night difference exceeds {envelope['max_dif_k']} K")
        if not (0 <= st.day_start_h < st.day_end_h <= 24):
            issues.append(f"{tag}: invalid day window {st.day_start_h}-{st.day_end_h} h")
        if not (0 < st.duration_days <= envelope["max_stage_days"]):
            issues.append(f"{tag}: invalid duration {st.duration_days} days")
        if not (envelope["rh_min"] <= st.rh_min_pct < st.rh_max_pct <= envelope["rh_max"]):
            issues.append(f"{tag}: invalid RH range {st.rh_min_pct}-{st.rh_max_pct} %")
    if recipe.absolute_min_c >= recipe.absolute_max_c:
        issues.append("absolute_min_c must be below absolute_max_c")
    return issues


def recipe_to_setpoint_table(recipe: ClimateRecipe, ramp_h: float = 1.5) -> List[Tuple[float, float]]:
    """Expand a validated recipe into (hour, setpoint) points for a TableSchedule."""
    points, t0 = [], 0.0
    for st in recipe.stages:
        for d in range(max(1, int(round(st.duration_days)))):
            base = t0 + 24.0 * d
            points += [(base + st.day_start_h, st.night_temp_c),
                       (base + min(st.day_start_h + ramp_h, st.day_end_h), st.day_temp_c),
                       (base + st.day_end_h, st.day_temp_c),
                       (base + min(st.day_end_h + ramp_h, 24.0), st.night_temp_c)]
        t0 += 24.0 * max(1, int(round(st.duration_days)))
    if points:
        points.insert(0, (0.0, recipe.stages[0].night_temp_c))
    return points


def recipe_to_scenario(recipe: ClimateRecipe, base_scenario: Dict, max_days: Optional[int] = None) -> Dict:
    """Build a scenario dict (YAML-serialisable) so the recipe can be simulated before deployment."""
    issues = validate_recipe(recipe)
    if issues:
        raise ValueError("recipe rejected by validator:\n  - " + "\n  - ".join(issues))
    pts = recipe_to_setpoint_table(recipe)
    total_h = pts[-1][0] if pts else 24.0
    if max_days:
        total_h = min(total_h, 24.0 * max_days)
    sc = dict(base_scenario)
    sc.update({
        "name": f"recipe_{recipe.crop.lower().replace(' ', '_')}",
        "description": f"LLM-drafted, validator-approved recipe: {recipe.crop}",
        "agronomy": recipe.rationale,
        "duration_h": total_h,
        "setpoint": {"type": "table", "points": [list(p) for p in pts]},
        "crop_limits": {"min": recipe.absolute_min_c, "max": recipe.absolute_max_c},
    })
    return sc


# ---------------------------------------------------------------- shift report
def shift_report(result, max_alarms: int = 60) -> ShiftReport:
    """Explain a simulation (or a logged real run) to the operator."""
    log = result.log
    alarm_lines = [json.dumps(a.as_dict()) for a in result.alarms[:max_alarms]]
    stats = {
        "duration_h": float(log["t_h"].iat[-1]),
        "temperature_true_min_max": [float(log["t_true"].min()), float(log["t_true"].max())],
        "setpoint_min_max": [float(log["setpoint"].min()), float(log["setpoint"].max())],
        "outdoor_min_max": [float(log["t_out"].min()), float(log["t_out"].max())],
        "hours_in_failsafe": float((log["mode"] == "FAILSAFE").mean() * log["t_h"].iat[-1]),
    }
    prompt = (f"Scenario: {result.scenario}\nController: {result.controller}\n"
              f"KPIs: {json.dumps(result.metrics)}\nStatistics: {json.dumps(stats)}\n"
              f"Alarm log ({len(result.alarms)} total, first {len(alarm_lines)} shown):\n"
              + "\n".join(alarm_lines))
    return _parse(SYSTEM_REPORT, prompt, ShiftReport)
