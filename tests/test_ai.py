import pytest

pytest.importorskip("sklearn")

from agriclimate.ai.sysid import LearnedThermalModel, excitation_experiment  # noqa: E402
from agriclimate.sim import Scenario, run_scenario  # noqa: E402


@pytest.fixture(scope="module")
def model():
    log = excitation_experiment(Scenario.load("tomato_greenhouse_spring"), hours=36, seed=1)
    return LearnedThermalModel().fit(log)


def test_identified_model_is_accurate_and_physical(model):
    assert model.metrics["one_step_r2"] > 0.6
    assert model.metrics["rollout_1h_rmse_K"] < 1.5
    from agriclimate.ai.sysid import ModelState

    # physical effect, not individual (collinear) coefficients: heater warms, pad cooling cools
    st = ModelState(T=28.0, T_prev=28.0, hf=0.0, cf=0.0)
    base = model.step(st, 0, 0, 0, t_out=30.0, rh_out=35.0, solar=300.0).T
    assert model.step(st, 1, 0, 0, 30.0, 35.0, 300.0).T > base
    assert model.step(st, 0, 1, 0, 30.0, 35.0, 300.0).T < base


def test_model_save_load(tmp_path, model):
    model.save(tmp_path / "m.json")
    m2 = LearnedThermalModel.load(tmp_path / "m.json")
    assert m2.explain() == pytest.approx(model.explain())


def test_ai_monitor_catches_frozen_sensor():
    sc = Scenario.load("greenhouse_sensor_actuator_faults")
    base = run_scenario(sc, "fuzzy_pi").metrics
    ai = run_scenario(sc, "fuzzy_pi+ai")
    assert ai.metrics["stress_h"] <= base["stress_h"]
    assert ai.metrics["max_abs_err_K"] < base["max_abs_err_K"]
    assert any(a.code == "SENSOR2_ANOMALY" and 41.5 < a.t_s / 3600 < 43.0 for a in ai.alarms)


def test_ai_monitor_has_no_false_alarms_on_healthy_run():
    r = run_scenario(Scenario.load("tomato_greenhouse_spring").copy(duration_h=36), "fuzzy_pi+ai")
    assert not [a for a in r.alarms if "ANOMALY" in a.code or "HEATER" in a.code]


def test_llm_validator_rejects_unsafe_recipe():
    from agriclimate.ai.llm_assistant import ClimateRecipe, RecipeStage, recipe_to_setpoint_table, validate_recipe

    ok = ClimateRecipe(crop="lettuce", facility="chamber", absolute_min_c=4, absolute_max_c=30,
                       rationale="", assumptions=[], stages=[RecipeStage(
                           name="grow", duration_days=2, day_temp_c=20, night_temp_c=16, day_start_h=6,
                           day_end_h=22, rh_min_pct=60, rh_max_pct=85, notes="")])
    assert validate_recipe(ok) == []
    assert recipe_to_setpoint_table(ok)[0] == (0.0, 16)
    bad = ok.model_copy(update={"stages": [ok.stages[0].model_copy(update={"day_temp_c": 55})]})
    assert validate_recipe(bad)
