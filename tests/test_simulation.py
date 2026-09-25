import pytest

from agriclimate.sim import Scenario, list_scenarios, run_scenario


@pytest.mark.parametrize("name", list_scenarios())
def test_every_scenario_loads_and_runs(name):
    sc = Scenario.load(name).copy(duration_h=4)
    r = run_scenario(sc, "fuzzy_pi")
    assert len(r.log) == int(4 * 3600 / sc.control_dt_s)
    assert r.log["t_true"].between(-30, 60).all()


def test_fuzzy_pi_beats_legacy_on_tomato_greenhouse():
    sc = Scenario.load("tomato_greenhouse_spring").copy(duration_h=24)
    new = run_scenario(sc, "fuzzy_pi").metrics
    old = run_scenario(sc, "legacy_fis").metrics
    assert new["rmse_K"] < 0.6 < old["rmse_K"]
    assert new["heat_cool_overlap_h"] == 0 and old["heat_cool_overlap_h"] > 0


def test_energy_balance_heater_only_warms():
    sc = Scenario.load("seed_germination_chamber").copy(duration_h=6)
    r = run_scenario(sc, "fuzzy_pi")
    assert r.metrics["heating_kwh"] > 0
    assert abs(r.log["t_true"].iloc[-1] - r.log["setpoint"].iloc[-1]) < 1.0


def test_periodic_internal_gain_follows_photoperiod():
    from agriclimate.sim.runner import disturbances_at

    sc = Scenario.load("vertical_farm_lettuce")
    assert disturbances_at(sc, 5.0)[1] == 0          # lights off before 06:00
    assert disturbances_at(sc, 12.0)[1] == 30000     # lights on
    assert disturbances_at(sc, 23.0)[1] == 0         # off after 22:00
    assert disturbances_at(sc, 36.0)[1] == 30000     # next day
