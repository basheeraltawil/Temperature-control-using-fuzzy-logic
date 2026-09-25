"""05 - Robustness to model mismatch.

The controllers were tuned on nominal parameters. A real building differs: glazing U-value,
thermal mass and heater capacity are uncertain by tens of percent. We change one parameter
at a time by -30 % / +30 % in the tomato greenhouse (24 h) and compare RMSE and actuator
travel. A robust controller degrades gracefully.

Outputs: output/05_robustness.md
"""
from concurrent.futures import ProcessPoolExecutor

from _common import md_table, save_text
from agriclimate.sim import Scenario, run_scenario

PARAMS = ["u_value", "mass_capacity", "heater_max_w", "solar_transmittance"]
FACTORS = [0.7, 1.0, 1.3]
CONTROLLERS = ["fuzzy_pi", "pid"]


def job(args):
    param, factor, ctrl = args
    sc = Scenario.load("tomato_greenhouse_spring").copy(duration_h=24)
    setattr(sc.facility, param, getattr(sc.facility, param) * factor)
    m = run_scenario(sc, ctrl).metrics
    return param, factor, ctrl, m["rmse_K"], m["actuator_travel"]


if __name__ == "__main__":
    jobs = [(p, f, c) for p in PARAMS for f in FACTORS for c in CONTROLLERS]
    with ProcessPoolExecutor() as ex:
        res = list(ex.map(job, jobs))
    rows = []
    for p in PARAMS:
        for c in CONTROLLERS:
            r = {f: (rm, tr) for pp, f, cc, rm, tr in res if pp == p and cc == c}
            rows.append((p, c, *(f"{r[f][0]:.2f}" for f in FACTORS), f"{r[0.7][1]:.0f} / {r[1.3][1]:.0f}"))
    text = "# 05 Robustness to parameter mismatch (tomato greenhouse, 24 h)\n\n"
    text += md_table(rows, ["parameter", "controller", "RMSE -30 % [K]", "RMSE nominal [K]", "RMSE +30 % [K]",
                            "travel -30 % / +30 %"])
    save_text("05_robustness.md", text)
