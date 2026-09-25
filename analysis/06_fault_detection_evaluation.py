"""06 - Fault detection and isolation: rules only, rules + AI monitor, three sensors.

Each fault is injected into sensor 1 of the tomato greenhouse at hour 20 of a 36 h run,
for three random seeds, with three monitoring set-ups:
  * 2 sensors, rules only (range, rate, disagreement -> mean of both)
  * 2 sensors, rules + AI anomaly monitor
  * 3 sensors, rules only (median = 2-out-of-3 voting)
We record
  * first alarm: delay until any alarm about the fault (incl. SENSOR_DISAGREE),
  * isolated: delay until sensor 1 itself is named (SENSOR1_*), i.e. the system knows
    which sensor is wrong,
  * false alarms: alarms naming a healthy sensor, or sensor alarms before the fault,
  * crop impact: RMSE of the true air temperature after the fault.

Outputs: output/06_fault_detection.md
"""
from concurrent.futures import ProcessPoolExecutor

import numpy as np

from _common import md_table, save_text
from agriclimate.sim import Scenario, run_scenario

FAULT_START = 20.0
FAULTS = {
    "drift +0.5 K/h": {"kind": "drift", "value": 0.5},
    "offset +2 K": {"kind": "offset", "value": 2.0},
    "frozen value": {"kind": "stuck"},
    "spikes 5 K": {"kind": "spike", "value": 5.0},
    "extra noise 0.5 K": {"kind": "noise", "value": 0.5},
}
SEEDS = [0, 1, 2]


SETUPS = {"2 sensors, rules": ("fuzzy_pi", 2), "2 sensors, rules + AI": ("fuzzy_pi+ai", 2),
          "3 sensors, rules (2oo3)": ("fuzzy_pi", 3)}


def job(args):
    label, seed, setup = args
    ctrl, n_sensors = SETUPS[setup]
    sc = Scenario.load("tomato_greenhouse_spring").copy(duration_h=36)
    sc.sensors_cfg = {**sc.sensors_cfg, "count": n_sensors}
    sc.faults = {"sensors": [{"sensor": 1, "start_h": FAULT_START, **FAULTS[label]}]}
    r = run_scenario(sc, ctrl, seed=seed)
    first, isolated, false = None, None, 0
    for a in r.alarms:
        t_h = a.t_s / 3600
        if not a.code.startswith("SENSOR"):
            continue
        healthy_named = a.code[6:7].isdigit() and a.code[6:7] != "1"
        if healthy_named or t_h < FAULT_START:
            false += 1
            continue
        first = t_h - FAULT_START if first is None else first
        if a.code.startswith("SENSOR1_") and isolated is None:
            isolated = t_h - FAULT_START
    after = r.log[r.log.t_h >= FAULT_START]
    rmse = float(np.sqrt(np.mean((after.t_true - after.setpoint) ** 2)))
    return label, setup, first, isolated, false, rmse


def delay(values, n):
    found = [v for v in values if v is not None]
    return f"{60 * np.mean(found):.0f} min ({len(found)}/{n})" if found else f"no (0/{n})"


if __name__ == "__main__":
    jobs = [(f, s, c) for f in FAULTS for s in SEEDS for c in SETUPS]
    with ProcessPoolExecutor() as ex:
        res = list(ex.map(job, jobs))
    rows = []
    for f in FAULTS:
        for setup in SETUPS:
            rr = [x for x in res if x[0] == f and x[1] == setup]
            rows.append((f, setup, delay([x[2] for x in rr], len(rr)), delay([x[3] for x in rr], len(rr)),
                         sum(x[4] for x in rr), f"{np.mean([x[5] for x in rr]):.2f}"))
    text = "# 06 Fault detection and isolation (sensor 1, fault at 20 h, 3 seeds)\n\n"
    text += md_table(rows, ["fault", "set-up", "first alarm", "faulty sensor isolated", "false alarms",
                            "RMSE after fault [K]"])
    save_text("06_fault_detection.md", text)
