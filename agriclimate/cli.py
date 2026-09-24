"""Command-line interface:  ``agriclimate <command> --help``"""
from __future__ import annotations

import argparse
import json
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]


def _fmt_table(df) -> str:
    cols = ["scenario", "controller", "rmse_K", "in_band_pct", "max_abs_err_K", "stress_h", "total_kwh",
            "actuator_travel", "heat_cool_overlap_h", "alarms"]
    head = "| " + " | ".join(cols) + " |\n|" + "---|" * len(cols) + "\n"
    rows = []
    for _, r in df[cols].iterrows():
        rows.append("| " + " | ".join(f"{v:.2f}" if isinstance(v, float) else str(v) for v in r.values) + " |")
    return head + "\n".join(rows) + "\n"


def _run_one(args):
    from .sim import Scenario, run_scenario

    name, ctrl, hours = args
    sc = Scenario.load(name)
    if hours:
        sc = sc.copy(duration_h=hours)
    return run_scenario(sc, ctrl)


def cmd_list(a):
    from .sim import Scenario, list_scenarios

    for n in list_scenarios():
        sc = Scenario.load(n)
        print(f"{n:38s} {sc.facility.name:20s} {sc.duration_h:5.0f} h  {sc.description}")


def cmd_run(a):
    from .sim import Scenario, metrics_table
    from .sim.plots import plot_comparison

    sc = Scenario.load(a.scenario)
    if a.hours:
        sc = sc.copy(duration_h=a.hours)
    ctrls = a.controllers or sc.controllers
    with ProcessPoolExecutor(max_workers=a.workers) as ex:
        results = list(ex.map(_run_one, [(a.scenario, c, a.hours) for c in ctrls]))
    print(_fmt_table(metrics_table(results)))
    if a.plot:
        print("plot:", plot_comparison(results, sc, a.plot))
    if a.csv:
        for r in results:
            p = Path(a.csv) / f"{sc.name}__{r.controller}.csv"
            p.parent.mkdir(parents=True, exist_ok=True)
            r.log.to_csv(p, index=False)
        print("logs:", a.csv)


def cmd_benchmark(a):
    import pandas as pd

    from .sim import Scenario, list_scenarios, metrics_table
    from .sim.plots import plot_comparison

    names = a.scenarios or list_scenarios()
    jobs = []
    for n in names:
        for c in a.controllers or Scenario.load(n).controllers:
            jobs.append((n, c, a.hours))
    with ProcessPoolExecutor(max_workers=a.workers) as ex:
        results = list(ex.map(_run_one, jobs))
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    df = metrics_table(results)
    df.to_csv(out / "benchmark.csv", index=False)
    (out / "benchmark.md").write_text(_fmt_table(df))
    print(_fmt_table(df))
    if a.plots:
        for n in names:
            sc = Scenario.load(n)
            if a.hours:
                sc = sc.copy(duration_h=a.hours)
            rs = [r for r in results if r.scenario == n]
            plot_comparison(rs, sc, out / f"{n}.png")
        print("plots written to", out)


def cmd_tune(a):
    from .ai.tuner import tune
    from .sim import Scenario

    scs = [Scenario.load(n).copy(duration_h=a.hours) if a.hours else Scenario.load(n) for n in a.scenarios]
    res = tune(scs, a.controller, maxiter=a.maxiter, popsize=a.popsize)
    print(json.dumps(res, indent=2))
    print("\nPaste into the scenario YAML:\ncontroller_params:\n  " + a.controller + ": "
          + json.dumps({k: float(f"{v:.4g}") for k, v in res["params"].items()}))


def cmd_identify(a):
    import pandas as pd

    from .ai.sysid import LearnedThermalModel, excitation_experiment
    from .sim import Scenario

    if a.csv:
        log = pd.read_csv(a.csv)
    else:
        log = excitation_experiment(Scenario.load(a.scenario), hours=a.hours)
    model = LearnedThermalModel(dt=a.dt).fit(log)
    print(json.dumps({"metrics": model.metrics, "coefficients_per_step": model.explain()}, indent=2))
    if a.out:
        model.save(a.out)
        print("saved", a.out)


def cmd_analyze_legacy(a):
    from .control.legacy_fis import DEFAULT_FIS
    from .fuzzy import fuzzy_pi_rulebase, load_fis
    from .sim.plots import plot_fis_surfaces

    legacy = load_fis(DEFAULT_FIS)
    gaps = legacy.coverage_gaps(n=101)
    print("Legacy FIS:", DEFAULT_FIS.name)
    print("  rules:", len(legacy.rules), "of", len(legacy.inputs[0].mfs) * len(legacy.inputs[1].mfs),
          "input combinations")
    for k, v in gaps.items():
        print(f"  no rule fires for {k:8s} on {100 * v:.1f} % of the input plane")
    for sensed, target in [(20, 20), (0, 0), (-5, 20), (35, 20), (10, 10)]:
        p, n = legacy.evaluate((sensed, target), default=(0.0, 0.0)).outputs
        print(f"  sensed={sensed:5.1f} target={target:5.1f} -> heater PWM {p:6.1f}  cooler PWM {n:6.1f}")
    if a.plot:
        print("surfaces:", plot_fis_surfaces(legacy, fuzzy_pi_rulebase(resolution=201), a.plot))


def cmd_export(a):
    from .control.fuzzy_pi import FuzzyPIController
    from .fuzzy import save_fis
    from .fuzzy.codegen import build_lut, to_c_header, to_iec_st

    out = Path(a.out)
    (out / "matlab").mkdir(parents=True, exist_ok=True)
    (out / "plc").mkdir(parents=True, exist_ok=True)
    (out / "firmware").mkdir(parents=True, exist_ok=True)
    ctrl = FuzzyPIController(use_lut=False)
    save_fis(ctrl.fis, out / "matlab" / "fuzzy_pi_climate.fis")
    xs, ys, tab = build_lut(lambda x, y: ctrl.fis(x, y)[0], (-1, 1), (-1, 1), a.size, a.size)
    (out / "firmware" / "fuzzy_pi_lut.h").write_text(to_c_header(xs, ys, tab, "fuzzy_pi_lut"))
    (out / "plc" / "FB_FuzzyPIClimate.st").write_text(to_iec_st(xs, ys, tab))
    print("exported:", *sorted(str(p.relative_to(out)) for p in out.rglob("*") if p.is_file()), sep="\n  ")


def cmd_advise(a):
    import yaml

    from .ai.llm_assistant import generate_recipe, recipe_to_scenario, validate_recipe
    from .sim import Scenario, run_scenario

    recipe = generate_recipe(a.request, a.facility)
    print(recipe.model_dump_json(indent=2))
    issues = validate_recipe(recipe)
    if issues:
        print("\nREJECTED by the deterministic validator:")
        for i in issues:
            print("  -", i)
        sys.exit(2)
    base = Scenario.load(a.base_scenario).raw
    sc_dict = recipe_to_scenario(recipe, base, max_days=a.max_days)
    Path(a.out).write_text(yaml.safe_dump(sc_dict, sort_keys=False))
    print("\nvalidated recipe scenario written to", a.out, "- review it before deployment")
    if a.simulate:
        r = run_scenario(Scenario.from_dict(sc_dict), "fuzzy_pi")
        print(json.dumps(r.metrics, indent=2))


def cmd_report(a):
    from .ai.llm_assistant import shift_report
    from .sim import Scenario, run_scenario

    sc = Scenario.load(a.scenario)
    if a.hours:
        sc = sc.copy(duration_h=a.hours)
    r = run_scenario(sc, a.controller)
    print(shift_report(r).model_dump_json(indent=2))


def main(argv=None):
    ap = argparse.ArgumentParser(prog="agriclimate", description="Fuzzy + AI agricultural climate control")
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("list", help="list scenarios").set_defaults(fn=cmd_list)

    p = sub.add_parser("run", help="simulate one scenario and compare controllers")
    p.add_argument("scenario")
    p.add_argument("-c", "--controllers", nargs="+", help="fuzzy_pi pid mpc legacy_fis, suffix +ai for anomaly detection")
    p.add_argument("--hours", type=float)
    p.add_argument("--plot")
    p.add_argument("--csv", help="directory for per-controller CSV logs")
    p.add_argument("--workers", type=int, default=None)
    p.set_defaults(fn=cmd_run)

    p = sub.add_parser("benchmark", help="run all (or selected) scenarios")
    p.add_argument("--scenarios", nargs="+")
    p.add_argument("-c", "--controllers", nargs="+")
    p.add_argument("--hours", type=float)
    p.add_argument("--out", default="results")
    p.add_argument("--plots", action="store_true")
    p.add_argument("--workers", type=int, default=None)
    p.set_defaults(fn=cmd_benchmark)

    p = sub.add_parser("tune", help="auto-tune controller gains on the digital twin")
    p.add_argument("scenarios", nargs="+")
    p.add_argument("--controller", default="fuzzy_pi", choices=["fuzzy_pi", "pid"])
    p.add_argument("--hours", type=float, default=36)
    p.add_argument("--maxiter", type=int, default=10)
    p.add_argument("--popsize", type=int, default=8)
    p.set_defaults(fn=cmd_tune)

    p = sub.add_parser("identify", help="learn a grey-box thermal model (from the twin or a CSV log)")
    p.add_argument("--scenario", default="tomato_greenhouse_spring")
    p.add_argument("--csv", help="real data with columns t_s,t_meas,heater,cooler,vent,t_out,rh_out,solar")
    p.add_argument("--hours", type=float, default=48)
    p.add_argument("--dt", type=float, default=60)
    p.add_argument("--out")
    p.set_defaults(fn=cmd_identify)

    p = sub.add_parser("analyze-legacy", help="analyse the original MATLAB FIS")
    p.add_argument("--plot")
    p.set_defaults(fn=cmd_analyze_legacy)

    p = sub.add_parser("export", help="export FIS / PLC Structured Text / C header")
    p.add_argument("--out", default="generated")
    p.add_argument("--size", type=int, default=21)
    p.set_defaults(fn=cmd_export)

    p = sub.add_parser("advise", help="LLM: draft a crop climate recipe from plain language (needs anthropic)")
    p.add_argument("request")
    p.add_argument("--facility", default="glass greenhouse")
    p.add_argument("--base-scenario", default="tomato_greenhouse_spring")
    p.add_argument("--out", default="scenarios/recipe_draft.yaml")
    p.add_argument("--max-days", type=int, default=7)
    p.add_argument("--simulate", action="store_true")
    p.set_defaults(fn=cmd_advise)

    p = sub.add_parser("report", help="LLM: operator shift report for a simulated run (needs anthropic)")
    p.add_argument("scenario")
    p.add_argument("--controller", default="fuzzy_pi+ai")
    p.add_argument("--hours", type=float)
    p.set_defaults(fn=cmd_report)

    a = ap.parse_args(argv)
    a.fn(a)


if __name__ == "__main__":
    main()
