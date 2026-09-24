"""Read and write MATLAB ``.fis`` files (FIS version 2.0 text format)."""
from __future__ import annotations

import re
from pathlib import Path
from typing import Dict, List, Union

from .engine import MamdaniFIS, MembershipFunction, Rule, Variable

_MF_RE = re.compile(r"'(?P<name>[^']*)'\s*:\s*'(?P<type>[^']*)'\s*,\s*\[(?P<params>[^\]]*)\]")


def _value(raw: str):
    raw = raw.strip()
    if raw.startswith("'") and raw.endswith("'"):
        return raw[1:-1]
    if raw.startswith("["):
        return [float(v) for v in raw.strip("[]").split()]
    try:
        return float(raw) if "." in raw or "e" in raw.lower() else int(raw)
    except ValueError:
        return raw


def _parse_rule(line: str, n_in: int, n_out: int) -> Rule:
    # e.g. "1 3, 1 1 (1) : 1"
    m = re.match(r"^\s*([-\d\s]+),([-\d\s]+)\(([\d.eE+-]+)\)\s*:\s*(\d)", line)
    if not m:
        raise ValueError(f"Cannot parse rule line: {line!r}")
    ante = [int(v) for v in m.group(1).split()]
    cons = [int(v) for v in m.group(2).split()]
    if len(ante) != n_in or len(cons) != n_out:
        raise ValueError(f"Rule {line!r} does not match {n_in} inputs / {n_out} outputs")
    return Rule(ante, cons, float(m.group(3)), int(m.group(4)))


def load_fis(path: Union[str, Path], resolution: int = 501) -> MamdaniFIS:
    sections: Dict[str, Dict[str, object]] = {}
    rules_raw: List[str] = []
    current = None
    for raw in Path(path).read_text().splitlines():
        line = raw.strip()
        if not line or line.startswith("%"):
            continue
        if line.startswith("[") and line.endswith("]"):
            current = line[1:-1]
            sections.setdefault(current, {})
            continue
        if current == "Rules":
            rules_raw.append(line)
            continue
        key, _, val = line.partition("=")
        if key.startswith("MF"):
            m = _MF_RE.search(val)
            if not m:
                raise ValueError(f"Cannot parse membership function: {line!r}")
            sections[current].setdefault("mfs", []).append(
                MembershipFunction(m["name"], m["type"], [float(p) for p in m["params"].split()]))
        else:
            sections[current][key.strip()] = _value(val)

    system = sections["System"]
    if str(system.get("Type", "mamdani")).lower() != "mamdani":
        raise ValueError("Only Mamdani FIS files are supported")

    def variables(prefix: str, count: int) -> List[Variable]:
        out = []
        for i in range(1, count + 1):
            s = sections[f"{prefix}{i}"]
            out.append(Variable(s["Name"], s["Range"], list(s.get("mfs", []))))
        return out

    inputs = variables("Input", int(system["NumInputs"]))
    outputs = variables("Output", int(system["NumOutputs"]))
    rules = [_parse_rule(r, len(inputs), len(outputs)) for r in rules_raw]
    return MamdaniFIS(
        system["Name"], inputs, outputs, rules,
        and_method=system.get("AndMethod", "min"), or_method=system.get("OrMethod", "max"),
        imp_method=system.get("ImpMethod", "min"), agg_method=system.get("AggMethod", "max"),
        defuzz_method=system.get("DefuzzMethod", "centroid"), resolution=resolution,
    )


def _fmt(v: float) -> str:
    return f"{v:.10g}"


def save_fis(fis: MamdaniFIS, path: Union[str, Path]) -> None:
    lines = [
        "[System]", f"Name='{fis.name}'", "Type='mamdani'", "Version=2.0",
        f"NumInputs={len(fis.inputs)}", f"NumOutputs={len(fis.outputs)}", f"NumRules={len(fis.rules)}",
        f"AndMethod='{fis.and_method}'", f"OrMethod='{fis.or_method}'", f"ImpMethod='{fis.imp_method}'",
        f"AggMethod='{fis.agg_method}'", f"DefuzzMethod='{fis.defuzz_method}'", "",
    ]
    for prefix, variables in (("Input", fis.inputs), ("Output", fis.outputs)):
        for i, var in enumerate(variables, 1):
            lines += [f"[{prefix}{i}]", f"Name='{var.name}'",
                      f"Range=[{_fmt(var.range[0])} {_fmt(var.range[1])}]", f"NumMFs={len(var.mfs)}"]
            for j, mf in enumerate(var.mfs, 1):
                lines.append(f"MF{j}='{mf.name}':'{mf.type}',[{' '.join(_fmt(p) for p in mf.params)}]")
            lines.append("")
    lines.append("[Rules]")
    for r in fis.rules:
        lines.append(f"{' '.join(map(str, r.antecedent))}, {' '.join(map(str, r.consequent))} "
                     f"({_fmt(r.weight)}) : {r.connective}")
    Path(path).write_text("\n".join(lines) + "\n")
