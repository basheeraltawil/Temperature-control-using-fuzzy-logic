"""Locate data directories (scenarios, original MATLAB FIS) for editable and regular installs."""
from __future__ import annotations

import os
import sys
from pathlib import Path
from typing import List

_REPO = Path(__file__).resolve().parents[1]


def candidates(sub: str) -> List[Path]:
    dirs = []
    if os.environ.get("AGRICLIMATE_HOME"):
        dirs.append(Path(os.environ["AGRICLIMATE_HOME"]) / sub)
    dirs += [Path.cwd() / sub, _REPO / sub, Path(sys.prefix) / "share" / "agriclimate" / sub]
    return dirs


def find_file(sub: str, name: str) -> Path:
    for d in candidates(sub):
        if (d / name).exists():
            return d / name
    raise FileNotFoundError(f"'{name}' not found in any of: {', '.join(str(d) for d in candidates(sub))} "
                            f"(set AGRICLIMATE_HOME to the repository checkout)")


def first_dir(sub: str) -> Path:
    for d in candidates(sub):
        if d.is_dir():
            return d
    return candidates(sub)[0]
