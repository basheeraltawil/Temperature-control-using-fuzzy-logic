"""Small helpers shared by the analysis scripts."""
import sys
from pathlib import Path

import matplotlib

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:          # run from a checkout without installing the package
    sys.path.insert(0, str(ROOT))

matplotlib.use("Agg")

OUT = Path(__file__).resolve().parent / "output"
OUT.mkdir(exist_ok=True)


def md_table(rows, headers):
    """Render a list of row tuples as a Markdown table."""
    lines = ["| " + " | ".join(headers) + " |", "|" + "---|" * len(headers)]
    for r in rows:
        lines.append("| " + " | ".join(f"{v:.3g}" if isinstance(v, float) else str(v) for v in r) + " |")
    return "\n".join(lines) + "\n"


def save_text(name, text):
    """Write a text/Markdown result file and print it."""
    (OUT / name).write_text(text)
    print(text)
