"""ROS 2 nodes for the agriclimate controller.

ROS 2 runs nodes with the system interpreter (/usr/bin/python3). If the
``agriclimate`` library was pip-installed into another interpreter (pyenv,
conda, venv) it is not importable here, so fall back to the repository
checkout: $AGRICLIMATE_HOME, or the first parent directory of this package
(source, build or install space) that contains ``agriclimate/__init__.py``.
"""
import importlib.util
import os
import sys
from pathlib import Path


def _ensure_agriclimate() -> None:
    if importlib.util.find_spec("agriclimate") is not None:
        return
    candidates = [Path(os.environ["AGRICLIMATE_HOME"])] if os.environ.get("AGRICLIMATE_HOME") else []
    candidates += list(Path(__file__).resolve().parents)
    for root in candidates:
        if (root / "agriclimate" / "__init__.py").is_file():
            sys.path.insert(0, str(root))
            return
    raise ModuleNotFoundError(
        "agriclimate library not found for " + sys.executable + ". Install it for this interpreter "
        "(/usr/bin/python3 -m pip install --user -e <repo>) or set AGRICLIMATE_HOME=<repo>.")


_ensure_agriclimate()


def _ensure_user_site() -> None:
    """conda and some shells set PYTHONNOUSERSITE, which hides ~/.local packages
    (e.g. scikit-learn installed with 'pip install --user') from the ROS nodes.
    Append the user site as a low-priority fallback when a dependency is missing."""
    import site

    if importlib.util.find_spec("sklearn") is not None:
        return
    user_site = site.getusersitepackages()
    if os.path.isdir(user_site) and user_site not in sys.path:
        sys.path.append(user_site)


_ensure_user_site()
