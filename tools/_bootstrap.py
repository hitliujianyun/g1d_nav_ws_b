"""Makes g1d_nav_py and unitree_sdk2py importable when they are not pip-installed."""
import os
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
for _p in (os.path.join(ROOT, "python"), os.path.join(ROOT, "third_party", "unitree_sdk2_python")):
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)
