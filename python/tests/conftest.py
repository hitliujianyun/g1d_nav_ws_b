import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
for p in (os.path.join(ROOT, "python"), os.path.join(ROOT, "third_party", "unitree_sdk2_python")):
    if p not in sys.path:
        sys.path.insert(0, p)
