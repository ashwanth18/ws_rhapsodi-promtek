import os
import sys

_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
for pkg in ("isaac_twin", "scoop_vision"):
    path = os.path.join(_SRC, pkg)
    if path not in sys.path:
        sys.path.insert(0, path)
