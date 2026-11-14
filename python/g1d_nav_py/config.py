"""Loads config/g1d_nav.yaml (shared with the C++ processes)."""
from __future__ import annotations

import os
from typing import Any, Dict, Optional

import yaml

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
DEFAULT_CONFIG = os.path.join(REPO_ROOT, "config", "g1d_nav.yaml")


def load_config(path: Optional[str] = None) -> Dict[str, Any]:
    path = path or os.environ.get("G1D_CONFIG") or DEFAULT_CONFIG
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def net_if(cfg: Dict[str, Any], override: Optional[str] = None) -> Optional[str]:
    """DDS network interface: --net-if override, else system.net_if; "auto" / "" -> None
    (CycloneDDS picks the interface, e.g. on a development PC)."""
    name = override if override is not None else get(cfg, "system.net_if", None)
    return None if name in (None, "", "auto") else str(name)


def get(cfg: Dict[str, Any], dotted: str, default: Any = None) -> Any:
    """cfg["a"]["b"] for "a.b", or default when any level is missing."""
    node: Any = cfg
    for key in dotted.split("."):
        if not isinstance(node, dict) or key not in node:
            return default
        node = node[key]
    return node
