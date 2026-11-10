"""Experiment configuration loading."""

from __future__ import annotations

import tomllib
from pathlib import Path
from typing import Any


def load_config(path: str | Path) -> dict[str, Any]:
    """Load a TOML experiment configuration."""
    with Path(path).open("rb") as handle:
        return tomllib.load(handle)
