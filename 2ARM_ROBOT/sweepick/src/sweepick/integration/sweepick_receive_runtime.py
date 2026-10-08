from sweepick.integration.sweepick_resource_paths import FIELD_MODULES
"""Use current local field tools or the preserved work6 snapshot without renaming either."""
import importlib
import os
import sys
from pathlib import Path

PROJECT = Path.home() / "DAPIER"
SNAPSHOT = PROJECT / ".local-artifacts/sim-data-factory-20261002"
FACTORY_ROOT = Path(os.environ.get("FACTORY_ROOT", SNAPSHOT if SNAPSHOT.is_dir() else PROJECT / "tjj-runtime/v1"))


def configure_runtime():
    paths = (FACTORY_ROOT / "factory/v1", FACTORY_ROOT / "source/production_source_g000")
    for path in reversed(paths):
        if path.is_dir() and str(path) not in sys.path:
            sys.path.insert(0, str(path))


def field_module(name):
    configure_runtime()
    renamed = FIELD_MODULES.get(name, "sweepick_" + name)
    try:
        return importlib.import_module(renamed)
    except ModuleNotFoundError as exc:
        if exc.name != renamed:
            raise
        return importlib.import_module("tjj_" + name)
