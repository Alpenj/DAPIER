"""Re-launch assignment scripts in the verified course conda environment."""

from __future__ import annotations

import os
import sys
from pathlib import Path


def ensure_course_environment() -> None:
    """Replace a VS Code system-Python launch with lerobot-vision Python."""

    target = Path.home() / "miniconda3" / "envs" / "lerobot-vision" / "bin" / "python"
    if not target.is_file() or Path(sys.executable).resolve() == target.resolve():
        return

    clean_environment = os.environ.copy()
    clean_environment.pop("PYTHONPATH", None)
    print(f"Python 자동 전환: {sys.executable} -> {target}", flush=True)
    os.execve(
        str(target),
        [str(target), str(Path(sys.argv[0]).resolve()), *sys.argv[1:]],
        clean_environment,
    )
