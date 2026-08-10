"""Read-only, privacy-conscious environment inventory for G0."""

from __future__ import annotations

from importlib.metadata import PackageNotFoundError, version
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
from typing import Any

SELECTED_DISTRIBUTIONS = (
    "mujoco",
    "so101-nexus",
    "lerobot",
    "gymnasium",
    "numpy",
    "pytest",
    "glfw",
    "dm-control",
    "torch",
)


def _run(arguments: list[str], *, timeout: float = 15.0) -> tuple[int | None, str]:
    try:
        completed = subprocess.run(
            arguments,
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout,
        )
    except (OSError, subprocess.TimeoutExpired):
        return None, ""
    return completed.returncode, completed.stdout.strip()


def _sanitized_path(value: str | None) -> str | None:
    if not value:
        return None
    home = str(Path.home())
    if value == home:
        return "$HOME"
    if value.startswith(f"{home}/"):
        return f"$HOME/{value[len(home) + 1 :]}"
    return value


def _os_release() -> dict[str, str]:
    result: dict[str, str] = {}
    try:
        lines = Path("/etc/os-release").read_text(encoding="utf-8").splitlines()
    except OSError:
        return result
    for line in lines:
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        result[key.lower()] = value.strip().strip('"')
    return result


def _distribution_versions() -> dict[str, str | None]:
    result: dict[str, str | None] = {}
    for name in SELECTED_DISTRIBUTIONS:
        try:
            result[name] = version(name)
        except PackageNotFoundError:
            result[name] = None
    return result


def _system_python() -> dict[str, str | None]:
    executable = (
        "/usr/bin/python3"
        if Path("/usr/bin/python3").is_file()
        else shutil.which("python3")
    )
    if not executable:
        return {"executable": None, "version": None}
    _, output = _run([executable, "--version"])
    return {"executable": executable, "version": output.removeprefix("Python ") or None}


def _gpu_inventory() -> dict[str, Any]:
    controllers: list[str] = []
    lspci = shutil.which("lspci")
    if lspci:
        _, output = _run([lspci, "-mm"])
        controllers = [
            line
            for line in output.splitlines()
            if '"VGA compatible controller"' in line
            or '"3D controller"' in line
            or '"Display controller"' in line
        ]

    nvidia: list[dict[str, str]] = []
    nvidia_smi = shutil.which("nvidia-smi")
    if nvidia_smi:
        code, output = _run(
            [
                nvidia_smi,
                "--query-gpu=name,driver_version,memory.total,compute_cap",
                "--format=csv,noheader,nounits",
            ]
        )
        if code == 0:
            for line in output.splitlines():
                fields = [field.strip() for field in line.split(",")]
                if len(fields) == 4:
                    nvidia.append(
                        {
                            "name": fields[0],
                            "driver_version": fields[1],
                            "memory_mib": fields[2],
                            "compute_capability": fields[3],
                        }
                    )

    return {
        "controllers": controllers,
        "nvidia": nvidia,
        "tools": {
            "nvidia_smi": bool(nvidia_smi),
            "glxinfo": bool(shutil.which("glxinfo")),
            "vulkaninfo": bool(shutil.which("vulkaninfo")),
        },
        "display": os.environ.get("DISPLAY"),
        "wayland_display": os.environ.get("WAYLAND_DISPLAY"),
        "mujoco_gl": os.environ.get("MUJOCO_GL"),
        "render_checked": False,
    }


def _ros_inventory() -> dict[str, Any]:
    ros2 = shutil.which("ros2")
    package_count: int | None = None
    if ros2:
        code, output = _run([ros2, "pkg", "list"], timeout=30.0)
        if code == 0:
            package_count = len([line for line in output.splitlines() if line.strip()])
    return {
        "distro": os.environ.get("ROS_DISTRO"),
        "ros2_executable": _sanitized_path(ros2),
        "package_count": package_count,
        "hardware_topics_or_serial_probed": False,
    }


def _uv_inventory() -> dict[str, str | None]:
    executable = shutil.which("uv")
    if not executable:
        return {"executable": None, "version": None}
    _, output = _run([executable, "--version"])
    return {
        "executable": _sanitized_path(executable),
        "version": output.removeprefix("uv ") or None,
    }


def _git_inventory(repo_root: Path) -> dict[str, Any]:
    code, revision = _run(["git", "-C", str(repo_root), "rev-parse", "HEAD"])
    status_code, status = _run(["git", "-C", str(repo_root), "status", "--porcelain"])
    return {
        "revision": revision if code == 0 else None,
        "worktree_has_changes": bool(status) if status_code == 0 else None,
    }


def collect_environment(repo_root: Path) -> dict[str, Any]:
    """Collect only the host facts needed to compare the documented matrix."""

    os_release = _os_release()
    system_python = _system_python()
    ros = _ros_inventory()
    packages = _distribution_versions()

    candidate_match = (
        os_release.get("id") == "ubuntu"
        and os_release.get("version_id", "").startswith("24.04")
        and ros["distro"] == "jazzy"
        and (system_python["version"] or "").startswith("3.12")
    )

    return {
        "os": {
            "id": os_release.get("id"),
            "version_id": os_release.get("version_id"),
            "pretty_name": os_release.get("pretty_name"),
            "kernel_release": platform.release(),
            "architecture": platform.machine(),
        },
        "python": {
            "system": system_python,
            "runtime": {
                "executable": _sanitized_path(sys.executable),
                "version": platform.python_version(),
            },
            "runtime_distributions": packages,
        },
        "uv": _uv_inventory(),
        "ros2": ros,
        "gpu": _gpu_inventory(),
        "repository": _git_inventory(repo_root),
        "matrix_comparison": {
            "matched_row": "Ubuntu 24.04 + ROS2 Jazzy + Python 3.12 candidate"
            if candidate_match
            else None,
            "base_axes_match": candidate_match,
            "full_nexus_lerobot_mujoco_ros2_compatibility_verified": False,
            "reason": (
                "OS/ROS/Python axes match the candidate row; the complete dependency combination remains unverified."
                if candidate_match
                else "The detected OS/ROS/Python axes do not exactly match the candidate row."
            ),
        },
        "scope": {
            "simulation_model_load_only": True,
            "render_checked": False,
            "recording_checked": False,
            "training_checked": False,
            "serial_checked": False,
            "hardware_control_attempted": False,
        },
    }
