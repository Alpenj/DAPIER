"""Deterministic SO-101 G1 scripted pick-and-lift recording gate.

The runtime in this module is simulation-only.  It never imports ROS 2,
opens a serial device, or constructs a physical LeRobot leader/follower.
"""

from __future__ import annotations

from dataclasses import dataclass
from importlib.metadata import PackageNotFoundError, version
from math import isfinite
import os
from pathlib import Path
import secrets
from typing import Any, Sequence

from .embodiment import SO101_CHANNEL_NAMES, EmbodimentSpec, so101_new_calibration_spec
from .gate import (
    EXPECTED_SOURCE_REVISIONS,
    MANIFEST_SCHEMA_VERSION,
    _digest_file,
    _digest_value,
    _embodiment_mapping,
    _git_has_changes,
    _git_revision,
    _read_json_object,
    _utc_now,
    _write_json_exclusive,
)
from .protocols import Frame, FrameContractError, validate_frame

G1_RECORD_ID = "DAPIER-2026-08-07-so101-g1-scripted-pick"
G1_SEED = 101
G1_RATE_HZ = 30
G1_FRAMES = 300
G1_EXECUTION_CONTRACT = {
    "step_mode": "synchronous",
    "observation_alignment": "post_action_readback",
    "action_reference": "absolute_target",
    "async_control": False,
    "control_period_ns": round(1_000_000_000 / G1_RATE_HZ),
}
G1_CONTROL_PERIOD_NS = 33_333_333
G1_TASK_DESCRIPTION = "Pick up the blue cube and hold it clear of the support."

# This is a deliberately explicit task adaptation, not a claim that the
# external checkout's default PickCube scene already works.  The source cube
# remains 50 mm and 50 g.  Thin collision pads represent rubber finger pads;
# the green tray is used only as a raised support for a lift-and-hold task.
G1_TASK_CONFIG: dict[str, Any] = {
    "task_id": "DAPIER-SO101-PaddedPickLift-v0",
    "source_scene": "pick_cube.xml",
    "object": {
        "geom": "cube_geom",
        "side_length_m": 0.05,
        "mass_kg": 0.05,
        "initial_center_z_m": 0.075,
        "supported_center_z_m": 0.069,
        "grasp_center_site_z_m": 0.060,
        "xy_placement": (
            "derived from gripperframe at grasp_open_action so the declared "
            "local grasp center has supported_center_z_m"
        ),
    },
    "support": {
        "body": "tray",
        "body_z_m": 0.038,
        "floor_top_z_m": 0.044,
        "used_as_goal": False,
    },
    "camera": {
        "name": "front",
        "target_body": "camera_target",
        "target_body_position_m": [0.24, 0.0, 0.10],
        "height": 120,
        "width": 160,
    },
    "finger_pads": {
        "design_gripper_percent": 60.0,
        "fixed": {
            "body": "gripper",
            "name": "dapier_fixed_finger_pad",
            "center_in_gripperframe_m": [-0.020, 0.0, 0.033],
        },
        "moving": {
            "body": "moving_jaw_so101_v1",
            "name": "dapier_moving_finger_pad",
            "center_in_gripperframe_m": [-0.020, 0.0, 0.087],
        },
        "half_size_m": [0.020, 0.020, 0.002],
        "friction": [2.0, 0.01, 0.001],
        "rgba": [0.08, 0.08, 0.08, 1.0],
    },
    "controller": {
        "initial_clear_action": [0.0, -45.0, 17.5, 90.0, 0.0, 100.0],
        "grasp_open_action": [0.0, -13.75, 26.25, 77.5, 0.0, 100.0],
        "grasp_closed_percent": 55.0,
        "lift_fraction": 0.65,
        "phase_boundaries": [0, 30, 110, 170, 190, 260, 300],
        "interpolation": "cubic_smoothstep_joint_target",
    },
    "evaluation": {
        "settle_frame_start": 20,
        "settle_frame_stop": 30,
        "hold_frame_start": 270,
        "hold_frame_stop": 300,
        "minimum_lift_m": 0.020,
        "require_bilateral_pad_contact_for_every_hold_frame": True,
        "require_no_support_contact_for_every_hold_frame": True,
    },
    "physics": {
        "model_timestep_s": 0.002,
        "substep_pattern": [16, 17, 17],
        "expected_total_substeps": 5000,
        "expected_simulated_duration_s": 10.0,
        "object_pose_changes_after_frame_zero": "physics_only",
        "weld_or_equality_grasp": False,
    },
}


@dataclass(frozen=True, slots=True)
class G1TraceFrame:
    frame_index: int
    timestamp_ns: int
    measured_action_units: tuple[float, ...]
    commanded_action: tuple[float, ...]
    image: Any
    cube_position: tuple[float, float, float]
    fixed_pad_contact: bool
    moving_pad_contact: bool
    support_contact: bool

    @property
    def bilateral_pad_contact(self) -> bool:
        return self.fixed_pad_contact and self.moving_pad_contact


def _lerobot_paths(lerobot_root: Path) -> dict[str, Path]:
    return {
        "environment_adapter": (lerobot_root / "src/lerobot/envs/so101_mujoco/env.py"),
        "dataset_writer": (lerobot_root / "src/lerobot/datasets/lerobot_dataset.py"),
        "pyproject": lerobot_root / "pyproject.toml",
    }


def _g1_manifest_input_payload(manifest: dict[str, Any]) -> dict[str, Any]:
    return {
        "gate": manifest["gate"],
        "seed": manifest["seed"],
        "rate_hz": manifest["rate_hz"],
        "frames": manifest["frames"],
        "execution_contract": manifest["execution_contract"],
        "implementation_revision": manifest["implementation"]["revision"],
        "source_revisions": manifest["source_revisions"],
        "embodiment": manifest["embodiment"],
        "model_sha256": manifest["model"]["sha256"],
        "calibration_sha256": manifest["model"]["calibration_sha256"],
        "lerobot": manifest["lerobot"],
        "task_config_digest": manifest["task_config_digest"],
    }


def initialize_g1_manifest(
    *,
    run_root: Path,
    repo_root: Path,
    model_path: Path,
    calibration_path: Path,
    lerobot_root: Path,
) -> Path:
    """Create a fresh immutable G1 manifest outside the DAPIER repository."""

    run_root = run_root.expanduser().resolve()
    repo_root = repo_root.expanduser().resolve(strict=True)
    model_path = model_path.expanduser().resolve(strict=True)
    calibration_path = calibration_path.expanduser().resolve(strict=True)
    lerobot_root = lerobot_root.expanduser().resolve(strict=True)

    if run_root.exists():
        raise ValueError(f"RUN_ROOT must not already exist: {run_root}")
    if run_root == repo_root or run_root.is_relative_to(repo_root):
        raise ValueError("RUN_ROOT must be outside the DAPIER repository")
    if not model_path.is_file() or not calibration_path.is_file():
        raise ValueError("model and calibration inputs must be regular files")

    lerobot_paths = _lerobot_paths(lerobot_root)
    missing = [name for name, path in lerobot_paths.items() if not path.is_file()]
    if missing:
        raise ValueError(f"LeRobot checkout is missing required files: {missing}")

    calibration_digest = _digest_file(calibration_path)
    embodiment = so101_new_calibration_spec(calibration_digest)
    lerobot = {
        "root": str(lerobot_root),
        "revision": _git_revision(lerobot_root),
        "worktree_has_changes_at_init": _git_has_changes(lerobot_root),
        "version_declared": "0.6.0",
        "source_boundary": (
            "existing dirty external checkout; custom so101_mujoco files are "
            "identified by path and SHA-256, not claimed by the Git revision"
        ),
        "files": {
            name: {"path": str(path), "sha256": _digest_file(path)}
            for name, path in lerobot_paths.items()
        },
    }
    manifest: dict[str, Any] = {
        "schema_version": MANIFEST_SCHEMA_VERSION,
        "record_id": G1_RECORD_ID,
        "created_at_utc": _utc_now(),
        "run_id": run_root.name,
        "nonce": secrets.token_hex(16),
        "gate": "G1",
        "seed": G1_SEED,
        "rate_hz": G1_RATE_HZ,
        "frames": G1_FRAMES,
        "execution_contract": G1_EXECUTION_CONTRACT,
        "repo_root": str(repo_root),
        "implementation": {
            "revision": _git_revision(repo_root),
            "worktree_has_changes_at_init": _git_has_changes(repo_root),
        },
        "source_revisions": dict(EXPECTED_SOURCE_REVISIONS),
        "source_revision_verification_mode": (
            "exact manifest match to the work contract; upstream checkout presence is not claimed"
        ),
        "embodiment": _embodiment_mapping(embodiment),
        "model": {
            "path": str(model_path),
            "filename": model_path.name,
            "sha256": _digest_file(model_path),
            "calibration_path": str(calibration_path),
            "calibration_filename": calibration_path.name,
            "calibration_sha256": calibration_digest,
            "source_boundary": (
                "existing untracked custom scene in the dirty external LeRobot checkout"
            ),
        },
        "lerobot": lerobot,
        "task_config": G1_TASK_CONFIG,
        "task_config_digest": _digest_value(G1_TASK_CONFIG),
        "provenance_contract": {
            "source": "scripted",
            "human_demo": False,
        },
    }
    manifest["input_digest"] = _digest_value(_g1_manifest_input_payload(manifest))

    run_root.mkdir(parents=True, exist_ok=False)
    manifest_path = run_root / "run-manifest.json"
    _write_json_exclusive(manifest_path, manifest)
    return manifest_path


def _smoothstep(
    start: Sequence[float], stop: Sequence[float], fraction: float
) -> tuple[float, ...]:
    if not 0.0 <= fraction <= 1.0:
        raise ValueError("smoothstep fraction must be in [0, 1]")
    weight = fraction * fraction * (3.0 - 2.0 * fraction)
    return tuple(
        float(left) if left == right else float(left * (1.0 - weight) + right * weight)
        for left, right in zip(start, stop, strict=True)
    )


def scripted_g1_action(frame_index: int) -> tuple[float, ...]:
    """Return the deterministic absolute six-channel command for one G1 frame."""

    if isinstance(frame_index, bool) or not isinstance(frame_index, int):
        raise ValueError("frame_index must be an integer")
    if not 0 <= frame_index < G1_FRAMES:
        raise ValueError(f"frame_index must be in [0, {G1_FRAMES - 1}]")

    controller = G1_TASK_CONFIG["controller"]
    high_open = tuple(controller["initial_clear_action"])
    low_open = tuple(controller["grasp_open_action"])
    low_closed = low_open[:5] + (float(controller["grasp_closed_percent"]),)
    high_closed = high_open[:5] + (float(controller["grasp_closed_percent"]),)
    lifted = _smoothstep(low_closed, high_closed, float(controller["lift_fraction"]))
    _, settle, descend, close, grasp, lift, end = controller["phase_boundaries"]

    if frame_index < settle:
        return high_open
    if frame_index < descend:
        return _smoothstep(
            high_open, low_open, (frame_index - settle) / (descend - settle)
        )
    if frame_index < close:
        return _smoothstep(
            low_open, low_closed, (frame_index - descend) / (close - descend)
        )
    if frame_index < grasp:
        return low_closed
    if frame_index < lift:
        return _smoothstep(low_closed, lifted, (frame_index - grasp) / (lift - grasp))
    if frame_index < end:
        return lifted
    raise AssertionError("unreachable frame phase")


def simulator_substeps(frame_index: int) -> int:
    if not 0 <= frame_index < G1_FRAMES:
        raise ValueError("frame_index outside G1 trace")
    pattern = G1_TASK_CONFIG["physics"]["substep_pattern"]
    return int(pattern[frame_index % len(pattern)])


def _frame_timestamp_ns(frame_index: int) -> int:
    return 10_000_000_000 + round(frame_index * 1_000_000_000 / G1_RATE_HZ)


def _write_text_exclusive(path: Path, text: str) -> None:
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o444)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            handle.write(text)
    except BaseException:
        try:
            os.close(descriptor)
        except OSError:
            pass
        raise


def _digest_tree(root: Path) -> str:
    entries: list[dict[str, str]] = []
    for path in sorted(
        candidate for candidate in root.rglob("*") if candidate.is_file()
    ):
        entries.append(
            {"path": path.relative_to(root).as_posix(), "sha256": _digest_file(path)}
        )
    return _digest_value(entries)


def _mujoco_name_id(mujoco: Any, model: Any, kind: Any, name: str) -> int:
    identifier = int(mujoco.mj_name2id(model, kind, name))
    if identifier < 0:
        raise ValueError(f"MuJoCo model is missing {name!r}")
    return identifier


def _model_channels(mujoco: Any, model: Any) -> tuple[Any, Any]:
    import numpy as np

    joint_ids = np.asarray(
        [
            _mujoco_name_id(mujoco, model, mujoco.mjtObj.mjOBJ_JOINT, name)
            for name in SO101_CHANNEL_NAMES
        ],
        dtype=np.int32,
    )
    actuator_ids = np.asarray(
        [
            _mujoco_name_id(mujoco, model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            for name in SO101_CHANNEL_NAMES
        ],
        dtype=np.int32,
    )
    return model.jnt_qposadr[joint_ids].astype(np.int32), actuator_ids


def _body_local_pad_pose(
    *,
    mujoco: Any,
    model: Any,
    data: Any,
    site_id: int,
    body_name: str,
    site_position: Sequence[float],
) -> tuple[list[float], list[float]]:
    import numpy as np

    body_id = _mujoco_name_id(mujoco, model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    site_rotation = data.site_xmat[site_id].reshape(3, 3)
    site_origin = data.site_xpos[site_id]
    body_rotation = data.xmat[body_id].reshape(3, 3)
    body_origin = data.xpos[body_id]
    world_position = site_origin + site_rotation @ np.asarray(site_position)
    local_position = body_rotation.T @ (world_position - body_origin)
    local_rotation = body_rotation.T @ site_rotation
    quaternion = np.zeros(4, dtype=np.float64)
    mujoco.mju_mat2Quat(quaternion, local_rotation.ravel())
    return local_position.tolist(), quaternion.tolist()


def _build_task_model(
    *,
    mujoco: Any,
    model_path: Path,
    embodiment: EmbodimentSpec,
) -> tuple[Any, str]:
    """Compile the source scene with the manifest-declared finger pads."""

    base_model = mujoco.MjModel.from_xml_path(str(model_path))
    base_data = mujoco.MjData(base_model)
    qpos_addresses, _ = _model_channels(mujoco, base_model)
    design_action = list(G1_TASK_CONFIG["controller"]["grasp_open_action"])
    design_action[5] = G1_TASK_CONFIG["finger_pads"]["design_gripper_percent"]
    base_data.qpos[qpos_addresses] = embodiment.action_to_sim(design_action)
    mujoco.mj_forward(base_model, base_data)
    site_id = _mujoco_name_id(
        mujoco, base_model, mujoco.mjtObj.mjOBJ_SITE, "gripperframe"
    )

    model_spec = mujoco.MjSpec.from_file(str(model_path))
    camera = G1_TASK_CONFIG["camera"]
    model_spec.body(camera["target_body"]).pos = camera["target_body_position_m"]
    object_config = G1_TASK_CONFIG["object"]
    site_rotation = base_data.site_xmat[site_id].reshape(3, 3)
    site_origin = base_data.site_xpos[site_id]
    local_z = float(object_config["grasp_center_site_z_m"])
    supported_z = float(object_config["supported_center_z_m"])
    local_x = (
        supported_z - site_origin[2] - site_rotation[2, 2] * local_z
    ) / site_rotation[2, 0]
    cube_world = site_origin + site_rotation @ [local_x, 0.0, local_z]
    model_spec.body(G1_TASK_CONFIG["support"]["body"]).pos = [
        float(cube_world[0]),
        float(cube_world[1]),
        G1_TASK_CONFIG["support"]["body_z_m"],
    ]
    model_spec.body("cube").pos = [
        float(cube_world[0]),
        float(cube_world[1]),
        object_config["initial_center_z_m"],
    ]
    pads = G1_TASK_CONFIG["finger_pads"]
    for key in ("fixed", "moving"):
        pad = pads[key]
        local_position, local_quaternion = _body_local_pad_pose(
            mujoco=mujoco,
            model=base_model,
            data=base_data,
            site_id=site_id,
            body_name=pad["body"],
            site_position=pad["center_in_gripperframe_m"],
        )
        model_spec.body(pad["body"]).add_geom(
            name=pad["name"],
            type=mujoco.mjtGeom.mjGEOM_BOX,
            pos=local_position,
            quat=local_quaternion,
            size=pads["half_size_m"],
            contype=1,
            conaffinity=1,
            friction=pads["friction"],
            rgba=pads["rgba"],
            group=3,
            density=0,
        )
    task_xml = model_spec.to_xml()
    model = model_spec.compile()
    if not isfinite(float(model.opt.timestep)):
        raise ValueError("compiled MuJoCo timestep is not finite")
    return model, task_xml


def _validate_runtime_manifest(
    *, manifest: dict[str, Any], manifest_path: Path
) -> tuple[Path, Path, Path, EmbodimentSpec]:
    errors: list[str] = []
    if manifest.get("schema_version") != MANIFEST_SCHEMA_VERSION:
        errors.append("manifest schema_version mismatch")
    if manifest.get("record_id") != G1_RECORD_ID:
        errors.append("manifest record_id mismatch")
    if (
        manifest.get("gate") != "G1"
        or manifest.get("seed") != G1_SEED
        or manifest.get("rate_hz") != G1_RATE_HZ
        or manifest.get("frames") != G1_FRAMES
    ):
        errors.append("manifest must select exact G1 seed/rate/frame count")
    if manifest.get("source_revisions") != EXPECTED_SOURCE_REVISIONS:
        errors.append("pinned source revisions mismatch")
    if manifest.get("task_config") != G1_TASK_CONFIG:
        errors.append("task_config mismatch")
    if manifest.get("execution_contract") != G1_EXECUTION_CONTRACT:
        errors.append("execution_contract mismatch")
    if manifest.get("task_config_digest") != _digest_value(G1_TASK_CONFIG):
        errors.append("task_config digest mismatch")
    try:
        expected_input = _digest_value(_g1_manifest_input_payload(manifest))
    except (KeyError, TypeError) as exc:
        expected_input = None
        errors.append(f"manifest input fields are incomplete: {exc}")
    if manifest.get("input_digest") != expected_input:
        errors.append("manifest input_digest mismatch")

    repo_root = Path(str(manifest.get("repo_root", ""))).resolve(strict=True)
    if manifest_path.parent == repo_root or manifest_path.parent.is_relative_to(
        repo_root
    ):
        errors.append("RUN_ROOT must be outside the DAPIER repository")
    if manifest.get("implementation", {}).get("revision") != _git_revision(repo_root):
        errors.append("DAPIER implementation revision changed after manifest creation")

    model_data = (
        manifest.get("model") if isinstance(manifest.get("model"), dict) else {}
    )
    model_path = Path(str(model_data.get("path", ""))).expanduser()
    calibration_path = Path(str(model_data.get("calibration_path", ""))).expanduser()
    if not model_path.is_file() or _digest_file(model_path) != model_data.get("sha256"):
        errors.append("model file identity mismatch")
    if not calibration_path.is_file() or _digest_file(
        calibration_path
    ) != model_data.get("calibration_sha256"):
        errors.append("calibration file identity mismatch")

    calibration_id = str(model_data.get("calibration_sha256", ""))
    try:
        embodiment = so101_new_calibration_spec(calibration_id)
    except ValueError as exc:
        errors.append(f"invalid calibration identity: {exc}")
        embodiment = so101_new_calibration_spec("sha256:" + "0" * 64)
    if manifest.get("embodiment") != _embodiment_mapping(embodiment):
        errors.append("SO-101 embodiment manifest mismatch")

    lerobot_data = (
        manifest.get("lerobot") if isinstance(manifest.get("lerobot"), dict) else {}
    )
    lerobot_root = Path(str(lerobot_data.get("root", ""))).expanduser()
    if not lerobot_root.is_dir():
        errors.append("LeRobot checkout is missing")
    else:
        if _git_revision(lerobot_root) != lerobot_data.get("revision"):
            errors.append("LeRobot Git revision changed after manifest creation")
        declared_files = lerobot_data.get("files", {})
        for name, path in _lerobot_paths(lerobot_root).items():
            expected = (
                declared_files.get(name, {}) if isinstance(declared_files, dict) else {}
            )
            if not path.is_file() or _digest_file(path) != expected.get("sha256"):
                errors.append(f"LeRobot {name} identity mismatch")
    if errors:
        raise ValueError("; ".join(errors))
    return repo_root, model_path, lerobot_root, embodiment


def _classify_frame_error(message: str) -> str:
    if "channel_names" in message:
        return "order"
    if "sequence_id" in message:
        return "sequence"
    if "timestamp" in message:
        return "timestamp"
    if "stale" in message:
        return "stale"
    return "schema"


def _simulate_g1(
    *, model: Any, embodiment: EmbodimentSpec, seed: int
) -> tuple[list[G1TraceFrame], dict[str, int], dict[str, Any]]:
    import mujoco
    import numpy as np

    if seed != G1_SEED:
        raise ValueError(f"G1 seed must be {G1_SEED}")
    data = mujoco.MjData(model)
    qpos_addresses, actuator_ids = _model_channels(mujoco, model)
    site_id = _mujoco_name_id(mujoco, model, mujoco.mjtObj.mjOBJ_SITE, "gripperframe")
    cube_joint_id = _mujoco_name_id(
        mujoco, model, mujoco.mjtObj.mjOBJ_JOINT, "cube_joint"
    )
    cube_qpos_address = int(model.jnt_qposadr[cube_joint_id])
    cube_body_id = _mujoco_name_id(mujoco, model, mujoco.mjtObj.mjOBJ_BODY, "cube")
    tray_body_id = _mujoco_name_id(mujoco, model, mujoco.mjtObj.mjOBJ_BODY, "tray")
    cube_geom_id = _mujoco_name_id(mujoco, model, mujoco.mjtObj.mjOBJ_GEOM, "cube_geom")
    pads = G1_TASK_CONFIG["finger_pads"]
    fixed_pad_id = _mujoco_name_id(
        mujoco, model, mujoco.mjtObj.mjOBJ_GEOM, pads["fixed"]["name"]
    )
    moving_pad_id = _mujoco_name_id(
        mujoco, model, mujoco.mjtObj.mjOBJ_GEOM, pads["moving"]["name"]
    )
    support_geom_ids = {
        _mujoco_name_id(mujoco, model, mujoco.mjtObj.mjOBJ_GEOM, name)
        for name in (
            "workbench",
            "tray_floor",
            "tray_wall_left",
            "tray_wall_right",
            "tray_wall_near",
            "tray_wall_far",
        )
    }

    low_action = G1_TASK_CONFIG["controller"]["grasp_open_action"]
    data.qpos[qpos_addresses] = embodiment.action_to_sim(low_action)
    mujoco.mj_forward(model, data)
    site_rotation = data.site_xmat[site_id].reshape(3, 3).copy()
    site_origin = data.site_xpos[site_id].copy()
    object_config = G1_TASK_CONFIG["object"]
    local_z = float(object_config["grasp_center_site_z_m"])
    supported_z = float(object_config["supported_center_z_m"])
    local_x = (
        supported_z - site_origin[2] - site_rotation[2, 2] * local_z
    ) / site_rotation[2, 0]
    cube_world = site_origin + site_rotation @ np.asarray([local_x, 0.0, local_z])
    model.body_pos[tray_body_id] = [
        float(cube_world[0]),
        float(cube_world[1]),
        G1_TASK_CONFIG["support"]["body_z_m"],
    ]

    mujoco.mj_resetData(model, data)
    initial_action = scripted_g1_action(0)
    initial_sim = embodiment.action_to_sim(initial_action)
    data.qpos[qpos_addresses] = initial_sim
    data.ctrl[actuator_ids] = initial_sim
    data.qpos[cube_qpos_address : cube_qpos_address + 3] = [
        float(cube_world[0]),
        float(cube_world[1]),
        object_config["initial_center_z_m"],
    ]
    data.qpos[cube_qpos_address + 3 : cube_qpos_address + 7] = [1.0, 0.0, 0.0, 0.0]
    mujoco.mj_forward(model, data)
    initial_cube_position = data.xpos[cube_body_id].astype(float).tolist()

    camera = G1_TASK_CONFIG["camera"]
    renderer = mujoco.Renderer(
        model, height=int(camera["height"]), width=int(camera["width"])
    )
    traces: list[G1TraceFrame] = []
    violations = {
        "schema": 0,
        "order": 0,
        "sequence": 0,
        "timestamp": 0,
        "stale": 0,
        "limit": 0,
    }
    previous_action_frame: Frame | None = None
    previous_readback_frame: Frame | None = None
    total_substeps = 0
    try:
        for frame_index in range(G1_FRAMES):
            timestamp_ns = _frame_timestamp_ns(frame_index)
            action = scripted_g1_action(frame_index)
            raw_readback = tuple(float(value) for value in data.qpos[qpos_addresses])
            try:
                measured = embodiment.sim_to_action(raw_readback)
            except ValueError:
                violations["limit"] += 1
                measured = tuple(float("nan") for _ in SO101_CHANNEL_NAMES)

            action_frame = Frame(
                embodiment_id=embodiment.embodiment_id,
                embodiment_revision=embodiment.embodiment_revision,
                channel_names=embodiment.channel_names,
                values=action,
                units=embodiment.action_units,
                calibration_id=embodiment.calibration_id,
                monotonic_timestamp_ns=timestamp_ns,
                sequence_id=frame_index,
                source="scripted",
            )
            readback_frame = Frame(
                embodiment_id=embodiment.embodiment_id,
                embodiment_revision=embodiment.embodiment_revision,
                channel_names=embodiment.channel_names,
                values=raw_readback,
                units=embodiment.sim_units,
                calibration_id=embodiment.calibration_id,
                monotonic_timestamp_ns=timestamp_ns,
                sequence_id=frame_index,
                source="sim_follower_readback",
            )
            for candidate, previous in (
                (Frame.from_mapping(action_frame.to_mapping()), previous_action_frame),
                (
                    Frame.from_mapping(readback_frame.to_mapping()),
                    previous_readback_frame,
                ),
            ):
                try:
                    validate_frame(
                        candidate,
                        spec=embodiment,
                        now_ns=timestamp_ns,
                        control_period_ns=G1_CONTROL_PERIOD_NS,
                        previous=previous,
                    )
                except FrameContractError as exc:
                    violations[_classify_frame_error(str(exc))] += 1
            previous_action_frame = action_frame
            previous_readback_frame = readback_frame

            try:
                sim_target = embodiment.action_to_sim(action)
            except ValueError:
                violations["limit"] += 1
                sim_target = raw_readback

            renderer.update_scene(data, camera=camera["name"])
            image = renderer.render().copy()
            data.ctrl[actuator_ids] = sim_target
            substeps = simulator_substeps(frame_index)
            for _ in range(substeps):
                mujoco.mj_step(model, data)
            total_substeps += substeps

            other_geoms: set[int] = set()
            for contact_index in range(data.ncon):
                contact = data.contact[contact_index]
                geom1 = int(contact.geom1)
                geom2 = int(contact.geom2)
                if geom1 == cube_geom_id:
                    other_geoms.add(geom2)
                elif geom2 == cube_geom_id:
                    other_geoms.add(geom1)
            traces.append(
                G1TraceFrame(
                    frame_index=frame_index,
                    timestamp_ns=timestamp_ns,
                    measured_action_units=tuple(float(value) for value in measured),
                    commanded_action=action,
                    image=image,
                    cube_position=tuple(
                        float(value) for value in data.xpos[cube_body_id]
                    ),
                    fixed_pad_contact=fixed_pad_id in other_geoms,
                    moving_pad_contact=moving_pad_id in other_geoms,
                    support_contact=bool(other_geoms & support_geom_ids),
                )
            )
    finally:
        renderer.close()

    runtime = {
        "initial_cube_position": initial_cube_position,
        "configured_cube_xy": [float(cube_world[0]), float(cube_world[1])],
        "model_timestep_s": float(model.opt.timestep),
        "total_substeps": total_substeps,
        "simulated_duration_s": total_substeps * float(model.opt.timestep),
        "mujoco_version": getattr(mujoco, "__version__", None),
    }
    return traces, violations, runtime


def evaluate_pick_lift(
    traces: Sequence[G1TraceFrame], *, runtime: dict[str, Any]
) -> dict[str, Any]:
    """Evaluate contact-backed lift-and-hold independently of the controller."""

    if len(traces) != G1_FRAMES:
        raise ValueError(f"expected {G1_FRAMES} trace frames, got {len(traces)}")
    import numpy as np

    evaluation = G1_TASK_CONFIG["evaluation"]
    settle_start = int(evaluation["settle_frame_start"])
    settle_stop = int(evaluation["settle_frame_stop"])
    hold_start = int(evaluation["hold_frame_start"])
    hold_stop = int(evaluation["hold_frame_stop"])
    positions = np.asarray([trace.cube_position for trace in traces], dtype=np.float64)
    settled_position = np.median(positions[settle_start:settle_stop], axis=0)
    lift = positions[:, 2] - settled_position[2]
    bilateral = np.asarray(
        [trace.bilateral_pad_contact for trace in traces], dtype=np.bool_
    )
    support = np.asarray([trace.support_contact for trace in traces], dtype=np.bool_)
    minimum_lift = float(evaluation["minimum_lift_m"])
    hold_lift_min = float(np.min(lift[hold_start:hold_stop]))
    hold_bilateral_count = int(np.count_nonzero(bilateral[hold_start:hold_stop]))
    hold_support_count = int(np.count_nonzero(support[hold_start:hold_stop]))
    hold_total = hold_stop - hold_start
    passed = bool(
        float(np.max(lift)) >= minimum_lift
        and hold_lift_min >= minimum_lift
        and hold_bilateral_count == hold_total
        and hold_support_count == 0
        and runtime["total_substeps"]
        == G1_TASK_CONFIG["physics"]["expected_total_substeps"]
    )
    return {
        "schema_version": 1,
        "record_id": G1_RECORD_ID,
        "gate": "G1",
        "task_id": G1_TASK_CONFIG["task_id"],
        "status": "PASS" if passed else "FAIL",
        "success": passed,
        "task_configuration": {
            "configured_cube_xy_m": runtime["configured_cube_xy"],
            "support_body_z_m": G1_TASK_CONFIG["support"]["body_z_m"],
            "finger_pad_half_size_m": G1_TASK_CONFIG["finger_pads"]["half_size_m"],
            "finger_pad_friction": G1_TASK_CONFIG["finger_pads"]["friction"],
        },
        "object": {
            "initial_position_m": runtime["initial_cube_position"],
            "settled_position_m": settled_position.tolist(),
            "max_position_m": positions[int(np.argmax(positions[:, 2]))].tolist(),
            "final_position_m": positions[-1].tolist(),
            "maximum_lift_from_settled_m": float(np.max(lift)),
            "final_lift_from_settled_m": float(lift[-1]),
            "minimum_lift_during_hold_m": hold_lift_min,
            "required_minimum_lift_m": minimum_lift,
        },
        "contact": {
            "bilateral_pad_contact_frames": int(np.count_nonzero(bilateral)),
            "bilateral_pad_contact_hold_frames": hold_bilateral_count,
            "required_bilateral_pad_contact_hold_frames": hold_total,
            "support_contact_hold_frames": hold_support_count,
            "required_support_contact_hold_frames": 0,
        },
        "timing": {
            "frames": len(traces),
            "rate_hz": G1_RATE_HZ,
            "total_substeps": runtime["total_substeps"],
            "simulated_duration_s": runtime["simulated_duration_s"],
        },
        "integrity": {
            "weld_or_equality_grasp": False,
            "object_pose_changes_after_frame_zero": "physics_only",
            "controller_used_by_evaluator": False,
        },
    }


def _write_lerobot_dataset(
    *, dataset_root: Path, traces: Sequence[G1TraceFrame], evaluation: dict[str, Any]
) -> dict[str, Any]:
    import numpy as np
    from lerobot.datasets import LeRobotDataset

    features = {
        "observation.state": {
            "dtype": "float32",
            "shape": (6,),
            "names": list(SO101_CHANNEL_NAMES),
        },
        "observation.images.front": {
            "dtype": "image",
            "shape": (
                G1_TASK_CONFIG["camera"]["height"],
                G1_TASK_CONFIG["camera"]["width"],
                3,
            ),
            "names": ["height", "width", "channel"],
        },
        "action": {
            "dtype": "float32",
            "shape": (6,),
            "names": list(SO101_CHANNEL_NAMES),
        },
        "next.reward": {"dtype": "float32", "shape": (1,), "names": None},
        "next.success": {"dtype": "bool", "shape": (1,), "names": None},
        "next.done": {"dtype": "bool", "shape": (1,), "names": None},
    }
    dataset = LeRobotDataset.create(
        repo_id="local/dapier-so101-g1-scripted-pick",
        fps=G1_RATE_HZ,
        features=features,
        root=dataset_root,
        robot_type="so101_mujoco",
        use_videos=False,
        image_writer_processes=0,
        image_writer_threads=2,
    )
    settled_z = evaluation["object"]["settled_position_m"][2]
    minimum_lift = evaluation["object"]["required_minimum_lift_m"]
    try:
        for index, trace in enumerate(traces):
            transition_success = bool(
                trace.cube_position[2] - settled_z >= minimum_lift
                and trace.bilateral_pad_contact
                and not trace.support_contact
            )
            dataset.add_frame(
                {
                    "observation.state": np.asarray(
                        trace.measured_action_units, dtype=np.float32
                    ),
                    "observation.images.front": np.asarray(trace.image, dtype=np.uint8),
                    "action": np.asarray(trace.commanded_action, dtype=np.float32),
                    "next.reward": np.atleast_1d(np.float32(float(transition_success))),
                    "next.success": np.atleast_1d(np.bool_(transition_success)),
                    "next.done": np.atleast_1d(np.bool_(index == G1_FRAMES - 1)),
                    "task": G1_TASK_DESCRIPTION,
                }
            )
        buffered = int(dataset.writer.episode_buffer["size"])
        if buffered != G1_FRAMES:
            raise ValueError(
                f"LeRobot episode buffer has {buffered} frames, expected {G1_FRAMES}"
            )
        dataset.save_episode(parallel_encoding=False)
        dataset.finalize()
    except BaseException:
        if dataset.has_pending_frames():
            dataset.clear_episode_buffer(delete_images=True)
        dataset.finalize()
        raise
    return {"buffered_frames_before_save": buffered}


def _validate_lerobot_dataset(
    *, dataset_root: Path, traces: Sequence[G1TraceFrame]
) -> dict[str, Any]:
    import numpy as np
    import pyarrow.parquet as parquet

    info = _read_json_object(dataset_root / "meta/info.json")
    parquet_paths = sorted(dataset_root.glob("data/chunk-*/*.parquet"))
    if not parquet_paths:
        raise ValueError("LeRobot dataset contains no Parquet data")
    tables = [parquet.read_table(path) for path in parquet_paths]
    import pyarrow as pa

    table = pa.concat_tables(tables) if len(tables) > 1 else tables[0]
    measured = np.asarray(table["observation.state"].to_pylist(), dtype=np.float32)
    commanded = np.asarray(table["action"].to_pylist(), dtype=np.float32)
    expected_measured = np.asarray(
        [trace.measured_action_units for trace in traces], dtype=np.float32
    )
    expected_commanded = np.asarray(
        [trace.commanded_action for trace in traces], dtype=np.float32
    )
    timestamps = np.asarray(table["timestamp"].to_pylist(), dtype=np.float64)
    expected_timestamps = np.arange(G1_FRAMES, dtype=np.float64) / G1_RATE_HZ
    frame_indices = np.asarray(table["frame_index"].to_pylist(), dtype=np.int64)
    episode_indices = np.asarray(table["episode_index"].to_pylist(), dtype=np.int64)
    image_rows = table["observation.images.front"].to_pylist()
    image_bytes_count = sum(bool(row.get("bytes")) for row in image_rows)
    measured_matches = int(
        np.count_nonzero(
            np.all(np.isclose(measured, expected_measured, atol=1e-5), axis=1)
        )
    )
    command_matches = int(
        np.count_nonzero(
            np.all(np.isclose(commanded, expected_commanded, atol=1e-5), axis=1)
        )
    )
    timestamp_error = float(np.max(np.abs(timestamps - expected_timestamps)))
    passed = bool(
        info.get("codebase_version") == "v3.0"
        and info.get("fps") == G1_RATE_HZ
        and info.get("total_episodes") == 1
        and info.get("total_frames") == G1_FRAMES
        and table.num_rows == G1_FRAMES
        and measured_matches == G1_FRAMES
        and command_matches == G1_FRAMES
        and timestamp_error <= 2e-6
        and np.array_equal(frame_indices, np.arange(G1_FRAMES))
        and np.count_nonzero(episode_indices == 0) == G1_FRAMES
        and image_bytes_count == G1_FRAMES
    )
    return {
        "passed": passed,
        "codebase_version": info.get("codebase_version"),
        "fps": info.get("fps"),
        "episodes": info.get("total_episodes"),
        "frames": info.get("total_frames"),
        "parquet_rows": table.num_rows,
        "measured_round_trip_matches": measured_matches,
        "command_round_trip_matches": command_matches,
        "timestamp_max_abs_error_s": timestamp_error,
        "image_bytes_rows": image_bytes_count,
        "tree_digest": _digest_tree(dataset_root),
    }


def _write_preview_video(path: Path, traces: Sequence[G1TraceFrame]) -> dict[str, Any]:
    import cv2

    width = int(G1_TASK_CONFIG["camera"]["width"])
    height = int(G1_TASK_CONFIG["camera"]["height"])
    writer = cv2.VideoWriter(
        str(path), cv2.VideoWriter_fourcc(*"mp4v"), G1_RATE_HZ, (width, height)
    )
    if not writer.isOpened():
        raise ValueError("OpenCV could not open the G1 preview writer")
    try:
        for trace in traces:
            writer.write(cv2.cvtColor(trace.image, cv2.COLOR_RGB2BGR))
    finally:
        writer.release()

    capture = cv2.VideoCapture(str(path))
    try:
        frames = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
        fps = float(capture.get(cv2.CAP_PROP_FPS))
    finally:
        capture.release()
    if frames != G1_FRAMES or abs(fps - G1_RATE_HZ) > 0.01:
        raise ValueError(f"preview validation failed: frames={frames}, fps={fps}")
    os.chmod(path, 0o444)
    return {
        "frames": frames,
        "fps": fps,
        "sha256": _digest_file(path),
    }


def _trace_payload(traces: Sequence[G1TraceFrame]) -> dict[str, Any]:
    return {
        "schema_version": 1,
        "record_id": G1_RECORD_ID,
        "gate": "G1",
        "frames": [
            {
                "frame_index": trace.frame_index,
                "timestamp_ns": trace.timestamp_ns,
                "measured_state": list(trace.measured_action_units),
                "commanded_action": list(trace.commanded_action),
                "cube_position_m": list(trace.cube_position),
                "fixed_pad_contact": trace.fixed_pad_contact,
                "moving_pad_contact": trace.moving_pad_contact,
                "support_contact": trace.support_contact,
            }
            for trace in traces
        ],
    }


def run_g1(
    *,
    manifest_path: Path,
    out_path: Path,
    seed: int,
    rate_hz: int,
    frames: int,
) -> tuple[str, Path]:
    """Run one fresh deterministic scripted G1 episode and write its receipt."""

    manifest_path = manifest_path.expanduser().resolve(strict=True)
    if manifest_path.name != "run-manifest.json":
        raise ValueError("manifest filename must be run-manifest.json")
    run_root = manifest_path.parent
    out_path = out_path.expanduser().resolve()
    if out_path.parent != run_root or out_path.name != "G1":
        raise ValueError("G1 output must be exactly $RUN_ROOT/G1")
    existing_entries = list(run_root.iterdir())
    if len(existing_entries) != 1 or existing_entries[0] != manifest_path:
        raise ValueError(
            "RUN_ROOT contains an existing artifact or receipt; refusing reuse"
        )
    if out_path.exists():
        raise ValueError("G1 output already exists; refusing reuse")
    if (seed, rate_hz, frames) != (G1_SEED, G1_RATE_HZ, G1_FRAMES):
        raise ValueError(
            f"G1 command must use seed={G1_SEED}, rate_hz={G1_RATE_HZ}, frames={G1_FRAMES}"
        )

    manifest = _read_json_object(manifest_path)
    _, model_path, _, embodiment = _validate_runtime_manifest(
        manifest=manifest, manifest_path=manifest_path
    )
    os.environ.setdefault("HF_HUB_OFFLINE", "1")
    import mujoco

    try:
        lerobot_version = version("lerobot")
    except PackageNotFoundError as exc:
        raise ValueError(
            "LeRobot distribution is not installed in this runtime"
        ) from exc
    if lerobot_version != manifest["lerobot"]["version_declared"]:
        raise ValueError(
            f"LeRobot runtime version mismatch: {lerobot_version} != "
            f"{manifest['lerobot']['version_declared']}"
        )

    out_path.mkdir(mode=0o755)
    model, task_xml = _build_task_model(
        mujoco=mujoco, model_path=model_path, embodiment=embodiment
    )
    task_model_path = out_path / "task-model.xml"
    _write_text_exclusive(task_model_path, task_xml)
    traces, violations, runtime = _simulate_g1(
        model=model, embodiment=embodiment, seed=seed
    )
    evaluation = evaluate_pick_lift(traces, runtime=runtime)
    evaluation.update(
        {
            "evaluated_at_utc": _utc_now(),
            "task_config_digest": manifest["task_config_digest"],
            "task_model_sha256": _digest_file(task_model_path),
        }
    )
    provenance = {
        "schema_version": 1,
        "record_id": G1_RECORD_ID,
        "gate": "G1",
        "source": "scripted",
        "human_demo": False,
        "description": (
            "Deterministic controller-generated pipeline check; this is not a human imitation demonstration."
        ),
        "seed": seed,
        "rate_hz": rate_hz,
        "frames": frames,
        "controller_digest": _digest_value(G1_TASK_CONFIG["controller"]),
        "task_config_digest": manifest["task_config_digest"],
    }
    _write_json_exclusive(out_path / "provenance.json", provenance)
    _write_json_exclusive(out_path / "task-evaluation.json", evaluation)
    _write_json_exclusive(out_path / "frame-trace.json", _trace_payload(traces))

    errors: list[str] = []
    if len(traces) != G1_FRAMES:
        errors.append("accepted frame count is not 300")
    if any(violations.values()):
        errors.append("one or more frame/safety violations occurred")
    if not evaluation["success"]:
        errors.append("contact-backed pick-and-lift evaluation failed")
    if provenance["source"] != "scripted" or provenance["human_demo"]:
        errors.append("scripted provenance is mislabeled")

    dataset_details: dict[str, Any] = {}
    dataset_validation: dict[str, Any] = {"passed": False}
    preview: dict[str, Any] = {}
    if not errors:
        dataset_root = out_path / "lerobot-v3"
        dataset_details = _write_lerobot_dataset(
            dataset_root=dataset_root, traces=traces, evaluation=evaluation
        )
        dataset_validation = _validate_lerobot_dataset(
            dataset_root=dataset_root, traces=traces
        )
        if not dataset_validation["passed"]:
            errors.append("LeRobot v3 dataset round-trip validation failed")
        preview = _write_preview_video(out_path / "preview.mp4", traces)

    status = "PASS" if not errors else "FAIL"
    metrics = {
        "episode": {"passed": int(status == "PASS"), "total": 1},
        "accepted_frames": {"passed": len(traces), "total": G1_FRAMES},
        "measured_action_pairs": {"passed": len(traces), "total": G1_FRAMES},
        "rendered_frames": {"passed": len(traces), "total": G1_FRAMES},
        "violations": violations,
        "provenance": {
            "source": provenance["source"],
            "human_demo": provenance["human_demo"],
        },
        "task_success": {
            "passed": int(evaluation["success"]),
            "total": 1,
            "maximum_lift_m": evaluation["object"]["maximum_lift_from_settled_m"],
            "minimum_lift_during_hold_m": evaluation["object"][
                "minimum_lift_during_hold_m"
            ],
            "bilateral_contact_hold_frames": evaluation["contact"][
                "bilateral_pad_contact_hold_frames"
            ],
            "support_contact_hold_frames": evaluation["contact"][
                "support_contact_hold_frames"
            ],
        },
        "dataset": dataset_validation,
        "dataset_writer": dataset_details,
        "preview": preview,
    }
    receipt = {
        "schema_version": 1,
        "record_id": G1_RECORD_ID,
        "gate": "G1",
        "run_id": manifest.get("run_id"),
        "nonce": secrets.token_hex(16),
        "created_at_utc": _utc_now(),
        "manifest_hash": _digest_file(manifest_path),
        "input_hash": manifest.get("input_digest"),
        "metrics": metrics,
        "status": status,
        "errors": errors,
        "claims_not_made": [
            "human imitation demonstration",
            "default external PickCube scene success without the declared task adaptation",
            "policy training or learned-policy evaluation",
            "ROS 2 adapter compatibility",
            "serial access or physical hardware control",
            "sim-to-real success",
        ],
    }
    receipt_path = out_path / "receipt.json"
    _write_json_exclusive(receipt_path, receipt)
    return status, receipt_path
