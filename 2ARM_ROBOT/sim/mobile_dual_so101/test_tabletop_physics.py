#!/usr/bin/env python3
"""Hardware-free regression: dynamic block, gravity/contact, and wrist cameras."""
import argparse
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace

import mujoco
import numpy as np

from replay_recorded_episode import build_tabletop, recorded_to_sim, block_contacts, load_episode, physics_settings
from mobile_dual_so101 import apply_control_as_pose
from vision_tabletop_pick import run as run_pick


def check(model_path, dataset, output):
    profile = json.loads(Path(__file__).with_name("tabletop_replay.json").read_text())
    profile["reference_block_center_m"] = [0.45, 0, 0.15]
    model = build_tabletop(model_path, profile)
    assert model.opt.cone == mujoco.mjtCone.mjCONE_ELLIPTIC
    assert model.opt.tolerance == 1e-10 and model.opt.noslip_iterations == 0
    data = mujoco.MjData(model)
    assert model.nu == 12
    assert model.joint("red_block_free").type == mujoco.mjtJoint.mjJNT_FREE
    assert model.body("red_block").mocapid == -1
    assert np.isclose(model.body("red_block").mass, profile["block_mass_kg"])
    assert model.geom("red_block_geom").contype and model.geom("red_block_geom").conaffinity
    assert not any(model.eq_type == mujoco.mjtEq.mjEQ_WELD)
    for side in ("left", "right"):
        assert model.camera(f"{side}_wrist_rgb").id >= 0
        for finger in ("fixed", "moving"):
            assert model.geom(f"{side}_dapier_{finger}_finger_pad").contype
        motor = [g for g in range(model.ngeom)
                 if model.geom_bodyid[g] == model.body(f"{side}_gripper").id
                 and model.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH
                 and model.geom_group[g] == 3
                 and model.mesh(int(model.geom_dataid[g])).name.endswith("sts3215_03a_v1")]
        assert motor and all(model.geom_contype[g] and model.geom_conaffinity[g] for g in motor)
    home = np.tile([0, -35, 55, 35, 0, 100], 2)
    apply_control_as_pose(model, data, recorded_to_sim(model, home, profile))
    initial = data.body("red_block").xpos.copy()
    for _ in range(1000):
        mujoco.mj_step(model, data)
    final = data.body("red_block").xpos.copy()
    assert initial[2] - final[2] > 0.10, (initial, final)
    assert abs(final[2] - profile["reference_block_size_m"][2] / 2) < 0.002, final
    assert np.linalg.norm(final[:2] - initial[:2]) < 0.005
    assert not any(block_contacts(model, data).values())
    assert not any(w.number for w in data.warning)
    print("PASS: free block falls, table supports it, no weld/attachment, both wrist cameras and pads exist")
    states, _, _, _ = load_episode(dataset, 0)
    if len(states) <= 700:
        raise ValueError("historical invalid-pinch check requires episode 0 frame 600")
    # The old positive fixture inserted the cube into an already closed jaw.
    # Keep it as a negative initial-condition check, not as grasp evidence.
    data = mujoco.MjData(model)
    apply_control_as_pose(model, data, recorded_to_sim(model, states[600], profile))
    fixed = data.geom("left_dapier_fixed_finger_pad")
    rotation = fixed.xmat.reshape(3, 3).copy()
    normal = rotation[:, 2]
    if normal @ (data.geom("left_dapier_moving_finger_pad").xpos - fixed.xpos) < 0:
        normal = -normal
    center = fixed.xpos + normal * (0.003 + profile["reference_block_size_m"][0] / 2)
    cube_rotation = np.column_stack([normal, rotation[:, 0], np.cross(normal, rotation[:, 0])])
    address = model.joint("red_block_free").qposadr[0]
    data.qpos[address:address + 3] = center
    mujoco.mju_mat2Quat(data.qpos[address + 3:address + 7], cube_rotation.ravel())
    mujoco.mj_forward(model, data)
    block_id = model.geom("red_block_geom").id
    overlap = max(-c.dist for c in data.contact if block_id in (c.geom1, c.geom2))
    assert overlap > .001, "reassess historical fixture if its initial geometry changes"
    print(f"PASS: historical deeply overlapping pinch is rejected ({overlap * 1000:.3f}mm)")

    # Acquire the cube dynamically from the table using depth, never a pose attachment.
    scene_path = Path(__file__).with_name("tabletop_replay.json")
    for disabled in (False, True):
        path = output / ("contact-off" if disabled else "contact-on")
        code = run_pick(SimpleNamespace(model=model_path, scene=scene_path, output=path,
            viewer=False, disable_finger_contact=disabled, entry_clearance=.003, grasp_force=1.0))
        report = json.loads((path / "report.json").read_text())
        assert code == (2 if disabled else 0), report
        assert report["table_pick_and_3s_hold_success"] is not disabled
        assert report["deepest_contact"]["depth_m"] < .001
        assert not any(report["mujoco_warning_counts"])
        assert report["source_hashes_unchanged"]
        assert not report["object_ground_truth_used_for_control"]
        assert not report["object_pose_writes_or_attachments"]
        if disabled:
            assert report["failure"]["phase"] == "close until virtual bilateral force"
            assert all(plan["phase"] != "lift" for plan in report["plans"])
        else:
            assert report["final_cube_position_m"][2] > .05
            for phase in ("descent with lateral finger clearance", "slow final 15mm approach"):
                assert report["phase_peak_metrics"][phase]["pad_normal_force_N"] < 1e-6
            assert max(p["pad_normal_force_N"] for p in report["phase_peak_metrics"].values()) < 3

    # Clone an actually acquired, settled hold as each ablation's initial state.
    # Removing contact must now drop the cube, not silently preserve an attachment.
    held = json.loads((output / "contact-on/trace.json").read_text())[-1]
    profile = json.loads(scene_path.read_text())
    results = []
    for disable_pads in (False, True):
        model = build_tabletop(model_path, profile)
        model.opt.timestep = 1 / 510
        data = mujoco.MjData(model)
        data.qpos[:] = held["qpos"]
        data.qvel[:] = held["qvel"]
        data.ctrl[:] = held["ctrl_rad"]
        if disable_pads:
            for side in ("left", "right"):
                for finger in ("fixed", "moving"):
                    geom = model.geom(f"{side}_dapier_{finger}_finger_pad")
                    geom.contype = geom.conaffinity = 0
        mujoco.mj_forward(model, data)
        initial_z = float(data.body("red_block").xpos[2])
        contact_steps = 0
        for _ in range(1500):
            mujoco.mj_step(model, data)
            contact_steps += int(block_contacts(model, data)["left"])
        dz = float(data.body("red_block").xpos[2] - initial_z)
        assert not any(w.number for w in data.warning)
        results.append({"pads_disabled": disable_pads, "bilateral_steps": contact_steps, "height_change_m": dz})
        if disable_pads:
            assert contact_steps == 0 and dz < -0.04, results
        else:
            assert contact_steps > 1400 and abs(dz) < .002, results
    summary = {"physics_settings": physics_settings(model),
               "acquired_hold_contact_ablation": results,
               "table_pick_success": True, "learned_policy_success": False}
    (output / "physics-regression.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary))
    print(f"PASS: table pickup, gentle approach, real contact acquisition and contact-removal drop; {output}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--output", type=Path, help="new directory for positive/negative evidence")
    args = parser.parse_args()
    output = args.output or Path(tempfile.mkdtemp(prefix="tabletop-physics-"))
    check(args.model, args.dataset, output)
