#!/usr/bin/env python3
"""Hardware-free regression: dynamic block, gravity/contact, and wrist cameras."""
import argparse
import json
from pathlib import Path

import mujoco
import numpy as np

from replay_recorded_episode import build_tabletop, recorded_to_sim, block_contacts, load_episode
from mobile_dual_so101 import apply_control_as_pose


def check(model_path, dataset):
    profile = json.loads(Path(__file__).with_name("tabletop_replay.json").read_text())
    profile["reference_block_center_m"] = [0.45, 0, 0.15]
    model = build_tabletop(model_path, profile)
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
        raise ValueError("pinch fixture requires episode 0 frames 600 and 700")
    results = []
    for disable_pads in (False, True):
        model = build_tabletop(model_path, profile)
        data = mujoco.MjData(model)
        apply_control_as_pose(model, data, recorded_to_sim(model, states[600], profile))
        fixed = data.geom("left_dapier_fixed_finger_pad")
        rotation = fixed.xmat.reshape(3, 3).copy()
        normal = rotation[:, 2]
        toward = data.geom("left_dapier_moving_finger_pad").xpos - fixed.xpos
        if normal @ toward < 0:
            normal = -normal
        # Fixture initial condition only, not a grasp policy: place the free
        # cube flush against the fingertip, then never write its pose again.
        center = fixed.xpos + normal * (0.003 + profile["reference_block_size_m"][0] / 2)
        cube_rotation = np.column_stack([normal, rotation[:, 0], np.cross(normal, rotation[:, 0])])
        quaternion = np.empty(4)
        mujoco.mju_mat2Quat(quaternion, cube_rotation.ravel())
        address = model.joint("red_block_free").qposadr[0]
        data.qpos[address:address + 3] = center
        data.qpos[address + 3:address + 7] = quaternion
        if disable_pads:
            for side in ("left", "right"):
                for finger in ("fixed", "moving"):
                    geom = model.geom(f"{side}_dapier_{finger}_finger_pad")
                    geom.contype = geom.conaffinity = 0
        mujoco.mj_forward(model, data)
        contact_steps = 0
        for step in range(1500):
            if step >= 500:
                alpha = min((step - 500) / 500, 1)
                data.ctrl[:] = recorded_to_sim(model, (1 - alpha) * states[600] + alpha * states[700], profile)
            mujoco.mj_step(model, data)
            contact_steps += int(block_contacts(model, data)["left"])
        dz = float(data.body("red_block").xpos[2] - center[2])
        assert not any(w.number for w in data.warning)
        results.append({"pads_disabled": disable_pads, "bilateral_steps": contact_steps, "height_change_m": dz})
        if disable_pads:
            assert contact_steps == 0 and dz < -0.04, results
        else:
            assert contact_steps > 1400 and dz > 0.03, results
    print(json.dumps({"pinch_and_carry_fixture": results, "table_pick_or_policy_success": False}))
    print("PASS: contact carries cube upward; removing contact makes it fall")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--dataset", type=Path, required=True)
    args = parser.parse_args()
    check(args.model, args.dataset)
