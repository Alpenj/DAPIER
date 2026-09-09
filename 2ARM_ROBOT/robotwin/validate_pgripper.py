"""Validate PGripper observations, every physics command, contacts and decoded camera data."""
import argparse
import hashlib
import json
from pathlib import Path

import cv2
import h5py
import numpy as np


def validate(folder):
    report = json.loads((folder / "report.json").read_text())
    assert report["success"], report.get("failure")
    with h5py.File(folder / "episode.hdf5", "r") as f:
        n, steps = int(f.attrs["frames"]), report["physics_steps"]
        assert f.attrs["success"] and n == report["frames"] == (steps+19)//20
        index = f["physics_step"][:]
        assert np.array_equal(index, np.arange(n)*20)
        assert np.allclose(f["timestamp_s"][:], index/500, rtol=0, atol=1e-10)
        state, action = f["observation/state_rad"][:], f["action_rad"][:]
        control, actual = f["expert/action_rad"][:], f["expert/state_500hz_rad"][:]
        assert state.shape == action.shape == (n, 12)
        assert control.shape == actual.shape == (steps, 12)
        assert np.isfinite(state).all() and np.isfinite(action).all()
        assert np.isfinite(control).all() and np.isfinite(actual).all()
        assert np.array_equal(action, control[index])
        assert np.array_equal(state[1:], actual[index[1:]-1]), "Observation/action timing mismatch"
        for raw, normalized in ((state, f["observation/state"][:]), (action, f["action"][:])):
            expected = raw.copy()
            expected[:, [5, 11]] /= 2.2028
            assert np.allclose(normalized, expected, rtol=0, atol=1e-7)
        valid = f["expert/contact_valid"][:]
        assert not valid[0] and valid[1:].all()
        assert np.isnan(f["expert/pad_force_N"][0]).all()
        force = f["expert/pad_force_500hz_N"][:]
        support = f["expert/table_support_500hz_N"][:]
        depth = f["expert/penetration_500hz_m"][:]
        block = f["expert/block_pose_500hz"][:]
        phase = np.char.strip(f["expert/phase_500hz"][:].astype(str))
        assert force.shape == (steps, 2, 2) and support.shape == depth.shape == (steps,)
        assert np.isfinite(force).all() and np.isfinite(support).all() and np.isfinite(depth).all()
        assert (force >= 0).all() and force.max() <= 10 and depth.max() <= .001
        assert np.isfinite(block).all() and block.shape == (steps, 7)
        assert np.allclose(np.linalg.norm(block[:, 3:], axis=1), 1, atol=1e-5)
        assert np.allclose(f["expert/pad_force_N"][1:], force[index[1:]-1])
        assert np.allclose(f["expert/block_pose"][1:], block[index[1:]-1])
        if report.get("camera_driven"):
            side = 0 if report["arm"] == "left" else 1
            close = np.flatnonzero(phase == "close")
            assert len(close) >= 165 and force[close[-165:], side].min() >= 1
            lift = np.isin(phase, ["vision lift", "hold"])
            assert force[lift, side].min() >= .3
            hold = phase == "hold"
            assert hold.sum() == 1500 and block[hold, 2].min() > .06
            assert support[hold].max() <= .01 and force[hold, 1-side].max() <= .01
            assert report["hold_seconds"] == 3 and report["maximum_coupling_error_m"] <= .0003
            assert not report["object_ground_truth_used_for_control"]
            assert not report["object_attachments"] and not report["hardware_execution"]
            assert report["object_pose_writes_after_initialization"] == 0
            assert report["runtime_qpos_writes_after_initialization"] == 0
            sources = [d["source"] for d in report["detections"]]
            assert sources[0] == "top RGB + metric depth"
            assert report["arm"]+" wrist RGB" in sources
            assert sources[-1] == report["arm"]+" wrist RGB visibility"
            assert not report["blank_top"] and not report["blank_wrist"]
        else:
            for side, close in enumerate(("donor close", "recipient close")):
                selection = np.flatnonzero(phase == close)
                assert len(selection) >= 165 and force[selection[-165:], side].min() >= 1
            donor_hold = np.isin(phase, ["donor lift", "donor present", "recipient orient away",
                                        "recipient approach", "recipient insert", "recipient close"])
            assert force[donor_hold, 0].min() >= .3
            recipient_hold = np.isin(phase, ["donor release", "donor retreat", "recipient hold",
                                            "recipient carry", "place approach", "place seat", "place support confirm"])
            assert force[recipient_hold, 1].min() >= .3
            alone = phase == "recipient hold"
            assert alone.sum() >= 1500 and force[alone, 0].max() <= .01 and block[alone, 2].min() > .06
            seat = phase == "place support confirm"
            assert seat.sum() >= 100 and support[seat].min() >= .05
            rest = phase == "table hold"
            assert rest.sum() >= 1500 and support[rest].min() >= .1 and force[rest].max() <= .01
            assert block[rest, 2].min() >= .019 and block[rest, 2].max() <= .022
            assert np.max(np.linalg.norm(np.diff(block[rest, :3], axis=0), axis=1)/.002) < .01
            assert np.max(np.linalg.norm(block[rest, :3]-block[rest, :3][0], axis=1)) < .001
        depth_valid_fraction = []
        for start in range(0, n, 32):
            chunk = f["observation/depth_m"][start:start+32]
            assert chunk.shape[1:] == (240, 320) and np.isfinite(chunk).all() and chunk.min() >= 0
            assert chunk.max() < 10
            depth_valid_fraction.extend((chunk > 0).mean(axis=(1, 2)).tolist())
        assert min(depth_valid_fraction) > .1, "Depth contains no usable surface"
        decoded = 0
        for name, dataset in f["observation/images"].items():
            assert len(dataset) == n
            for encoded in dataset:
                image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
                assert image is not None and image.shape == (240, 320, 3)
                decoded += 1
            transforms = f["camera_metadata/"+name+"/world_transform"][:]
            assert transforms.shape == (n, 4, 4) and np.isfinite(transforms).all()
        assert decoded == n*3
    video = cv2.VideoCapture(str(folder / "preview.mp4"))
    frames = 0
    while True:
        ok, frame = video.read()
        if not ok:
            break
        assert frame.shape == (480, 640, 3)
        frames += 1
    video.release()
    assert frames == n
    result = dict(passed=True, frames=n, decoded_images=decoded, video_frames=frames,
        physics_steps=steps, maximum_pad_force_N=float(force.max()),
        maximum_penetration_m=float(depth.max()), minimum_depth_valid_fraction=min(depth_valid_fraction),
        hdf5_sha256=hashlib.file_digest((folder/"episode.hdf5").open("rb"), "sha256").hexdigest()
            if hasattr(hashlib, "file_digest") else hashlib.sha256((folder/"episode.hdf5").read_bytes()).hexdigest())
    (folder / "validation.json").write_text(json.dumps(result, indent=2))
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folder", type=Path)
    args = parser.parse_args()
    print(json.dumps(validate(args.folder.resolve()), indent=2))
