#!/usr/bin/env python3
"""Held-out ACT diagnostics; recorded observations are not closed-loop task success."""
import argparse
import hashlib
import io
import json
from pathlib import Path

import numpy as np
from train_act_baseline import RGB_KEYS, INPUT_KEYS, numeric_stats


def metrics(predicted, target, valid):
    error = np.asarray(predicted) - np.asarray(target)
    if error.ndim != 3 or error.shape[-1] != 12 or valid.shape != error.shape[:2]:
        raise ValueError("expected NxTx12 error and NxT mask")
    error = error[valid]
    if not len(error) or not np.isfinite(error).all():
        raise ValueError("empty or non-finite evidence")
    return {"mae_per_joint": np.abs(error).mean(0).tolist(),
            "rmse_per_joint": np.sqrt(np.square(error).mean(0)).tolist(), "valid_pairs": len(error)}


def digest(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def run(args):
    import pyarrow as pa
    import pyarrow.parquet as pq
    import torch
    from PIL import Image
    from lerobot.policies.act.modeling_act import ACTPolicy
    from lerobot.policies.factory import make_pre_post_processors
    from safetensors.torch import load_file

    root, checkpoint, output = args.dataset.resolve(), args.checkpoint.resolve(), args.output.resolve()
    if output.exists() or root == output or root in output.parents:
        raise ValueError("use a new output directory outside the original dataset")
    paths = sorted(root.rglob("*.parquet")) + sorted((root / "meta").glob("*.json"))
    before = {str(p): digest(p) for p in paths}
    contract = json.loads(args.contract.read_text())
    if contract["train_episodes"] != [0, 1, 2] or contract["heldout_episodes"] != [3]:
        raise ValueError("unexpected split")
    if digest(root / "meta/info.json") != contract["info_sha256"]:
        raise ValueError("dataset changed since training")
    info = json.loads((root / "meta/info.json").read_text())
    columns = ["episode_index", "frame_index", "action", *sorted(INPUT_KEYS)]
    table = pa.concat_tables([pq.read_table(p, columns=columns) for p in sorted((root / "data").rglob("*.parquet"))])
    training = table.filter(pa.compute.is_in(table["episode_index"], value_set=pa.array([0, 1, 2])))
    saved_stats = load_file(str(checkpoint / "policy_preprocessor_step_3_normalizer_processor.safetensors"))
    if len(training) != contract["train_frames"]:
        raise ValueError("training frame count differs")
    for key in ("observation.state", "action"):
        for name, value in numeric_stats(training[key].to_pylist()).items():
            if not np.allclose(value, contract["normalization"][key][name], rtol=1e-6, atol=1e-6):
                raise ValueError("split contract does not contain training-only statistics")
            if not np.allclose(value, saved_stats[f"{key}.{name}"].numpy(), rtol=1e-6, atol=1e-6):
                raise ValueError("checkpoint normalizer differs from training-only statistics")
    heldout = table.filter(pa.compute.equal(table["episode_index"], 3)).sort_by("frame_index")
    n = len(heldout)
    if n != contract["heldout_frames"] or heldout["frame_index"].to_pylist() != list(range(n)):
        raise ValueError("held-out frames differ from contract")
    states = np.asarray(heldout["observation.state"].to_pylist(), dtype=np.float32)
    actions = np.asarray(heldout["action"].to_pylist(), dtype=np.float32)
    if not np.isfinite(states).all() or not np.isfinite(actions).all():
        raise ValueError("invalid held-out numeric input")
    policy = ACTPolicy.from_pretrained(checkpoint).to(args.device).eval()
    if set(policy.config.input_features) != INPUT_KEYS or policy.config.n_action_steps != 1:
        raise ValueError("expected every-frame two-camera ACT")
    pre, post = make_pre_post_processors(policy.config, pretrained_path=str(checkpoint),
        preprocessor_overrides={"device_processor": {"device": args.device}})
    images = {key: heldout[key].to_pylist() for key in RGB_KEYS}

    def observation(indices, mode):
        batch = {"observation.state": torch.from_numpy(states[indices].copy())}
        for key in RGB_KEYS:
            image_indices = (indices + n // 2) % n if mode == key else indices
            decoded = []
            for index in image_indices:
                with Image.open(io.BytesIO(images[key][int(index)]["bytes"])) as im:
                    rgb = np.asarray(im.convert("RGB")).copy()
                if rgb.shape != (240, 320, 3):
                    raise ValueError("unexpected wrist RGB shape")
                decoded.append(torch.from_numpy(rgb).permute(2, 0, 1).float() / 255.0)
            batch[key] = torch.stack(decoded)
            if mode == "blank_both":
                batch[key].zero_()
        assert set(batch) == INPUT_KEYS  # Reference actions never enter inference.
        return batch

    predictions = {}
    for mode in ("normal", *RGB_KEYS, "blank_both"):
        chunks = []
        with torch.inference_mode():
            policy.reset()
            for start in range(0, n, args.batch_size):
                indices = np.arange(start, min(start + args.batch_size, n))
                chunks.append(post(policy.predict_action_chunk(pre(observation(indices, mode)))).numpy())
        predictions[mode] = np.concatenate(chunks)
        print(f"Finished {mode}: {n} held-out observations", flush=True)
    indices = np.arange(n)[:, None] + np.arange(policy.config.chunk_size)
    valid = indices < n
    target = actions[np.minimum(indices, n - 1)]
    mean = np.asarray(contract["normalization"]["action"]["mean"])
    predictions["hold_current_state"] = np.broadcast_to(states[:, None, :], target.shape)
    predictions["train_mean_action"] = np.broadcast_to(mean, target.shape)
    report = {
        "schema_version": 1, "kind": "offline held-out diagnostics, not task success",
        "heldout_episode": 3, "frames": n, "chunk_size": policy.config.chunk_size,
        "joint_names": info["features"]["action"]["names"],
        "units": "arm degrees; gripper percent 0..100",
        "ground_truth_actions_passed_to_policy": False, "hardware_execution": False,
        "checkpoint_model_sha256": digest(checkpoint / "model.safetensors"),
        "checkpoint_config_sha256": digest(checkpoint / "config.json"),
        "split_contract_sha256": digest(args.contract),
        "checkpoint_files_sha256": {p.name: digest(p) for p in sorted(checkpoint.iterdir()) if p.is_file()},
        "checkpoint_train_only_statistics_verified": True,
        "metrics": {key: {"first_action": metrics(value[:, :1], target[:, :1], valid[:, :1]),
                          "full_chunk": metrics(value, target, valid)} for key, value in predictions.items()},
        "visual_action_change_mae_per_joint": {
            key: np.abs(predictions[key][:, 0] - predictions["normal"][:, 0]).mean(0).tolist()
            for key in (*RGB_KEYS, "blank_both")},
        "limitations": ["One held-out episode is not generalization evidence.",
                        "Image sensitivity is not proof of correct visual reasoning.",
                        "Recorded observations do not test policy-induced drift or task success."],
    }
    if before != {str(p): digest(p) for p in paths}:
        raise RuntimeError("source dataset changed during evaluation")
    report["source_hashes_unchanged"] = True
    output.mkdir(parents=True, exist_ok=False)
    np.savez_compressed(output / "predictions.npz", target=target, valid=valid, **predictions)
    (output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"output": str(output), "frames": n, "source_unchanged": True}))


def self_test():
    truth = np.zeros((2, 3, 12))
    predicted = truth.copy()
    predicted[:, 2] = 1000
    valid = np.array([[True, True, False], [True, True, False]])
    assert metrics(predicted, truth, valid)["rmse_per_joint"] == [0.0] * 12
    assert metrics(truth + 2, truth, valid)["mae_per_joint"] == [2.0] * 12
    for candidate, mask in [(truth + np.nan, valid), (truth, np.zeros_like(valid)), (truth[:, :, :11], valid)]:
        try:
            metrics(candidate, truth, mask)
        except ValueError:
            pass
        else:
            raise AssertionError("invalid evidence accepted")
    print("PASS: exact errors, padding excluded, non-finite/empty rejected")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--dataset", type=Path)
    parser.add_argument("--checkpoint", type=Path)
    parser.add_argument("--contract", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--device", choices=("cpu", "cuda"), default="cuda")
    parser.add_argument("--batch-size", type=int, default=16)
    args = parser.parse_args()
    if args.self_test:
        self_test()
    elif any(getattr(args, key) is None for key in ("dataset", "checkpoint", "contract", "output")) or args.batch_size < 1:
        parser.error("dataset, checkpoint, contract, new output and positive batch size required")
    else:
        run(args)
