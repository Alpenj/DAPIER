#!/usr/bin/env python3
"""Local LeRobot ACT baseline with episode-isolated normalization.

Reuses the installed trainer, not a new ACT implementation. Raw depth remains
in the dataset for geometry; this baseline consumes both wrist RGBs and state.
"""
import copy
import hashlib
import json
import logging
from pathlib import Path
import sys

import numpy as np

RGB_KEYS = ("observation.images.left_wrist", "observation.images.right_wrist")
INPUT_KEYS = {"observation.state", *RGB_KEYS}


def numeric_stats(values):
    values = np.asarray(values, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] != 12 or not len(values) or not np.isfinite(values).all():
        raise ValueError("expected finite non-empty Nx12 training values")
    return {"mean": values.mean(0), "std": values.std(0), "min": values.min(0),
            "max": values.max(0), "count": np.array([len(values)])}


def install_split_adapter(trainer):
    from lerobot.utils.constants import IMAGENET_STATS
    original = trainer.make_train_eval_datasets

    def make_split(cfg):
        if cfg.env is not None or cfg.policy.type != "act" or cfg.policy.push_to_hub:
            raise ValueError("this entrypoint only permits local offline ACT training")
        if cfg.save_checkpoint_to_hub or cfg.job.is_remote or cfg.dataset.streaming:
            raise ValueError("remote publishing/training and streaming are not supported")
        if set(cfg.policy.input_features) != INPUT_KEYS or cfg.dataset.eval_split != 0.25:
            raise ValueError("expected two wrist RGBs + state and a 25% episode holdout")
        root = Path(cfg.dataset.root).resolve()
        output = Path(cfg.output_dir).resolve()
        if root == output or root in output.parents or output in root.parents:
            raise ValueError("training output must be separate from the original dataset")
        info = json.loads((root / "meta/info.json").read_text())
        if info["total_episodes"] != 4 or info["fps"] != 15 or info["robot_type"] != "bi_so_follower":
            raise ValueError("baseline contract expects the four-episode 15Hz bimanual recording")
        alignment = json.loads((root / "meta/dapier_base_alignment.json").read_text())
        if alignment.get("stored_action") != "actual sent command in configured follower units":
            raise ValueError("actual-sent-action provenance missing")
        train, heldout = original(cfg)
        if train.episodes != [0, 1, 2] or heldout.episodes != [3]:
            raise ValueError("unexpected episode split; do not silently change this baseline")
        actual_train = set(train.hf_dataset.data.column("episode_index").to_pylist())
        actual_eval = set(heldout.hf_dataset.data.column("episode_index").to_pylist())
        if actual_train != {0, 1, 2} or actual_eval != {3}:
            raise ValueError("loaded frames do not match the split")
        stats = {key: numeric_stats(train.hf_dataset.data.column(key).to_pylist())
                 for key in ("observation.state", "action")}
        for key in RGB_KEYS:
            stats[key] = {name: np.asarray(value, dtype=np.float32) for name, value in IMAGENET_STATS.items()}
        # Fix only this invocation: the installed fork and source metadata remain unchanged.
        train.meta.stats = copy.deepcopy(stats)
        heldout.meta.stats = copy.deepcopy(stats)
        if not cfg.resume:
            output.mkdir(parents=True, exist_ok=False)
        report = {
            "schema_version": 1, "train_episodes": train.episodes, "heldout_episodes": heldout.episodes,
            "train_frames": train.num_frames, "heldout_frames": heldout.num_frames,
            "numeric_stats_source": "training episodes only", "rgb_stats_source": "fixed ImageNet",
            "policy_inputs": sorted(INPUT_KEYS), "action_units": "arm degrees, gripper percent 0..100",
            "depth_role": "retained unchanged for geometry/IK; not an ACT RGB channel",
            "base_alignment_reapplied": False, "hardware_execution": False,
            "normalization": {key: {k: v.tolist() for k, v in values.items()} for key, values in stats.items()},
            "info_sha256": hashlib.sha256((root / "meta/info.json").read_bytes()).hexdigest(),
        }
        contract = output / "split_contract.json"
        if cfg.resume and json.loads(contract.read_text()) != report:
            raise ValueError("resume split/statistics differ from the saved contract")
        if not cfg.resume:
            contract.write_text(json.dumps(report, indent=2) + "\n")
        handler = logging.FileHandler(output / "train.log")
        handler.setLevel(logging.INFO)
        handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
        logging.getLogger().addHandler(handler)
        logging.info("Verified train=%s (%d frames), heldout=%s (%d frames); train-only numeric stats",
                     train.episodes, train.num_frames, heldout.episodes, heldout.num_frames)
        return train, heldout

    trainer.make_train_eval_datasets = make_split
    return original


def self_test():
    train = np.arange(36, dtype=float).reshape(3, 12)
    validation = np.full((2, 12), 10000.0)
    stats = numeric_stats(train)
    assert np.allclose(stats["mean"], train.mean(0))
    assert not np.allclose(stats["mean"], np.concatenate([train, validation]).mean(0))
    assert stats["count"].tolist() == [3]
    for bad in [[], [[0] * 11], [[float("nan")] * 12]]:
        try:
            numeric_stats(bad)
        except ValueError:
            pass
        else:
            raise AssertionError("invalid numeric data accepted")
    print("PASS: training-only moments, heldout exclusion, invalid data")


if __name__ == "__main__":
    if sys.argv[1:] == ["--self-test"]:
        self_test()
    else:
        from lerobot.scripts import lerobot_train
        native_split = install_split_adapter(lerobot_train)
        try:
            lerobot_train.main()
        finally:
            lerobot_train.make_train_eval_datasets = native_split
