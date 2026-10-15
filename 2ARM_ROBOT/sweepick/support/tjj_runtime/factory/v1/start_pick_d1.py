#!/usr/bin/env python3
"""START-PICK D1: what START was trained on, what its wrist inputs show, what it predicts on teacher observations, and
what it does from nine fixed Train resets (local SIM only; no training, no teacher run, no Val / Test run, no hardware).

  coverage    START's training lineage (stored run records) and the reset ranges of Train69 (focus 3 / other 66) and Val10
  visibility  stored wrist frames of the Train69 demonstrations at reset / approach / descent / first left-pad contact, the
              policy's real preprocessed input, and the stored Val 5001 student frames (red-pixel proxy; frames saved to look at)
  offline     START in deployment mode on the teacher's C25-boundary observations from the reset to the RECEIVE hand-over
              boundary of every Train parent: k = 0..24, arm rad, gripper opening fraction
  run         nine new original-reset runs, START -> RECEIVE_E -> RELEASE_B, fixed before execution: 5000 / 5002 / 5008 and
              the first / middle / last other Train parent of G000 and of G001 in manifest order
  summary     this run's own tables (used by `act-sim results`)
  video       first failure and first pick_lift success among the six additional Train parents

Switches read SIM ground truth (SIM-ORACLE-GATED THREE-POLICY). SIM ground truth is used for analysis, never as a policy input.
"""
import argparse
import hashlib
import json
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import train_rollout_diagnostic as td   # noqa: E402
import receive_candidate as rc   # noqa: E402
import receive_boundary_repair as rb   # noqa: E402
import receive_entry_r2 as r2   # noqa: E402
import receive_onset_r3 as r3   # noqa: E402
import receive_fit_check as fc   # noqa: E402
import receive_r3_recheck as rk   # noqa: E402
import receive_r3_val as rv   # noqa: E402

FACTORY_ROOT = td.FACTORY_ROOT
RUN = "start-pick-d1-20261006-01"
LABEL = "SIM-ORACLE-GATED THREE-POLICY / START-PICK D1 (fixed Train9 development diagnostic)"
FOCUS = (5000, 5002, 5008)
MODELS = dict(START=rk.MODELS["START"], RELEASE_B=rk.MODELS["RELEASE_B"], E=rk.MODELS["E"])
PICK_PHASES = ("left_approach", "left_descend", "left_grasp", "left_lift")
K = rc.PREFIX
GROUPS = fc.GROUPS
MOVING = fc.MOVING
SOURCES = rv.SOURCES + ("factory/v1/start_pick_d1.py",)
VAL_SCREEN = FACTORY_ROOT / "evaluation" / rv.RUN


def load(path):
    return json.loads(Path(path).read_text())


def run_dir():
    return FACTORY_ROOT / "evaluation" / RUN


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def manifest():
    return load(td.DATASET / "manifest.json")["episodes"]


def train_rows():
    return [(i, e) for i, e in enumerate(manifest()) if e["split"] == "train" and e["training_eligible"]]


def train9():
    """5000 / 5002 / 5008, then per generation the first / middle (floor((n-1)/2)) / last other Train parent in manifest order."""
    rows = train_rows()
    picked = [dict(seed=s, manifest_index=next(i for i, e in rows if e["seed"] == s), generation=next(e["generation"] for i, e in rows if e["seed"] == s), group="focus 3", rule="required") for s in FOCUS]
    for gen in ("G000", "G001"):
        rest = [(i, e) for i, e in rows if e["generation"] == gen and e["seed"] not in FOCUS]
        for name, k in (("first", 0), ("middle", (len(rest) - 1) // 2), ("last", len(rest) - 1)):
            i, e = rest[k]
            picked.append(dict(seed=e["seed"], manifest_index=i, generation=gen, group="additional 6", rule=f"{gen} {name} of {len(rest)} other Train parents (position {k})"))
    return picked


def red_mask(rgb):
    """Analysis-only colour proxy for the red block in a raw wrist frame; not a recognition result and never a runtime gate."""
    r, g, b = (rgb[..., i].astype(np.int32) for i in range(3))
    return (r > 90) & (r > 1.8 * g) & (r > 1.8 * b)


# ----------------------------------------------------------------------------------------------
# coverage: lineage and reset ranges (stored records only)
# ----------------------------------------------------------------------------------------------

def coverage(args):
    target = run_dir() / "coverage.json"
    if target.exists():
        raise FileExistsError(f"{target} exists; nothing is overwritten")
    run_dir().mkdir(parents=True, exist_ok=True)
    lineage, p = [], MODELS["START"].parent / "run.json"
    while True:
        r = load(p)
        seeds = r.get("train_seeds") or sorted({manifest()[i]["seed"] for i in r.get("train_episodes", [])})
        lineage.append(dict(training=p.parent.name, kind=r.get("kind"), train_parents=len(r.get("train_episodes", [])), parent_seeds=seeds if len(seeds) <= 3 else f"{len(seeds)} Train parents",
                            updates=r.get("budget_updates", r.get("steps")), warm_start=r.get("warm_start")))
        w = r.get("warm_start")
        if not w or not (Path(w).parent / "run.json").exists():
            break
        p = Path(w).parent / "run.json"

    def resets(rows):
        out = []
        for i, e in rows:
            meta = load(Path(e["path"]) / "metadata.json")
            out.append((e["seed"], e["generation"], np.array(meta["requested_initial_state"]["q"]), np.array(meta["sampling"]["block_xy_yaw"])))
        return out

    tr = resets(train_rows())
    val = resets([(i, e) for i, e in enumerate(manifest()) if e["split"] == "val"])
    focus = [x for x in tr if x[0] in FOCUS]

    def ranges(items):
        q, b = np.array([x[2] for x in items]), np.array([x[3] for x in items])
        rng = lambda a: [round(float(a.min()), 3), round(float(a.max()), 3)]
        return dict(parents=len(items), left_arm_q0_rad=[rng(q[:, j]) for j in GROUPS["left_arm"]], right_arm_q0_rad=[rng(q[:, j]) for j in GROUPS["right_arm"]],
                    left_gripper_open=rng(q[:, td.LEFT_GRIPPER]), right_gripper_open=rng(q[:, td.RIGHT_GRIPPER]), block_x_m=rng(b[:, 0]), block_y_m=rng(b[:, 1]), block_yaw_rad=rng(b[:, 2]))

    def nearest(items):
        out = []
        for seed, gen, q, b in items:
            d = [(float(np.linalg.norm(b[:2] - f[3][:2]) * 1000), float(np.abs(q[list(rc.ARM)] - f[2][list(rc.ARM)]).max()), abs(float(q[td.RIGHT_GRIPPER] - f[2][td.RIGHT_GRIPPER])), f[0]) for f in focus]
            k = min(d)
            out.append(dict(seed=seed, generation=gen, block_xy_to_nearest_focus_mm=round(k[0], 1), nearest_focus=k[3], arm_q0_max_diff_to_that_focus_rad=round(k[1], 3),
                            right_gripper_open=round(float(q[td.RIGHT_GRIPPER]), 3), block_yaw_rad=round(float(b[2]), 3)))
        return out

    groups = {"focus 3 (5000 / 5002 / 5008)": focus, "other Train 66": [x for x in tr if x[0] not in FOCUS], "other Train G000": [x for x in tr if x[0] not in FOCUS and x[1] == "G000"],
              "other Train G001": [x for x in tr if x[0] not in FOCUS and x[1] == "G001"], "Val10 (description of the finished screen only)": val}
    data = dict(run_id=RUN, lineage_newest_first=lineage,
                lineage_reading="normalization is the Train69 one throughout. The weights saw all 69 Train parents for 5,000 + 5,000 updates (the first of those warm-started from step-004000), then seed 5000 "
                                "alone for 20,000 updates, then 5000 / 5002 / 5008 for 20,000 updates. Earlier Train69 exposure exists; it is not evidence that the current weights still grasp on the 69",
                cameras="two wrist RGB cameras (left_wrist_rgb, right_wrist_rgb), 160 x 120, one mounted on each gripper; no overhead camera is a policy input",
                reset_ranges={k: ranges(v) for k, v in groups.items()}, focus_resets=[dict(seed=f[0], q0=f[2].round(3).tolist(), block_xy_yaw=f[3].round(3).tolist()) for f in focus],
                distance_to_nearest_focus=dict(note="descriptive statistics, not a safety boundary", other_train=nearest([x for x in tr if x[0] not in FOCUS]), val=nearest(val)))
    for name in ("other_train", "val"):
        d = data["distance_to_nearest_focus"][name]
        data["distance_to_nearest_focus"][name + "_summary"] = dict(block_xy_mm=[min(x["block_xy_to_nearest_focus_mm"] for x in d), float(np.median([x["block_xy_to_nearest_focus_mm"] for x in d])),
                                                                               max(x["block_xy_to_nearest_focus_mm"] for x in d)],
                                                                   arm_q0_max_diff_rad=[min(x["arm_q0_max_diff_to_that_focus_rad"] for x in d), float(np.median([x["arm_q0_max_diff_to_that_focus_rad"] for x in d])),
                                                                                        max(x["arm_q0_max_diff_to_that_focus_rad"] for x in d)])
    target.write_text(json.dumps(data, indent=2))
    print(json.dumps(dict(lineage=lineage, ranges={k: {a: v[a] for a in ("parents", "left_gripper_open", "right_gripper_open", "block_x_m", "block_y_m", "block_yaw_rad")} for k, v in data["reset_ranges"].items()},
                          other_train=data["distance_to_nearest_focus"]["other_train_summary"], val=data["distance_to_nearest_focus"]["val_summary"]), indent=1))
    print(f"[written] {target}")


# ----------------------------------------------------------------------------------------------
# visibility: what the wrist inputs show
# ----------------------------------------------------------------------------------------------

def visibility(args):
    import cv2
    import torch
    from sim_data_factory.r2_data import policy_observation
    target = run_dir() / "visibility.json"
    if target.exists():
        raise FileExistsError(f"{target} exists; nothing is overwritten")
    (run_dir() / "frames").mkdir(parents=True, exist_ok=True)
    st = rc.stats()
    picked = {p["seed"] for p in train9()}
    per, sheets = [], {}
    for i, e in train_rows():
        path = Path(e["path"])
        phases = [d.get("phase") for d in load(path / "diagnostics.json")]
        rows = load(path / "task_check_rows.json")
        rgb = {s: np.load(path / f"cameras/{s}_wrist_rgb.npz")["rgb"] for s in ("left", "right")}
        first = lambda name: next((k for k, p in enumerate(phases) if p == name), None)
        touch = next((k for k, r in enumerate(rows[:len(rgb["left"])]) if max(r["pad_force_N"]["left"]) > 0), None)
        moments = dict(reset=0, approach_mid=(first("left_descend") or 0) // 2, descend_start=first("left_descend"), first_left_pad_touch=touch)
        end = first("left_lift") or len(phases) - 1
        counts = {s: np.array([int(red_mask(rgb[s][k]).sum()) for k in range(end + 1)]) for s in rgb}
        item = dict(seed=e["seed"], generation=e["generation"], group="focus 3" if e["seed"] in FOCUS else "other 66",
                    moments={m: None if k is None else dict(step=k, t=round(k * td.DT, 2), left_red_px=int(counts["left"][k]), right_red_px=int(counts["right"][k])) for m, k in moments.items()},
                    left_first_visible_s=next((round(k * td.DT, 2) for k in range(end + 1) if counts["left"][k] > 0), None),
                    right_first_visible_s=next((round(k * td.DT, 2) for k in range(end + 1) if counts["right"][k] > 0), None),
                    left_visible_share_before_lift=round(float((counts["left"] > 0).mean()), 3), right_visible_share_before_lift=round(float((counts["right"] > 0).mean()), 3),
                    descend_start_s=None if moments["descend_start"] is None else round(moments["descend_start"] * td.DT, 2))
        per.append(item)
        if e["seed"] in picked:
            tiles = []
            for m, k in moments.items():
                if k is None:
                    continue
                pair = np.concatenate([rgb["left"][k], rgb["right"][k]], axis=0)
                pair = cv2.resize(pair, (320, 480), interpolation=cv2.INTER_NEAREST)
                cv2.putText(pair, f"{m} t={k * td.DT:.1f}s L{counts['left'][k]} R{counts['right'][k]}", (4, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
                tiles.append(pair)
            sheet = np.concatenate(tiles, axis=1)
            name = f"train{e['seed']}_teacher_wrist_left_top_right_bottom.png"
            cv2.imwrite(str(run_dir() / "frames" / name), cv2.cvtColor(sheet, cv2.COLOR_RGB2BGR))
            sheets[str(e["seed"])] = name
    # the policy's real preprocessed input: the same 160 x 120 pixels, scaled and normalised, nothing cropped or resized
    e = next(e for i, e in train_rows() if e["seed"] == 5000)
    raw = {s: np.load(Path(e["path"]) / f"cameras/{s}_wrist_rgb.npz")["rgb"][0] for s in ("left", "right")}
    with np.load(Path(e["path"]) / "episode.npz", allow_pickle=False) as a:
        q0 = a["q_state"][0].astype(np.float32)
    batch = policy_observation(q0, raw, st)
    key = next(k for k in batch if "left" in k)
    back = (batch[key] * torch.tensor([.229, .224, .225])[:, None, None] + torch.tensor([.485, .456, .406])[:, None, None]) * 255
    preprocessing = dict(tensor_keys=sorted(batch), image_tensor_shape=list(batch[key].shape), raw_shape=list(raw["left"].shape),
                         denormalised_tensor_equals_raw_pixels=bool(np.array_equal(np.rint(back.permute(1, 2, 0).numpy()).astype(np.uint8), raw["left"])),
                         statement="policy_observation: uint8 HWC -> CHW float / 255 -> ImageNet mean / std; no crop, no resize, no augmentation; model input shape (3, 120, 160) as in config.json")
    # stored Val 5001 student frames (the screen's E run): observation check only, never a label
    val_case = next(Path(r["path"]) for r in load(VAL_SCREEN / "results.json")["results"] if r["seed"] == 5001 and r["receive_model"] == "E")
    vrgb = {s: np.load(val_case / f"cameras/{s}_wrist_rgb.npz")["rgb"] for s in ("left", "right")}
    vcounts = {s: [int(red_mask(f).sum()) for f in vrgb[s]] for s in vrgb}
    tiles = []
    for k in (0, len(vrgb["left"]) // 2, 285 if len(vrgb["left"]) > 285 else len(vrgb["left"]) - 1, len(vrgb["left"]) - 1):
        pair = cv2.resize(np.concatenate([vrgb["left"][k], vrgb["right"][k]], axis=0), (320, 480), interpolation=cv2.INTER_NEAREST)
        cv2.putText(pair, f"Val5001 student t={k * td.DT:.2f}s L{vcounts['left'][k]} R{vcounts['right'][k]}", (4, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
        tiles.append(pair)
    cv2.imwrite(str(run_dir() / "frames" / "val5001_student_E_wrist_left_top_right_bottom.png"), cv2.cvtColor(np.concatenate(tiles, axis=1), cv2.COLOR_RGB2BGR))
    val5001 = dict(source=str(val_case), frames=len(vrgb["left"]), left_red_px=dict(at_reset=vcounts["left"][0], max=max(vcounts["left"]), frames_visible=int(sum(c > 0 for c in vcounts["left"]))),
                   right_red_px=dict(at_reset=vcounts["right"][0], max=max(vcounts["right"]), frames_visible=int(sum(c > 0 for c in vcounts["right"]))),
                   video_inset="recovery_trial.render pastes run['wrist']['left'][k] (top) and ['right'][k] (bottom), i.e. these stored policy-input frames resized to 240 x 180; "
                               "the large left panel of the video is a separate third-person render that is NOT a policy input",
                   sheet="val5001_student_E_wrist_left_top_right_bottom.png")

    def summarise(items):
        mom = {}
        for m in ("reset", "approach_mid", "descend_start", "first_left_pad_touch"):
            xs = [p["moments"][m] for p in items if p["moments"][m]]
            mom[m] = dict(parents=len(xs), left_visible=sum(x["left_red_px"] > 0 for x in xs), right_visible=sum(x["right_red_px"] > 0 for x in xs),
                          left_red_px_median=float(np.median([x["left_red_px"] for x in xs])) if xs else None)
        first = [p["left_first_visible_s"] for p in items if p["left_first_visible_s"] is not None]
        lead = [p["descend_start_s"] - p["left_first_visible_s"] for p in items if p["left_first_visible_s"] is not None and p["descend_start_s"] is not None]
        return dict(parents=len(items), moments=mom, left_never_visible_before_lift=sum(p["left_first_visible_s"] is None for p in items),
                    left_first_visible_s=[min(first), float(np.median(first)), max(first)] if first else None,
                    seconds_visible_before_descend_start=[round(min(lead), 2), round(float(np.median(lead)), 2), round(max(lead), 2)] if lead else None,
                    right_ever_visible_before_lift=sum(p["right_first_visible_s"] is not None for p in items))

    data = dict(run_id=RUN, proxy="red pixels (R > 90, R > 1.8 G, R > 1.8 B) in the raw 160 x 120 wrist frame; analysis only. 0 pixels is not proof that the block cannot be located, "
                                  "and > 0 pixels is not proof that the policy uses it",
                preprocessing=preprocessing, summary={"all Train69": summarise(per), "focus 3": summarise([p for p in per if p["group"] == "focus 3"]), "other 66": summarise([p for p in per if p["group"] == "other 66"])},
                val5001_student=val5001, sheets=sheets, parents=per)
    target.write_text(json.dumps(data, indent=2))
    print(json.dumps(dict(preprocessing=preprocessing, summary=data["summary"], val5001={k: val5001[k] for k in ("frames", "left_red_px", "right_red_px")}, sheets=sheets), indent=1))
    print(f"[written] {target}")


# ----------------------------------------------------------------------------------------------
# offline: START on the teacher's C25-boundary observations, reset -> hand-over boundary
# ----------------------------------------------------------------------------------------------

def offline(args):
    import arm_output_trial as trial
    import frozen_transfer_check as ft
    from dapier_act_policy import ACTPolicy
    from sim_data_factory.reset_sampler import make_task_episode
    target = run_dir() / "offline.json"
    if target.exists():
        raise FileExistsError(f"{target} exists; nothing is overwritten")
    run_dir().mkdir(parents=True, exist_ok=True)
    st = rc.stats()
    ids = [i for i, e in train_rows()]
    replay = trial.TrialEpisodes(td.DATASET, ids, required_split="train", stats=st, arm_target="measured_relative")
    policy = ACTPolicy.from_pretrained(MODELS["START"], local_files_only=True, strict=True).cuda().eval()
    policy.check_normalization(st)
    first_index, total = {}, 0
    for i, ep in replay.episodes.items():
        first_index[i], total = total, total + len(ep["action"])
    parents, note = [], []
    for row in replay.rows:
        i, path = row["manifest_index"], Path(row["path"])
        ep = replay.episodes[i]
        phases = [d.get("phase") for d in load(path / "diagnostics.json")]
        start, _, why = rb.semantic_bounds(load(path / "task_check_rows.json"), load(path / "metadata.json")["task_check"]["events"])
        if start is None:
            start = next(k for k, p in enumerate(phases) if p == "transfer_approach")
            note.append(dict(seed=row["seed"], reason=f"hand-over boundary not found ({why}); the teacher's first transfer_approach step is used instead"))
        ts = list(range(0, start + 1, K))
        pred = fc.predict(policy, [replay[first_index[i] + t] for t in ts])[:, :K]
        n = len(ep["action"])
        steps = np.array(ts)[:, None] + np.arange(K)[None]
        tgt = ep["action"][np.minimum(steps, n - 1)].astype(np.float64)
        anchor = ep["state"][ts].astype(np.float64)
        bounds = []
        for j, t in enumerate(ts):
            b = dict(t=round(t * td.DT, 2), phase=phases[t])
            for g, cols in GROUPS.items():
                want, got = tgt[j, -1, cols] - anchor[j, cols], pred[j, -1, cols] - anchor[j, cols]
                b[g] = dict(mae=round(float(np.abs(pred[j] - tgt[j])[:, cols].mean()), 4), max_err=round(float(np.abs(pred[j] - tgt[j])[:, cols].max()), 4),
                            teacher_move=round(float(np.linalg.norm(want)), 4), predicted_move=round(float(np.linalg.norm(got)), 4),
                            progress=None if np.linalg.norm(want) <= MOVING else round(float(got @ want / (want @ want)), 2))
            b["right_gripper_k24"] = [round(float(tgt[j, -1, td.RIGHT_GRIPPER]), 3), round(float(pred[j, -1, td.RIGHT_GRIPPER]), 3)]
            b["left_gripper_k24"] = [round(float(tgt[j, -1, td.LEFT_GRIPPER]), 3), round(float(pred[j, -1, td.LEFT_GRIPPER]), 3)]
            bounds.append(b)

        def agg(sel, g):
            xs = [b[g] for b in sel]
            pr = [x["progress"] for x in xs if x["progress"] is not None]
            return dict(boundaries=len(xs), mae=round(float(np.mean([x["mae"] for x in xs])), 4), max_err=round(max(x["max_err"] for x in xs), 4), teacher_move_mean=round(float(np.mean([x["teacher_move"] for x in xs])), 4),
                        predicted_move_mean=round(float(np.mean([x["predicted_move"] for x in xs])), 4), moving=len(pr), progress_median=None if not pr else round(float(np.median(pr)), 2),
                        progress_min=None if not pr else round(min(pr), 2))

        per_phase = {}
        for ph in PICK_PHASES:
            sel = [b for b in bounds if b["phase"] == ph]
            if sel:
                per_phase[ph] = {g: agg(sel, g) for g in GROUPS}
        still = [b for b in bounds if b["right_arm"]["teacher_move"] < 0.005]
        parents.append(dict(seed=row["seed"], generation=row["generation"], group="focus 3" if row["seed"] in FOCUS else "other 66", handover_boundary_s=round(start * td.DT, 2), boundaries=len(ts),
                            all={g: agg(bounds, g) for g in GROUPS}, per_phase=per_phase, first_boundary=bounds[0],
                            right_arm_when_teacher_waits=dict(boundaries=len(still), predicted_move_mean=round(float(np.mean([b["right_arm"]["predicted_move"] for b in still])), 4) if still else None,
                                                              predicted_move_max=round(max(b["right_arm"]["predicted_move"] for b in still), 4) if still else None),
                            right_gripper_max_abs_err_k24=round(max(abs(b["right_gripper_k24"][0] - b["right_gripper_k24"][1]) for b in bounds), 3), boundaries_detail=bounds))

    def group(items):
        g = lambda f: [f(p) for p in items]
        med = lambda xs: round(float(np.median(xs)), 4)
        la = [p["per_phase"]["left_approach"]["left_arm"] for p in items if "left_approach" in p["per_phase"]]
        return dict(parents=len(items), left_arm_mae_median=med(g(lambda p: p["all"]["left_arm"]["mae"])), left_arm_mae_max=max(g(lambda p: p["all"]["left_arm"]["mae"])),
                    left_arm_max_err_median=med(g(lambda p: p["all"]["left_arm"]["max_err"])), right_arm_mae_median=med(g(lambda p: p["all"]["right_arm"]["mae"])),
                    right_arm_mae_max=max(g(lambda p: p["all"]["right_arm"]["mae"])),
                    left_approach_progress_median_of_parents=None if not la else round(float(np.median([x["progress_median"] for x in la if x["progress_median"] is not None])), 2),
                    parents_with_left_approach_progress_below_half=sum(x["progress_median"] is not None and x["progress_median"] < 0.5 for x in la),
                    first_boundary_left_arm_progress_median=round(float(np.median([p["first_boundary"]["left_arm"]["progress"] for p in items if p["first_boundary"]["left_arm"]["progress"] is not None] or [np.nan])), 2),
                    parents_first_boundary_left_progress_below_half=sum(p["first_boundary"]["left_arm"]["progress"] is not None and p["first_boundary"]["left_arm"]["progress"] < 0.5 for p in items),
                    first_boundary_right_gripper_abs_err_median=med(g(lambda p: abs(p["first_boundary"]["right_gripper_k24"][0] - p["first_boundary"]["right_gripper_k24"][1]))),
                    parents_first_boundary_right_gripper_err_above_0p2=sum(abs(p["first_boundary"]["right_gripper_k24"][0] - p["first_boundary"]["right_gripper_k24"][1]) > 0.2 for p in items),
                    right_arm_predicted_move_when_teacher_waits_median=med([p["right_arm_when_teacher_waits"]["predicted_move_mean"] for p in items if p["right_arm_when_teacher_waits"]["predicted_move_mean"] is not None] or [0.0]),
                    left_gripper_mae_median=med(g(lambda p: p["all"]["left_gripper"]["mae"])), right_gripper_mae_median=med(g(lambda p: p["all"]["right_gripper"]["mae"])))

    # loader path against the runner path on three fixed reset observations
    picked = train9()
    fixed = [5000, next(p["seed"] for p in picked if "G000 first" in p["rule"]), next(p["seed"] for p in picked if "G001 first" in p["rule"])]
    correspondence = {}
    for seed in fixed:
        demo = ft.demonstration(seed)
        meta = demo["metadata"]
        ctx = make_task_episode(seed, meta["requested_initial_state"]["q"], meta["sampling"]["block_xy_yaw"])
        i = demo["index"]
        ep = replay.episodes[i]
        chunk, anchor, q_live, command, trace = fc.runner_prediction(policy, ctx, demo["integration_state"][0], {s: ep["rgb"][s][0] for s in ("left", "right")})
        item = replay[first_index[i]]
        loader = fc.predict(policy, [item])[0]
        correspondence[str(seed)] = dict(runner_q_equals_stored_q=bool(np.array_equal(q_live.astype(np.float32), ep["state"][0])),
                                         runner_anchor_equals_loader_anchor=bool(np.array_equal(anchor, item["observation.measured_q"].numpy())),
                                         chunk_runner_vs_loader_max_abs_diff=float(np.abs(chunk - loader).max()), executed_command_equals_chunk_k0=bool(np.array_equal(command, chunk[0])),
                                         projected_at_k0=bool(any(trace["projection_mask"])))
    data = dict(run_id=RUN, mode="eval, latent zero; inputs = two wrist RGB + q_state; C25-boundary teacher observations from t = 0 to the RECEIVE hand-over boundary; k = 0..24",
                units="arm rad, gripper opening fraction", moving_threshold_rad=MOVING, hand_over_boundary_notes=note, parents_analysed=len(parents),
                groups={"focus 3": group([p for p in parents if p["group"] == "focus 3"]), "other 66": group([p for p in parents if p["group"] == "other 66"]),
                        "other G000": group([p for p in parents if p["group"] == "other 66" and p["generation"] == "G000"]),
                        "other G001": group([p for p in parents if p["group"] == "other 66" and p["generation"] == "G001"])},
                loader_runner_correspondence=correspondence, parents=parents)
    target.write_text(json.dumps(data, indent=2))
    print(json.dumps(dict(notes=note, groups=data["groups"], correspondence=correspondence), indent=1))
    for p in parents:
        if p["seed"] in {x["seed"] for x in picked}:
            f = p["first_boundary"]
            print(p["seed"], p["group"], "L mae", p["all"]["left_arm"]["mae"], "max", p["all"]["left_arm"]["max_err"], "| first boundary L move teacher/pred/progress", f["left_arm"]["teacher_move"], f["left_arm"]["predicted_move"],
                  f["left_arm"]["progress"], "| R grip k24 tgt/pred", f["right_gripper_k24"], "| approach progress", p["per_phase"].get("left_approach", {}).get("left_arm", {}).get("progress_median"))
    print(f"[written] {target}")


# ----------------------------------------------------------------------------------------------
# run: nine original-reset rollouts
# ----------------------------------------------------------------------------------------------

def identity():
    import arm_output_trial as trial
    got = {k: sha(p / "model.safetensors") for k, p in MODELS.items()}
    ref = load(VAL_SCREEN / "protocol.json")["identity"]["models"]
    want = dict(START=ref["START"], RELEASE_B=ref["RELEASE_B"], E=ref["E"])
    detail = {k: dict(path=str(p), model=got[k], config=sha(p / "config.json"), normalization=sha(p.parent / "normalization.npz")) for k, p in MODELS.items()}
    if any(detail[k][f] != want[k][f] for k in detail for f in ("model", "config", "normalization")):
        raise RuntimeError("a read-only model / config / normalization differs from the Val screen protocol identity")
    picked = train9()
    resets = {}
    for p in picked:
        meta = load(Path(manifest()[p["manifest_index"]]["path"]) / "metadata.json")
        resets[str(p["seed"])] = hashlib.sha256(json.dumps([meta["requested_initial_state"]["q"], meta["sampling"]["block_xy_yaw"]], sort_keys=True).encode()).hexdigest()
    body = dict(run_id=RUN, models=detail, manifest_sha256=sha(td.DATASET / "manifest.json"), train9=picked, resets=resets, sources={p: sha(FACTORY_ROOT / p) for p in SOURCES},
                contract=rb.CONTRACT_TEXT["REPAIRED_RECEIVE"],
                execution=dict(chain="START -> RECEIVE_E -> RELEASE_B", chunk=50, prefix=K, control_hz=round(1 / td.DT), arm_bound_policy="project", ensemble="off", smoothing="off",
                               horizon_steps=rc.HORIZON, torch_seed=4100, policy_inputs=["left / right wrist RGB", "measured q_state"]),
                backend=trial.environment_record(), environment={k: __import__("os").environ.get(k) for k in rk.ENV_KEYS})
    body["sha256"] = hashlib.sha256(json.dumps(body, sort_keys=True).encode()).hexdigest()
    return body


def case_dir(ident, seed):
    return run_dir() / "runs" / ident["sha256"][:12] / f"s{seed}"


def complete(out, ident, seed):
    try:
        r, meta = load(out / "result.json"), load(out / "metadata.json")
    except (FileNotFoundError, json.JSONDecodeError):
        return None
    need = ("task_success", "first_failure_stage", "moments", "child_settings", "phases", "projection", "events", "start_segment")
    ok = (r.get("identity_sha256") == ident["sha256"] and r.get("case") == [seed] and r.get("job") == RUN and meta.get("complete") and r.get("boundary_contract") == "repaired"
          and all(k in r for k in need) and all((out / f).exists() for f in ("episode.npz", "action_trace.json", "task_check_rows.json", "predicted_chunks.npz")))
    return r if ok else None


def start_segment(out, seed, result):
    """START's own stretch next to the stored demonstration of the same parent (reference trajectory only, not the only right
    action once the states differ): per C25 boundary the state difference and the executed 25 commands."""
    import frozen_transfer_check as ft
    demo = ft.demonstration(seed)
    with np.load(out / "episode.npz", allow_pickle=False) as a:
        q, cmd = a["q_state"].astype(np.float64), a["q_cmd"].astype(np.float64)
    rows = load(out / "task_check_rows.json")
    trace = load(out / "action_trace.json")
    entry = next((s["step"] for s in result["switches"] if s["to"] == "RECEIVE"), None)
    last = len(cmd) if entry is None else entry
    L, R = GROUPS["left_arm"], GROUPS["right_arm"]
    bounds = []
    for b in range(0, last, K):
        if b + 1 > len(cmd) or b >= len(demo["q_cmd"]):
            break
        n = min(K, len(cmd) - b, len(demo["q_cmd"]) - b)
        mine, teach = cmd[b:b + n], demo["q_cmd"][b:b + n]
        dq = q[b] - demo["q_state"][b]
        row = dict(t=round(b * td.DT, 2), executed=n, teacher_phase=demo["phases"][b], projected_steps=int(sum(any(t.get("projection_mask", [])) for t in trace[b:b + n])),
                   state_minus_teacher=dict(left_arm=round(float(np.abs(dq[L]).max()), 4), right_arm=round(float(np.abs(dq[R]).max()), 4), left_gripper=round(float(dq[td.LEFT_GRIPPER]), 3),
                                            right_gripper=round(float(dq[td.RIGHT_GRIPPER]), 3)))
        for g, cols in GROUPS.items():
            want, got = teach[-1][cols] - demo["q_state"][b][cols], mine[-1][cols] - q[b][cols]
            row[g] = dict(command_minus_teacher_max=round(float(np.abs(mine - teach)[:, cols].max()), 4), student_move=round(float(np.linalg.norm(got)), 4), teacher_move=round(float(np.linalg.norm(want)), 4),
                          progress=None if np.linalg.norm(want) <= MOVING else round(float(got @ want / (want @ want)), 2))
        row["right_gripper_cmd_end"] = [round(float(mine[-1][td.RIGHT_GRIPPER]), 3), round(float(teach[-1][td.RIGHT_GRIPPER]), 3)]
        bounds.append(row)
    left_d = [r["gripper_block_distance_m"]["left"] for r in rows]
    block0 = np.array(rows[0]["block_pos"])
    return dict(boundaries=bounds, left_tcp_block_min_mm=round(1000 * min(left_d), 1), left_tcp_block_last_mm=round(1000 * left_d[-1], 1),
                left_pad_max_N=round(max(max(r["pad_force_N"]["left"]) for r in rows), 2), left_pad_steps_one_pad=int(sum(max(r["pad_force_N"]["left"]) > 0 and min(r["pad_force_N"]["left"]) == 0 for r in rows)),
                left_pad_steps_both_pads=int(sum(min(r["pad_force_N"]["left"]) > 0 for r in rows)), block_moved_before_lift_mm=round(1000 * float(max(np.linalg.norm(np.array(r["block_pos"][:2]) - block0[:2]) for r in rows[:last + 1])), 1),
                right_gripper_measured_min=round(float(q[:last + 1, td.RIGHT_GRIPPER].min()), 3), right_gripper_measured_at_reset=round(float(q[0, td.RIGHT_GRIPPER]), 3),
                first_boundary_with_command_departure=next((dict(t=r["t"], state_minus_teacher=r["state_minus_teacher"], left_arm=r["left_arm"], right_arm=r["right_arm"], right_gripper_cmd_end=r["right_gripper_cmd_end"])
                                                            for r in bounds if r["left_arm"]["command_minus_teacher_max"] > 0.05 or r["right_arm"]["command_minus_teacher_max"] > 0.05
                                                            or abs(r["right_gripper_cmd_end"][0] - r["right_gripper_cmd_end"][1]) > 0.1), None))


def one(args):
    ident = identity()
    if ident["sha256"] != args.identity:
        raise RuntimeError("identity changed between the parent process and this case")
    parent = next(p for p in ident["train9"] if p["seed"] == args.seed)
    out = case_dir(ident, args.seed)
    if out.exists():
        raise FileExistsError(f"{out} exists; nothing is overwritten")
    out.parent.mkdir(parents=True, exist_ok=True)
    rc.MODELS["RECEIVE"] = MODELS["E"]
    loaded, original = {}, rc.load_policy

    def remember(name):
        got = original(name)
        loaded[name] = got[0]
        return got

    rc.load_policy = remember
    started = time.time()
    try:
        result = rc.execute(out, args.seed, ("START", "RECEIVE", "B"), None, f"{LABEL}: Train parent {args.seed} START -> RECEIVE_E -> RELEASE_B", contract="repaired")
    finally:
        rc.load_policy = original
    rows = load(out / "task_check_rows.json")
    trace = load(out / "action_trace.json")
    with np.load(out / "episode.npz", allow_pickle=False) as a:
        states, q = a["integration_state"].copy(), a["q_state"].copy()
    rgb = {s: np.load(out / f"cameras/{s}_wrist_rgb.npz")["rgb"] for s in ("left", "right")}

    def moment(step):
        if step is None or step >= len(rgb["left"]):
            return None
        return dict(step=int(step), sim_time_s=round(float(states[step][0]), 2), state_sha256=rk.array_sha(states[step]), q_sha256=rk.array_sha(q[step]),
                    rgb_sha256=dict(left=rk.array_sha(rgb["left"][step]), right=rk.array_sha(rgb["right"][step])))

    entry = next((s for s in result["switches"] if s["to"] == "RECEIVE"), None)
    gate = next((s for s in result["switches"] if s["to"] == "B"), None)
    success, fault = result["task_success"], result["first_fault"]
    stage = None if success else "START" if entry is None else "RECEIVE" if gate is None else "RELEASE_B"
    kind = None if success else ("fault" if fault is not None else "task not completed within the 40 s horizon")
    end = len(rgb["left"]) if entry is None else entry["step"]
    red = {s: [int(red_mask(f).sum()) for f in rgb[s][:end]] for s in rgb}
    result.update(job=RUN, kind="full", classification=LABEL, split="train", case=[args.seed], parent=parent, receive_model="E", receive_name="RECEIVE_ONSET_RECOVERY", repeat=1,
                  identity_sha256=ident["sha256"], chain="START -> RECEIVE_ONSET_RECOVERY -> RELEASE_B", handover_event=result["handover_s"] is not None,
                  completed_40s=result["termination"] == "horizon_completed", first_failure_stage=stage, first_failure_kind=kind,
                  transfer=None if entry is None else r3.arm_motion(out, entry["step"]),
                  counts=dict(commands_dispatched=sum(t.get("ctrl_applied") is not None for t in trace), completed_transitions=result["completed_transitions"],
                              sim_elapsed_s=result["last_completed_state_sim_s"], wall_s=round(time.time() - started, 1)),
                  moments=dict(reset=moment(0), receive_entry=moment(entry["step"] if entry else None), release_b_entry=moment(gate["step"] if gate else None)),
                  start_segment=start_segment(out, args.seed, result),
                  start_wrist_red_px=dict(left_at_reset=red["left"][0], left_first_visible_s=next((round(k * td.DT, 2) for k, c in enumerate(red["left"]) if c > 0), None), left_max=max(red["left"]),
                                          left_visible_share=round(float(np.mean([c > 0 for c in red["left"]])), 3), right_max=max(red["right"])),
                  contract_checks=rb.contract_checks(result), phases=r2.phase_details(result, rows, result["receive_entry_s"] or 0.0),
                  child_settings=rk.child_settings({("RECEIVE_E" if k == "RECEIVE" else "RELEASE_B" if k == "B" else k): p for k, p in loaded.items()}))
    (out / "result.json").write_text(json.dumps(result, indent=2, default=str))
    print(json.dumps(dict(seed=args.seed, group=parent["group"], events={k: round(v, 2) for k, v in result["events"].items()}, receive_entry=result["receive_entry_s"],
                          ready=result["phases"]["semantic_ready_s"], release_b=result["gate_s"], termination=result["termination"], first_failure_stage=stage, first_failure_kind=kind,
                          task_success=success, final_support=result["final_support"]), default=str))


def table_row(r):
    row = rk.table_row(r)
    ev, seg = r["events"], r["start_segment"]
    row.update(group=r["parent"]["group"], generation=r["parent"]["generation"], rule=r["parent"]["rule"], reach_s=ev.get("reach"), left_grasp_s=ev.get("left_grasp"),
               completed_40s=r["completed_40s"], counts=r["counts"], receive_entered=r["receive_entered"], release_b_entered=r["gate_reached"], handover_event=r["handover_event"],
               start_segment={k: seg[k] for k in seg if k != "boundaries"}, start_wrist_red_px=r["start_wrist_red_px"])
    return row


def tally(results):
    out = {}
    for name in ("focus 3", "additional 6"):
        rs = [r for r in results if r["parent"]["group"] == name]
        entered, reached = [r for r in rs if r["receive_entered"]], [r for r in rs if r["gate_reached"]]
        out[name] = dict(runs=len(rs), reach=sum("reach" in r["events"] for r in rs), left_grasp=sum("left_grasp" in r["events"] for r in rs), pick_lift=sum(r["pick_lift_s"] is not None for r in rs),
                         receive_entered=len(entered), receive_ready_of_entered="N/A (RECEIVE not entered)" if not entered else f"{sum(r['phases']['semantic_ready_s'] is not None for r in entered)}/{len(entered)}",
                         release_b_success_of_reached="N/A (RELEASE_B not reached)" if not reached else f"{sum(r['task_success'] for r in reached)}/{len(reached)}",
                         handover_event=sum(r["handover_event"] for r in rs), task_success=sum(r["task_success"] for r in rs),
                         first_failure_stage={k: sum(r["first_failure_stage"] == k for r in rs) for k in ("START", "RECEIVE", "RELEASE_B")},
                         first_failure_kind={k: sum(r["first_failure_kind"] == k for r in rs) for k in ("fault", "task not completed within the 40 s horizon")},
                         start_projection=dict(runs=sum(r["projection"]["START"]["projected_steps"] > 0 for r in rs), steps_total=sum(r["projection"]["START"]["projected_steps"] for r in rs),
                                               longest_s_max=max((r["projection"]["START"]["longest_s"] for r in rs), default=0.0),
                                               max_raw_excess_rad=round(max((r["projection"]["START"]["max_raw_excess_rad"] for r in rs), default=0.0), 4),
                                               axes=sorted({a for r in rs for a in r["projection"]["START"]["per_axis"]})))
    return out


def run(args):
    for name in ("coverage.json", "visibility.json", "offline.json"):
        if not (run_dir() / name).exists():
            raise FileNotFoundError(f"{name} is missing: run `{name.split('.')[0]}` first")
    ident = identity()
    plan = [p["seed"] for p in ident["train9"]]
    (run_dir() / "logs").mkdir(exist_ok=True)
    proto = run_dir() / "protocol.json"
    if proto.exists():
        stored = load(proto)
        if stored["identity"]["sha256"] != ident["sha256"] or stored["cases"] != plan:
            raise RuntimeError("protocol.json of this job has another identity or case list: nothing is run, nothing is overwritten")
        print(f"[resumed job] protocol of {stored['fixed_before_execution']} (identity {ident['sha256'][:12]}); completed cases of this job are not run again and are not new runs")
    else:
        proto.write_text(json.dumps(dict(
            run_id=RUN, name=LABEL, fixed_before_execution=time.strftime("%Y-%m-%dT%H:%M:%S%z"), identity=ident, cases=plan,
            selection="5000 / 5002 / 5008, then for G000 and for G001 the first / middle (floor((n-1)/2)) / last of the other Train parents in manifest order; fixed before the first rollout; "
                      "a fixed development diagnostic, not a sample that estimates a Train69 success rate",
            order="as listed; sequential, one new process per case", budget="nine new original-reset runs, one each; no repeat, no replacement, no Val / Test run",
            chain="START -> RECEIVE_E -> RELEASE_B for every case (no RECEIVE_C); the later policies run only if their unchanged gates are reached",
            not_called=["select", "teacher", "train", "any optimizer step", "Val / Test evaluation"], training=False, teacher_collection=False, val_test=False, hardware_access=False), indent=2))
    status = []
    for seed in plan:
        out = case_dir(ident, seed)
        if complete(out, ident, seed) is not None:
            status.append(dict(case=seed, status="completed (same job resumed: stored case of this identity, not a new run)"))
            print(f"[stored case of this job] {seed}")
            continue
        if out.exists():
            status.append(dict(case=seed, status="NOT RUN: an incomplete folder of this identity exists; neither reused nor overwritten", path=str(out)))
            print(f"[not run] {seed}: incomplete folder {out}")
            continue
        done = subprocess.run([sys.executable, str(Path(__file__).resolve()), "one", "--seed", str(seed), "--identity", ident["sha256"]], capture_output=True, text=True)
        (run_dir() / "logs" / f"{ident['sha256'][:12]}_s{seed}.log").write_text(done.stdout + "\n--- stderr ---\n" + done.stderr)
        ok = done.returncode == 0 and complete(out, ident, seed) is not None
        status.append(dict(case=seed, returncode=done.returncode, status="completed (new run)" if ok else "executor_error (not a task result)", **({} if ok else dict(stderr_tail=done.stderr[-600:]))))
        line = [x for x in done.stdout.splitlines() if x.startswith("{")]
        print(line[-1][:700] if ok and line else f"[executor_error, not a task failure] {seed}: returncode {done.returncode}: {done.stderr[-300:]}", flush=True)
    results = [r for seed in plan if (r := complete(case_dir(ident, seed), ident, seed)) is not None]
    off = {p["seed"]: p for p in load(run_dir() / "offline.json")["parents"]}
    link = []
    for r in results:
        dep = r["start_segment"]["first_boundary_with_command_departure"]
        o = off[r["seed"]]
        same = None if dep is None else next((b for b in o["boundaries_detail"] if abs(b["t"] - dep["t"]) < 1e-6), None)
        link.append(dict(seed=r["seed"], group=r["parent"]["group"], task_success=r["task_success"], pick_lift=r["pick_lift_s"] is not None, first_failure_stage=r["first_failure_stage"],
                         student_first_command_departure=dep, teacher_observation_prediction_at_that_boundary=None if same is None else {g: same[g] for g in ("left_arm", "right_arm")} | dict(
                             right_gripper_k24=same["right_gripper_k24"]),
                         teacher_observation_whole_pick=dict(left_arm=o["all"]["left_arm"], right_arm=o["all"]["right_arm"], first_boundary_left=o["first_boundary"]["left_arm"],
                                                             first_boundary_right_gripper_k24=o["first_boundary"]["right_gripper_k24"])))
    settings = [r["child_settings"] for r in results]
    (run_dir() / "results.json").write_text(json.dumps(dict(
        run_id=RUN, note=f"{LABEL}; switches read SIM ground truth; fixed weights; nine fixed Train resets, one run each; not a Train69 success rate; not added to any earlier table",
        identity_sha256=ident["sha256"], identity_unchanged_during_run=identity()["sha256"] == ident["sha256"], planned_cases=len(plan), completed_cases=len(results),
        executor_errors=[s for s in status if not s["status"].startswith("completed")], execution_status=status, tally=tally(results), teacher_vs_student=link,
        child_settings=dict(first=settings[0] if settings else None, pids_distinct=len({s["pid"] for s in settings}), all_policies_eval=all(not v for s in settings for v in s["policy_training_flag"].values()),
                            torch_initial_seeds=sorted({s["rng"]["torch_initial_seed"] for s in settings})),
        table=[table_row(r) for r in results], results=results), indent=2, default=str))
    print(f"[written] {run_dir() / 'results.json'}: {len(results)} of {len(plan)} cases completed, {len([s for s in status if not s['status'].startswith('completed')])} executor error(s) / not run; "
          f"identity {ident['sha256'][:12]}")
    summary(args)


def summary(args=None):
    path = run_dir() / "results.json"
    if not path.exists():
        print(f"START-PICK D1 ({RUN}): unavailable, no results.json")
        return
    data = load(path)
    fmt = lambda v: "-" if v is None else (f"{v:.2f}" if isinstance(v, float) else str(v))
    print(f"\n=== START-PICK D1 ({RUN}) — {LABEL}; 전환은 SIM 정답 기반 ===")
    print(f"고정 Train9 원래 reset, START -> RECEIVE_E -> RELEASE_B 각 1회, 가중치 고정. Train69 성공률 추정이 아니다. "
          f"완료 {data['completed_cases']}/{data['planned_cases']}건, 실행기 오류 {len(data['executor_errors'])}건, identity {data['identity_sha256'][:12]}.")
    print("\n| 묶음 | 실행 | reach | left_grasp | pick_lift | RECEIVE 진입 | 수신 준비(진입 중) | RELEASE_B 도달 중 성공 | 전체 유효 | 첫 실패 START/RECEIVE/RELEASE_B |\n|---|---|---|---|---|---|---|---|---|---|")
    for k, v in data["tally"].items():
        f = v["first_failure_stage"]
        print(f"| {k} | {v['runs']} | {v['reach']} | {v['left_grasp']} | {v['pick_lift']} | {v['receive_entered']} | {v['receive_ready_of_entered']} | {v['release_b_success_of_reached']} | "
              f"{v['task_success']}/{v['runs']} | {f['START']}/{f['RECEIVE']}/{f['RELEASE_B']} |")
    print("\n| parent | 묶음 | reach | pick_lift | RECEIVE 진입 | handover | 첫 fault | 첫 실패 정책 | 왼손-블록 최소 mm | 성공 |\n|---|---|---|---|---|---|---|---|---|---|")
    for t in data["table"]:
        print(f"| {t['seed']} | {t['group']} {t['generation']} | {fmt(t['reach_s'])} | {fmt(t['pick_lift_s'])} | {fmt(t['receive_entry_s'])} | {fmt(t['handover_event_s'])} | {fmt(t['first_fault_s'])} | "
              f"{t['first_failure_stage'] or '-'} | {t['start_segment']['left_tcp_block_min_mm']} | {'예' if t['task_success'] else '아니오'} |")
    print(f"  결과: {path}\n  보고서: {run_dir() / 'REPORT.md'}\n  분석: coverage.json, visibility.json, offline.json (같은 폴더)")


def video(args):
    import recovery_trial as rt
    ident = identity()
    data = load(run_dir() / "results.json")
    extra = [t for t in data["table"] if t["group"] == "additional 6"]
    picks = [("first_failure_additional_train", next((t for t in extra if not t["task_success"]), None)), ("first_pick_lift_additional_train", next((t for t in extra if t["pick_lift_s"] is not None), None))]
    done = set()
    for tag, t in picks:
        if t is None:
            print(f"[no such category] {tag}")
            continue
        if t["seed"] in done:
            print(f"[same parent as an earlier video] {tag}: {t['seed']}")
            continue
        done.add(t["seed"])
        out = run_dir() / "videos" / f"{tag}_train{t['seed']}_{ident['sha256'][:12]}.mp4"
        if out.exists():
            print(f"[exists] {out}")
            continue
        r = complete(case_dir(ident, t["seed"]), ident, t["seed"])
        row = rt.teacher_row(f"STUDENT / SIM-ORACLE-GATED / Train {t['seed']}: {r['chain']}", Path(r["path"]))
        ev = dict(r["events"])
        ev.update({f"SWITCH->{'RELEASE_B' if s['to'] == 'B' else s['to']}": s["sim_time_s"] for s in r["switches"]})
        row["events"] = dict(sorted(ev.items(), key=lambda kv: kv[1]))
        row["end_text"] = (f"ENDED at SIM {r['last_completed_state_sim_s']:.2f}s: {r['termination']}" + ("" if r["task_success"] else f" (first failure in {r['first_failure_stage']})"),
                           "VALID HANDOVER, right hand holds" if r["task_success"] else f"NO VALID HANDOVER; final support: {r['final_support']}")
        rt.render([row], out, f"{LABEL} / parent {t['seed']} ({tag.replace('_', ' ')})", 2000, False, right_gripper=True)
        print(f"[written] {out}")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    for name in ("coverage", "visibility", "offline", "run", "summary", "video"):
        sub.add_parser(name)
    o = sub.add_parser("one")
    o.add_argument("--seed", type=int, required=True)
    o.add_argument("--identity", required=True)
    args = parser.parse_args(argv)
    import arm_output_trial  # noqa: F401
    dict(coverage=coverage, visibility=visibility, offline=offline, run=run, one=one, summary=summary, video=video)[args.command](args)


if __name__ == "__main__":
    main()
