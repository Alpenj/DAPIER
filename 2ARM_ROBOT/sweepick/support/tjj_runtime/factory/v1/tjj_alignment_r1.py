#!/usr/bin/env python3
"""TJJ SYSTEM-ALIGNMENT R1 / work6: the 3-view observation path wired end to end, top-view recovery for stored episodes,
depth perception checked in SIM, and a PLACE attempt that is separate from RELEASE_B (SIM / code only; no hardware, no
training run, no Val / Test in training or normalization).

  inventory   which stored episodes hold which cameras (shape, dtype, capture time, K / T, full integration state)
  sidecar     top-view RGB + depth for the correction episodes that have none, rendered from their stored integration
              states on a separate MjData (no mj_step, nothing of the original episode is changed)
  connect     collector -> recorder -> dataset -> batch -> policy input features -> vision backbone -> inference with three
              views: checkpoint migration, CUDA forward with a backbone hook, a three-update training smoke (no checkpoint),
              a one-second collection / inference smoke
  perceive    object position and area-clear from the top depth on stored Train reset frames, against the stored state
  place       one PLACE attempt from the stored end state of a mixed handover: the right arm sets the block down in a fixed
              target region, opens and retreats; scored by a SIM-truth scorer and, apart, by the sensor-based observer
  all         every step that is not complete yet

The top camera of the SIM is `os30a_UNVERIFIED` (monochrome intensity in three channels + metric depth): SIM_ONLY /
REAL_MODALITY_UNVERIFIED. A forward pass is a connection check, never a recognition or task result.
"""
import argparse
import copy
import hashlib
import json
import os
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import train_rollout_diagnostic as td   # noqa: E402
import receive_candidate as rc   # noqa: E402
import receive_r3_recheck as rk   # noqa: E402
import start_pick_d1 as d1   # noqa: E402
import start_pick_r1 as p1   # noqa: E402
import receive_bridge_r1 as br   # noqa: E402
import constraint_causal_c0 as c0   # noqa: E402
import receive_expand_r1 as ex   # noqa: E402
import server_s0 as s0   # noqa: E402
import tjj_perception as tp   # noqa: E402
import tjj_task_manager as tm   # noqa: E402

FACTORY_ROOT = td.FACTORY_ROOT
RUN = "tjj-system-alignment-r1-work6-20261006-01"
TITLE = "TJJ SYSTEM-ALIGNMENT R1 / work6 / 3-view path, top-view recovery, PLACE"
TOP, TOP_KEY = "os30a_UNVERIFIED", "observation.images.top"
CAMERAS = ("left_wrist_rgb", "right_wrist_rgb", TOP)
MODALITY = "SIM_ONLY / REAL_MODALITY_UNVERIFIED (SIM top view: monochrome intensity replicated in three channels, revision os30a-mono-v1, provisional extrinsics / fovy)"
TABLE_Z = 0.0
SOURCE_REGION = tm.Region("source", "world", (0.20, 0.00), 0.06, TABLE_Z)
TARGET_REGION = tm.Region("place-target-fixed-v0", "world", (0.22, -0.18), 0.05, TABLE_Z, fixed=True)
TARGET_REGION_V1 = tm.Region("place-target-fixed-v1", "world", (0.28, -0.06), 0.04, TABLE_Z, fixed=True)     # inside the top camera's view of the table (v0's target was not)
PLACE_VERSIONS = dict(v0=dict(region=TARGET_REGION, retreats=((0.06, 0.03),)), v1=dict(region=TARGET_REGION_V1, retreats=((0.06, 0.03), (0.03, 0.05), (0.0, 0.06), (0.0, 0.03))))
PLACE_SOURCE = ex.run_dir() / "runs" / "collect_ea57389b3edb" / "collect_5119"
ROI = ((0.08, 0.45), (-0.30, 0.30))

load, sha, array_sha = rk.load, rk.sha, rk.array_sha


def run_dir():
    return FACTORY_ROOT / "evaluation" / RUN


def scene_model():
    from integration_scenes import task_env
    return task_env("desk", grippers="both").model


def top_sensor(width=160, height=120):
    from sim_data_factory.sensor import SimulatedCameraSensor
    return SimulatedCameraSensor(width=width, height=height, camera_ids=CAMERAS, depth_camera_ids=(TOP,))


def render(model, data, sensor, state):
    import mujoco
    mujoco.mj_setState(model, data, np.asarray(state), int(mujoco.mjtState.mjSTATE_INTEGRATION))
    mujoco.mj_forward(model, data)
    return sensor.observe(model, data)


# ----------------------------------------------------------------------------------------------
# Inventory
# ----------------------------------------------------------------------------------------------

def camera_row(path):
    path = Path(path)
    row = dict(path=str(path), cameras={}, integration_state=None)
    try:
        with np.load(path / "episode.npz", allow_pickle=False) as a:
            row["integration_state"] = list(a["integration_state"].shape)
            times, n = a["sim_time"][:-1].copy(), len(a["q_cmd"])
    except FileNotFoundError:
        row["missing"] = "episode.npz"
        return row
    row["transitions"] = n
    for cam in CAMERAS:
        f = path / "cameras" / f"{cam}.npz"
        if not f.exists():
            row["cameras"][cam] = None
            continue
        with np.load(f, allow_pickle=False) as c:
            rgb = c["rgb"]
            first = rgb[0]
            row["cameras"][cam] = dict(rgb=list(rgb.shape), dtype=str(rgb.dtype), depth_m=list(c["depth_m"].shape) if "depth_m" in c.files else None,
                                       gray3=bool(np.array_equal(first[..., 0], first[..., 1]) and np.array_equal(first[..., 1], first[..., 2])),
                                       K=bool("K" in c.files), T_world_camera_optical=bool("T_world_camera_optical" in c.files),
                                       capture_time_equals_precommand_state_time=bool(len(c["capture_time"]) == n and np.array_equal(c["capture_time"], times)))
    meta = load(path / "metadata.json") if (path / "metadata.json").exists() else {}
    row.update(observation_revision=meta.get("observation_revision"), camera_ids_in_metadata=meta.get("camera_ids"), student_allowlist=meta.get("student_allowlist"))
    return row


def correction_sets():
    """Every correction episode the RECEIVE trainings really read, and the 15 G corrections, with their training rows."""
    st, views, index = br.datasets()
    out = []
    for name, label in (("existing", "earlier corrections (re-cut)"), ("actual_entry", "R2 actual-entry corrections"), ("onset", "R3 onset correction"), ("bridge", "Bridge corrections")):
        if views.get(name) is None:
            continue
        for r in views[name].rows:
            out.append(dict(group=label, id=str(r.get("id")), seed=r["seed"], path=r["path"], rows=list(r["receive"]), view_key=r["view_key"]))
    for e in load(ex.run_dir() / "training_manifest.json")["main_pool"]:
        out.append(dict(group="G corrections (EXPAND pool)", id=e["case"], seed=e["parent"], path=e["path"], rows=list(e["rows_in_episode"]), view_key=e["view_key"]))
    return out, views


def inventory(args=None):
    target = run_dir() / "inventory.json"
    if target.exists():
        return load(target)
    run_dir().mkdir(parents=True, exist_ok=True)
    man = d1.manifest()
    groups = {}
    for split, label in (("train", "Train69 demonstrations"), ("val", "development Val10 demonstrations (structure only; never used for training or normalization)")):
        groups[label] = [dict(seed=e["seed"], **camera_row(e["path"])) for e in man if e["split"] == split and (split != "train" or e["training_eligible"])]
    corrections, _ = correction_sets()
    for c in corrections:
        groups.setdefault(c["group"], []).append(dict(seed=c["seed"], id=c["id"], training_rows=c["rows"], **camera_row(c["path"])))
    student = [dict(case=c, **camera_row(ex.case_dir(load(ex.run_dir() / "identity_evaluate.json"), c))) for c in ex.eval_cases()[:4]]
    groups["student evaluation runs (first four of EXPAND, as examples)"] = student
    summary = {}
    for label, rows in groups.items():
        has = lambda r, cam: r["cameras"].get(cam) is not None
        summary[label] = dict(episodes=len(rows), parents=len({r["seed"] for r in rows if "seed" in r}), frames=int(sum(r.get("transitions", 0) for r in rows)),
                              with_top_view=sum(has(r, TOP) for r in rows), with_top_depth=sum(has(r, TOP) and r["cameras"][TOP]["depth_m"] is not None for r in rows),
                              with_both_wrists=sum(has(r, CAMERAS[0]) and has(r, CAMERAS[1]) for r in rows), with_full_integration_state=sum(r["integration_state"] is not None for r in rows),
                              top_is_gray3=sorted({r["cameras"][TOP]["gray3"] for r in rows if has(r, TOP)}),
                              capture_time_aligned=all(c["capture_time_equals_precommand_state_time"] for r in rows for c in r["cameras"].values() if c))
    real = FACTORY_ROOT / "reports/STATIC13-20260930-091453/A-os30a.registered-metadata.json"
    real_note = None
    if real.exists():
        h = load(real)["header"]
        real_note = dict(file=str(real), color=[h["color_width"], h["color_height"]], depth=[h["depth_width"], h["depth_height"]], point_cloud=[h["pc_width"], h["pc_height"]],
                         capture_complete=load(real)["capture_complete"], accepted_for_execution=load(real)["accepted_for_execution"],
                         note="one stored real OS30A capture attempt of 2026-09-30 (laptop): a colour stream and a depth stream exist; the capture helper timed out and the capture was not accepted. "
                              "It is not a calibration: K / T / depth registration of the real camera are unverified here")
    data = dict(run_id=RUN, written=time.strftime("%Y-%m-%dT%H:%M:%S%z"), modality=MODALITY, existing_preflight="no OS30A preflight process was running and no earlier inventory of this kind was found; "
                "the stored real capture of 2026-09-30 is referenced, not repeated", real_os30a_capture=real_note, test_split="not opened", summary=summary, groups=groups)
    target.write_text(json.dumps(data, indent=2, default=str))
    print(json.dumps(summary, indent=1))
    return data


# ----------------------------------------------------------------------------------------------
# Sidecar: top view from stored states
# ----------------------------------------------------------------------------------------------

def sidecar_path(item):
    return run_dir() / "sidecar" / f"{item['view_key']}_s{item['seed']}" / f"{TOP}.npz"


def sidecar(args=None):
    import mujoco
    target = run_dir() / "sidecar_manifest.json"
    if target.exists():
        return load(target)
    corrections, _ = correction_sets()
    model, sensor = scene_model(), top_sensor()
    data = mujoco.MjData(model)
    entries, started = [], time.time()
    try:
        for item in corrections:
            path, (a, b) = Path(item["path"]), item["rows"]
            out = sidecar_path(item)
            if (path / "cameras" / f"{TOP}.npz").exists():
                entries.append(dict(item, status="has its own top view; no sidecar needed"))
                continue
            with np.load(path / "episode.npz", allow_pickle=False) as e:
                states, times = e["integration_state"].copy(), e["sim_time"].copy()
            if b > len(states) - 1:
                entries.append(dict(item, status="UNRECOVERABLE: rows outside the stored states"))
                continue
            stored = {s: np.load(path / f"cameras/{s}_wrist_rgb.npz")["rgb"] for s in ("left", "right")}
            rgb, depth, checks = [], [], []
            K = T = None
            sample = sorted({a, (a + b) // 2, b - 1})
            for k in range(a, b):
                obs = render(model, data, sensor, states[k])
                rgb.append(obs[TOP]["rgb"])
                depth.append(obs[TOP]["depth_m"])
                if K is None:
                    K, T = obs[TOP]["K"], obs[TOP]["T_world_camera_optical"]
                elif not np.array_equal(T, obs[TOP]["T_world_camera_optical"]):
                    raise RuntimeError("the top camera moved")
                if k in sample:                                             # the wrist views rendered the same way must be the stored wrist frames
                    for s in ("left", "right"):
                        d = np.abs(obs[f"{s}_wrist_rgb"]["rgb"].astype(np.int16) - stored[s][k].astype(np.int16))
                        checks.append(dict(row=k, side=s, pixels_differing=int((d.max(-1) > 0).sum()), max_abs=int(d.max())))
            out.parent.mkdir(parents=True, exist_ok=True)
            np.savez_compressed(out, rgb=np.stack(rgb), depth_m=np.stack(depth).astype(np.float32), K=K, T_world_camera_optical=T, capture_time=times[a:b], rows=np.array([a, b]))
            entries.append(dict(item, status="rendered from the stored integration states", frames=b - a, sidecar=str(out), sidecar_sha256=sha(out),
                                source_files={f: sha(path / f) for f in ("episode.npz", "cameras/left_wrist_rgb.npz", "cameras/right_wrist_rgb.npz")},
                                wrist_rerender_check=dict(samples=len(checks), identical=sum(c["pixels_differing"] == 0 for c in checks), max_pixels_differing=max(c["pixels_differing"] for c in checks),
                                                          max_abs=max(c["max_abs"] for c in checks), rows=checks),
                                gray3=bool(np.array_equal(rgb[0][..., 0], rgb[0][..., 1])), depth_range_m=[float(np.min(depth)), float(np.max(depth))]))
            print(f"[sidecar] {item['group']} {item['id']}: {b - a} frames", flush=True)
    finally:
        sensor.close()
    done = [e for e in entries if e.get("sidecar")]
    manifest = dict(run_id=RUN, written=time.strftime("%Y-%m-%dT%H:%M:%S%z"), modality=MODALITY, camera=TOP,
                    method="for each training row the stored full integration state is set on a separate MjData, mj_forward, then the unchanged SimulatedCameraSensor renders the top view "
                           "(RGB + metric depth). No mj_step, no policy, no change of the stored episode, action or normalization",
                    renderer=dict(MUJOCO_GL=os.environ.get("MUJOCO_GL"), host=s0.host()), resolution=[160, 120],
                    counts=dict(episodes_checked=len(entries), sidecars=len(done), frames=int(sum(e["frames"] for e in done)), parents=len({e["seed"] for e in done}),
                                already_had_top_view=sum(e["status"].startswith("has") for e in entries), unrecoverable=sum(e["status"].startswith("UNRECOVERABLE") for e in entries),
                                by_group={g: dict(episodes=sum(e["group"] == g for e in done), frames=int(sum(e["frames"] for e in done if e["group"] == g)), parents=len({e["seed"] for e in done if e["group"] == g}))
                                          for g in dict.fromkeys(e["group"] for e in done)}),
                    wrist_rerender=dict(samples=int(sum(e["wrist_rerender_check"]["samples"] for e in done)), identical=int(sum(e["wrist_rerender_check"]["identical"] for e in done)),
                                        max_pixels_differing=max((e["wrist_rerender_check"]["max_pixels_differing"] for e in done), default=0), max_abs=max((e["wrist_rerender_check"]["max_abs"] for e in done), default=0),
                                        note="differences of +-1 in a few shadow pixels were seen before between two renders of one state; they are counted, not hidden"),
                    not_recovered="student evaluation runs are not needed for training and get no sidecar; they keep their states and can be rendered the same way",
                    seconds=round(time.time() - started, 1), entries=entries)
    target.write_text(json.dumps(manifest, indent=2, default=str))
    print(json.dumps(dict(counts=manifest["counts"], wrist=manifest["wrist_rerender"] | dict(note=None), seconds=manifest["seconds"]), default=str))
    return manifest


# ----------------------------------------------------------------------------------------------
# 3-view: dataset item, policy, collector, inference
# ----------------------------------------------------------------------------------------------

def image_tensor(image, device=None):
    import torch
    image = np.asarray(image)
    if image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("raw RGB uint8 required")
    rgb = torch.tensor(image, device=device).permute(2, 0, 1).float() / 255
    return (rgb - torch.tensor([.485, .456, .406], device=device)[:, None, None]) / torch.tensor([.229, .224, .225], device=device)[:, None, None]


class ThreeView:
    """A stored 2-view training view plus the top frames of the same rows: item = the unchanged item + the top image."""
    def __init__(self, view, tops):
        self.view, self.tops = view, tops
        for key, ep in view.episodes.items():
            if len(tops[key]) != len(ep["action"]):
                raise ValueError(f"view {key}: {len(tops[key])} top frames for {len(ep['action'])} rows")

    def __len__(self):
        return len(self.view)

    def __getitem__(self, index):
        item = self.view[index]
        key, t = self.view.indices[index]
        item[TOP_KEY] = image_tensor(self.tops[key][t])
        return item


def three_view_sets():
    """Train69 receiving view (top from the demonstrations' own files) and the G corrections (top from the sidecars)."""
    side = {e["view_key"]: e for e in load(run_dir() / "sidecar_manifest.json")["entries"] if e.get("sidecar")}
    st, views, index, _ = ex.datasets()
    man = d1.manifest()
    train_tops = {}
    for r in views["train"].rows:
        a, b = r["receive"]
        with np.load(Path(man[r["manifest_index"]]["path"]) / "cameras" / f"{TOP}.npz", allow_pickle=False) as c:
            train_tops[r["view_key"]] = c["rgb"][a:b]
    g_tops = {}
    for r in views["expand"].rows:
        e = side[r["view_key"]]
        if sha(e["sidecar"]) != e["sidecar_sha256"]:
            raise RuntimeError("a sidecar changed")
        g_tops[r["view_key"]] = np.load(e["sidecar"])["rgb"]
    return st, ThreeView(views["train"], train_tops), ThreeView(views["expand"], g_tops)


def three_view_policy(checkpoint):
    """The stored 2-view policy class and weights with a third camera in its input features. Returns (policy, migration record)."""
    from dapier_act_policy import ACTPolicy
    from lerobot.configs.types import PolicyFeature, FeatureType
    two = ACTPolicy.from_pretrained(Path(checkpoint), local_files_only=True, strict=True)
    cfg = copy.deepcopy(two.config)
    cfg.input_features = dict(cfg.input_features)
    cfg.input_features[TOP_KEY] = PolicyFeature(FeatureType.VISUAL, (3, 120, 160))
    three = type(two)(cfg)
    result = three.load_state_dict(two.state_dict(), strict=True)
    a, b = {n: tuple(p.shape) for n, p in two.named_parameters()}, {n: tuple(p.shape) for n, p in three.named_parameters()}
    record = dict(source=str(checkpoint), source_model_sha256=sha(Path(checkpoint) / "model.safetensors"), image_features=list(cfg.image_features), strict_load=str(result),
                  parameters_two_view=int(sum(p.numel() for p in two.parameters())), parameters_three_view=int(sum(p.numel() for p in three.parameters())),
                  same_names_shapes_and_order=bool(list(a.items()) == list(b.items())), new_parameters=sorted(set(b) - set(a)), missing_parameters=sorted(set(a) - set(b)),
                  why="the vision backbone is shared by all cameras and the camera tokens get a fixed (sinusoidal) 2-D position embedding, so a third camera adds tokens, not parameters",
                  meaning="every weight carries over; the encoder now sees one more block of tokens it was never trained with, so this is a starting point for training, not a usable 3-view policy",
                  optimizer="parameter names, shapes and order are the same, so a stored AdamW state can be mapped one to one; whether to carry it over is a training decision, not made here")
    return two, three, record


def backbone_counter(policy):
    calls = []
    hook = policy.model.backbone.register_forward_hook(lambda module, args, output: calls.append(dict(input=list(args[0].shape), device=str(args[0].device), feature_map=list(output["feature_map"].shape))))
    return calls, hook


def connect(args=None):
    import torch
    from torch.profiler import profile, ProfilerActivity
    from torch.utils.data import default_collate
    import arm_output_trial as trial
    import single_demo_trial as sd
    from sim_data_factory import student_eval
    from dapier_act_policy import MEASURED_Q_KEY
    target = run_dir() / "connect.json"
    if target.exists():
        return load(target)
    models = c0.bridge_models()
    st, train3, g3 = three_view_sets()
    out = dict(run_id=RUN, written=time.strftime("%Y-%m-%dT%H:%M:%S%z"), modality=MODALITY, key_order=["observation.images.left_wrist", "observation.images.right_wrist", TOP_KEY, "observation.state (q12)"])
    # 1) checkpoint migration for every model of the chain
    out["checkpoints"] = {}
    migrated = {}
    for name in ("START_FROZEN", "START_R1_E", "RECEIVE_FROZEN", "RECEIVE_BRIDGE_CTRL", "RECEIVE_BRIDGE_ADAPT", "RELEASE_B"):
        two, three, rec = three_view_policy(models[name]["path"])
        out["checkpoints"][name] = dict(stored_input_cameras=[k for k in two.config.image_features], stored_is_three_view=False, migration=rec)
        migrated[name] = (two, three)
    data_ckpt = ex.training_dir() / ex.CHECKPOINT
    two, three, rec = three_view_policy(data_ckpt)
    out["checkpoints"]["RECEIVE_DATA (EXPAND, not adopted)"] = dict(stored_input_cameras=list(two.config.image_features), stored_is_three_view=False, migration=rec)
    out["chain_status"] = "every stored checkpoint of the chain is 2-view (two wrist cameras); none was trained with the top view; the 3-view models made here are migrated starting points only"
    # 2) dataset -> batch -> backbone -> output on CUDA
    two, three = migrated["RECEIVE_FROZEN"]
    two, three = two.cuda().eval(), three.cuda().eval()
    three.check_normalization(st)
    items = [train3[0], train3[len(train3) // 2], g3[0], g3[len(g3) - 1]]
    batch, _ = trial.split_meta(default_collate(items))
    batch = {k: v.cuda() for k, v in batch.items()}
    obs = {k: v for k, v in batch.items() if k.startswith("observation.")}
    calls3, hook3 = backbone_counter(three)
    calls2, hook2 = backbone_counter(two)
    with torch.inference_mode(), profile(activities=[ProfilerActivity.CPU, ProfilerActivity.CUDA]) as prof:
        y3 = three.predict_canonical_chunk(dict(obs))
        torch.cuda.synchronize()
    with torch.inference_mode():
        y2 = two.predict_canonical_chunk({k: v for k, v in obs.items() if k != TOP_KEY})
        zero = dict(obs)
        zero[TOP_KEY] = torch.zeros_like(obs[TOP_KEY])
        y3_other_top = three.predict_canonical_chunk(zero)
    hook3.remove()
    hook2.remove()
    rows = sorted(((float(getattr(e, "device_time_total", None) or getattr(e, "cuda_time_total", 0) or 0), e.key, int(e.count)) for e in prof.key_averages()), reverse=True)
    rows = [r for r in rows if r[0] > 0]
    out["forward"] = dict(policy="RECEIVE_FROZEN weights in the 3-view model", batch_keys={k: dict(shape=list(v.shape), device=str(v.device)) for k, v in batch.items()},
                          parameter_devices=sorted({str(p.device) for p in three.parameters()}), output=dict(shape=list(y3.shape), device=str(y3.device), finite=bool(torch.isfinite(y3).all())),
                          backbone_calls_first_forward=calls3[:3], backbone_calls_per_forward=dict(three_view=len(calls3) // 2, two_view=len(calls2)),
                          cuda_kernel_events=len(rows), top_kernels=[dict(name=k[:60], device_time_us=round(t, 1), calls=c) for t, k, c in rows[:8]],
                          output_changes_with_top_image=dict(max_abs_three_vs_two_view=float((y3 - y2).abs().max()), max_abs_real_top_vs_zero_top=float((y3 - y3_other_top).abs().max()),
                                                             meaning="the top image reaches the decoder (the output moves when it changes); the size of the change says nothing about usefulness"),
                          note="a connection check on four stored training observations: not recognition, not a student result")
    # 3) three ordinary training updates on CUDA (no checkpoint is written; this is not the 3-view training)
    trainable = copy.deepcopy(three).train()
    optimizer = torch.optim.AdamW(trainable.get_optim_params(), lr=1e-4, weight_decay=1e-4)
    with np.load(rc.HORIZON_WEIGHTS, allow_pickle=False) as a:
        r = torch.tensor(a["r"], dtype=torch.float32, device="cuda")
    rng, losses, tcalls = np.random.default_rng(4100), [], []
    thook = trainable.model.backbone.register_forward_hook(lambda module, args, output: tcalls.append(tuple(args[0].shape)))
    for step in range(3):
        idx = [train3[int(i)] for i in rng.integers(0, len(train3), 6)] + [g3[int(i)] for i in rng.integers(0, len(g3), 2)]
        b, _ = trial.split_meta(default_collate(idx))
        b = {k: v.cuda() for k, v in b.items()}
        loss, parts = sd.loss_terms(trainable, b, r)
        optimizer.zero_grad(set_to_none=True)
        loss.backward()
        torch.nn.utils.clip_grad_norm_(trainable.parameters(), 1.0)
        optimizer.step()
        losses.append(round(float(loss), 5))
    thook.remove()
    grad_seen = bool(any(p.grad is not None and float(p.grad.abs().sum()) > 0 for n, p in trainable.named_parameters() if n.startswith("model.backbone")))
    out["training_smoke"] = dict(updates=3, batch="6 Train69 receiving + 2 G corrections, all with the top view", losses=losses, loss_device="cuda:0", backbone_inputs_per_update=len(tcalls) // 3,
                                 backbone_gradient_nonzero=grad_seen, optimizer="fresh AdamW (lr 1e-4, backbone 1e-5, wd 1e-4)", checkpoint_written=False,
                                 note="proves dataset -> batch -> encoder -> loss -> backward with three views on CUDA; it is not the 3-view training")
    out["training_entrypoint"] = dict(status="NOT RUN (prepared)", data=dict(old="Train69 receiving view with the demonstrations' own top frames", new="the 15 G corrections with their top-view sidecars",
                                                                           replay_corrections="earlier / R2 / R3 / Bridge corrections also have sidecars and can be added through the same ThreeView wrapper",
                                                                           observations=dict(train69_receiving=len(train3), g_corrections=len(g3))),
                                      how="tjj_alignment_r1.three_view_policy(<checkpoint>) + ThreeView items + the unchanged loss / optimizer settings; one pre-set run to be instructed, no ratio or update sweep",
                                      open_points=["which checkpoint of each role is the start", "whether the stored AdamW state is carried over", "START also needs its pick-stretch view wrapped (its demonstrations have the top view)"])
    # 4) collector -> recorder -> inference for one second with three cameras and the 3-view model (connection smoke)
    Base = student_eval.Observations
    current = {}

    class ThreeCameraObservations(Base):
        def __init__(self, *a, **k):
            super().__init__(width=160, height=120, camera_ids=CAMERAS, depth_camera_ids=(TOP,))

        def observe(self, m, d, c):
            got = super().observe(m, d, c)
            current["top"] = got[TOP]["rgb"]
            return got

    base_observation = student_eval.policy_observation

    def observation3(q, wrist_rgb, stats, *, device=None):
        b = base_observation(q, wrist_rgb, stats, device=device)
        b[TOP_KEY] = image_tensor(current["top"], device=device)
        return b

    start3 = migrated["START_R1_E"][1].cuda().eval()
    icalls, ihook = backbone_counter(start3)
    saved = (student_eval.Observations, student_eval.policy_observation, rc.HORIZON, rc.load_policy)
    student_eval.Observations, student_eval.policy_observation, rc.HORIZON = ThreeCameraObservations, observation3, 50
    rc.load_policy = lambda name: (start3, rc.stats(), dict(checkpoint="START_R1_E weights in the 3-view model (untrained top input)", model_sha256=None, config_sha256=None, normalization_sha256=None))
    smoke = run_dir() / "collector_smoke_s5119"
    try:
        if not smoke.exists():
            rc.execute(smoke, 5119, ("START",), None, "TJJ 3-view collector / inference connection smoke (1 s; untrained top input; not a task result)", contract="repaired")
    finally:
        student_eval.Observations, student_eval.policy_observation, rc.HORIZON, rc.load_policy = saved
        ihook.remove()
    row = camera_row(smoke)
    trace = load(smoke / "action_trace.json")
    out["collector_smoke"] = dict(path=str(smoke), seed=5119, control_steps=len(trace), cameras=row["cameras"], inference_calls=sum(t["inference_wall_s"] > 0 for t in trace),
                                  backbone_calls=len(icalls), backbone_calls_per_inference=len(icalls) / max(1, sum(t["inference_wall_s"] > 0 for t in trace)),
                                  commands_finite=bool(np.isfinite(np.array([t["q_cmd"] for t in trace])).all()),
                                  note="the three cameras are rendered, recorded (top with depth) and fed to the encoder at every inference; the model's top input is untrained, so the motion means nothing")
    # one simultaneous 3-camera frame for a direct look
    import cv2
    frames = [np.load(smoke / f"cameras/{c}.npz")["rgb"][25] for c in CAMERAS]
    depth = np.load(smoke / f"cameras/{TOP}.npz")["depth_m"][25]
    dimg = cv2.applyColorMap(np.clip((depth - depth.min()) / max(1e-6, depth.max() - depth.min()) * 255, 0, 255).astype(np.uint8), cv2.COLORMAP_VIRIDIS)
    sheet = np.concatenate([cv2.cvtColor(f, cv2.COLOR_RGB2BGR) for f in frames] + [dimg], axis=1)
    sheet = cv2.resize(sheet, (sheet.shape[1] * 3, sheet.shape[0] * 3), interpolation=cv2.INTER_NEAREST)
    (run_dir() / "frames").mkdir(exist_ok=True)
    cv2.imwrite(str(run_dir() / "frames" / "three_camera_same_step.png"), sheet)
    out["simultaneous_frame"] = dict(file=str(run_dir() / "frames" / "three_camera_same_step.png"), step=25, order=["left wrist", "right wrist", "top (gray3)", "top depth (colour map)"])
    target.write_text(json.dumps(out, indent=2, default=str))
    print(json.dumps(dict(migration={k: (v["migration"]["strict_load"], v["migration"]["new_parameters"]) for k, v in out["checkpoints"].items()},
                          forward={k: out["forward"][k] for k in ("backbone_calls_per_forward", "cuda_kernel_events", "output_changes_with_top_image")},
                          smoke={k: out["training_smoke"][k] for k in ("losses", "backbone_inputs_per_update", "backbone_gradient_nonzero")},
                          collector={k: out["collector_smoke"][k] for k in ("control_steps", "inference_calls", "backbone_calls")}), default=str)[:2400])
    return out


# ----------------------------------------------------------------------------------------------
# Depth perception on stored Train reset frames
# ----------------------------------------------------------------------------------------------

def perceive(args=None):
    import mujoco
    target = run_dir() / "perception_check.json"
    if target.exists():
        return load(target)
    model = scene_model()
    data = mujoco.MjData(model)
    block = model.body("red_block").id
    rows = []
    for e in [x for x in d1.manifest() if x["split"] == "train" and x["training_eligible"]]:
        path = Path(e["path"])
        with np.load(path / "cameras" / f"{TOP}.npz", allow_pickle=False) as c:
            depth, K, T, stamp = c["depth_m"][0], c["K"][0], c["T_world_camera_optical"][0], float(c["capture_time"][0])
            last_depth = c["depth_m"][-1]
        with np.load(path / "episode.npz", allow_pickle=False) as a:
            s0_, s1_ = a["integration_state"][0], a["integration_state"][-2]
        mujoco.mj_setState(model, data, s0_, int(mujoco.mjtState.mjSTATE_INTEGRATION))
        mujoco.mj_forward(model, data)
        truth = data.xpos[block].copy()                                       # scoring only; never an input of the perception
        obs, clusters = tp.observe_object(depth, K, T, stamp, surface_z=TABLE_Z, roi=ROI)
        region = tm.Region("source-at-truth", "world", (float(truth[0]), float(truth[1])), 0.04, TABLE_Z)
        occupied, after = tp.observe_area(depth, K, T, stamp, region), tp.observe_area(last_depth, K, T, stamp, region)
        rows.append(dict(seed=e["seed"], detected=obs is not None, clusters=len(clusters), xy_error_mm=None if obs is None else round(1000 * float(np.hypot(obs.position_xyz[0] - truth[0], obs.position_xyz[1] - truth[1])), 2),
                         top_height_mm=None if obs is None else round(1000 * (obs.position_xyz[2] - TABLE_Z), 1), source_region_clear_at_reset=occupied.clear, source_region_clear_at_end=after.clear,
                         points_above_surface_at_end=after.points_above_surface))
    err = np.array([r["xy_error_mm"] for r in rows if r["xy_error_mm"] is not None])
    data_ = dict(run_id=RUN, written=time.strftime("%Y-%m-%dT%H:%M:%S%z"), modality=MODALITY, calibrated=False, frames="the reset frame and the last frame of each Train69 demonstration (stored top depth, K, T)",
                 inputs="top depth + K + T only; the stored block position is used for scoring, never as an input", roi=ROI, object_rule="largest cluster 8-120 mm above the table whose XY extent is at most 90 mm",
                 episodes=len(rows), detected=int(sum(r["detected"] for r in rows)), xy_error_mm=dict(median=round(float(np.median(err)), 2), p90=round(float(np.percentile(err, 90)), 2), max=round(float(err.max()), 2)) if len(err) else None,
                 wrong_cluster_over_20mm=int((err > 20).sum()), source_region_reported_occupied_at_reset=int(sum(not r["source_region_clear_at_reset"] for r in rows)),
                 source_region_reported_clear_at_end=int(sum(r["source_region_clear_at_end"] for r in rows)),
                 limits=["SIM depth is exact and noise-free; the real OS30A depth, K and T are unverified", "an arm inside the region counts as 'not clear' (by design: the hand must be away)",
                         "the object rule is size-based; no recognition of what the object is"], rows=rows)
    target.write_text(json.dumps(data_, indent=2, default=str))
    print(json.dumps({k: data_[k] for k in ("episodes", "detected", "xy_error_mm", "wrong_cluster_over_20mm", "source_region_reported_occupied_at_reset", "source_region_reported_clear_at_end")}))
    return data_


# ----------------------------------------------------------------------------------------------
# PLACE: right arm sets the block down in a fixed target region, opens, retreats
# ----------------------------------------------------------------------------------------------

class PlaceActor:
    """Scripted joint-space PLACE (trajectory teacher v0). Left arm holds its command; only the right arm and gripper move.
    Phases: carry above the target -> lower -> open -> retreat -> hold. No simulator object pose is read after planning
    starts except the hand-to-object transform at the start (a robot would estimate it from its cameras: recorded as such)."""

    def __init__(self, model, data, region, *, speed=0.6, retreats=((0.06, 0.03),)):
        import mujoco
        from sim_data_factory.teacher import Planner, TeacherConfig, UP, SIDE_OFFSET
        self.m, self.failure, self.done, self.phase, self.log = model, None, False, "plan", []
        self.plan, cfg = Planner(model), TeacherConfig()
        adr = model.jnt_qposadr[model.actuator_trnid[:, 0]]
        base = data.qpos.copy()
        cmd = data.ctrl.copy()
        self.plan.pose(base, cmd)
        p, rot = self.plan.site_frame("right")
        block_body = model.body("red_block").id
        rel = rot.T @ (data.xpos[block_body] - p)                             # block centre in the right tool frame at the start
        rel_rot = rot.T @ data.xmat[block_body].reshape(3, 3)
        a_cur, n_cur = rot[:, 0], rot[:, 2]
        rr_now = self.plan.radial("right", p)
        beta = float(np.arctan2(-a_cur @ UP, a_cur @ rr_now))
        self.info = dict(region=region.region_id, target_center_xy=list(region.center_xy), block_in_right_tool_mm=(1000 * rel).round(2).tolist(), approach_pitch_deg=round(float(np.degrees(beta)), 1),
                         start_site_xyz=p.round(4).tolist(), hand_to_object="read once from the simulator at the start of PLACE; on the robot this comes from the wrist / top cameras")
        open_gear = float(model.actuator_ctrlrange[11, 1])
        self.segments = []

        def goal_for(block_center):
            rr = self.plan.radial("right", np.array([block_center[0], block_center[1], p[2]]))
            yaw = float(np.arctan2(rr[1], rr[0]) - np.arctan2(rr_now[1], rr_now[0]))
            Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
            a_new, n_new = Rz @ a_cur, Rz @ n_cur
            site = np.asarray(block_center) - (Rz @ rot) @ rel
            ctrl, ik = self.plan.solve(base, cmd, "right", site, a_new, [n_new, -n_new], cfg)
            return ctrl, ik, site

        hover = (region.center_xy[0], region.center_xy[1], TABLE_Z + 0.02 + 0.06)
        down = (region.center_xy[0], region.center_xy[1], TABLE_Z + 0.02 + 0.003)   # block bottom 3 mm above the surface; it settles when released
        g1, ik1, s1 = goal_for(hover)
        g2, ik2, s2 = goal_for(down)
        self.info.update(ik_hover=ik1 if isinstance(ik1, dict) else str(ik1), ik_down=ik2 if isinstance(ik2, dict) else str(ik2))
        if g1 is None or g2 is None:
            self.failure = dict(reason="place_planning_failure", detail=f"IK hover={ik1} down={ik2}", phase="plan", time=float(data.time))
            return
        c1, c2 = cmd.copy(), cmd.copy()
        c1[6:11], c2[6:11] = g1[6:11], g2[6:11]
        c3 = c2.copy()
        c3[11] = open_gear
        self.plan.pose(base, c3)
        p3, rot3 = self.plan.site_frame("right")
        back, tried = None, []
        for along, up in retreats:                                            # first reachable retreat: back along the approach axis and up
            back, ikb = self.plan.solve(base, c3, "right", p3 - along * rot3[:, 0] + np.array([0, 0, up]), rot3[:, 0], [rot3[:, 2], -rot3[:, 2]], cfg)
            tried.append(dict(back_m=along, up_m=up, ik=ikb if isinstance(ikb, dict) else str(ikb)))
            if back is not None:
                break
        c4 = c3.copy()
        if back is not None:
            c4[6:11] = back[6:11]
        self.info.update(retreat_candidates=tried, retreat_planned=back is not None)
        carry = ("right", np.r_[np.c_[rel_rot, rel], [[0, 0, 0, 1]]])
        self.info["preview"] = dict(carry_to_hover=str(self.plan.path_collisions(base, cmd, c1, carry=carry, allow_pad_block=("right",))),
                                    lower=str(self.plan.path_collisions(base, c1, c2, carry=carry, allow_pad_block=("right",))), note="recorded; the single attempt is executed whatever the preview says")
        dur = lambda a, b: max(0.8, float(np.abs(b - a).max()) / speed)
        self.segments = [("carry", cmd.copy(), c1, dur(cmd, c1)), ("lower", c1, c2, dur(c1, c2)), ("open", c2, c3, 0.8), ("settle", c3, c3, 0.5), ("retreat", c3, c4, dur(c3, c4)), ("hold", c4, c4, 1.5)]
        self.index, self.t = 0, 0.0
        self.trace = []

    def next_command(self, context):
        from sim_data_factory import core
        dt = float(context["control_dt"])
        if self.failure:
            raise RuntimeError("PLACE already failed")
        name, a, b, T = self.segments[self.index]
        if self.phase != name:
            self.phase = name
            self.log.append(dict(phase=name, time=float(context["data"].time)))
        u = min(1.0, self.t / T)
        s = 35 * u**4 - 84 * u**5 + 70 * u**6 - 20 * u**7
        gear = a + (b - a) * s
        self.t += dt
        if self.t >= T:
            self.t = 0.0
            if self.index + 1 < len(self.segments):
                self.index += 1
            else:
                self.done = True
        q = gear.copy()
        q[[5, 11]] = q[[5, 11]] / core.MOTOR_MAX_RAD
        q = np.clip(q, np.r_[self.m.actuator_ctrlrange[:5, 0], 0, self.m.actuator_ctrlrange[6:11, 0], 0], np.r_[self.m.actuator_ctrlrange[:5, 1], 1, self.m.actuator_ctrlrange[6:11, 1], 1])
        self.trace.append(dict(time=float(context["data"].time), phase=name))
        return q, dict(phase=name, controller="PLACE_TRAJ_V0")


def place_truth(model, states, region):
    """SIM-truth scorer (never an input of the task manager): block centre in the region, on the table, no pad touching it,
    the right gripper clear of it, for the last second."""
    import mujoco
    from sim_data_factory.task_success import ContactModel
    cm, d = ContactModel(model), mujoco.MjData(model)
    rows = []
    for s in states:
        mujoco.mj_setState(model, d, s, int(mujoco.mjtState.mjSTATE_INTEGRATION))
        mujoco.mj_forward(model, d)
        ev = cm.evidence(d)
        xy = d.xpos[cm.block_body][:2]
        rows.append(dict(time=float(d.time), in_region=bool(np.hypot(xy[0] - region.center_xy[0], xy[1] - region.center_xy[1]) <= region.radius_m), on_table=ev["table_support_N"] > 0,
                         pads_off=max(max(ev["pad_force_N"]["left"]), max(ev["pad_force_N"]["right"])) < 0.01, right_clear_m=ev["gripper_block_distance_m"]["right"],
                         block_xy=[float(xy[0]), float(xy[1])], block_bottom_clearance_m=ev["block_bottom_clearance_m"], forbidden=ev["forbidden_contacts"]))
    last = [r for r in rows if r["time"] >= rows[-1]["time"] - 1.0]
    ok = all(r["in_region"] and r["on_table"] and r["pads_off"] and r["right_clear_m"] >= 0.01 for r in last)
    return dict(placed=bool(ok), final=rows[-1], last_second=dict(samples=len(last), in_region=all(r["in_region"] for r in last), on_table=all(r["on_table"] for r in last), pads_off=all(r["pads_off"] for r in last),
                                                                 right_gripper_clear_min_m=min(r["right_clear_m"] for r in last)),
                first_on_table_s=next((r["time"] for r in rows if r["on_table"]), None), first_pads_off_s=next((r["time"] for r in rows if r["on_table"] and r["pads_off"]), None),
                forbidden_contact_samples=sum(bool(r["forbidden"]) for r in rows),
                definition="block centre within the target radius, supported by the table, no pad force, right gripper at least 10 mm away, for the last 1.0 s; first-demo values for this 40 mm block")


def place(args=None):
    version = getattr(args, "version", None) or "v0"
    TARGET_REGION = PLACE_VERSIONS[version]["region"]
    import mujoco
    import frozen_transfer_check as ft
    from sim_data_factory import core
    from sim_data_factory.recorder import EpisodeRecorder
    from sim_data_factory.reset_sampler import make_task_episode
    from sim_data_factory.task_success import ContactModel
    target = run_dir() / ("place" if version == "v0" else f"place_{version}") / "result.json"
    if target.exists():
        return load(target)
    target.parent.mkdir(parents=True, exist_ok=True)
    source = load(PLACE_SOURCE / "result.json")
    if not source["strict_valid_handover"] or source["final_support"] != "right hand":
        raise RuntimeError("the stored start is not a completed handover with the right hand holding")
    with np.load(PLACE_SOURCE / "episode.npz", allow_pickle=False) as a:
        start_state = a["integration_state"][-1].copy()
    seed = source["spec"]["seed"]
    meta = ft.demonstration(seed)["metadata"]
    ctx = make_task_episode(seed, meta["requested_initial_state"]["q"], meta["sampling"]["block_xy_yaw"])
    m, d = ctx["model"], ctx["data"]
    spec = int(mujoco.mjtState.mjSTATE_INTEGRATION)
    mujoco.mj_setState(m, d, start_state, spec)
    mujoco.mj_forward(m, d)
    back = np.empty(mujoco.mj_stateSize(m, spec))
    mujoco.mj_getState(m, d, back, spec)
    ctx["initial"] = core.snapshot(ctx)
    cm = ContactModel(m)
    ctx["substep_check"] = cm.substep_failure
    sensor = top_sensor()
    actor = PlaceActor(m, d, TARGET_REGION, retreats=PLACE_VERSIONS[version]["retreats"])
    out = target.parent / "episode"
    rec = EpisodeRecorder(out, dict(contract_revision=core.REVISION, observation_revision="os30a-mono-v1", seed=seed, control_dt=ctx["control_dt"], classification="TJJ PLACE attempt (scripted trajectory teacher v0), SIM only",
                                    hardware_access=False, object_attachment=False, external_forces=False, runtime_qpos_writes=0, restored_from=dict(case=str(PLACE_SOURCE), state="last stored state (handover complete, right hand holds)",
                                                                                                                                                   restore_identity=bool(np.array_equal(back, start_state))),
                                    target_region=dict(id=TARGET_REGION.region_id, center_xy=list(TARGET_REGION.center_xy), radius_m=TARGET_REGION.radius_m, fixed=True), cameras_recorded=list(CAMERAS)))
    started, stop = time.time(), "completed"
    try:
        if actor.failure:
            stop = "planning_failure"
        else:
            for _ in range(1500):
                if not core.rollout(ctx, actor, sensor, rec, 1):
                    stop = "physics_fault"
                    break
                if actor.done:
                    break
    finally:
        sensor.close()
    result = dict(run_id=RUN, written=time.strftime("%Y-%m-%dT%H:%M:%S%z"), kind=f"PLACE attempt, version {version} (one run per version; v1 = v0 with a target the top camera sees and more retreat candidates)", controller="PLACE_TRAJ_V0: scripted joint trajectory of the right arm (IK of the unchanged planner); not a learned policy",
                  start=dict(case=str(PLACE_SOURCE), seed=seed, note="the end state of a mixed collection run (student START + teacher RECEIVE + student RELEASE_B); PLACE starts after RELEASE_B, it does not replace it"),
                  target_region=dict(id=TARGET_REGION.region_id, center_xy=list(TARGET_REGION.center_xy), radius_m=TARGET_REGION.radius_m, surface_z_m=TABLE_Z,
                                     fixed="the first demo's fixed point; a variable target needs a goal input to the controller, which this trajectory has (the region argument) and the learned policies do not"),
                  plan=actor.info, stop=stop, planning_failure=actor.failure, terminal_failure=ctx.get("terminal_failure"), phases=actor.log, transitions=len(rec.commands), wall_s=round(time.time() - started, 1))
    if rec.commands:
        rec.finish(dict(place_stop=stop))
        with np.load(out / "episode.npz", allow_pickle=False) as a:
            states, q = a["integration_state"].copy(), a["q_state"].copy()
        truth = place_truth(m, states, TARGET_REGION)
        top = np.load(out / "cameras" / f"{TOP}.npz")
        k = len(top["rgb"]) - 1
        plan_ = actor.plan
        plan_.pose(states[k][1:1 + m.nq], core.decode_command(np.clip(q[k], np.r_[m.actuator_ctrlrange[:5, 0], 0, m.actuator_ctrlrange[6:11, 0], 0], np.r_[m.actuator_ctrlrange[:5, 1], 1, m.actuator_ctrlrange[6:11, 1], 1]), m))
        tcp, _ = plan_.site_frame("right")                                    # forward kinematics of the measured joints
        seen = tp.verify_place(top["depth_m"][k], top["K"][k], top["T_world_camera_optical"][k], float(top["capture_time"][k]), TARGET_REGION, gripper_opening=float(q[k][11]), gripper_tcp_xyz=tcp)
        src = tp.observe_area(top["depth_m"][k], top["K"][k], top["T_world_camera_optical"][k], float(top["capture_time"][k]), SOURCE_REGION)
        result.update(sim_truth=truth, sensor_observer=dict(placed=seen.placed, object_in_region=seen.object_in_region, object_supported=seen.object_supported, gripper_separated=seen.gripper_separated,
                                                            evidence=seen.evidence, validity=dict(frame_id=seen.validity.frame_id, stamp_s=seen.validity.stamp_s, calibrated=seen.validity.calibrated),
                                                            inputs="last top depth frame + K + T, measured right gripper opening, forward kinematics of the measured joints"),
                      observer_agrees_with_truth=bool(seen.placed == truth["placed"]), source_region_clear=dict(clear=src.clear, points_above_surface=src.points_above_surface),
                      episode=str(out), cameras=camera_row(out)["cameras"])
        # the task manager on this very run: perception from the recorded top depth, manipulation = the run just made
        with np.load(PLACE_SOURCE / "episode.npz", allow_pickle=False) as a0:
            reset_state = a0["integration_state"][0].copy()
        s2 = top_sensor()
        first = render(m, mujoco.MjData(m), s2, reset_state)[TOP]             # the top view of the stored run's reset state (that run recorded wrists only)
        s2.close()
        class RunPerception:
            kind = tm.Kind.SIM

            def observe_object(self, task, now):
                o, _ = tp.observe_object(first["depth_m"], first["K"], first["T_world_camera_optical"], now, surface_z=TABLE_Z, roi=ROI)
                return o

            def observe_area(self, region, now):
                a_ = tp.observe_area(top["depth_m"][k], top["K"][k], top["T_world_camera_optical"][k], now, region)
                return a_

            def verify_place(self, task, now):
                v = tp.verify_place(top["depth_m"][k], top["K"][k], top["T_world_camera_optical"][k], now, task.target_region, gripper_opening=float(q[k][11]), gripper_tcp_xyz=tcp)
                return v

        class RunArms:
            kind = tm.Kind.SIM

            def pick(self, target_, now):
                return tm.CommandResult(tm.Status.SUCCEEDED, self.kind, "stored mixed run: pick by START_R1_E (student)", now, dict(case=str(PLACE_SOURCE)))

            def coordinate(self, task, now):
                return tm.CommandResult(tm.Status.SUCCEEDED, self.kind, "stored mixed run: hand-over (teacher RECEIVE + student RELEASE_B); not PLACE", now, dict(case=str(PLACE_SOURCE)))

            def place(self, task, now):
                ok = stop == "completed"
                return tm.CommandResult(tm.Status.SUCCEEDED if ok else tm.Status.FAILED, self.kind, f"PLACE_TRAJ_V0 {stop}", now, dict(episode=str(out)))

            def stow(self, now):
                return tm.CommandResult(tm.Status.NOT_IMPLEMENTED, tm.Kind.NOT_IMPLEMENTED, "no stow motion yet", now)

        clock = iter(np.arange(0.0, 100.0, 0.01))
        manager = tm.TaskManager(RunPerception(), tm.StationaryBase(), RunArms(), tm.MockCleaner(), tm.MockPower(), clock=lambda: float(next(clock)))
        manager.start(tm.Task("tjj-sim-place-1", SOURCE_REGION, TARGET_REGION, needs_two_arm_coordination=True))
        manager.run()
        result["task_manager"] = dict(final_state=manager.state.value, failure=manager.failure, system_success=manager.system_success, module_kinds_used=sorted(k.value for k in manager.kinds_used),
                                      transitions=[f"{e.state} -> {e.next_state} [{e.module_kind}] {e.reason}" for e in manager.events],
                                      note="the sequence is replayed over stored / just-made SIM pieces to exercise the interfaces; the cleaner and the battery are MOCK, so this is never a system success")
        import cv2
        frames_ = [0, len(top["rgb"]) // 3, 2 * len(top["rgb"]) // 3, k]
        tiles = []
        for f in frames_:
            row_ = np.concatenate([np.load(out / f"cameras/{c}.npz")["rgb"][f] for c in CAMERAS], axis=1)
            tiles.append(cv2.cvtColor(row_, cv2.COLOR_RGB2BGR))
        sheet = cv2.resize(np.concatenate(tiles, axis=0), None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST)
        (run_dir() / "frames").mkdir(exist_ok=True)
        cv2.imwrite(str(run_dir() / "frames" / f"place_{version}_three_cameras.png"), sheet)
        result["frames"] = dict(file=str(run_dir() / "frames" / f"place_{version}_three_cameras.png"), rows=[dict(step=int(f), sim_time_s=float(top["capture_time"][f])) for f in frames_], columns=["left wrist", "right wrist", "top (gray3)"])
    target.write_text(json.dumps(result, indent=2, default=str))
    print(json.dumps(dict(stop=stop, failure=actor.failure, terminal=ctx.get("terminal_failure"), truth=None if "sim_truth" not in result else dict(placed=result["sim_truth"]["placed"], last=result["sim_truth"]["last_second"], final=result["sim_truth"]["final"]),
                          observer=None if "sensor_observer" not in result else {k: result["sensor_observer"][k] for k in ("placed", "object_in_region", "object_supported", "gripper_separated")},
                          manager=result.get("task_manager", {}).get("final_state"), source_clear=result.get("source_region_clear")), default=str)[:2400])
    return result


def everything(args=None):
    import subprocess
    me = [sys.executable, str(Path(__file__).resolve())]
    for step in ("inventory", "sidecar", "connect", "perceive", "place"):
        print(f"--- {step} --- {time.strftime('%H:%M:%S')}", flush=True)
        code = subprocess.run(me + [step]).returncode
        if code:
            print(f"[step failed] {step}: return code {code}; the independent later steps still run", flush=True)
    print(f"=== ALL DONE {time.strftime('%H:%M:%S')} ===", flush=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    for name in ("inventory", "sidecar", "connect", "perceive", "all"):
        sub.add_parser(name)
    sub.add_parser("place").add_argument("--version", choices=sorted(PLACE_VERSIONS), default="v0")
    args = parser.parse_args(argv)
    import arm_output_trial  # noqa: F401
    return dict(inventory=inventory, sidecar=sidecar, connect=connect, perceive=perceive, place=place, all=everything)[args.command](args)


if __name__ == "__main__":
    main()
