#!/usr/bin/env python3
"""RECEIVE candidate: one copy of START trained on the receiving phase only (local SIM / learning only; no hardware).

START (three-parent step-020000) and B (recovery-subset b step-005000) are read-only. RECEIVE = all START weights + its
AdamW state + 5,000 updates on (a) the receiving phase of the Train69 demonstrations, served as index views, and (b) the
receiving phase of valid teacher corrections that start from the student's own approach states.

  teacher   the existing re-entered HandoverTeacher, once from each of three fixed stored approach states (seed 5000)
  protocol  phase views, sample plan and their direct checks, fixed before training
  train     the one candidate, 5,000 updates, batch 8 = 4 Train receiving + 4 correction receiving
  run       same-state comparison (5 states x START / RECEIVE, then B at the gate) and original-reset comparison
            (Train 5000 / 5002 / 5008 x START->B / START->RECEIVE->B), every rollout in its own process
  video     REPLAY/run1 control + candidate, seed 5000 original reset control + candidate

Every policy switch reads SIM ground truth (the existing checker events and contact forces): the runs are
SIM-ORACLE-GATED THREE-POLICY CANDIDATE runs, not privileged-information-free student runs. No time, wrist angle or pad
distance is a switch condition or a stop condition.
"""
import argparse
import hashlib
import json
import os
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import train_rollout_diagnostic as td   # noqa: E402
import live_release_check as lr   # noqa: E402

FACTORY_ROOT = td.FACTORY_ROOT
RUN = "receive-candidate-20261005-01"
START_TRAINING = FACTORY_ROOT / "training/three-parent-c25-20261004-01"
START_CHECKPOINT = "step-020000"
RECEIVE_TRAINING = FACTORY_ROOT / "training" / RUN
RECEIVE_CHECKPOINT = "step-005000"
MODELS = dict(START=lr.MODELS["START"], B=lr.MODELS["B"], RECEIVE=RECEIVE_TRAINING / RECEIVE_CHECKPOINT)
LIVE = FACTORY_ROOT / "evaluation/live-release-check-20261004-01/runs"
STRICT_5008 = FACTORY_ROOT / "evaluation/three-parent-c25-20261004-01/eval/step-020000/run1/case_03_G000_s5008"
CORRECTION = FACTORY_ROOT / "evaluation/handover-correction-20261004-01"
HORIZON_WEIGHTS = FACTORY_ROOT / "evaluation/first-grasp-20261003-01/horizon_weights.npz"
# stored student approach states; step 900 only selects stored material, it is not a runtime condition
TEACHER_STATES = (("REPLAY1_0900", LIVE / "REPLAY/run1", 900, 5000), ("CONTROL1_0900", LIVE / "CONTROL/run1", 900, 5000), ("RECOVERY1_0900", LIVE / "RECOVERY/run1", 900, 5000))
REUSED_CORRECTION = "s5008_strict_0410"
LOCAL_STATES = (("CONTROL1_0900", LIVE / "CONTROL/run1", 900, 5000, "in the new correction fit"), ("REPLAY1_0900", LIVE / "REPLAY/run1", 900, 5000, "in the new correction fit"),
                ("REPLAY2_0900", LIVE / "REPLAY/run2", 900, 5000, "not in the fit"), ("RECOVERY1_0900", LIVE / "RECOVERY/run1", 900, 5000, "in the new correction fit"),
                ("s5008_strict_0755", STRICT_5008, 755, 5008, "not in the fit"))
FULL_SEEDS = (5000, 5002, 5008)
CHAINS = dict(control=("START", "B"), candidate=("START", "RECEIVE", "B"))
LOCAL_CHAINS = dict(control=("START", "B"), candidate=("RECEIVE", "B"))
UPDATES, BATCH, SEED = 5000, 8, 4100
HORIZON, PREFIX = 2000, 25
ARM = lr.ARM
LABEL = "SIM-ORACLE-GATED THREE-POLICY CANDIDATE"
ENTRY = dict(RECEIVE="existing checker pick_lift event in the run so far AND the left hand holds the block now (both left pads >= grip_force_min_N, "
                     "table_support_N == 0), checked where the active policy is due for a regular C25 inference; once",
             B=lr.GATE["predicate"] + "; " + lr.GATE["when"], skip="if the B gate already holds at a boundary, B is entered directly")


def load(path):
    return json.loads(Path(path).read_text())


def run_dir():
    return FACTORY_ROOT / "evaluation" / RUN


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def frozen_identity():
    return {k: {f: sha(MODELS[k] / f) for f in ("model.safetensors", "config.json")} | dict(normalization=sha(MODELS[k].parent / "normalization.npz")) for k in ("START", "B")}


def stats():
    with np.load(START_TRAINING / "normalization.npz", allow_pickle=False) as a:
        return {k: {s: a[f"{k}_{s}"].copy() for s in ("mean", "std")} for k in ("state", "action")}


def stored_state(path, step):
    path = Path(path)
    with np.load(path / "episode.npz", allow_pickle=False) as a:
        states, q = a["integration_state"].copy(), a["q_state"].astype(np.float64)
    rgb = {s: np.load(path / f"cameras/{s}_wrist_rgb.npz")["rgb"][step].copy() for s in ("left", "right")}
    return dict(states=states[:step + 1], q=q[:step + 1], rgb=rgb, step=step)


# ----------------------------------------------------------------------------------------------
# Receiving phase: bounds from recorded SIM ground truth (never a policy input)
# ----------------------------------------------------------------------------------------------

def receive_bounds(rows, events, offset=0, start_at_first_row=False):
    """[start, end) row indices of the receiving phase. start: first regular C25 boundary (absolute step % 25 == 0) at or
    after the checker's pick_lift where the left hand holds the block; end: first such boundary at or after the checker's
    right_grasp where both hands hold it now and the table does not support it (the live-release gate)."""
    from sim_data_factory.task_success import CheckerConfig
    g = CheckerConfig().grip_force_min_N
    left = lambda r: min(r["pad_force_N"]["left"]) >= g and r["table_support_N"] == 0.0
    both = lambda r: left(r) and min(r["pad_force_N"]["right"]) >= g
    boundary = lambda j: (offset + j) % PREFIX == 0
    if "pick_lift" not in events or "right_grasp" not in events:
        return None, None, "the checker has no pick_lift / right_grasp event"
    lift, grasp = int(round(events["pick_lift"] / td.DT)) - offset, int(round(events["right_grasp"] / td.DT)) - offset
    start = 0 if start_at_first_row else next((j for j in range(max(lift, 0), len(rows)) if boundary(j) and left(rows[j])), None)
    end = next((j for j in range(max(grasp, 0), len(rows)) if boundary(j) and both(rows[j])), None)
    if start is None or end is None or end <= start:
        return start, end, "no C25 boundary with the required hold state"
    return start, end, None


def quality(model, states, rows, lo, hi):
    """Diagnostic contact / slip numbers of the stored states [lo, hi] (SIM ground truth; no threshold is derived from them)."""
    import mujoco
    from sim_data_factory.task_success import CheckerConfig
    g = CheckerConfig().grip_force_min_N
    d = mujoco.MjData(model)
    spec = int(mujoco.mjtState.mjSTATE_INTEGRATION)
    site, block = model.site("left_cube_grasp").id, model.body("red_block").id
    pos, rot = [], []
    for k in range(lo, hi + 1):
        mujoco.mj_setState(model, d, states[k], spec)
        mujoco.mj_forward(model, d)
        r = d.site_xmat[site].reshape(3, 3)
        pos.append(r.T @ (d.xpos[block] - d.site_xpos[site]))
        rot.append(r.T @ d.xmat[block].reshape(3, 3))
    pos, rot = np.array(pos), np.array(rot)
    seg = rows[lo:hi + 1]
    fr = np.array([r["pad_force_N"]["right"] for r in seg])
    fl = np.array([r["pad_force_N"]["left"] for r in seg])
    two = fr.min(1) >= g
    first_two = int(np.argmax(two)) if two.any() else len(seg)
    push = fr.max(1)[:first_two]                              # right-hand contact on the block before it holds with both pads
    held = fl.min(1) >= g
    angle = np.degrees(np.arccos(np.clip((np.einsum("ij,kij->k", rot[0], rot) - 1) / 2, -1, 1)))
    disp = np.linalg.norm(pos - pos[0], axis=1) * 1000
    runs, best, cur = two.astype(int), 0, 0
    for v in runs:
        cur = cur + 1 if v else 0
        best = max(best, cur)
    return dict(window_s=[round(lo * td.DT, 2), round(hi * td.DT, 2)],
                right_before_two_pad_hold=dict(peak_N=round(float(push.max()) if len(push) else 0.0, 2), contact_s=round(float((push > 0).sum() * td.DT), 2),
                                               impulse_Ns=round(float(push.sum() * td.DT), 3)),
                right_two_pad_hold=dict(first_s=None if not two.any() else round((lo + first_two) * td.DT, 2), longest_s=round(best * td.DT, 2), at_window_end=bool(two[-1])),
                right_pad_peak_N=round(float(fr.max()), 2), left_pad_peak_N=round(float(fl.max()), 2), left_hold_lost_steps=int((~held).sum()),
                block_in_left_site=dict(start_mm=(pos[0] * 1000).round(2).tolist(), end_mm=(pos[-1] * 1000).round(2).tolist(),
                                        max_displacement_mm=round(float(disp.max()), 2), end_displacement_mm=round(float(disp[-1]), 2),
                                        max_rotation_deg=round(float(angle.max()), 2), end_rotation_deg=round(float(angle[-1]), 2)))


# ----------------------------------------------------------------------------------------------
# Teacher corrections from the fixed student approach states
# ----------------------------------------------------------------------------------------------

def teacher(args):
    import handover_correction_trial as hc
    import paired_release_check as pr
    from sim_data_factory import student_eval
    target = run_dir() / "teacher_collection.json"
    if target.exists():
        raise FileExistsError(f"{target} exists; nothing is overwritten")
    run_dir().mkdir(parents=True, exist_ok=True)
    fixed = [dict(id=name, source=str(path), step=step, seed=seed, state_sha256=hc.state_sha(stored_state(path, step)["states"][-1])) for name, path, step, seed in TEACHER_STATES]
    (run_dir() / "teacher_selection.json").write_text(json.dumps(dict(run_id=RUN, fixed_before_teacher=time.strftime("%Y-%m-%dT%H:%M:%S%z"), states=fixed,
                                                                       rule="three states, one teacher run each, no retry; step 900 selects stored material only"), indent=2))
    original = student_eval.Observations
    results = []
    for name, path, step, seed in TEACHER_STATES:
        g = stored_state(path, step)
        sensors = []

        def first_stored(*a, **k):                                   # the recorded first observation is the stored frame of that state
            student_eval.Observations = original                    # the helper subclasses the real class
            sensors.append(pr.stored_first_sensor(dict(rgb=g["rgb"])))
            return sensors[-1]

        student_eval.Observations = first_stored
        try:
            entry = hc.continuation(run_dir() / "corrections" / name, seed, g["states"], g["q"], g["states"][-1],
                                    dict(classification="teacher_recovery_from_student_approach_state", source_student_case=str(path), source_step=step,
                                         source_state_sha256=hc.state_sha(g["states"][-1]), note="a child of the Train parent, not a new independent initial condition"))
        finally:
            student_eval.Observations = original
        entry.update(id=name, seed=seed, source_step=step, first_frame_is_stored=True, rerender_equals_stored=sensors[-1].rerender_equals_stored if sensors else None)
        results.append(entry)
        print(json.dumps({k: entry.get(k) for k in ("id", "reentry", "stop", "transitions", "events", "admitted", "teacher_failure", "terminal_failure")}, default=str), flush=True)
    target.write_text(json.dumps(dict(run_id=RUN, results=results, admitted=[r["id"] for r in results if r.get("admitted")]), indent=2, default=str))
    print(f"[written] {target}")


def correction_entries():
    """Valid corrections with their receiving phase: the new ones of this run + the stored s5008_strict_0410."""
    out, excluded = [], []
    new = load(run_dir() / "teacher_collection.json")["results"]
    old = next(r for r in load(CORRECTION / "collection.json")["results"] if r["id"] == REUSED_CORRECTION)
    for r, origin in [(r, "new") for r in new] + [(old, "reused, not re-run")]:
        if not r.get("admitted"):
            excluded.append(dict(id=r["id"], reason=f"teacher run not admitted: stop={r.get('stop')}, failure={r.get('teacher_failure') or r.get('terminal_failure')}"))
            continue
        path = Path(r["path"])
        for file, digest in r["files"].items():
            if sha(path / file) != digest:
                raise ValueError(f"correction file changed: {path / file}")
        meta = load(path / "metadata.json")
        if meta["parent"]["split"] != "train" or meta["parent"]["seed"] != r["seed"]:
            raise ValueError(f"{r['id']}: lineage / split mismatch")
        rows = load(path / "task_check_rows.json")
        start, end, why = receive_bounds(rows, r["events"], offset=r["source_step"], start_at_first_row=True)
        if why:
            excluded.append(dict(id=r["id"], reason=why))
            continue
        out.append(dict(id=r["id"], seed=r["seed"], path=str(path), origin=origin, source_step=r["source_step"], suffix_transitions=r["transitions"],
                        receive=[start, end], receive_abs_s=[round((r["source_step"] + start) * td.DT, 2), round((r["source_step"] + end) * td.DT, 2)], files=r["files"]))
    return out, excluded


# ----------------------------------------------------------------------------------------------
# Phase views and the sample plan
# ----------------------------------------------------------------------------------------------

def segment_view(st, dataset, segments):
    """TrialEpisodes.__getitem__ on [start, end) slices: the anchor is the slice's own q_meas[t], targets come from the same
    slice, and steps past its end are padding. Slices are numpy views; nothing is copied or relabelled."""
    import arm_output_trial as trial
    view = trial.TrialEpisodes.__new__(trial.TrialEpisodes)
    view.arm_target, view.stats, view.required_split, view.dataset = "measured_relative", st, "train", str(dataset)
    view.episodes, view.rows = {}, []
    for key, ep, (a, b), row in segments:
        view.episodes[key] = dict(state=ep["state"][a:b], action=ep["action"][a:b], rgb={s: ep["rgb"][s][a:b] for s in ("left", "right")})
        view.rows.append({**row, "view_key": key, "receive": [a, b]})
    view.indices = [(i, t) for i, ep in view.episodes.items() for t in range(len(ep["action"]))]
    h, w, c = next(iter(view.episodes.values()))["rgb"]["left"].shape[1:]
    view.image_shape = (c, h, w)
    return view


def datasets():
    import arm_output_trial as trial
    st = stats()
    episodes = trial.load_manifest()["episodes"]
    train_ids = [i for i, e in enumerate(episodes) if e["split"] == "train" and e["training_eligible"]]
    replay = trial.TrialEpisodes(td.DATASET, train_ids, required_split="train", stats=st, arm_target="measured_relative")
    segments, excluded = [], []
    for row in replay.rows:
        path = Path(row["path"])
        start, end, why = receive_bounds(load(path / "task_check_rows.json"), load(path / "metadata.json")["task_check"]["events"])
        if why:
            excluded.append(dict(manifest_index=row["manifest_index"], seed=row["seed"], reason=why))
            continue
        segments.append((row["manifest_index"], replay.episodes[row["manifest_index"]], (start, end), dict(manifest_index=row["manifest_index"], seed=row["seed"], split=row["split"],
                                                                                                           parent_physics_group=row["parent_physics_group"])))
    train_view = segment_view(st, td.DATASET, segments)
    entries, c_excluded = correction_entries()
    c_segments = []
    for i, e in enumerate(entries):
        path = Path(e["path"])
        with np.load(path / "episode.npz", allow_pickle=False) as a:
            state, action = a["q_state"][:-1].astype(np.float32), a["q_cmd"].astype(np.float32)
        rgb = {s: np.load(path / f"cameras/{s}_wrist_rgb.npz")["rgb"] for s in ("left", "right")}
        if not (len(state) == len(action) == len(rgb["left"]) == len(rgb["right"]) == e["suffix_transitions"]):
            raise ValueError(f"{path}: N+1/N or camera alignment mismatch")
        c_segments.append((100000 + i, dict(state=state, action=action, rgb=rgb), tuple(e["receive"]), e))
    corr_view = segment_view(st, run_dir(), c_segments) if c_segments else None
    return st, replay, train_view, corr_view, excluded, c_excluded


def build_plan(train_view, corr_view):
    """Per update: slots 0-3 Train receiving (parent first over the admitted parents, then a uniform step of its phase);
    slots 4-7 correction receiving (parent first over the parents that have one, then one of its corrections, then a step)."""
    rng = np.random.default_rng(SEED)

    def table(view):
        off, total, length = {}, 0, {}
        for key, ep in view.episodes.items():
            off[key], length[key], total = total, len(ep["action"]), total + len(ep["action"])
        return off, length

    def balanced(pool, count):
        blocks = -(-count // len(pool))
        return rng.permuted(np.tile(np.asarray(pool), (blocks, 1)), axis=1).reshape(-1)[:count]

    n = UPDATES
    off, length = table(train_view)
    p = balanced(list(train_view.episodes), n * 4)
    t = np.floor(rng.random(n * 4) * np.array([length[k] for k in p])).astype(np.int64)
    out = dict(train_key=p.reshape(n, 4), train_t=t.reshape(n, 4), train_index=(np.array([off[k] for k in p]) + t).reshape(n, 4))
    c_off, c_length = table(corr_view)
    by_seed = {}
    for r in corr_view.rows:
        by_seed.setdefault(r["seed"], []).append(r["view_key"])
    seeds = balanced(sorted(by_seed), n * 4)
    pick = rng.random(n * 4)
    keys = np.array([by_seed[s][int(u * len(by_seed[s]))] for s, u in zip(seeds.tolist(), pick.tolist())])
    ct = np.floor(rng.random(n * 4) * np.array([c_length[k] for k in keys])).astype(np.int64)
    out.update(corr_parent=seeds.reshape(n, 4), corr_key=keys.reshape(n, 4), corr_t=ct.reshape(n, 4), corr_index=(np.array([c_off[k] for k in keys]) + ct).reshape(n, 4))
    return out


def check_plan(plan, replay, train_view, corr_view):
    """Direct checks: the plan addresses what it says, Train and correction pools are disjoint and Train-only, a view item
    equals the original episode item inside the phase, and a chunk never crosses the phase end (padding mask)."""
    arm = list(ARM)
    for view, a, b, c in ((train_view, "train_index", "train_key", "train_t"), (corr_view, "corr_index", "corr_key", "corr_t")):
        for i, k, t in zip(plan[a].ravel()[:4000], plan[b].ravel()[:4000], plan[c].ravel()[:4000]):
            assert view.indices[int(i)] == (int(k), int(t))
    assert not set(train_view.episodes) & set(corr_view.episodes)
    assert all(r["split"] == "train" for r in train_view.rows) and {r["seed"] for r in corr_view.rows} <= {r["seed"] for r in replay.rows}
    seed_of = {r["view_key"]: r["seed"] for r in corr_view.rows}
    assert all(seed_of[int(k)] == int(s) for k, s in zip(plan["corr_key"].ravel(), plan["corr_parent"].ravel()))
    row = train_view.rows[0]
    key, (a, b) = row["view_key"], row["receive"]
    full_start = next(i for i, (k, t) in enumerate(replay.indices) if k == key)
    view_start = next(i for i, (k, t) in enumerate(train_view.indices) if k == key)
    n, std = b - a, train_view.stats["action"]["std"]
    full_ep = replay.episodes[key]
    assert np.shares_memory(train_view.episodes[key]["action"], full_ep["action"]) and np.shares_memory(train_view.episodes[key]["rgb"]["left"], full_ep["rgb"]["left"])
    for t in (0, max(n - 60, 0), n - 10, n - 1):
        item, same = train_view[view_start + t], replay[full_start + a + t]
        pad = item["action_is_pad"].numpy()
        valid = ~pad
        assert pad.sum() == max(0, t + 50 - n) and (not pad.any() or pad[-1])                       # padding only past the phase end
        assert np.array_equal(item["action"].numpy()[valid], same["action"].numpy()[valid])        # the original targets inside the phase
        assert all(np.array_equal(item[k].numpy(), same[k].numpy()) for k in item if k.startswith("observation"))
        j = min(3, n - 1 - t)
        want = (full_ep["action"][a + t + j][arm] - full_ep["state"][a + t][arm]) / std[arm]
        assert np.allclose(item["action"][j, arm].numpy(), want, atol=1e-6)                        # one anchor: this observation's q_meas
    return True


def protocol(args):
    import arm_output_trial as trial
    target = run_dir() / "protocol.json"
    if target.exists():
        raise FileExistsError(f"{target} exists; nothing is overwritten")
    st, replay, train_view, corr_view, excluded, c_excluded = datasets()
    if corr_view is None:
        target.write_text(json.dumps(dict(run_id=RUN, status="INSUFFICIENT_DATA: no valid receiving correction", corrections_excluded=c_excluded), indent=2))
        raise SystemExit("no valid receiving correction: training is not started")
    plan = build_plan(train_view, corr_view)
    check_plan(plan, replay, train_view, corr_view)
    np.savez_compressed(run_dir() / "sample_plan.npz", **plan)
    lengths = [len(e["action"]) for e in train_view.episodes.values()]
    config = load(START_TRAINING / START_CHECKPOINT / "config.json")
    usage = __import__("shutil").disk_usage(FACTORY_ROOT)
    record = dict(
        run_id=RUN, name=LABEL, fixed_before_training=time.strftime("%Y-%m-%dT%H:%M:%S%z"), hardware_access=False, val_test=False,
        frozen=frozen_identity(),
        start=dict(checkpoint=str(START_TRAINING / START_CHECKPOINT), optimizer_state_sha256=sha(START_TRAINING / START_CHECKPOINT / "training_state.pt"),
                   use="read-only: all weights and the stored AdamW state are loaded into a new policy object; nothing re-initialised; saved only under training/" + RUN),
        config_read_from_checkpoint={k: config.get(k) for k in ("type", "chunk_size", "n_action_steps", "kl_weight", "optimizer_lr", "optimizer_lr_backbone", "optimizer_weight_decay", "use_vae")},
        data=dict(train_view=dict(dataset=str(td.DATASET), manifest_sha256=sha(td.DATASET / "manifest.json"), parents=len(train_view.rows), transitions=len(train_view),
                                  phase_length_steps=dict(min=min(lengths), median=float(np.median(lengths)), max=max(lengths)), excluded=excluded,
                                  phases=[dict(manifest_index=r["manifest_index"], seed=r["seed"], receive=r["receive"]) for r in train_view.rows]),
                  corrections=[{k: r[k] for k in ("id", "seed", "origin", "source_step", "receive", "receive_abs_s", "suffix_transitions", "path")} for r in corr_view.rows],
                  corrections_transitions=len(corr_view), corrections_excluded=c_excluded,
                  not_used=["any student prefix action", "commands of other runs", "the original teacher's action at the same time stamp", "teacher steps after the gate (release, retreat)",
                            "REPLAY/run2 approach state", "s5008_strict_0755", "s5000_project_1178 / _1678 (they start with both hands already holding: no receiving phase)", "Val / Test"],
                  phase_rule=dict(start="first C25 boundary at or after the checker's pick_lift where the left hand holds (corrections: the first teacher-controlled step)",
                                  end="first C25 boundary at or after the checker's right_grasp where both hands hold now and table support is 0 (exclusive)",
                                  source="recorded SIM ground truth; never a policy input")),
        sample_plan=dict(file=str(run_dir() / "sample_plan.npz"), sha256=sha(run_dir() / "sample_plan.npz"), seed=SEED, updates=UPDATES, batch=BATCH,
                         slots={"0-3": "Train receiving view: parent first (balanced permutation blocks), then a uniform step of that parent's phase",
                                "4-7": "correction receiving view: parent first (balanced over the parents that have a valid correction), then one of its corrections, then a uniform step"},
                         planned_corrections={r["id"]: int((plan["corr_key"] == r["view_key"]).sum()) for r in corr_view.rows},
                         planned_correction_parents={str(int(k)): int(v) for k, v in zip(*np.unique(plan["corr_parent"], return_counts=True))},
                         note="a computation budget and an operating choice, not an optimal ratio"),
        unchanged=dict(normalization_sha256=sha(START_TRAINING / "normalization.npz"), horizon_weights_sha256=sha(HORIZON_WEIGHTS),
                       loss="B horizon weighting (single_demo_trial.loss_terms)", optimizer="AdamW over policy.get_optim_params(), lr 1e-4 / wd 1e-4 as in the existing continuation path, clip 1.0"),
        evaluation=dict(checkpoint="step-005000 only, fixed before training", local_states=[dict(id=n, step=s, seed=sd, fit=f) for n, _, s, sd, f in LOCAL_STATES],
                        local_chains={k: " -> ".join(v) for k, v in LOCAL_CHAINS.items()}, full_seeds=list(FULL_SEEDS), full_chains={k: " -> ".join(v) for k, v in CHAINS.items()},
                        entry=ENTRY, order="local: control first on even state index, candidate first on odd; full: 5000 control first, 5002 candidate first, 5008 control first",
                        budget="10 suffixes + 6 full runs, one each, whatever the results", success="whole history valid + handover + final right-hand-only support",
                        video="REPLAY1_0900 control + candidate; seed 5000 original reset control + candidate"),
        environment=dict(MUJOCO_GL=os.environ.get("MUJOCO_GL"), python=sys.executable, backend=trial.environment_record(), disk_free_bytes=usage.free))
    target.write_text(json.dumps(record, indent=2, default=str))
    print(json.dumps(dict(train_parents=len(train_view.rows), train_transitions=len(train_view), train_excluded=excluded, phase_length=record["data"]["train_view"]["phase_length_steps"],
                          corrections=record["data"]["corrections"], corrections_excluded=c_excluded, planned=record["sample_plan"]["planned_corrections"]), indent=1, default=str))
    print(f"[written] {target}")


def train(args):
    import shutil
    import warnings
    import arm_output_trial as trial
    import single_demo_trial as sd
    import torch
    from torch.utils.data import default_collate
    from dapier_act_policy import ACTPolicy
    record = load(run_dir() / "protocol.json")
    out = RECEIVE_TRAINING
    load_report = trial.gpu_other_load()
    if [p for p in load_report.get("other_processes", []) if isinstance(p["used_memory_mib"], int) and p["used_memory_mib"] > 1000]:
        raise RuntimeError("another heavy GPU job is running")
    trial.apply_reproducibility_settings()
    st, replay, train_view, corr_view, _, _ = datasets()
    if sha(run_dir() / "sample_plan.npz") != record["sample_plan"]["sha256"]:
        raise RuntimeError("sample plan changed since the protocol")
    with np.load(run_dir() / "sample_plan.npz", allow_pickle=False) as a:
        plan = {k: a[k] for k in a.files}
    before = frozen_identity()
    if before != record["frozen"]:
        raise RuntimeError("START / B changed since the protocol")
    source = START_TRAINING / START_CHECKPOINT
    policy = ACTPolicy.from_pretrained(source, local_files_only=True, strict=True).cuda()
    policy.check_normalization(st)
    optimizer = torch.optim.AdamW(policy.get_optim_params(), lr=1e-4, weight_decay=1e-4)
    state = torch.load(source / "training_state.pt", map_location="cpu", weights_only=False)
    optimizer.load_state_dict(state["optimizer"])
    groups = [dict(lr=g["lr"], weight_decay=g["weight_decay"], tensors=len(g["params"])) for g in optimizer.param_groups]
    with np.load(HORIZON_WEIGHTS, allow_pickle=False) as a:
        r = torch.tensor(a["r"], dtype=torch.float32, device="cuda")
    out.mkdir(parents=True, exist_ok=False)
    shutil.copy2(START_TRAINING / "normalization.npz", out / "normalization.npz")
    consumed_e = np.full((UPDATES, BATCH), -1, dtype=np.int64)
    consumed_t = np.full((UPDATES, BATCH), -1, dtype=np.int64)
    torch.manual_seed(SEED)
    started_wall, started = time.strftime("%Y-%m-%dT%H:%M:%S%z"), time.monotonic()
    with (out / "metrics.jsonl").open("x") as log, warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        for step in range(1, UPDATES + 1):
            u = step - 1
            items = [train_view[int(i)] for i in plan["train_index"][u]] + [corr_view[int(i)] for i in plan["corr_index"][u]]
            batch, meta = trial.split_meta(default_collate(items))
            consumed_e[u], consumed_t[u] = meta["meta_episode"].numpy(), meta["meta_t"].numpy()
            batch = {k: v.cuda(non_blocking=True) for k, v in batch.items()}
            policy.train()
            loss, parts = sd.loss_terms(policy, batch, r)
            if not torch.isfinite(loss):
                raise RuntimeError(f"non-finite loss at step {step}")
            optimizer.zero_grad(set_to_none=True)
            loss.backward()
            grad = torch.nn.utils.clip_grad_norm_(policy.parameters(), 1.0, error_if_nonfinite=True)
            optimizer.step()
            if step == 1 or step % 100 == 0:
                entry = dict(step=step, elapsed_s=round(time.monotonic() - started, 2), loss=loss.item(), gradient_norm=grad.item(), **parts)
                log.write(json.dumps(entry) + "\n")
                log.flush()
                if step % 1000 == 0 or step == 1:
                    print(f"[RECEIVE][{step:05d}] loss={entry['loss']:.4f} arm={parts['arm_l1_term']:.4f} grip={parts['gripper_l1_term']:.4f} grad={entry['gradient_norm']:.3f} "
                          f"t={entry['elapsed_s']:.0f}s", flush=True)
        name = out / RECEIVE_CHECKPOINT
        policy.save_pretrained(name)
        torch.save(dict(optimizer=optimizer.state_dict(), step=UPDATES, cpu_rng=torch.get_rng_state(), cuda_rng=torch.cuda.get_rng_state()), name / "training_state.pt")
    seconds = round(time.monotonic() - started, 2)
    expected_e = np.concatenate([plan["train_key"], plan["corr_key"]], axis=1)
    expected_t = np.concatenate([plan["train_t"], plan["corr_t"]], axis=1)
    if not (np.array_equal(consumed_e, expected_e) and np.array_equal(consumed_t, expected_t)):
        raise RuntimeError("consumed samples differ from the plan")
    np.savez_compressed(out / "consumed_samples.npz", episode=consumed_e, timestep=consumed_t)
    after = frozen_identity()
    seed_of = {row["view_key"]: row["seed"] for row in train_view.rows}
    train_counts = np.unique([seed_of[int(k)] for k in consumed_e[:, :4].ravel()], return_counts=True)
    run = dict(kind="DAPIER ACT RECEIVE candidate (SIM only)", run_id=RUN, policy_variant=policy.config.type, dataset=str(td.DATASET), train_episodes=[int(r["manifest_index"]) for r in train_view.rows],
               holdout_episodes=[], correction_segments=[{k: r[k] for k in ("id", "seed", "origin", "receive", "path")} for r in corr_view.rows], contract_revision=trial.load_manifest()["contract_revision"],
               control_dt=td.DT, hardware_access=False, task_success=None, seed=SEED, batch_size=BATCH, steps=UPDATES, warm_start=str(source),
               optimizer_state_loaded=True, optimizer_state_sha256=sha(source / "training_state.pt"), optimizer_step_at_load=int(state["step"]), optimizer_param_groups=groups,
               sample_plan_sha256=record["sample_plan"]["sha256"], consumed_sha256=trial.array_sha(consumed_e, consumed_t), consumed_equals_plan=True, samples=int(consumed_e.size),
               consumed=dict(train_pool_samples=int(consumed_e[:, :4].size), correction_pool_samples=int(consumed_e[:, 4:].size),
                             train_parents_distinct=int(len(train_counts[0])), train_parent_samples_min_max=[int(train_counts[1].min()), int(train_counts[1].max())],
                             corrections={r["id"]: int((consumed_e == r["view_key"]).sum()) for r in corr_view.rows},
                             correction_parents={str(s): int(sum((consumed_e == r["view_key"]).sum() for r in corr_view.rows if r["seed"] == s)) for s in sorted({r["seed"] for r in corr_view.rows})}),
               frozen_before=before, frozen_after=after, frozen_unchanged=before == after, started=started_wall, training_seconds=seconds, environment=trial.environment_record(),
               gpu_other_load_at_start=load_report, deterministic_warnings=sorted({str(w.message).split("\n")[0][:200] for w in caught}))
    (out / "run.json").write_text(json.dumps(run, indent=2, default=str))
    print(json.dumps(dict(seconds=seconds, consumed=run["consumed"], frozen_unchanged=run["frozen_unchanged"]), indent=1))
    print(f"[written] {name}")


# ----------------------------------------------------------------------------------------------
# Rollouts: a chain of policies, switched at the existing checker conditions (SIM ground truth)
# ----------------------------------------------------------------------------------------------

class Chain:
    """One active Student at a time; a replacement discards the old chunk and carries nothing over."""

    def __init__(self, first):
        self.active, self.retired, self.switches = first, [], []

    @property
    def failure(self):
        return self.active.failure

    def next_command(self, context):
        return self.active.next_command(context)

    def due_for_inference(self):
        return self.active.chunk is not None and self.active.chunk_index == self.active.execution_prefix

    def replace(self, successor, info):
        self.active.discard_chunk()
        self.retired.append(self.active)
        self.active = successor
        self.switches.append(info)

    def discard_chunk(self):
        self.active.discard_chunk()

    @property
    def trace(self):
        return [t for s in self.retired for t in s.trace] + self.active.trace


def load_policy(name):
    saved, lr.MODELS = lr.MODELS, {**lr.MODELS, **MODELS}
    try:
        return lr.load_policy(name)
    finally:
        lr.MODELS = saved


def gate_history(prefix_states, recorded_states, current_state, contract):
    """States the gate checker sees. The recorder holds N+1 states, so its last one already is the current state:
    "legacy" appends the current state once more (as receive-candidate-20261005-01 ran); "repaired" appends it only if the
    recorder's last state has another time stamp. Nothing else is removed: equal poses at different times stay."""
    own = list(recorded_states)
    if contract == "legacy" or not own or float(own[-1][0]) != float(current_state[0]):
        own = own + [current_state]
    return np.concatenate([prefix_states, np.stack(own)])


def execute(out, seed, chain, restore=None, label="", contract="legacy"):
    """START (or the first policy of the chain) controls; at a regular C25 boundary the next policy of the chain takes over
    when its entry condition holds. The world state is never written; the successor infers from that moment's RGB and q.

    contract="legacy" is the behaviour of receive-candidate-20261005-01, unchanged. contract="repaired" (used only by
    receive_boundary_repair) counts the current state once in the gate history and, while RECEIVE is the active policy,
    checks the same B readiness at every 50 Hz control step: when it holds, the rest of RECEIVE's chunk is discarded and B
    starts. Normal chunk consumption stays C25; no checker threshold differs between the two contracts."""
    if contract not in ("legacy", "repaired"):
        raise ValueError("unknown boundary contract")
    import mujoco
    import torch
    import arm_output_trial as trial
    import arm_projection_trial as ap
    import frozen_transfer_check as ft
    import paired_release_check as pr
    from sim_data_factory import core
    from sim_data_factory.recorder import EpisodeRecorder
    from sim_data_factory.reset_sampler import make_task_episode
    from sim_data_factory.student_eval import Observations, Student, include_terminal
    from sim_data_factory.task_success import CheckerConfig, ContactModel, evaluate_states
    trial.apply_reproducibility_settings()
    torch.set_num_threads(4)
    torch.manual_seed(4100)
    meta = ft.demonstration(seed)["metadata"]
    ctx = make_task_episode(seed, meta["requested_initial_state"]["q"], meta["sampling"]["block_xy_yaw"])
    m, d = ctx["model"], ctx["data"]
    spec = int(mujoco.mjtState.mjSTATE_INTEGRATION)
    cm = ContactModel(m)
    cfg = CheckerConfig()
    step0, restore_ok = 0, None
    prefix_states, prefix_q = np.empty((0, mujoco.mj_stateSize(m, spec))), np.empty((0, 12))
    if restore is not None:
        step0, prefix_states, prefix_q = restore["step"], restore["states"][:-1], restore["q"][:-1]
        mujoco.mj_setState(m, d, restore["states"][-1], spec)
        mujoco.mj_forward(m, d)
        back = np.empty(mujoco.mj_stateSize(m, spec))
        mujoco.mj_getState(m, d, back, spec)
        restore_ok = bool(np.array_equal(back, restore["states"][-1]))
        ctx["initial"] = core.snapshot(ctx)
        sensor = pr.stored_first_sensor(dict(rgb=restore["rgb"]))
    else:
        sensor = Observations(width=160, height=120, camera_ids=("left_wrist_rgb", "right_wrist_rgb"), depth_camera_ids=())
    ctx["substep_check"] = cm.substep_failure
    loaded = {name: load_policy(name) for name in dict.fromkeys(chain)}
    student = lambda name: Student(loaded[name][0], loaded[name][1], sensor, PREFIX, None, "project")
    loaded[chain[0]][0].reset()
    actor = Chain(student(chain[0]))
    position = 0
    rec = EpisodeRecorder(out, dict(
        contract_revision=core.REVISION, seed=seed, control_dt=ctx["control_dt"], classification=LABEL, label=label, chain=list(chain), boundary_contract=contract, hardware_access=False,
        object_attachment=False, external_forces=False, runtime_qpos_writes=0, teacher_behavior_calls=0, policies={k: v[2] for k, v in loaded.items()}, entry=ENTRY,
        restored_from=None if restore is None else dict(step=step0, restore_identity=restore_ok, first_observation="the stored RGB / q of that state"),
        execution=dict(prefix=PREFIX, arm_bound_policy="project", ensemble="off", end_sim_s=HORIZON * td.DT),
        policy_inputs=["current measured canonical q_state", "raw left/right wrist RGB"], compiled_model_sha256=ctx["compiled_model_sha256"]))
    checks, started, phase_start = [], time.time(), step0

    def history():
        states_now = gate_history(prefix_states, [s["integration_state"] for s in rec.states], core.snapshot(ctx)["integration_state"], contract)
        return evaluate_states(m, states_now, control_dt=ctx["control_dt"])["events"]

    try:
        for step in range(step0, HORIZON):
            boundary = position < len(chain) - 1 and actor.due_for_inference()
            event_check = contract == "repaired" and chain[position] == "RECEIVE" and actor.active.chunk is not None and not boundary
            if boundary or event_check:
                ev = cm.evidence(d)
                left = min(ev["pad_force_N"]["left"]) >= cfg.grip_force_min_N and ev["table_support_N"] == 0.0
                both = left and min(ev["pad_force_N"]["right"]) >= cfg.grip_force_min_N
                target, events = None, None
                if left and (boundary or both):
                    events = history()
                    if both and events.get("right_grasp") is not None:
                        target = len(chain) - 1                                          # the existing both-hands gate: B (directly, if RECEIVE was not entered yet)
                    elif boundary and chain[position + 1] == "RECEIVE" and events.get("pick_lift") is not None:
                        target = position + 1
                if boundary or events is not None:
                    checks.append(dict(step=step, time=round(float(d.time), 2), c25_boundary=bool(boundary), left_holds=bool(left), both_hands=bool(both),
                                       events=None if events is None else sorted(events), to=None if target is None else chain[target]))
                if target is not None:
                    before = np.empty(mujoco.mj_stateSize(m, spec))
                    mujoco.mj_getState(m, d, before, spec)
                    last = actor.active.trace[-1]
                    info = dict(step=step, sim_time_s=round(float(d.time), 2), frm=chain[position], to=chain[target], skipped=[chain[i] for i in range(position + 1, target)],
                                boundary_contract=contract, at_c25_boundary=bool(boundary), phase_start_step=phase_start, local_control_index=step - phase_start,
                                chunk_index_at_switch=int(actor.active.chunk_index), chunk_steps_discarded=int(PREFIX - actor.active.chunk_index),
                                events_at_switch={k: round(v, 2) for k, v in events.items()}, state_before=lr.frames(m, d, cm),
                                last_command_before=dict(q_raw=last["q_raw"], q_applied=last["q_cmd"], q_measured=last["q_measured"]))
                    loaded[chain[target]][0].reset()
                    actor.replace(student(chain[target]), info)
                    after = np.empty_like(before)
                    mujoco.mj_getState(m, d, after, spec)
                    info["world_state_unchanged_by_switch"] = bool(np.array_equal(before, after))
                    position, phase_start = target, step
            before_time, before_count = float(d.time), len(rec.commands)
            progressed = core.rollout(ctx, actor, sensor, rec, 1)
            if actor.active.trace:
                row = actor.active.trace[-1]
                row.update(ctrl_applied=d.ctrl.tolist() if d.time > before_time else None, complete_transition=len(rec.commands) > before_count, policy=chain[position])
                sw = actor.switches[-1] if actor.switches else None
                if sw is not None and "first_command_after" not in sw and sw["step"] == step:
                    prev = np.array(sw["last_command_before"]["q_applied"])
                    sw["first_command_after"] = dict(q_raw=row["q_raw"], q_applied=row["q_cmd"], chunk_index=row["selected_chunk_index"],
                                                     anchor_equals_measured=bool(np.array_equal(actor.active.chunks[0]["anchor"], np.asarray(row["q_measured"], dtype=np.float32))),
                                                     target_change_arm_max_rad=float(np.abs((np.array(row["q_cmd"]) - prev)[ARM]).max()),
                                                     target_change_gripper=[float((np.array(row["q_cmd"]) - prev)[g]) for g in (td.LEFT_GRIPPER, td.RIGHT_GRIPPER)])
            if not progressed:
                actor.discard_chunk()
                break
    finally:
        sensor.close()
    fault = ctx.get("terminal_failure")
    states = np.concatenate([prefix_states, np.stack([s["integration_state"] for s in rec.states])])
    q = np.concatenate([prefix_q, np.stack([s["q_state"] for s in rec.states])])
    check = evaluate_states(m, states, q, control_dt=ctx["control_dt"])
    rows = check.pop("rows")
    prefix_valid = check["valid"]
    check = include_terminal(check, fault)
    events = check["events"]
    g = cfg.grip_force_min_N
    L = np.array([min(r["pad_force_N"]["left"]) >= g for r in rows])
    R = np.array([min(r["pad_force_N"]["right"]) >= g for r in rows])
    off = np.array([max(r["pad_force_N"]["left"]) < cfg.release_force_max_N for r in rows])
    last = rows[-1]
    support = ("table" if last["table_support_N"] > 0 else "both hands" if L[-1] and R[-1] else "left hand" if L[-1] else "right hand" if R[-1] else "other (no two-pad hold)")
    gate = next((s for s in actor.switches if s["to"] == "B"), None)
    entry = next((s for s in actor.switches if s["to"] == "RECEIVE"), None)
    t_gate = gate["sim_time_s"] if gate else None
    release = [i for i in range(len(rows)) if off[i] and R[i] and (gate is None or i >= gate["step"])]
    retreat = [i for i in release if rows[i]["gripper_block_distance_m"]["left"] >= cfg.donor_clearance_m]
    trace = actor.trace
    dispatched = [t for t in trace if t.get("ctrl_applied") is not None]
    lo, hi = m.actuator_ctrlrange[ARM, 0], m.actuator_ctrlrange[ARM, 1]
    projection = {}
    for name in dict.fromkeys(chain):
        part = [t for t in dispatched if t["policy"] == name]
        if not part:
            projection[name] = dict(dispatched=0, projected_steps=0, longest_s=0.0, max_raw_excess_rad=0.0, per_axis={})
            continue
        mask = np.array([t["projection_mask"] for t in part], dtype=bool)
        raw = np.array([t["q_raw"] for t in part], dtype=np.float64)
        ex = np.maximum(raw[:, ARM] - hi, 0) + np.maximum(lo - raw[:, ARM], 0)
        projection[name] = dict(dispatched=len(part), projected_steps=int(mask.any(1).sum()), longest_s=round(ap.longest(mask.any(1)) * td.DT, 2),
                                max_raw_excess_rad=float(ex.max()), per_axis={m.actuator(i).name: int(c) for i, c in zip(ARM, mask.sum(0)) if c})
    # receiving window for the diagnostics: same rule for both chains, from the rows (not from the switch that only one chain has)
    lift = events.get("pick_lift")
    w_lo = step0 if restore is not None else next((k for k in range(int(round(lift / td.DT)) if lift else len(rows), len(rows)) if k % PREFIX == 0 and L[k]), None)
    w_hi = gate["step"] if gate else len(rows) - 1
    receiving = None if w_lo is None or w_hi <= w_lo else quality(m, states, rows, w_lo, w_hi)
    handover = events.get("handover")
    success = bool(check["valid"] and check["handover_success"] and support == "right hand" and (restore is None or (handover is not None and handover > step0 * td.DT)))
    result = dict(
        label=label, classification=LABEL, boundary_contract=contract, seed=seed, chain=" -> ".join(chain), start="original reset" if restore is None else f"restored stored state, step {step0} ({step0 * td.DT:.2f} s)",
        restore_identity=restore_ok, gate_source="SIM_GT", switches=actor.switches, receive_entered=entry is not None, receive_entry_s=entry["sim_time_s"] if entry else None,
        gate_reached=gate is not None, gate_s=t_gate, boundary_checks=len(checks), completed_transitions=len(rec.commands), last_completed_state_sim_s=round(float(states[-1][0]), 3),
        termination="horizon_completed" if fault is None else fault["reason"], first_fault=fault,
        fault_stage=None if fault is None else ("after the gate (B)" if gate else f"before the gate ({chain[position]} in control)"),
        events=events, pick_lift_s=lift, right_grasp_s=events.get("right_grasp"), handover_s=handover, prefix_checker_valid=prefix_valid, run_valid=bool(check["valid"]),
        terminal_handover_success=bool(check["handover_success"]), task_success=success, final_support=support, last_pad_force_N=last["pad_force_N"],
        last_block_clearance_m=last["block_bottom_clearance_m"], left_release_while_right_holds_first_s=None if not release else round(release[0] * td.DT, 2),
        left_clear_by_donor_clearance_first_s=None if not retreat else round(retreat[0] * td.DT, 2), projection=projection, receiving=receiving,
        wall_s=round(time.time() - started, 1), path=str(out))
    rec.finish(dict(chain_result=json.loads(json.dumps(result, default=str))))
    (out / "task_check_rows.json").write_text(json.dumps(rows[step0:]))
    (out / "action_trace.json").write_text(json.dumps(trace))
    chunks = [c for s in actor.retired + [actor.active] for c in s.chunks]
    if chunks:
        np.savez_compressed(out / "predicted_chunks.npz", time=np.array([c["time"] for c in chunks]), anchor=np.stack([c["anchor"] for c in chunks]), actions=np.stack([c["actions"] for c in chunks]))
    (out / "result.json").write_text(json.dumps(result, indent=2, default=str))
    return result


def one(args):
    if args.kind == "local":
        name, path, step, seed, _ = next(s for s in LOCAL_STATES if s[0] == args.id)
        out = run_dir() / "runs" / "local" / name / args.chain
        result = execute(out, seed, LOCAL_CHAINS[args.chain], stored_state(path, step), f"{name} {args.chain}")
        result.update(kind="local", state=name)
    else:
        out = run_dir() / "runs" / "full" / f"s{args.id}" / args.chain
        result = execute(out, int(args.id), CHAINS[args.chain], None, f"seed {args.id} {args.chain}")
        result.update(kind="full")
    result["chain_name"] = args.chain
    (out / "result.json").write_text(json.dumps(result, indent=2, default=str))
    print(json.dumps(dict(kind=args.kind, id=args.id, chain=result["chain"], receive=result["receive_entry_s"], gate=result["gate_s"], termination=result["termination"],
                          last_s=result["last_completed_state_sim_s"], events={k: round(v, 2) for k, v in result["events"].items()}, run_valid=result["run_valid"],
                          task_success=result["task_success"], final_support=result["final_support"]), default=str))


def table_row(r):
    fault = r["first_fault"]
    text = None if fault is None else f"{fault['time']:.2f} s {fault['reason']} {fault.get('forbidden_contacts') or fault.get('detail') or ''}".strip()
    if fault is None and not r["task_success"]:
        text = f"no fault; ended at 40 s with {r['final_support']}"
    q = r["receiving"] or {}
    return dict(kind=r["kind"], id=r.get("state") or r["seed"], chain=r["chain"], pick_lift_s=r["pick_lift_s"], receive_entry_s=r["receive_entry_s"], gate_s=r["gate_s"],
                failure_before_gate=None if r["gate_reached"] else text, failure_after_gate=text if r["gate_reached"] else None, handover_s=r["handover_s"], run_valid=r["run_valid"],
                final_support=r["final_support"], task_success=r["task_success"],
                projection={k: f"{v['projected_steps']}/{v['dispatched']} (max {v['max_raw_excess_rad']:.3f} rad)" for k, v in r["projection"].items()},
                switch_target_change_arm_max_rad={s["to"]: round(s["first_command_after"]["target_change_arm_max_rad"], 4) for s in r["switches"] if "first_command_after" in s},
                world_unchanged_by_switch=[s["world_state_unchanged_by_switch"] for s in r["switches"]],
                right_push=q.get("right_before_two_pad_hold"), right_two_pad_hold=q.get("right_two_pad_hold"), right_pad_peak_N=q.get("right_pad_peak_N"),
                block_in_left=q.get("block_in_left_site"), left_hold_lost_steps=q.get("left_hold_lost_steps"),
                left_release_s=r["left_release_while_right_holds_first_s"], left_retreat_s=r["left_clear_by_donor_clearance_first_s"])


def teacher_reference():
    """The same receiving diagnostics on teacher material: the three Train demonstrations and the corrections used."""
    import frozen_transfer_check as ft
    from integration_scenes import task_env
    m = task_env("desk", grippers="both").model
    out = {}
    for seed in FULL_SEEDS:
        demo = ft.demonstration(seed)
        a, b, why = receive_bounds(demo["rows"], demo["events"])
        out[f"demonstration s{seed}"] = why or quality(m, demo["integration_state"], demo["rows"], a, b)
    for e in correction_entries()[0]:
        with np.load(Path(e["path"]) / "episode.npz", allow_pickle=False) as z:
            states = z["integration_state"].copy()
        out[f"correction {e['id']}"] = quality(m, states, load(Path(e["path"]) / "task_check_rows.json"), e["receive"][0], e["receive"][1])
    return out


def run(args):
    if not (MODELS["RECEIVE"] / "model.safetensors").exists():
        raise FileNotFoundError("RECEIVE step-005000 does not exist: run teacher, protocol and train first")
    record = load(run_dir() / "protocol.json")
    before = frozen_identity()
    jobs = []
    for i, (name, *_rest) in enumerate(LOCAL_STATES):
        jobs += [("local", name, c) for c in (("control", "candidate") if i % 2 == 0 else ("candidate", "control"))]
    for i, seed in enumerate(FULL_SEEDS):
        jobs += [("full", str(seed), c) for c in (("control", "candidate") if i % 2 == 0 else ("candidate", "control"))]
    (run_dir() / "logs").mkdir(exist_ok=True)
    for kind, ident, chain in jobs:
        out = run_dir() / "runs" / kind / (ident if kind == "local" else f"s{ident}") / chain
        if (out / "result.json").exists():
            print(f"[reused] {kind} {ident} {chain}")
            continue
        done = subprocess.run([sys.executable, str(Path(__file__).resolve()), "one", "--kind", kind, "--id", ident, "--chain", chain], capture_output=True, text=True)
        (run_dir() / "logs" / f"{kind}_{ident}_{chain}.log").write_text(done.stdout + "\n--- stderr ---\n" + done.stderr)
        line = [x for x in done.stdout.splitlines() if x.startswith("{")]
        print(line[-1] if line else f"[INVALID EXECUTION, not a task result] {kind} {ident} {chain}: {done.stderr[-500:]}", flush=True)
    results = [load(p) for p in sorted((run_dir() / "runs").glob("*/*/*/result.json"))]
    order = {(k, i, c): n for n, (k, i, c) in enumerate(jobs)}
    results.sort(key=lambda r: order[(r["kind"], r.get("state") or str(r["seed"]), r["chain_name"])])
    after = frozen_identity()
    (run_dir() / "results.json").write_text(json.dumps(dict(
        run_id=RUN, note=f"{LABEL}; switches read SIM ground truth; Train parents only; one run per condition, not a success rate",
        frozen_unchanged=before == after == record["frozen"], frozen=after, table=[table_row(r) for r in results], teacher_reference=teacher_reference(), results=results), indent=2, default=str))
    for row in (table_row(r) for r in results):
        print(json.dumps({k: row[k] for k in ("kind", "id", "chain", "pick_lift_s", "receive_entry_s", "gate_s", "failure_before_gate", "failure_after_gate", "handover_s", "final_support", "task_success")}, default=str))
    print(f"[written] {run_dir() / 'results.json'} ({len(results)} of {len(jobs)} runs; START / B unchanged: {before == after == record['frozen']})")


def video(args):
    import recovery_trial as rt
    for name, folder, title in (("REPLAY1_0900_control_candidate.mp4", run_dir() / "runs/local/REPLAY1_0900", "REPLAY/run1 step 900 (time from the restored state)"),
                                ("seed5000_original_reset_control_candidate.mp4", run_dir() / "runs/full/s5000", "seed 5000 original reset")):
        out = run_dir() / "videos" / name
        if out.exists():
            print(f"[reused] {out}")
            continue
        rows = []
        for chain in ("control", "candidate"):
            if not (folder / chain / "result.json").exists():
                print(f"[unavailable] {folder / chain}: no result; this row is not replaced by another run")
                continue
            r = load(folder / chain / "result.json")
            row = rt.teacher_row(f"{LABEL}: {chain.upper()} {r['chain']}", folder / chain)
            ev = dict(r["events"])
            ev.update({f"SWITCH->{s['to']}": s["sim_time_s"] for s in r["switches"]})
            row["events"] = dict(sorted(((k, v) for k, v in ev.items() if v >= row["start"] - 1e-9), key=lambda kv: kv[1]))
            row["end_text"] = (f"ENDED at SIM {r['last_completed_state_sim_s']:.2f}s: {r['termination']}" + ("" if r["gate_reached"] else " (B GATE NOT REACHED)"),
                               "VALID HANDOVER, right hand holds" if r["task_success"] else f"no valid handover; final support: {r['final_support']}")
            rows.append(row)
        if rows:
            rt.render(rows, out, f"{LABEL} / {title}", 2000, False, right_gripper=True)
            print(f"[written] {out}")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    for name in ("teacher", "protocol", "train", "run", "video"):
        sub.add_parser(name)
    o = sub.add_parser("one")
    o.add_argument("--kind", choices=("local", "full"), required=True)
    o.add_argument("--id", required=True)
    o.add_argument("--chain", choices=("control", "candidate"), required=True)
    args = parser.parse_args(argv)
    import arm_output_trial  # noqa: F401
    dict(teacher=teacher, protocol=protocol, train=train, run=run, one=one, video=video)[args.command](args)


if __name__ == "__main__":
    main()
