#!/usr/bin/env python3
"""TJJ MVP R2 / work6: perception repair, phase regression of the 3-view candidates, and the full MVP chain from the
original reset with sensor-driven transitions (SIM only; no hardware; no Val / Test in training; Test not opened).

  perception  the v1 and the v2 object selector on the 69 Train reset frames (stored top depth) -> perception_before_after.json
  phase       oracle-gated reference runs of the 3-view candidates on the fixed development sets (46 new runs); the 2-view
              baseline is read from the stored RECEIVE-EXPAND R1 runs, not run again
  chain       OBSERVE -> START_3V -> RECEIVE_3V -> RELEASE_B -> PLACE -> ARM_STOW -> ORIGINAL_AREA_REOBSERVE ->
              CLEAN_RESUME_REQUEST from the original reset on pre-set cases. Every transition and every motion command is
              decided from the top RGB-D, the wrist RGB and the measured joints. Simulator truth is read only by the
              evaluator after the run and by the unchanged contact checker that ends a run on a forbidden contact.
  all         every step that is not complete yet (training is tjj_mvp_r2_train.py)

SIM_ONLY / REAL_MODALITY_UNVERIFIED. START_3V / RECEIVE_3V are candidates; no default model is replaced.
"""
import argparse
import hashlib
import json
import os
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import train_rollout_diagnostic as td   # noqa: E402
import receive_candidate as rc   # noqa: E402
import receive_boundary_repair as rb   # noqa: E402
import receive_r3_recheck as rk   # noqa: E402
import start_pick_d1 as d1   # noqa: E402
import start_pick_r1 as p1   # noqa: E402
import receive_bridge_r1 as br   # noqa: E402
import constraint_causal_c0 as c0   # noqa: E402
import post_c0_pilot_r1 as pr1   # noqa: E402
import receive_expand_r1 as ex   # noqa: E402
import tjj_alignment_r1 as al   # noqa: E402
import tjj_mvp_r2_train as tr   # noqa: E402
import tjj_perception as tp   # noqa: E402
import tjj_task_manager as tm   # noqa: E402
import server_s0 as s0   # noqa: E402

FACTORY_ROOT = td.FACTORY_ROOT
RUN = tr.RUN
TITLE = "TJJ MVP R2 / work6 / 3-view candidates, sensor perception, PLACE + ARM_STOW, full chain from the original reset"
LABEL_PHASE = "student-action / SIM-ORACLE-GATED reference (contract G) / TJJ MVP R2 phase regression"
LABEL_CHAIN = "student-action / SENSOR-GATED / TJJ MVP R2 full chain"
TOP, TOP_KEY, CAMERAS, TABLE_Z, ROI = al.TOP, al.TOP_KEY, al.CAMERAS, al.TABLE_Z, al.ROI
WORKERS = 2
CHAIN_CASES = (5035, 5040, 5109, 5119)                 # Train parents fixed before any run: two of G000, two of G001 (the source-fit parents)
TARGET = tm.Region("place-target-fixed-v1", "world", (0.28, -0.06), 0.04, TABLE_Z, fixed=True)
SOURCE_RADIUS = 0.05
HOLD_BAND = (0.74, 0.80)                               # measured gripper opening while a 40 mm block is held: 0.765-0.79 in the Train69 demonstrations (1st-99th percentile)
STABLE_STEPS, LIFT_M, LIFT_STEPS, RELEASED, APART_M, DONE_STEPS = 10, 0.015, 15, 0.81, 0.05, 50
ARMED_OPEN = 0.805                                     # above the hold band: the demonstrations open the left gripper to ~0.84 before the grasp and to ~0.84 again at the release
DEV_CASES = (5121, 5112)                               # development parents for wiring this tool; never part of the pre-set evaluation
HANDOVER_LIMIT, TOTAL_LIMIT = 2000, 3200               # 40 s for START .. RELEASE_B as before; 64 s for the whole chain
STOW_REFERENCE_SEED = 5000                             # the stow pose is the reset pose of Train 5000 (both arms raised to the sides, outside the top view)
SOURCES = ex.SOURCES + ("factory/v1/tjj_mvp_r2.py", "factory/v1/tjj_mvp_r2_train.py", "factory/v1/tjj_alignment_r1.py", "factory/v1/tjj_perception.py", "factory/v1/tjj_task_manager.py")

load, sha, array_sha = rk.load, rk.sha, rk.array_sha


def run_dir():
    return FACTORY_ROOT / "evaluation" / RUN


def models():
    m = c0.bridge_models()
    for role, name in (("start", "START_3V"), ("receive", "RECEIVE_3V")):
        run = tr.trained(role)
        if run is None:
            raise RuntimeError(f"{name} is not trained")
        m[name] = dict(path=str(tr.checkpoint(role)), model=run["model_sha256"], config=sha(tr.checkpoint(role) / "config.json"), normalization=sha(tr.training_dir(role) / "normalization.npz"))
    return m


# ----------------------------------------------------------------------------------------------
# Kinematic self model (what a URDF + calibrated camera gives on the robot): FK and self exclusion from measured joints
# ----------------------------------------------------------------------------------------------

class SelfModel:
    def __init__(self, model):
        import mujoco
        from sim_data_factory.teacher import Planner
        self.m, self.plan = model, Planner(model)
        self.geoms = [g for g in range(model.ngeom) if model.body(int(model.geom_bodyid[g])).name.startswith(("left_", "right_"))]
        self.block_adr = int(model.jnt_qposadr[model.body_jntadr[model.body("red_block").id]])
        self.mujoco = mujoco

    def base(self, block_xyz=None):
        """A planning state made from nothing but the model's reference pose; the object is put where perception saw it (or far away)."""
        q = np.array(self.m.qpos0, dtype=float)
        q[self.block_adr:self.block_adr + 3] = (5.0, 5.0, 0.02) if block_xyz is None else block_xyz
        q[self.block_adr + 3:self.block_adr + 7] = (1, 0, 0, 0)
        return q

    def pose(self, q_state, block_xyz=None):
        from sim_data_factory import core
        q = np.clip(np.asarray(q_state, dtype=float), np.r_[self.m.actuator_ctrlrange[:5, 0], 0, self.m.actuator_ctrlrange[6:11, 0], 0], np.r_[self.m.actuator_ctrlrange[:5, 1], 1, self.m.actuator_ctrlrange[6:11, 1], 1])
        gear = core.decode_command(q, self.m)
        return self.plan.pose(self.base(block_xyz), gear), gear

    def fk(self, q_state):
        self.pose(q_state)
        return {s: self.plan.site_frame(s) for s in ("left", "right")}

    def robot_mask(self, q_state, margin=0.012):
        pd, _ = self.pose(q_state)
        boxes = []
        for g in self.geoms:
            aabb = self.m.geom_aabb[g]
            boxes.append((pd.geom_xpos[g].copy(), pd.geom_xmat[g].reshape(3, 3).copy(), aabb[:3].copy(), aabb[3:].copy() + margin))

        def mask(points):
            out = np.zeros(len(points), dtype=bool)
            for pos, rot, centre, half in boxes:
                local = (points - pos) @ rot - centre
                out |= np.all(np.abs(local) <= half, axis=1)
            return out

        return mask


# ----------------------------------------------------------------------------------------------
# Perception before / after on the stored Train reset frames
# ----------------------------------------------------------------------------------------------

def perception(args=None):
    import cv2
    import mujoco
    target = run_dir() / "perception_before_after.json"
    if target.exists():
        return load(target)
    run_dir().mkdir(parents=True, exist_ok=True)
    model = al.scene_model()
    me, data = SelfModel(model), mujoco.MjData(model)
    block = model.body("red_block").id
    rows, tiles = [], []
    for e in [x for x in d1.manifest() if x["split"] == "train" and x["training_eligible"]]:
        path = Path(e["path"])
        with np.load(path / "cameras" / f"{TOP}.npz", allow_pickle=False) as c:
            depth, K, T, rgb = c["depth_m"][0], c["K"][0], c["T_world_camera_optical"][0], c["rgb"][0]
        with np.load(path / "episode.npz", allow_pickle=False) as a:
            state, q = a["integration_state"][0], a["q_state"][0]
        mujoco.mj_setState(model, data, state, int(mujoco.mjtState.mjSTATE_INTEGRATION))
        mujoco.mj_forward(model, data)
        truth = data.xpos[block].copy()                                       # evaluator only
        before, _ = tp.observe_object(depth, K, T, 0.0, surface_z=TABLE_Z, roi=ROI)
        best, cands, counts = tp.select_object(depth, K, T, surface_z=TABLE_Z, roi=ROI, robot_mask=me.robot_mask(q))
        err = lambda xy: None if xy is None else round(1000 * float(np.hypot(xy[0] - truth[0], xy[1] - truth[1])), 2)
        row = dict(seed=e["seed"], before_xy_error_mm=err(None if before is None else before.position_xyz), after_xy_error_mm=err(None if best is None else best["centroid_xyz"]),
                   after_top_height_mm=None if best is None else round(1000 * best["top_height_m"], 1), candidates=len(cands), removed_as_robot=counts["removed_as_robot"])
        rows.append(row)
        if row["before_xy_error_mm"] is not None and row["before_xy_error_mm"] > 20 and len(tiles) < 19:
            img = cv2.resize(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), (320, 240), interpolation=cv2.INTER_NEAREST)

            def px(w):
                pc = np.linalg.inv(T) @ np.r_[w, 1]
                return int(2 * (K[0, 0] * pc[0] / pc[2] + K[0, 2])), int(2 * (K[1, 1] * pc[1] / pc[2] + K[1, 2]))

            cv2.circle(img, px(truth), 5, (0, 255, 0), 1)                     # green circle: stored block position (evaluator)
            if before is not None:
                cv2.drawMarker(img, px(np.array(before.position_xyz)), (0, 0, 255), cv2.MARKER_TILTED_CROSS, 9, 1)     # red x: v1 selection
            if best is not None:
                cv2.drawMarker(img, px(np.array(best["centroid_xyz"])), (255, 128, 0), cv2.MARKER_CROSS, 11, 1)        # blue +: v2 selection
            cv2.putText(img, f"{e['seed']} v1 {row['before_xy_error_mm']} v2 {row['after_xy_error_mm']} mm", (4, 12), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (255, 0, 0), 1)
            tiles.append(img)
    while len(tiles) % 5:
        tiles.append(np.zeros_like(tiles[0]))
    (run_dir() / "frames").mkdir(exist_ok=True)
    cv2.imwrite(str(run_dir() / "frames" / "object_selection_before_after.png"), np.concatenate([np.concatenate(tiles[i:i + 5], 1) for i in range(0, len(tiles), 5)], 0))
    stat = lambda key: (lambda v: dict(detected=len(v), median=round(float(np.median(v)), 2), p90=round(float(np.percentile(v, 90)), 2), max=round(float(v.max()), 2), over_20mm=int((v > 20).sum()),
                                       over_10mm=int((v > 10).sum())))(np.array([r[key] for r in rows if r[key] is not None]))
    data_ = dict(run_id=RUN, written=time.strftime("%Y-%m-%dT%H:%M:%S%z"), modality=al.MODALITY, frames="reset frame of each Train69 demonstration (stored top depth + K + T, stored q_state)", episodes=len(rows),
                 before=dict(rule="v1: largest cluster 8-120 mm above the table with XY extent <= 90 mm; top face = points within 4 mm of the highest point", xy_error_mm=stat("before_xy_error_mm")),
                 after=dict(rule="v2: table plane removed (height band above the known surface), workspace ROI, points inside the robot's own link boxes (forward kinematics of the measured joints, 12 mm margin) "
                                 "removed, clusters that are object-sized and not cut by the height band preferred, top face = highest height bin holding >= 15 % of the cluster's points",
                            xy_error_mm=stat("after_xy_error_mm"), not_detected=[r["seed"] for r in rows if r["after_xy_error_mm"] is None]),
                 gross_error_causes=dict(edge_pixels="a few pixels at a block edge read 9-11 mm above the top face; v1 took them as the top face, so the centroid moved ~20 mm to that corner",
                                         arm_selected="an arm or gripper inside the ROI was a larger 'small' cluster than the block"),
                 inputs="top depth + K + T + measured joints + the robot's kinematic model; the stored block pose is read only to score", calibrated=False,
                 real_robot_note="the same rule needs the real camera's K / T and a URDF of the arms; both are unverified for the real OS30A (PROVISIONAL)",
                 sheet=dict(file=str(run_dir() / "frames" / "object_selection_before_after.png"), legend="green circle = stored block position (evaluator), red x = v1 selection, blue + = v2 selection; the frames v1 got wrong by > 20 mm"),
                 rows=rows)
    target.write_text(json.dumps(data_, indent=2, default=str))
    print(json.dumps(dict(before=data_["before"]["xy_error_mm"], after=data_["after"]["xy_error_mm"], not_detected=data_["after"]["not_detected"])))
    return data_


# ----------------------------------------------------------------------------------------------
# Three-camera observation for every run of this tool
# ----------------------------------------------------------------------------------------------

def install_three_cameras():
    from sim_data_factory import student_eval
    Base, base_observation, current = student_eval.Observations, student_eval.policy_observation, {}

    class ThreeCameraObservations(Base):
        def __init__(self, *a, **k):
            super().__init__(width=160, height=120, camera_ids=CAMERAS, depth_camera_ids=(TOP,))

        def observe(self, m, d, c):
            got = super().observe(m, d, c)
            current.update(top=got[TOP])
            return got

    def observation3(q, wrist_rgb, stats, *, device=None):
        b = base_observation(q, wrist_rgb, stats, device=device)
        b[TOP_KEY] = al.image_tensor(current["top"]["rgb"], device=device)   # a 2-view policy ignores this key; a 3-view policy reads it
        return b

    student_eval.Observations, student_eval.policy_observation = ThreeCameraObservations, observation3
    return current


def cuda_record(loaded, profile_case=None):
    return pr1.cuda_evidence({k: v for k, v in loaded.items()}, stored_case=None) | dict(image_features={k: list(v[0].config.image_features) for k, v in loaded.items()})


# ----------------------------------------------------------------------------------------------
# Phase regression (oracle-gated reference mode, contract G)
# ----------------------------------------------------------------------------------------------

def phase_cases():
    fit = load(ex.run_dir() / "training_manifest.json")["source_fit"]["parents"]
    out = [f"phase:start3v:{b}:{s}" for b, seeds in (("train9", p1.TRAIN9), ("val10", p1.VAL10)) for s in seeds]
    out += [f"phase:receive3v:{b}:{s}" for b, seeds in (("train9", p1.TRAIN9), ("sourcefit", fit), ("val10", p1.VAL10), ("probe", (5000, 5008))) for s in seeds]
    return out


def identity():
    import arm_output_trial as trial
    body = dict(run_id=RUN, models=models(), manifest_sha256=p1.MANIFEST_SHA, phase_cases=phase_cases(), chain_cases=list(CHAIN_CASES), sources={p: sha(FACTORY_ROOT / p) for p in SOURCES},
                contract=rb.CONTRACT_TEXT["REPAIRED_RECEIVE"], target_region=dict(center_xy=list(TARGET.center_xy), radius_m=TARGET.radius_m),
                gates=dict(hold_band=list(HOLD_BAND), stable_steps=STABLE_STEPS, lift_m=LIFT_M, lift_steps=LIFT_STEPS, released=RELEASED, apart_m=APART_M, done_steps=DONE_STEPS),
                backend=trial.environment_record(), host=s0.host(), environment={k: os.environ.get(k) for k in rk.ENV_KEYS})
    body["sha256"] = hashlib.sha256(json.dumps(body, sort_keys=True).encode()).hexdigest()
    return body


def case_dir(ident, case):
    return run_dir() / "runs" / ident["sha256"][:12] / case.replace(":", "_")


def complete(ident, case):
    out = case_dir(ident, case)
    try:
        r, meta = load(out / "result.json"), load(out / "metadata.json")
    except (FileNotFoundError, json.JSONDecodeError):
        return None
    return r if r.get("identity_sha256") == ident["sha256"] and r.get("mvp_case") == case and meta.get("complete") else None


def phase_one(case, ident, out):
    _, kind, bundle, seed = case.split(":")
    seed = int(seed)
    started, timers = time.time(), {}
    log, _ = pr1.install_runtime(dict(gate50=True), timers)                   # contract G of POST-C0 (SIM-truth switch): reference mode only
    install_three_cameras()
    m = ident["models"]
    start = "START_3V" if kind == "start3v" else ("START_FROZEN" if bundle == "probe" else "START_R1_E")
    rc.MODELS["START"], rc.MODELS["RECEIVE"] = Path(m[start]["path"]), Path(m["RECEIVE_3V"]["path"])
    loaded, original = {}, rc.load_policy

    def remember(name):
        got = original(name)
        loaded[name] = got
        return got

    rc.load_policy = remember
    try:
        result = rc.execute(out, seed, ("START", "RECEIVE", "B"), None, f"{LABEL_PHASE}: {case} {start} -> RECEIVE_3V -> RELEASE_B", contract="repaired")
    finally:
        rc.load_policy = original
    got = {k: v[2]["model_sha256"] for k, v in loaded.items()}
    if got.get("START") != m[start]["model"] or got.get("RECEIVE", m["RECEIVE_3V"]["model"]) != m["RECEIVE_3V"]["model"]:
        raise RuntimeError(f"a loaded policy is not the intended one: {got}")
    rows = load(out / "task_check_rows.json")
    entry = next((s for s in result["switches"] if s["to"] == "RECEIVE"), None)
    gate = next((s for s in result["switches"] if s["to"] == "B"), None)
    strict = bool(result["task_success"]) and log["faults"] == 0
    fault = result["first_fault"]
    result.update(job=RUN, classification=LABEL_PHASE, mvp_case=case, spec=dict(kind=kind, bundle=bundle, seed=seed, start=start, receive="RECEIVE_3V"), identity_sha256=ident["sha256"],
                  strict_valid_handover=strict, first_failure_stage=None if strict else ("START" if entry is None else "RECEIVE" if gate is None else "RELEASE_B"),
                  first_failure_kind=None if strict else ("fault" if fault is not None else "task not completed within the 40 s horizon"), ready_s=None if gate is None else gate["sim_time_s"],
                  right_first_touch_s=p1.contact_summary(rows, result)["right"]["first_touch_s"], cameras=al.camera_row(out)["cameras"], cuda=cuda_record(loaded),
                  timing=dict(wall_s=round(time.time() - started, 1), checker_s=round(timers.get("checker_s", 0.0), 2)),
                  child_settings=dict(rk.child_settings({k: v[0] for k, v in loaded.items()}), host=s0.host(), workers=WORKERS))
    (out / "result.json").write_text(json.dumps(result, indent=2, default=str))
    print(json.dumps(dict(case=case, pick=result["pick_lift_s"], entry=result["receive_entry_s"], ready=result["ready_s"], handover=result["handover_s"], strict=strict, stage=result["first_failure_stage"]), default=str))


# ----------------------------------------------------------------------------------------------
# Sensor-driven chain
# ----------------------------------------------------------------------------------------------

class Gates:
    """Transitions from joint feedback, forward kinematics and the top depth. No contact force, no simulator state."""

    def __init__(self, me):
        self.me, self.hist, self.armed, self.count = me, [], dict(left=False, right=False), {}
        self.hold_start_z = None

    def update(self, q):
        fk = self.me.fk(q)
        self.hist.append(dict(left=float(q[5]), right=float(q[11]), lz=float(fk["left"][0][2]), apart=float(np.linalg.norm(fk["left"][0] - fk["right"][0]))))
        for side in ("left", "right"):
            if self.hist[-1][side] >= ARMED_OPEN:
                self.armed[side] = True                                         # the gripper was open before it closed on something
        return fk

    def holding(self, side):
        h = self.hist[-STABLE_STEPS:]
        return (self.armed[side] and len(h) == STABLE_STEPS and all(HOLD_BAND[0] <= x[side] <= HOLD_BAND[1] for x in h)
                and max(x[side] for x in h) - min(x[side] for x in h) < 0.01)

    def run(self, name, ok, steps):
        self.count[name] = self.count.get(name, 0) + 1 if ok else 0
        return self.count[name] >= steps

    def picked_and_lifted(self, source_occupied):
        """Left gripper stopped on an object-wide opening, the left tool point rose LIFT_M since then, and the top depth no longer shows the object where it was observed."""
        hold = self.holding("left")
        if not hold:
            self.hold_start_z = None
            self.count["lift"] = 0
            return False, dict(holding=False)
        if self.hold_start_z is None:
            self.hold_start_z = min(x["lz"] for x in self.hist[-STABLE_STEPS:])
        rise = self.hist[-1]["lz"] - self.hold_start_z
        ok = self.run("lift", rise >= LIFT_M and not source_occupied, LIFT_STEPS)
        return ok, dict(holding=True, rise_m=round(rise, 4), object_still_at_source=bool(source_occupied))

    def both_hold(self):
        return self.holding("left") and self.holding("right")

    def handed_over(self):
        h = self.hist[-1]
        return self.run("done", h["left"] >= RELEASED and h["apart"] >= APART_M and self.holding("right"), DONE_STEPS)


class Script:
    """A joint-space trajectory actor (deterministic planner output), used for PLACE and ARM_STOW."""

    def __init__(self, model, segments, controller):
        self.m, self.segments, self.controller = model, segments, controller
        self.failure, self.done, self.index, self.t, self.phase, self.log = None, not segments, 0, 0.0, None, []

    def next_command(self, context):
        from sim_data_factory import core
        name, a, b, T = self.segments[self.index]
        if self.phase != name:
            self.phase = name
            self.log.append(dict(phase=name, time=float(context["data"].time)))
        u = min(1.0, self.t / T)
        gear = a + (b - a) * (35 * u**4 - 84 * u**5 + 70 * u**6 - 20 * u**7)
        self.t += float(context["control_dt"])
        if self.t >= T:
            self.t = 0.0
            if self.index + 1 < len(self.segments):
                self.index += 1
            else:
                self.done = True
        q = gear.copy()
        q[[5, 11]] = q[[5, 11]] / core.MOTOR_MAX_RAD
        lo, hi = np.r_[self.m.actuator_ctrlrange[:5, 0], 0, self.m.actuator_ctrlrange[6:11, 0], 0], np.r_[self.m.actuator_ctrlrange[:5, 1], 1, self.m.actuator_ctrlrange[6:11, 1], 1]
        return np.clip(q, lo, hi), dict(phase=name, controller=self.controller)


def held_object(me, q, top, h_obj):
    """Where the object in the right hand is, from the top depth (robot's own boxes removed) or, if the hand hides it, from
    the grasp point of the measured joints. Returns (centre xyz, source of the estimate, details)."""
    p, rot = me.fk(q)["right"]
    pts = tp.back_project(top["depth_m"], top["K"], top["T_world_camera_optical"])
    near = (np.linalg.norm(pts - p, axis=1) <= 0.06) & (pts[:, 2] >= TABLE_Z + 0.015)
    cand = pts[near & ~me.robot_mask(q, margin=0.006)(pts)]
    if len(cand) >= 20:
        top_h, face = tp.robust_top(cand, TABLE_Z)
        centre = np.array([face[:, 0].mean(), face[:, 1].mean(), TABLE_Z + top_h - h_obj / 2])
        return centre, "top depth near the right tool point", dict(points=int(len(cand)), top_height_m=round(top_h, 4))
    return p.copy(), "grasp point of the measured joints (object not visible to the top camera)", dict(points=int(len(cand)))


def plan_place(me, q, gear, top, h_obj, region):
    """PLACE plan from sensors only: carry the held object over the fixed target, lower until its bottom is 3 mm above the
    table (object height from the first observation), open, retreat."""
    from sim_data_factory.teacher import TeacherConfig, UP
    plan, cfg = me.plan, TeacherConfig()
    centre, how, detail = held_object(me, q, top, h_obj)
    base = me.base(centre)
    plan.pose(base, gear)
    p, rot = plan.site_frame("right")
    rel = rot.T @ (centre - p)
    a_cur, n_cur, rr_now = rot[:, 0], rot[:, 2], plan.radial("right", p)
    info = dict(object_estimate_xyz=centre.round(4).tolist(), object_estimate_from=how, estimate_detail=detail, object_in_right_tool_mm=(1000 * rel).round(1).tolist(), object_height_m=round(h_obj, 4))

    def goal_for(block_centre):
        rr = plan.radial("right", np.array([block_centre[0], block_centre[1], p[2]]))
        yaw = float(np.arctan2(rr[1], rr[0]) - np.arctan2(rr_now[1], rr_now[0]))
        Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
        ctrl, ik = plan.solve(base, gear, "right", np.asarray(block_centre) - (Rz @ rot) @ rel, Rz @ a_cur, [Rz @ n_cur, -(Rz @ n_cur)], cfg)
        return ctrl, ik

    g1, ik1 = goal_for((region.center_xy[0], region.center_xy[1], TABLE_Z + h_obj / 2 + 0.06))
    g2, ik2 = goal_for((region.center_xy[0], region.center_xy[1], TABLE_Z + h_obj / 2 + 0.003))
    info.update(ik_hover=ik1 if isinstance(ik1, dict) else str(ik1), ik_down=ik2 if isinstance(ik2, dict) else str(ik2))
    if g1 is None or g2 is None:
        return None, info
    c1, c2 = gear.copy(), gear.copy()
    c1[6:11], c2[6:11] = g1[6:11], g2[6:11]
    c3 = c2.copy()
    c3[11] = float(me.m.actuator_ctrlrange[11, 1])
    plan.pose(base, c3)
    p3, rot3 = plan.site_frame("right")
    back, tried = None, []
    placed_base = me.base((region.center_xy[0], region.center_xy[1], TABLE_Z + h_obj / 2))          # the object where it is being put (a plan, not a reading)
    for along, up in ((0.06, 0.03), (0.03, 0.05), (0.0, 0.06), (0.03, 0.03), (0.0, 0.03), (0.02, 0.02)):
        back, ikb = plan.solve(base, c3, "right", p3 - along * rot3[:, 0] + np.array([0, 0, up]), rot3[:, 0], [rot3[:, 2], -rot3[:, 2]], cfg)
        hit = None
        if back is not None:
            cand = c3.copy()
            cand[6:11] = back[6:11]
            hit = plan.path_collisions(placed_base, c3, cand, block_qpos=None)      # the planner's own preview: arm-arm, self and table contacts
            if hit is not None:
                back = None
        tried.append(dict(back_m=along, up_m=up, ik=ikb if isinstance(ikb, dict) else str(ikb), preview=None if hit is None else str(hit)[:160], ok=back is not None))
        if back is not None:
            break
    c4 = c3.copy()
    if back is not None:
        c4[6:11] = back[6:11]
    info.update(retreat=tried, retreat_planned=back is not None)
    if back is None:
        return None, info
    dur = lambda a, b: max(0.8, float(np.abs(b - a).max()) / 0.6)
    return [("carry", gear.copy(), c1, dur(gear, c1)), ("lower", c1, c2, dur(c1, c2)), ("open", c2, c3, 0.8), ("settle", c3, c3, 0.5), ("retreat", c3, c4, dur(c3, c4)), ("hold", c4, c4, 0.6)], info


def stow_pose(model):
    from sim_data_factory import core
    import frozen_transfer_check as ft
    q0 = np.asarray(ft.demonstration(STOW_REFERENCE_SEED)["metadata"]["requested_initial_state"]["q"], dtype=float)
    gear = core.decode_command(q0, model)
    return gear, q0


def plan_stow(me, gear, placed_xyz):
    """Both arms to the stow pose: the right arm first (it stands next to the placed object), each arm in two moves (lift the
    upper joints, then pan / roll). The object is where PLACE verification saw it; previews are recorded, physics decides."""
    target, q0 = stow_pose(me.m)
    plan, base = me.plan, me.base(placed_xyz)
    segs, previews, cur = [], [], gear.copy()
    for side, o in (("right", 6), ("left", 0)):
        a = cur.copy()
        b = a.copy()
        b[o + 1:o + 4] = target[o + 1:o + 4]                                 # shoulder lift, elbow, wrist flex first: the hand goes up and back
        c = b.copy()
        c[o:o + 5] = target[o:o + 5]
        c[o + 5] = target[o + 5]
        for name, x, y in ((f"{side}_lift", a, b), (f"{side}_swing", b, c)):
            previews.append(dict(move=name, preview=str(plan.path_collisions(base, x, y, block_qpos=None))))
            segs.append((name, x, y, max(0.8, float(np.abs(y - x).max()) / 0.6)))
        cur = c
    segs.append(("stow_hold", cur, cur, 0.6))
    return segs, dict(reference=f"reset pose of Train {STOW_REFERENCE_SEED}", stow_q=q0.round(4).tolist(), order=["right", "left"], previews=previews)


def chain_one(case, ident, out):
    import mujoco
    import torch
    import arm_output_trial as trial
    import frozen_transfer_check as ft
    from sim_data_factory import core, student_eval
    from sim_data_factory.recorder import EpisodeRecorder
    from sim_data_factory.reset_sampler import make_task_episode
    from sim_data_factory.task_success import ContactModel, evaluate_states
    seed = int(case.split(":")[1])
    started = time.time()
    current = install_three_cameras()
    trial.apply_reproducibility_settings()
    torch.set_num_threads(4)
    torch.manual_seed(4100)
    meta = ft.demonstration(seed)["metadata"]
    ctx = make_task_episode(seed, meta["requested_initial_state"]["q"], meta["sampling"]["block_xy_yaw"])
    m, d = ctx["model"], ctx["data"]
    cm = ContactModel(m)
    ctx["substep_check"] = cm.substep_failure                                 # the unchanged contact checker ends the run on a forbidden contact
    me = SelfModel(m)
    mods = ident["models"]
    rc.MODELS.update(START=Path(mods["START_3V"]["path"]), RECEIVE=Path(mods["RECEIVE_3V"]["path"]))
    loaded = {k: rc.load_policy(k) for k in ("START", "RECEIVE", "B")}
    sensor = student_eval.Observations()
    rec = EpisodeRecorder(out, dict(contract_revision=core.REVISION, observation_revision="os30a-mono-v1", seed=seed, control_dt=ctx["control_dt"], classification=LABEL_CHAIN, hardware_access=False,
                                    object_attachment=False, external_forces=False, runtime_qpos_writes=0, policies={k: v[2] for k, v in loaded.items()}, cameras_recorded=list(CAMERAS),
                                    transitions_decided_by="top RGB-D, wrist RGB (policy input), measured joints; no simulator state"))
    events, stage_log, frames = [], {}, {}
    note = lambda name, **kw: events.append(dict(event=name, step=len(rec.commands), sim_time_s=round(float(d.time), 2), **kw))
    keep = lambda name: frames.setdefault(name, len(rec.commands))
    student = lambda key: student_eval.Student(loaded[key][0], loaded[key][1], sensor, rc.PREFIX, None, "project")
    failure, stage = None, "OBSERVE"
    # ---- OBSERVE (top RGB-D at the reset) ------------------------------------------------------
    first = sensor.observe(m, d, ctx)[TOP]
    q = core.encode_state(m, d)
    dev = case.startswith("devplace:")                                        # development only: PLACE / STOW wiring from the stored end of a mixed collection run
    if dev:
        with np.load(ex.run_dir() / "runs" / "collect_ea57389b3edb" / f"collect_{seed}" / "episode.npz", allow_pickle=False) as a0:
            end_state = a0["integration_state"][-1].copy()
    best, cands, counts = tp.select_object(first["depth_m"], first["K"], first["T_world_camera_optical"], surface_z=TABLE_Z, roi=ROI, robot_mask=me.robot_mask(q))
    stage_log["OBSERVE"] = dict(ok=best is not None, candidates=len(cands), counts=counts, object=best)
    actor = None
    if best is None:
        failure = dict(stage="OBSERVE", reason="no object-sized cluster in the workspace")
    else:
        obj_xy, h_obj = np.array(best["centroid_xyz"][:2]), float(best["top_height_m"])
        source = tm.Region("original-object-area", "world", (float(obj_xy[0]), float(obj_xy[1])), SOURCE_RADIUS, TABLE_Z)
        note("OBSERVE", object_xy=obj_xy.round(4).tolist(), object_height_m=round(h_obj, 4))
        stage, actor, gates = "START_3V", student("START"), Gates(me)
        loaded["START"][0].reset()
        if dev:
            mujoco.mj_setState(m, d, end_state, int(mujoco.mjtState.mjSTATE_INTEGRATION))
            mujoco.mj_forward(m, d)
            ctx["initial"] = core.snapshot(ctx)
            sensor.observe(m, d, ctx)
            segs, place_info = plan_place(me, core.encode_state(m, d), d.ctrl.copy(), current["top"], h_obj, TARGET)
            stage, actor = "PLACE", Script(m, segs or [], "PLACE_SENSOR_V2")
            if segs is None:
                failure = dict(stage="PLACE", reason="place planning failed", detail=place_info)

    def source_occupied():
        t = current["top"]
        b, _, _ = tp.select_object(t["depth_m"], t["K"], t["T_world_camera_optical"], surface_z=TABLE_Z, roi=((source.center_xy[0] - 0.03, source.center_xy[0] + 0.03), (source.center_xy[1] - 0.03, source.center_xy[1] + 0.03)),
                                   robot_mask=me.robot_mask(core.encode_state(m, d)), min_points=8)
        return b is not None and abs(b["top_height_m"] - h_obj) < 0.012

    if not ("place_info" in dir()):
        place_info = None
    stow_info = verify = area = None
    try:
        while failure is None and stage not in ("CLEAN_RESUME_REQUEST",):
            step = len(rec.commands)
            if step >= TOTAL_LIMIT or (step >= HANDOVER_LIMIT and stage in ("START_3V", "RECEIVE_3V", "RELEASE_B")):
                failure = dict(stage=stage, reason="time limit" + (" (40 s for START .. RELEASE_B)" if step < TOTAL_LIMIT else " (whole chain)"))
                break
            q = core.encode_state(m, d)
            if stage in ("START_3V", "RECEIVE_3V", "RELEASE_B"):
                gates.update(q)
                if stage == "START_3V":
                    ok, why = gates.picked_and_lifted(source_occupied() if gates.holding("left") else True)
                    if ok:
                        note("SWITCH START_3V -> RECEIVE_3V", basis=why)
                        keep("START_3V done")
                        stage_log["START_3V"] = dict(ok=True, steps=step)
                        actor.discard_chunk()
                        stage, actor = "RECEIVE_3V", student("RECEIVE")
                        loaded["RECEIVE"][0].reset()
                        gates.armed["right"] = gates.hist[-1]["right"] >= ARMED_OPEN
                elif stage == "RECEIVE_3V":
                    if gates.both_hold():
                        note("SWITCH RECEIVE_3V -> RELEASE_B", basis=dict(left_opening=gates.hist[-1]["left"], right_opening=gates.hist[-1]["right"]))
                        keep("RECEIVE_3V done")
                        stage_log["RECEIVE_3V"] = dict(ok=True, steps=step)
                        actor.discard_chunk()
                        stage, actor = "RELEASE_B", student("B")
                        loaded["B"][0].reset()
                elif gates.handed_over():
                    note("SWITCH RELEASE_B -> PLACE", basis=dict(left_opening=gates.hist[-1]["left"], tool_points_apart_m=round(gates.hist[-1]["apart"], 3), right_opening=gates.hist[-1]["right"]))
                    keep("RELEASE_B done")
                    stage_log["RELEASE_B"] = dict(ok=True, steps=step)
                    actor.discard_chunk()
                    segs, place_info = plan_place(me, q, d.ctrl.copy(), current["top"], h_obj, TARGET)
                    if segs is None:
                        failure = dict(stage="PLACE", reason="place planning failed", detail=place_info)
                        break
                    stage, actor = "PLACE", Script(m, segs, "PLACE_SENSOR_V2")
            elif stage == "PLACE" and actor.done:
                t = current["top"]
                tcp = me.fk(q)["right"][0]
                verify = tp.verify_place(t["depth_m"], t["K"], t["T_world_camera_optical"], float(d.time), TARGET, gripper_opening=float(q[11]), gripper_tcp_xyz=tcp)
                keep("PLACE done")
                stage_log["PLACE"] = dict(ok=verify.placed, steps=step, phases=actor.log, verification=dict(placed=verify.placed, object_in_region=verify.object_in_region, object_supported=verify.object_supported,
                                                                                                           gripper_separated=verify.gripper_separated, evidence=verify.evidence))
                if not verify.placed:
                    failure = dict(stage="PLACE", reason="sensor verification: object not placed (in region / supported / gripper separated)")
                    break
                note("PLACE verified by sensors")
                placed = verify.evidence["clusters_in_region"][0]["centroid_xyz"]
                segs, stow_info = plan_stow(me, d.ctrl.copy(), (placed[0], placed[1], TABLE_Z + h_obj / 2))
                stage, actor = "ARM_STOW", Script(m, segs, "ARM_STOW_V1")
            elif stage == "ARM_STOW" and actor.done:
                keep("ARM_STOW done")
                stage_log["ARM_STOW"] = dict(ok=True, steps=step, phases=actor.log)
                note("ARM_STOW finished")
                t = current["top"]
                area = tp.observe_area(t["depth_m"], t["K"], t["T_world_camera_optical"], float(d.time), source)
                pts = tp.back_project(t["depth_m"], t["K"], t["T_world_camera_optical"])
                robot_in_view = int(me.robot_mask(q, margin=0.0)(pts[(np.hypot(pts[:, 0] - source.center_xy[0], pts[:, 1] - source.center_xy[1]) <= source.radius_m + 0.03)]).sum())
                stage_log["ORIGINAL_AREA_REOBSERVE"] = dict(ok=bool(area.clear and area.validity.valid), clear=area.clear, valid=area.validity.valid, points_above_surface=area.points_above_surface,
                                                            robot_points_over_the_area=robot_in_view)
                keep("REOBSERVE")
                if not (area.clear and area.validity.valid):
                    failure = dict(stage="ORIGINAL_AREA_REOBSERVE", reason="the original area is not clear or not visible", points_above_surface=area.points_above_surface)
                    break
                note("CLEAN_RESUME_REQUEST emitted", basis=dict(points_above_surface=0, region=source.center_xy))
                stage_log["CLEAN_RESUME_REQUEST"] = dict(ok=True, emitted=True, note="a request only: no cleaning device is connected and nothing was cleaned")
                stage = "CLEAN_RESUME_REQUEST"
                break
            if not core.rollout(ctx, actor, sensor, rec, 1):
                t = ctx.get("terminal_failure") or {}
                failure = dict(stage=stage, reason=t.get("reason", "rollout stopped"), time=t.get("time"), forbidden_contacts=t.get("forbidden_contacts"), detail=t.get("detail"))
                break
    finally:
        sensor.close()
    if failure is not None:
        stage_log.setdefault(failure["stage"], {}).update(ok=False, failure=failure)
    if rec.commands:
        rec.finish(dict(chain_final_stage=stage, chain_failure=failure))
    else:                                                                     # nothing was commanded (OBSERVE failed): one idle transition so that the run has a record
        hold = Script(m, [("idle", d.ctrl.copy(), d.ctrl.copy(), 0.02)], "IDLE")
        s_ = student_eval.Observations()
        core.rollout(ctx, hold, s_, rec, 1)
        s_.close()
        rec.finish(dict(chain_final_stage=stage, chain_failure=failure))
    # ---- evaluator: simulator truth, read only after the run ------------------------------------
    with np.load(out / "episode.npz", allow_pickle=False) as a:
        states, qs = a["integration_state"].copy(), a["q_state"].copy()
    check = evaluate_states(m, states, qs.astype(np.float64), control_dt=td.DT)
    rows = check.pop("rows")
    truth_xy = np.array(rows[0]["block_pos"][:2])
    truth = dict(events={k: round(v, 2) for k, v in check["events"].items()}, checker_valid=bool(check["valid"]), failures=check["failures"][:3],
                 observe_xy_error_mm=None if best is None else round(1000 * float(np.linalg.norm(obj_xy - truth_xy)), 2),
                 final_block_xy=[round(x, 4) for x in rows[-1]["block_pos"][:2]], final_block_on_table=rows[-1]["table_support_N"] > 0,
                 final_block_in_target=bool(np.hypot(rows[-1]["block_pos"][0] - TARGET.center_xy[0], rows[-1]["block_pos"][1] - TARGET.center_xy[1]) <= TARGET.radius_m),
                 final_pads_off=max(max(rows[-1]["pad_force_N"]["left"]), max(rows[-1]["pad_force_N"]["right"])) < 0.01,
                 final_block_outside_original_area=None if best is None else bool(np.hypot(rows[-1]["block_pos"][0] - obj_xy[0], rows[-1]["block_pos"][1] - obj_xy[1]) > SOURCE_RADIUS))
    sw = {e["event"]: e["sim_time_s"] for e in events}
    ev = check["events"]
    agree = dict(pick_lift_true_at_switch=None if "SWITCH START_3V -> RECEIVE_3V" not in sw else bool("pick_lift" in ev and ev["pick_lift"] <= sw["SWITCH START_3V -> RECEIVE_3V"] + 1e-9),
                 right_grasp_true_near_switch=None if "SWITCH RECEIVE_3V -> RELEASE_B" not in sw else bool("right_grasp" in ev and ev["right_grasp"] <= sw["SWITCH RECEIVE_3V -> RELEASE_B"] + 0.5),
                 handover_true_at_switch=None if "SWITCH RELEASE_B -> PLACE" not in sw else bool("handover" in ev and ev["handover"] <= sw["SWITCH RELEASE_B -> PLACE"] + 1e-9),
                 place_true=None if "PLACE" not in stage_log or "verification" not in stage_log["PLACE"] else bool(truth["final_block_in_target"] and truth["final_block_on_table"]))
    order = ["OBSERVE", "START_3V", "RECEIVE_3V", "RELEASE_B", "PLACE", "ARM_STOW", "ORIGINAL_AREA_REOBSERVE", "CLEAN_RESUME_REQUEST"]
    stages = {s: (None if s not in stage_log else bool(stage_log[s].get("ok"))) for s in order}
    success = stage == "CLEAN_RESUME_REQUEST" and failure is None
    # ---- frame sheet: the three cameras at the key moments --------------------------------------
    import cv2
    tiles = []
    n = len(rec.commands)
    cams = {c: np.load(out / f"cameras/{c}.npz")["rgb"] for c in CAMERAS}
    marks = [("reset / OBSERVE", 0)] + [(k, min(v, n - 1)) for k, v in frames.items()] + [("end", n - 1)]
    for name, f in marks:
        row = cv2.cvtColor(np.concatenate([cams[c][f] for c in CAMERAS], axis=1), cv2.COLOR_RGB2BGR)
        row = cv2.resize(row, None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST)
        cv2.putText(row, f"{name}  t={f * td.DT:.2f}s", (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        tiles.append(row)
    (run_dir() / "frames").mkdir(exist_ok=True)
    sheet = run_dir() / "frames" / f"chain_s{seed}.png"
    cv2.imwrite(str(sheet), np.concatenate(tiles, axis=0))
    cuda = cuda_record(loaded)
    result = dict(job=RUN, classification=LABEL_CHAIN, mvp_case=case, seed=seed, identity_sha256=ident["sha256"], path=str(out), final_stage=stage, mvp_chain_sim_success=bool(success),
                  first_failure_stage=None if failure is None else failure["stage"], failure=failure, stages=stages, stage_log=stage_log, events=events, place_plan=place_info, stow_plan=stow_info,
                  control_inputs=["top RGB-D + K + T", "wrist RGB (policy input)", "measured q_state", "kinematic model of the arms", "fixed target region"],
                  simulator_truth_used_in_control="none (the unchanged contact checker can end a run; it does not choose a transition or a command)",
                  evaluator_truth=truth, gate_vs_truth=agree, transitions=n, sim_elapsed_s=round(n * td.DT, 2), frames=dict(file=str(sheet), rows=[dict(name=k, step=int(v), sim_time_s=round(v * td.DT, 2)) for k, v in marks],
                                                                                                                         columns=["left wrist", "right wrist", "top (gray3)"]),
                  cuda=cuda, timing=dict(wall_s=round(time.time() - started, 1)), cameras=al.camera_row(out)["cameras"],
                  note="CLEAN_RESUME_REQUEST is a request emitted by the task chain; no cleaning device exists here and nothing was cleaned")
    (out / "result.json").write_text(json.dumps(result, indent=2, default=str))
    print(json.dumps(dict(case=case, final=stage, success=success, first_failure=result["first_failure_stage"], failure=failure, stages=stages, truth=dict(events=truth["events"], observe_err=truth["observe_xy_error_mm"],
                                                                                                                                                            in_target=truth["final_block_in_target"]), agree=agree), default=str)[:1800])


def one(args):
    ident = identity()
    if ident["sha256"] != args.identity:
        raise RuntimeError("identity changed between the parent process and this case")
    out = case_dir(ident, args.case)
    if out.exists():
        raise FileExistsError(f"{out} exists; nothing is overwritten")
    out.parent.mkdir(parents=True, exist_ok=True)
    (phase_one if args.case.startswith("phase:") else chain_one)(args.case, ident, out)


def batch(cases, name):
    ident = identity()
    marker = run_dir() / "evaluation_identity.json"
    if marker.exists():
        if load(marker)["sha256"] != ident["sha256"]:
            raise RuntimeError("evaluation_identity.json holds another identity (model, source or host changed): nothing is run, nothing is overwritten")
    else:
        marker.write_text(json.dumps(dict(ident, fixed_before_execution=time.strftime("%Y-%m-%dT%H:%M:%S%z")), indent=2, default=str))
    (run_dir() / "logs").mkdir(exist_ok=True)
    groups, started = [cases[0::2], cases[1::2]], time.time()

    def worker(group):
        out = []
        for case in group:
            if complete(ident, case) is not None:
                continue
            folder = case_dir(ident, case)
            if folder.exists():
                kept = run_dir() / "attempts" / f"{folder.name}_{time.strftime('%Y%m%dT%H%M%S')}"
                kept.parent.mkdir(exist_ok=True)
                folder.rename(kept)
                out.append(dict(case=case, kept=str(kept), note="incomplete folder of an earlier attempt"))
            t0 = time.time()
            done = subprocess.run([sys.executable, str(Path(__file__).resolve()), "one", "--case", case, "--identity", ident["sha256"]], capture_output=True, text=True)
            (run_dir() / "logs" / f"{folder.name}_{time.strftime('%H%M%S')}.log").write_text(done.stdout + "\n--- stderr ---\n" + done.stderr)
            ok = done.returncode == 0 and complete(ident, case) is not None
            if not ok:
                out.append(dict(case=case, returncode=done.returncode, status="infrastructure_error (not a task result)", stderr_tail=done.stderr[-900:]))
            line = [x for x in done.stdout.splitlines() if x.startswith("{")]
            print((line[-1][:700] if ok and line else f"[infrastructure_error] {case}: {done.stderr[-500:]}") + f"  [{time.time() - t0:.0f}s]", flush=True)
        return out

    extra = []
    with ThreadPoolExecutor(max_workers=WORKERS) as pool:
        for x in pool.map(worker, groups):
            extra += x
    f = run_dir() / "attempts.json"
    f.write_text(json.dumps((load(f) if f.exists() else []) + extra, indent=2))
    done = [c for c in cases if complete(ident, c) is not None]
    sf = run_dir() / "batch_status.json"
    status = load(sf) if sf.exists() else {}
    status[name] = dict(planned=len(cases), completed=len(done), missing=[c for c in cases if c not in done], workers=WORKERS, batch_wall_s=round(time.time() - started, 1))
    sf.write_text(json.dumps(status, indent=2))
    print(f"[{name}] {len(done)}/{len(cases)} cases; batch wall {status[name]['batch_wall_s']}s", flush=True)


def phase(args=None):
    batch(phase_cases(), "phase")


def chain(args=None):
    batch([f"chain:{s}" for s in CHAIN_CASES], "chain")


def everything(args=None):
    me = [sys.executable, str(Path(__file__).resolve())]
    for step in ("perception", "chain", "phase"):
        print(f"--- {step} --- {time.strftime('%H:%M:%S')}", flush=True)
        code = subprocess.run(me + [step]).returncode
        if code:
            print(f"[step failed] {step}: return code {code}; the independent later steps still run", flush=True)
    report = Path(__file__).with_name("tjj_mvp_r2_report.py")
    if report.exists():
        print(f"--- report --- {time.strftime('%H:%M:%S')}", flush=True)
        subprocess.run([sys.executable, str(report), "all"])
    print(f"=== ALL DONE {time.strftime('%H:%M:%S')} ===", flush=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    for name in ("perception", "phase", "chain", "all"):
        sub.add_parser(name)
    o = sub.add_parser("one")
    o.add_argument("--case", required=True)
    o.add_argument("--identity", required=True)
    args = parser.parse_args(argv)
    import arm_output_trial  # noqa: F401
    return dict(perception=perception, phase=phase, chain=chain, one=one, all=everything)[args.command](args)


if __name__ == "__main__":
    main()
