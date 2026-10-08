"""The FULL-CHAIN mode of the PICK02 session, bound to the functions that exist.

ONE state machine runs the part after the lift: RECEIVE FullEpisodeHandoff -> TaskManager -> ReceiveManipulation ->
ReceiveSession (COORDINATE -> RELEASE -> PLACE -> VERIFY_PLACE -> ARM_STOW -> REOBSERVE_CLEAR -> CLEAN_RESUME_REQUEST).
This module does not run a second one. It is the LOCAL OWNER that state machine is given: the same process that picked
and holds the block, and the only one that reads or writes an arm.

  writer        Owner.apply_dispatch  -> sweepick_move_a.run / grip (the executor of the PICK02 stages: checked trajectory,
                                         motion profile, progress watch, settle, torque state) once per planned waypoint or
                                         per commanded gripper / arm target. No goal stream of its own.
  observation   Owner.observation     -> read-only registers of both arms + the session recorder's camera frames
  plan          RECEIVE adapter + receive planner over the SIM teacher (carry, right approach, right insert)
  load transfer LoadTransfer          -> the left Jaw's hold command backed off by the left profile's own squeeze step
  release       Release               -> left jaws to their open tick, then the teacher's own retreat directions / hops
  PLACE / STOW  Downstream            -> teacher.plan_move with the block carried by the right hand; executor 'return'
  perception    Area                  -> the session's look (top depth through the board) on the target / source region

What needs evidence from the real robot is read from three files that do not exist until it has been taken
(FIELD below). Without them `preflight` says NOT READY and names them; nothing is defaulted, and the left gripper's
profile is never used for the right gripper."""
from sweepick.integration.sweepick_resource_paths import PRIVATE_CONFIG, RECEIVE_MODULES
import hashlib
import json
import os
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
CODEX = Path(os.environ.get("SWEEPICK_RECEIVE_DIR", Path(__file__).resolve().parents[1]))
RUNTIME = Path.home() / "DAPIER/tjj-runtime/v1"
MJCF = Path.home() / "DAPIER/.local-workspaces/so101/lerobot/src/lerobot/envs/so101_mujoco/assets/so101_new_calib.xml"
FIELD = dict(
    right_jaw=(PRIVATE_CONFIG / "sweepick_jaw_profile_right.json", "measured device profile of the RIGHT gripper (free-stroke residual, contact onset, rest residual), same schema as the left file plus device_identity and measurement_evidence",
               "the right close (COORDINATE cannot become HANDOFF_CANDIDATE), the right open at the place"),
    right_wrist=(PRIVATE_CONFIG / "sweepick_right_wrist_reference.json", "the right wrist camera read with the right jaws open at a known pose: jaw-tip reference and where a block between the right jaws appears (as the left one has)",
                 "object_at_tool for the right hand: the grasp label, the support record and therefore the left release"),
    support=(PRIVATE_CONFIG / "sweepick_support_reference.json", "how the load transfer shows on THIS robot: the small right-hand lift used (right_lift_m), the left-arm joints whose Present_Load falls when the right hand takes the block and by how much (load_joints, drop_raw), read in one supervised transfer",
             "the load transfer and the support record, and therefore the left release"),
    place=(PRIVATE_CONFIG / "sweepick_place_target.json", "the place target (centre and surface height in the model frame) confirmed on the real desk; a candidate file exists as sweepick_place_target.candidate.json",
           "PLACE, VERIFY_PLACE"))
ALL = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"); ARM = ALL[:5]
STOPPING = ("FAULT_HOLDING", "FAULT_JOINTS_RELEASED", "BUS_LOST", "STOPPED_HOLDING", "NO_PROGRESS_HOLDING", "RELEASE_UNCONFIRMED", "REFUSED")


def codex():
    """RECEIVE modules from where they were delivered; imported, not copied or changed."""
    os.environ.setdefault("FACTORY_ROOT", str(RUNTIME)); os.environ.setdefault("DAPIER_SO101_MJCF", str(MJCF))
    import importlib
    if "SWEEPICK_RECEIVE_DIR" in os.environ:
        if str(CODEX) not in sys.path:
            sys.path.insert(0, str(CODEX))
        return {n: importlib.import_module(f"sweepick_261008_{n}") for n in ("receive_real_adapter", "receive_plan", "receive_entry", "receive_session", "full_episode")}
    return {n: importlib.import_module(RECEIVE_MODULES[n]) for n in ("receive_real_adapter", "receive_plan", "receive_entry", "receive_session", "full_episode")}


def tool_pose(model, q12, site="left_cube_grasp"):
    import mujoco
    from sweepick.control import sweepick_joint_kinematics as sweepick_kin
    d = mujoco.MjData(model); sweepick_kin.set_q12(model, d, np.asarray(q12, float)); mujoco.mj_forward(model, d)
    i = model.site(site).id; T = np.eye(4); T[:3, :3], T[:3, 3] = d.site_xmat[i].reshape(3, 3), d.site_xpos[i]
    return T, d


def held_pose(q12_at_close, q12_now, block_centre_desk, block_yaw, *, model, pad_geoms, source):
    """What this session knows about the block in the left hand, with every part named by what it is:
    OBSERVED (read by a sensor in this state), ESTIMATED (computed from observations of an earlier state), ASSUMED
    (a physical assumption), UNIDENTIFIED. Same vocabulary as RECEIVE version of this function: `known`, `missing`,
    `object_pose_model` is None until the pose in the hand has been OBSERVED. The 4x4 put together from the estimated and
    assumed parts is given separately as `coarse_preview_pose`: good for previewing the carry and the right approach,
    not for putting the right jaws around the block."""
    Tc, dc = tool_pose(model, q12_at_close); Tn, _ = tool_pose(model, q12_now)
    pads = [dc.geom_xpos[model.geom(n).id].copy() for n in pad_geoms]; mid = (pads[0] + pads[1]) / 2; jaw = pads[1] - pads[0]; jaw[2] = 0; jaw /= np.linalg.norm(jaw)
    c = np.array(block_centre_desk, float); c[:2] += jaw[:2] * float(np.dot(mid[:2] - c[:2], jaw[:2]))
    yaw = float(np.arctan2(jaw[1], jaw[0]) - np.pi / 2); yaw += round((block_yaw - yaw) / (np.pi / 2)) * np.pi / 2
    Tb = np.eye(4); Tb[:3, :3] = [[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]]; Tb[:3, 3] = c; rel = np.linalg.inv(Tc) @ Tb
    comp = dict(along_jaw_axis=dict(cls="ASSUMED", basis="the closed pads put the block at their midpoint (pad compliance not known)"),
                across_jaw_axis=dict(cls="ESTIMATED", basis="top depth of the block on the desk before the pick through the candidate chain; 4 .. 14 mm off against the hand in the runs of 2026-10-08", uncertainty_m=0.014),
                height=dict(cls="ESTIMATED", basis="candidate desk height plus half the block at the close, carried rigidly with the tool"),
                yaw=dict(cls="ASSUMED", basis="squared to the jaws by the closed pads", uncertainty_deg=3.0), tilt=dict(cls="ASSUMED", basis="the block stood flat on the desk when it was closed on"), slip_after_close=dict(cls="UNIDENTIFIED"))
    return dict(object_pose_model=None, tool_from_object=None, status="MISSING", object_pose_source=source, kind="partial grasp evidence; candidate mapping and frame chain", components=comp,
                known=dict(desk_centre_before_pick=list(map(float, block_centre_desk)), desk_yaw_before_pick=float(block_yaw), candidate_pad_midpoint_at_close=mid.tolist(), candidate_tool_motion=(Tn @ np.linalg.inv(Tc)).tolist()),
                object_pose_uncertainty=dict(measured=False, height="MISSING", across_pad_width="MISSING", orientation="not identified in the model frame", slip_after_close="not observed"),
                missing=["held object height/tool offset", "held across-pad-width offset", "held orientation in model frame"], pose_role="COARSE_PREVIEW", final_insertion_ready=False,
                observed_components=["measured joints at close and now", "desk observation before close"], estimated_components=["tool motion by candidate FK", "pad midpoint along jaw axis"],
                assumed_components=["upright desk object", "nearest face squared by pads", "no slip after close"], unidentified_components=["current transverse offset", "pad compliance", "slip", "current orientation error"],
                coarse_preview_pose=(Tn @ rel).tolist(), coarse_tool_from_object=rel.tolist(), coarse_use="carry and right-approach PREVIEW only; the right insert needs the block OBSERVED in the hand")


def held_seen(depth_mm, K, T_world_camera, predicted, *, model, q12, half=0.02, back_project=None, inside=None):
    """The block in the hand as the top depth sees it NOW: the points of its top face near where the estimate puts it,
    robot points removed. Gives centre and height as OBSERVED, or None when too few points are there (sparse depth is
    unknown, not absent). Yaw stays as assumed. Not run on the real robot yet."""
    import mujoco
    from sweepick.control import sweepick_joint_kinematics as sweepick_kin
    from sweepick.integration import sweepick_manipulation_session as p1
    d = mujoco.MjData(model); sweepick_kin.set_q12(model, d, np.asarray(q12, float)); mujoco.mj_forward(model, d)
    import tjj_perception as tp
    Kd = np.array(K, float).reshape(3, 3).copy(); Kd[:2] /= 2; pts = (back_project or tp.back_project)(np.asarray(depth_mm, float) / 1000.0, Kd, np.array(T_world_camera, float))
    P = np.array(predicted, float); near = (np.hypot(pts[:, 0] - P[0, 3], pts[:, 1] - P[1, 3]) < 2 * half) & (np.abs(pts[:, 2] - (P[2, 3] + half)) < half) & ~(inside or p1.inside)(model, d, pts, "", 0.004)
    top = pts[near]
    if len(top) < 50:
        return None
    zt = float(np.percentile(top[:, 2], 90)); face = top[top[:, 2] > zt - 0.006]; out = P.copy(); out[:2, 3] = np.median(face[:, :2], 0); out[2, 3] = zt - half
    return dict(object_pose_model=out.tolist(), points=int(len(face)), moved_from_estimate_m=[float(v) for v in out[:3, 3] - P[:3, 3]],
                components=dict(centre_xy=dict(cls="OBSERVED", basis="top depth, top face of the held block"), height=dict(cls="OBSERVED", basis="top depth"), yaw=dict(cls="ASSUMED", basis="squared by the left pads"), slip_after_close=dict(cls="UNIDENTIFIED")),
                object_pose_uncertainty=dict(measured=True, centre_m=0.005, centre_basis="depth reads about 2 % long at this range and the top face is seen with a few hundred points", yaw="assumed"))


def scene_bindings(model, look_scene, table_z, *, model_sha, source_sha):
    """The frames RECEIVE adapter asks for: the table's own centre and orientation with the recorded candidate surface height (RECEIVE version of this function)."""
    import mujoco
    d = mujoco.MjData(model); mujoco.mj_forward(model, d)
    def body(n):
        T = np.eye(4); b = d.body(n); T[:3, 3], T[:3, :3] = b.xpos, b.xmat.reshape(3, 3); return T.tolist()
    g = model.geom("table"); table = d.geom("table"); Tt = np.eye(4); Tt[:3, :3] = table.xmat.reshape(3, 3); Tt[:3, 3] = table.xpos + Tt[:3, 2] * g.size[2]; Tt[2, 3] = float(table_z)
    return dict(frame="assembled dual model world (left base at its model pose)", model_sha256=model_sha, source_sha256=source_sha, table_top_z_m=float(table_z), T_model_left_base=body("left_base"), T_model_right_base=body("right_base"),
                T_model_camera=look_scene.get("T_world_camera"), T_model_table=Tt.tolist(), T_task_from_model=np.eye(4).tolist(), provenance="candidate compiled bases/table centre + recorded camera/chain height; compiler checks both bases", geometry_verified=False)


def field_refs():
    """The three records only the real robot can give. Each: the loaded document or None, with what it is and what waits for it."""
    out = {}
    for name, (path, what, blocks) in FIELD.items():
        doc = None
        if path.exists():
            try:
                doc = json.loads(path.read_text())
            except Exception as e:
                doc = None; what = f"{what} (file unreadable: {e})"
        ok = bool(doc) and doc.get("source_kind") == "REAL" and bool(doc.get("measurement_evidence")) and doc.get("field_confirmed", True) is True
        out[name] = dict(file=str(path), present=bool(doc), usable=ok, doc=doc if ok else None, what=what, blocks=blocks)
    return out


class ScopeDone(RuntimeError):
    """The OBSERVE scope reached its planned end (not a failure)."""


class Scoped:
    """RECEIVE FullEpisodeHandoff with the scope of this session said out loud. FULL: unchanged. OBSERVE: the same state machine
    is run up to the right contact and the load-transfer lift; it never gets a support record, so it never releases the
    left hand; the owner then runs the planned return and this reports where the block is."""
    def __init__(self, inner, owner, scope):
        self.inner, self.owner, self.scope, self.abandon, self.area = inner, owner, scope, owner.abandon, getattr(inner, "area", None)

    def ready(self, out, state):
        r = self.inner.ready(out, state); r["scope"] = self.scope; return r

    def __call__(self, ctx):
        h = self.inner(ctx); o = self.owner
        if self.scope == "observe" and o.scope_result is not None:
            return dict(result="OBSERVED", scope="OBSERVE", commands_sent=True, object_owner="LEFT" if o.scope_result["left_still_loaded"] else "UNKNOWN", left_at_lift=o.scope_result["left_at_lift"], end=o.scope_result, observation_file="SUPPORT_OBSERVATION.json",
                        why="right contact and load-transfer observation done; the left hand was not released; the right arm is home and released", stages=h.get("stages"), state_machine_result=h.get("result"), recording_errors=h.get("recording_errors"))
        if self.scope == "observe":
            h["scope"] = "OBSERVE (ended before its planned end)"
        return h


class LoadTransfer:
    """The 'controlled load transfer' reference inside RECEIVE contract (the left Jaw's hold command is kept until support is
    confirmed; arm postures may change): the right hand, closed on the block, is raised by the short lift named in the
    support reference, as one checked hop of the teacher's own straight-line planner, run once by the executor. Whether the
    load went over is then read from the left arm's Present_Load against its value at the start (Owner.support). Without the
    support reference this is NOT READY; no lift height or load drop is assumed here."""
    def __init__(self, owner):
        self.o, self.name, self.goal = owner, "right hand raised by the support reference's lift; left Jaw hold kept", None

    def begin(self, phase):
        self.goal = None; raw = getattr(self.o, "last_raw", None); self.o.load_baseline = None if raw is None else {n: float(v) for n, v in raw["load"]["left"].items()}; self.o.note("load_transfer_begin", phase=phase, left_load_baseline=self.o.load_baseline)

    def next_command(self, ctx):
        obs = ctx["observation"]; q = list(ctx["measured_q12"]); left, right = obs.get("left") or {}, obs.get("right") or {}; lift = self.o.lift_m()
        if lift is None:
            return q, dict(status="NOT_READY", missing=["the right-hand lift of the load transfer (approved for an observation session, or from the support reference in the full task)"])
        ref = dict(right_lift_m=lift)
        if left.get("command") is None or right.get("command") is None:
            return q, dict(status="NOT_READY", missing=["both gripper commands"])
        if self.goal is None:
            t = self.o.teacher; base = ctx["data"].qpos.copy(); t.cmd = self.o.gear(q); adr = t.plan.cm.block_qpos
            hops, why = t.cartesian_hops(base, t.cmd, "right", np.array([0.0, 0.0, 1.0]), float(ref["right_lift_m"]), allow=("left", "right"), block_qpos=base[adr:adr + 7].copy())
            if not hops:
                return q, dict(status="NOT_READY", missing=[f"a checked right-hand lift of {ref['right_lift_m']} m from this pose ({why})"])
            self.goal = t.to_canonical(hops[-1]).tolist()
        g = list(q); g[6:11] = self.goal[6:11]; g[5], g[11] = left["command"], right["command"]
        return g, dict(reference=self.name, right_lift_m=float(ref["right_lift_m"]))


class Release:
    """The release / retreat reference: left jaws to their open tick (the session's own open target), then the left hand away
    along the teacher's own retreat directions as short checked hops. Called only after the session has a support record."""
    def __init__(self, owner, distance_m=None):
        self.o, self.name, self.goals = owner, "left open, then teacher retreat hops", None

    def begin(self, phase):
        self.goals = None; self.o.note("release_begin", phase=phase)

    def next_command(self, ctx):
        obs = ctx["observation"]; q = list(ctx["measured_q12"]); right = obs.get("right") or {}; left = obs.get("left") or {}
        if right.get("command") is None or left.get("command") is None:
            return q, dict(status="NOT_READY", missing=["both gripper commands"])
        q[11] = right["command"]; open_q = self.o.left_open_q
        if (left.get("actual") or 0.0) < open_q - self.o.left_profile["floor"]:
            q[5] = open_q; return q, dict(reference=self.name, step="open")
        q[5] = open_q
        if self.goals is None:
            t = self.o.teacher; base = ctx["data"].qpos.copy(); t.cmd = self.o.gear(q); a_r = None
            for direction in t.retreat_directions(base, t.cmd, a_r):
                hops, why = t.cartesian_hops(base, t.cmd, "left", direction, t.cfg.retreat_m if hasattr(t.cfg, "retreat_m") else 0.06, allow=("left", "right"), block_qpos=base[t.plan.cm.block_qpos:t.plan.cm.block_qpos + 7].copy())
                if hops:
                    self.goals = [t.to_canonical(hops[-1]).tolist()]; self.o.note("release_retreat_planned", direction=[float(v) for v in direction], hops=len(hops)); break
            if self.goals is None:
                return q, dict(status="NOT_READY", missing=["a collision-free left retreat from this pose (teacher.cartesian_hops found none)"])
        g = list(self.goals[0]); g[5] = open_q; g[6:] = q[6:]
        return g, dict(reference=self.name, step="away")


class Owner:
    """The one local owner. Everything it sends goes through `run` / `grip` (sweepick_move_a) inside the recorder's lease that
    the hand-over already holds; everything it reports is read back from registers or cameras."""
    def __init__(self, out, episode, refs, *, p1, run=None, grip=None, run_kw=None, read=None, frames=None, look=None, check=None, clock=time.monotonic, sleep=time.sleep, state=None):
        from sweepick.control import sweepick_trajectory_executor as mv
        self.out, self.ep, self.refs, self.p1 = Path(out), episode, refs, p1
        self.run, self.grip, self.run_kw, self.clock, self.sleep = run or mv.run, grip or mv.grip, dict(run_kw or {}), clock, sleep
        self._read, self._frames, self._look, self._check = read or self.read_registers, frames or self.recorder_frames, look or p1.look, check or self.segment_check
        self.mp = self.run_kw.get("mapping") or p1.mapping(); self.msha = self.run_kw.get("mapping_sha") or hashlib.sha256(p1.CAND.read_bytes()).hexdigest()
        self.seq, self.k, self.pending, self.done, self.stopped, self.session, self.notes = 0, 0, {}, set(), None, None, []
        self.held, self.carry, self.block_owner, self.state_fn = None, None, "left", state
        d = json.loads(mv.JAW_PROFILE.read_text()); lo, hi = float(self.mp["left"]["gripper"]["lo"]), float(self.mp["left"]["gripper"]["hi"]); R = hi - lo
        self.left_profile = dict(step=1.0 / R, floor=float(d["residual_floor_ticks"]["value"]) / R, source="sweepick_jaw_profile_left.json (one tick of the left servo; residual floor)")
        self.left_onset_command, self.left_open_q, self.home, self.right_closed = None, None, {}, False
        self.last_frame_n = {}; self.last_images = {}; self.insert = None; self.scope, self.right_lift_m, self.scope_result, self.support_rows = "full", None, None, []
        self.teacher = None; self.load_baseline = None; self.transfer_seen = False; self.cycle = None; self.notes_sent = []; self.idle_sig, self.idle_since = None, clock()
        self.idle_limit_s, self.idle_basis = 100 * float(d["confirm_s"]["value"]), "100 times the Jaw profile's confirmation time"
        self.observe_s = 10 * float(d["confirm_s"]["value"])                      # how long the state after the lift is recorded in the OBSERVE scope: ten of the Jaw profile's confirmation times

    # -- records ------------------------------------------------------------------------------------------------------
    def note(self, kind, **k):
        k = _plain(k)                                                          # images stay in the recorder's own frame files; the event keeps their frame number and stamp
        self.notes.append(dict(kind=kind, **k))
        try:
            if self.ep is not None:
                self.ep.event(kind, **k)
        except Exception as e:                                                 # the record's quality, not the motion's outcome
            self.notes.append(dict(kind="recording_error", what=f"{type(e).__name__}: {e}"))

    def gear(self, q12):
        return codex()["receive_plan"].gear(self.teacher, q12)

    # -- reading ------------------------------------------------------------------------------------------------------
    def read_registers(self):
        """Both arms, read-only: Present_Position, Goal_Position, Torque_Enable, Status, Present_Load. Measured and goal are separate registers and stay separate fields."""
        from sweepick.integration.sweepick_observation_session import open_readonly_bus
        raw = dict(ticks={}, goal={}, torque={}, status={}, load={})
        for side in ("left", "right"):
            b = open_readonly_bus(side)
            try:
                for key, reg in (("ticks", "Present_Position"), ("goal", "Goal_Position"), ("torque", "Torque_Enable"), ("status", "Status"), ("load", "Present_Load")):
                    raw[key][side] = {n: float(v) for n, v in b.sync_read(reg, normalize=False).items()}
            finally:
                b.disconnect()
        return raw

    def recorder_frames(self):
        """The newest frame of each of the session recorder's readers (no camera is opened here)."""
        out = {}
        if self.ep is None:
            return out
        from sweepick.integration.sweepick_observation_session import cadence_limit
        for name, r in getattr(self.ep, "wrists", {}).items():
            f, _ = r.get(); out[name] = None if f is None else dict(n=f["n"], recv_monotonic_s=f["recv_monotonic_s"], frame=f["frame"], cadence_limit_s=cadence_limit(getattr(getattr(r, "reader", r), "intervals", [])))
        t = getattr(self.ep, "top", None); f = t.get() if t is not None else None
        out["top"] = None if f is None else dict(n=f["n"], recv_monotonic_s=f["recv_monotonic_s"], frame=f.get("bgr"), depth=f.get("zd"), cadence_limit_s=cadence_limit(getattr(t, "intervals", [])),
                                                 sensor_stamps={k: f.get(k) for k in ("sdk_color_stamp_s", "sdk_depth_stamp_s", "helper_receipt_monotonic_s")})
        return out

    def raw(self):
        t0 = self.clock(); r = self._read(); self.seq += 1
        r.update(t_bus_monotonic=self.clock(), bus_seq=self.seq, cycle_start=t0, usable=True, snapshot_only=False); return r

    def observation(self, stage, task, now):
        """One observation packet in the shape RECEIVE session takes. Every record that claims something (motion result, insert,
        support, left away, acknowledgement) was made by a function of this owner from this read; it is stamped with this read."""
        tg = codex()["receive_session"].tg
        if self.cycle is None:                                                  # a second observation in the same step of the task manager: a new read (it will be stamped later than that step's 'now' and the session will wait for the next cycle)
            fr_ = self._frames(); self.cycle = (self.raw(), fr_)
        (raw, fr), self.cycle = self.cycle, None; stamp, seq = raw["t_bus_monotonic"], raw["bus_seq"]
        tel = {s: dict(goal_ticks=raw["goal"][s], feedback_stamp=stamp, seq=seq, fresh=True, source="independent motor read: Present_Position and Goal_Position registers", provenance=tg.INDEPENDENT,
                       torque=bool(all(raw["torque"][s][n] for n in ALL)), velocity=0.0, load=raw["load"][s].get("gripper"), current=None, current_units=None, current_baseline=None) for s in ("left", "right")}
        def img(f, src, cam):
            """A camera packet keeps the frame's OWN receipt time and sequence number. It is tied to this cycle (the cycle's stamp
            and seq, which the session compares) only when the reader's own cadence rule (sweepick_local_loop.cadence_limit:
            older than two of its recent frame intervals = a frame was missed) calls it fresh at the time of this bus read.
            A frame that is not fresh keeps its own stamp, is marked so, and does not pass as this cycle's image."""
            if f is None:
                return None
            lim = f.get("cadence_limit_s"); age = stamp - f["recv_monotonic_s"]; fresh = bool(lim is not None and 0.0 <= age <= lim); same = self.last_frame_n.get(cam) == f["n"]; self.last_frame_n[cam] = f["n"]
            return dict(rgb=f.get("frame"), n=f["n"], frame_seq=f["n"], recv_monotonic_s=f["recv_monotonic_s"], sensor_stamps=f.get("sensor_stamps"), age_at_bus_read_s=age, cadence_limit_s=lim, same_frame_as_last_cycle=same,
                        fresh=fresh, feedback_stamp=stamp if fresh else f["recv_monotonic_s"], seq=seq if fresh else None, source=src, validity="reader cadence rule" if lim is not None else "not judged: the reader has not shown enough frame intervals yet")
        ev = dict(wrist=dict(left=img(fr.get("left_wrist"), "left wrist reader of the session recorder", "left_wrist"), right=img(fr.get("right_wrist"), "right wrist reader of the session recorder", "right_wrist")), os30a=img(fr.get("top"), "SDK top reader of the session recorder", "top"),
                  servo_status={s: raw["status"][s] for s in ("left", "right")})
        if ev["wrist"]["left"] is None or ev["wrist"]["right"] is None:
            ev["wrist"] = None if ev["wrist"]["left"] is None and ev["wrist"]["right"] is None else ev["wrist"]
        obs = codex()["receive_entry"].owner_observation(dict(raw, ticks=raw["ticks"]), self.mp, executor_pid=os.getpid(), telemetry=tel, evidence=ev)
        self.last_raw, self.last_obs = raw, obs; self.last_images = dict(left_wrist=(ev.get("wrist") or {}).get("left") if ev.get("wrist") else None, right_wrist=(ev.get("wrist") or {}).get("right") if ev.get("wrist") else None, top=ev.get("os30a"))
        fr = {k: (f if (self.last_images.get(k) or {}).get("fresh") else None) for k, f in fr.items()}      # everything below (held pose, object at the tool, support, left away) uses a frame only when it is this cycle's fresh frame
        held = self.held_packet(obs, raw, fr, stamp, seq)
        if held is not None:
            obs["held_object"] = held
        obs["object_evidence"] = self.object_evidence(obs, fr, stamp, seq)
        if self.scope == "observe" and self.right_closed:
            self.observe_row(obs, raw, "after the right-hand lift" if "load_transfer" in self.done else "right closed, before the lift")
        for key, rec in list(self.pending.items()):                             # what the executor did since the last read, stamped with THIS read
            obs[key] = dict(rec, feedback_stamp=stamp, seq=seq); del self.pending[key]
        if self.session is not None and getattr(self.session, "phase", None) in ("LOAD_TRANSFER", "RELEASE"):
            sup = self.support(obs, raw, stamp, seq)
            if sup is not None:
                obs["support_confirmation"] = sup
            away = self.left_away(obs, raw, fr, stamp, seq)
            if away is not None:
                obs["left_open_and_away"] = away
        return obs

    def held_packet(self, obs, raw, fr, stamp, seq):
        """The held-object record of this cycle. Before the right insert the coarse estimate is not enough: the block has to be
        OBSERVED in the hand (top depth) or the record carries no pose, and RECEIVE session then waits (NOT_READY)."""
        if self.held is None:
            return None
        need_seen = "RIGHT_APPROACH" in self.done and "RIGHT_INSERT" not in self.done      # the arms stand at the hand-over pose: the next waypoint puts the right jaws around the block
        T_tool, _ = tool_pose(self.p1.model(), obs["q12"], site=f"{self.block_owner}_cube_grasp" if self.block_owner in ("left", "right") else "left_cube_grasp")
        rel = self.held.get("seen_tool_from_object") if self.held.get("seen_tool_from_object") is not None else self.held["coarse_tool_from_object"]
        if need_seen and self.held.get("seen_tool_from_object") is None:
            top = fr.get("top")
            seen = None if top is None or top.get("depth") is None or self.scene_look is None else self.held_seen_fn(top["depth"], self.scene_look.get("K"), self.scene_look.get("T_world_camera"), T_tool @ np.array(rel), model=self.p1.model(), q12=obs["q12"])
            if seen is None:
                self.note("held_pose_not_observed", why="the top depth did not show the block's top face in the hand; the right insert is not opened on the estimate")
                return dict(object_pose_model=None, pose_frame=self.scene["frame"], pose_source=None, object_pose_uncertainty=None, feedback_stamp=stamp, seq=seq, components=self.held["components"], missing=self.held["missing"])
            self.held["seen_tool_from_object"] = (np.linalg.inv(T_tool) @ np.array(seen["object_pose_model"])).tolist(); self.held["seen"] = seen; self.held["seen_seq"] = seq; rel = self.held["seen_tool_from_object"]; self.note("held_pose_observed", moved_from_estimate_m=seen["moved_from_estimate_m"], points=seen["points"])
        seen = self.held.get("seen"); self.last_held_pose = (T_tool @ np.array(rel)).tolist()      # where the block is NOW (with whoever carries it): the scene of the next segment's collision check
        return dict(object_pose_model=(T_tool @ np.array(rel)).tolist(), pose_frame=self.scene["frame"], feedback_stamp=stamp, seq=seq, transforms=dict(tool_from_object=rel, tool=f"{self.block_owner}_cube_grasp"),
                    pose_role="CURRENT_OBSERVATION" if seen and self.held.get("seen_seq") == seq else "CURRENT_TRACKED_ESTIMATE", source_kind="REAL", final_insertion_ready=bool(seen),
                    observed_components=(["block centre and height in the hand (top depth at the hand-over pose)"] if seen else []) + ["measured joints now"], estimated_components=[] if seen else ["block position in the hand from the desk observation before the close", "pad midpoint along the jaw axis"],
                    assumed_components=["nearest face squared by the pads", "rigid with the tool since " + ("it was observed" if seen else "the close")], unidentified_components=["slip", "pad compliance"] + ([] if seen else ["current transverse offset", "current orientation error"]),
                    pose_source=("top depth of the block in the hand, carried with the measured tool" if seen else "COARSE ESTIMATE: grasp geometry + measured joints (estimated and assumed parts named in components)"),
                    object_pose_uncertainty=(seen or {}).get("object_pose_uncertainty") or dict(measured=False, across_jaw_axis_m=0.014, note="coarse preview estimate"), components=(seen or self.held)["components"], grasp_evidence=self.held.get("grasp_evidence"))

    def object_evidence(self, obs, fr, stamp, seq):
        """Is the block at the RIGHT tool point? From the right wrist image, which needs the right wrist reference. Without it: not valid (unknown), never 'yes'."""
        ref = self.refs["right_wrist"]["doc"]; f = fr.get("right_wrist"); at = None; valid = False
        if self.scope == "observe":
            # No right wrist reference exists before this session (it is made from this session's frames). Here the question
            # 'is the block at the right tool' is answered from what IS observed: the block's pose seen in the hand by the top
            # depth, carried with the measured joints, against where the right pads stand by the measured joints.
            held = obs.get("held_object") or {}; rel = None
            if self.held is not None and self.held.get("seen_tool_from_object") is not None and held.get("object_pose_model") is not None:
                import mujoco
                from sweepick.control import sweepick_joint_kinematics as sweepick_kin
                m = self.p1.model(); d = mujoco.MjData(m); sweepick_kin.set_q12(m, d, np.array(obs["q12"], float)); mujoco.mj_forward(m, d)
                pads = [d.geom_xpos[m.geom(n).id].copy() for n in ("right_pgripper_pad_1", "right_pgripper_pad_2")]; c = np.array(held["object_pose_model"])[:3, 3]; jaw = pads[1] - pads[0]; gap = float(np.linalg.norm(jaw)); jaw /= gap
                v = c - (pads[0] + pads[1]) / 2; along = float(np.dot(v, jaw)); across = float(np.linalg.norm(v - along * jaw)); rel = dict(pad_gap_m=gap, block_along_jaw_m=along, block_across_m=across)
                at = bool(abs(along) <= gap / 2 and across <= self.held_half * np.sqrt(2)); valid = True                # the centre between the pads, and no further from their line than the block's own half diagonal
            return dict(object_at_tool=at, evidence_valid=valid, fresh=True, source="OBSERVE scope: block pose seen in the hand (top depth) and measured joints against the right pads; the right wrist image is recorded, not judged" if valid else "block not observed in the hand: not evaluated",
                        relation=rel, feedback_stamp=stamp, seq=seq, object_on_support=False, co_motion_observed=False, hand_raised_since_contact=False, hand_raised=True)
        if ref is not None and f is not None and f.get("frame") is not None:
            o = self.p1.wrist_object(f["frame"]); valid = True
            at = bool(o.get("visible") and abs(o["centre_px"] - float(ref["block_between_jaws_centre_px"])) <= float(ref["centre_tolerance_px"]))
        return dict(object_at_tool=at, evidence_valid=valid, fresh=True, source="right wrist image against the right wrist reference" if valid else "no right wrist reference: not evaluated", feedback_stamp=stamp, seq=seq,
                    object_on_support=False, co_motion_observed=False, hand_raised_since_contact=False, hand_raised=True)

    def support(self, obs, raw, stamp, seq):
        """'The right hand carries the block', from three things read in this cycle: the right jaws are loaded (residual above the
        RIGHT profile's floor), the block is at the right tool in the right wrist image, and the left jaws no longer press
        (left residual at or below the LEFT profile's no-load floor after the preload was backed off). Without the right
        profile or the right wrist reference there is no record; a contact alone is never support."""
        tg = codex()["receive_session"].tg; rj = self.refs["right_jaw"]["doc"]; left, right, oe = obs["left"], obs["right"], obs["object_evidence"]
        if self.scope == "observe":                                             # no support judgement exists yet: this scope measures what one will be made from, and never produces a support record
            return None
        if rj is None or not oe.get("evidence_valid") or left.get("command") is None or right.get("command") is None:
            return None
        Rr = float(self.mp["right"]["gripper"]["hi"] - self.mp["right"]["gripper"]["lo"]); r_res = abs(right["actual"] - right["command"]); ref = self.refs["support"]["doc"]
        motor = r_res > float(rj["residual_floor_ticks"]["value"]) / Rr; vision = bool(oe.get("object_at_tool")); drop = None
        if ref is not None and self.load_baseline is not None and "load_transfer" in self.done:
            drop = {n: abs(self.load_baseline[n]) - abs(raw["load"]["left"][n]) for n in ref["load_joints"]}
            if all(v >= float(ref["drop_raw"]) for v in drop.values()):
                self.transfer_seen = True                                         # seen once with the left hand still holding; the left opening that follows changes the left load for another reason
        transfer = bool(self.transfer_seen); l_res = drop
        e = lambda ok, src, **k: dict(observed=bool(ok), fresh=True, source=src, feedback_stamp=stamp, seq=seq, **k)
        rec = dict(label=tg.SUPPORT_CONFIRMED if motor and vision and transfer else tg.UNKNOWN, confirmed=bool(motor and vision and transfer), source_kind="REAL", source="sweepick_full_chain.Owner.support (registers and right wrist image of this cycle)", session_id=self.session.session_id,
                   executor_pid=os.getpid(), feedback_stamp=stamp, seq=seq, provenance=tg.INDEPENDENT, motor_evidence=e(motor, "right gripper residual above the right profile floor", residual=r_res),
                   vision_evidence=e(vision, "block at the right tool in the right wrist image"), load_transfer_evidence=e(transfer, "left arm Present_Load fell by the support reference's amount after the right-hand lift", left_load_drop=l_res))
        self.note("support_evaluated", confirmed=rec["confirmed"], motor=bool(motor), vision=vision, load_transfer=bool(transfer))
        return rec if rec["confirmed"] else None

    def left_away(self, obs, raw, fr, stamp, seq):
        """The left hand is open and away: its gripper stands at its open tick (register), its retreat ended HOLDING (executor), and the left wrist image no longer shows the block between the left jaws (the detector of the pick)."""
        tg = codex()["receive_session"].tg; left = obs["left"]
        if "left_retreat" not in self.done or left.get("actual") is None:
            return None
        opened = left["actual"] >= self.left_open_q - self.left_profile["floor"]; f = fr.get("left_wrist"); gone = None if f is None or f.get("frame") is None else not self.p1.wrist_object(f["frame"]).get("visible")
        e = lambda ok, src: dict(observed=bool(ok), fresh=True, source=src, feedback_stamp=stamp, seq=seq)
        return dict(open=bool(opened), away=bool(gone), source_kind="REAL", source="left gripper register, executor retreat result, left wrist image", session_id=self.session.session_id, executor_pid=os.getpid(), provenance=tg.INDEPENDENT,
                    feedback_stamp=stamp, seq=seq, motor_evidence=e(opened, "left gripper at its open tick; retreat ended HOLDING"), vision_evidence=e(bool(gone), "no block between the left jaws in the left wrist image"))

    # -- writing: the existing executor ------------------------------------------------------------------------------------
    def waypoint(self):
        s = self.session; c = getattr(s, "corrector", None)
        if s is None or c is None or getattr(c, "phase", None) not in (None, "COORDINATE"):
            return None
        return c.plan["waypoints"][c.index]

    def segment_check(self, doc, side, raw):
        """The commanded path with the block in the scene, by the teacher's own collision check (as the PICK02 stages do): the block
        carried by whoever holds it; allowed contacts are that hand's pads on the block (both hands' at the hand-over)."""
        import mujoco
        from sweepick.control import sweepick_joint_kinematics as sweepick_kin
        from sweepick.control import sweepick_trajectory_profile as tt
        from sim_data_factory.teacher import HandoverTeacher
        p1 = self.p1; T = None if self.last_held_pose is None else np.array(self.last_held_pose); m = p1.model(None if T is None else (T[:3, 3], float(np.arctan2(T[1, 0], T[0, 0])))); t = HandoverTeacher(m); d = mujoco.MjData(m); traj = doc["trajectory"]
        adr = t.cm.block_qpos; block_q = m.qpos0[adr:adr + 7].copy(); other = "right" if side == "left" else "left"; allow = ("left", "right") if self.block_owner in ("both", "right") or self.near_handover else ("left",)
        def ctrl(arm):
            q = p1.rc.ticks_to_q12(self.mp, {side: arm, other: raw["ticks"][other]}); sweepick_kin.set_q12(m, d, q); return d.qpos.copy(), d.qpos[t.plan.adr].copy()
        carrier = self.block_owner if self.block_owner in ("left", "right") else "left"; rel = None
        if T is not None and carrier == side:
            base, c = ctrl({n: raw["ticks"][side][n] for n in ALL}); t.plan.pose(base, c, block_q); ps, rot = t.plan.site_frame(side); rb = T[:3, :3]; rel = np.eye(4); rel[:3, :3] = rot.T @ rb; rel[:3, 3] = rot.T @ (T[:3, 3] - ps)
        first = None; steps = traj["steps"]
        for k in range(steps):
            base, c = ctrl({n: float(traj["goals"][n][k]) if n in traj["goals"] else raw["ticks"][side][n] for n in ALL})
            bq = t.plan.carried_block(base, c, (side, rel)) if rel is not None else block_q
            bad = t.plan.collisions(base, c, bq if T is not None else None, allow_pad_block=allow, clearance=0.0)
            if bad:
                first = dict(step=k, pairs=[list(map(str, b)) for b in bad[:4]]); break
        return dict(schema="tjj.object-collision-result.v1", identity=tt.identity(doc["side"], traj, self.msha), nominal_verdict="CLEAR" if first is None else "CONTACT", first_nominal_contact=first, nominal_steps_checked=steps,
                    scene=f"assembled dual model, desk at the real height, block held by {self.block_owner}", allowed=[f"{a} pad - block" for a in allow], check="sim_data_factory.teacher.Planner.collisions (unchanged)", mapping_status="CANDIDATE (verified=false)")

    def segment(self, name, side, *, target=None, rest=None, power=None):
        """One checked executor run of one arm (or its gripper): plan -> collision check with the block -> run. Returns the executor's log. Raises on anything but HOLDING / RETURNED_RELEASED, so that the caller's state machine stops."""
        raw = self._read(); other = "right" if side == "left" else "left"; d = self.out / "handoff" / f"{self.k:02d}_{name}_{side}"; self.k += 1
        on = [n for n in ALL if raw["torque"][side][n]]; want = target if target is not None else rest
        if target is not None and all(round(raw["ticks"][side][n]) == round(v) for n, v in target.items()) and set(target) <= set(on):
            self.note("segment", name=name, side=side, result="HOLDING", wrote=False, why="already at the target ticks"); return dict(result="HOLDING", end=dict(ticks=raw["ticks"][side]), wrote=False, folder=str(d))
        tj = power or [n for n in ALL if n in on or n in want or (n in ARM and any(j in ARM for j in want))]           # an arm move holds the whole arm; a gripper move adds the gripper to what is on
        mode = "return" if rest is not None else ("next" if on else "out")
        kw = dict(level="model", torque_joints=tj, other_arm_ticks=raw["ticks"][other], **self.run_kw); kw.update(dict(rest=rest) if rest is not None else dict(target=target))
        code, plan = self.run("plan" if mode != "return" else "return", side, d / "plan", **kw)
        if plan["result"] != "PLANNED":
            raise RuntimeError(f"{name} ({side}): the executor did not plan it: {plan.get('refused') or plan['result']}")
        chk = lambda doc, tag: self._write_check(d, tag, self._check(doc, side, raw))
        c = chk(json.loads((d / "plan/TRAJECTORY.json").read_text()), "path")
        if c.get("nominal_verdict") != "CLEAR":
            raise RuntimeError(f"{name} ({side}): the commanded path touches something that is not an intended contact: {c.get('first_nominal_contact')}")
        if self.ep is not None:
            self._safe(lambda: self.ep.mark(f"handoff_{name}_{side}", owner=f"sweepick_move_a.run ({side} arm)"))
        code, log = self.run(mode, side, d / "run", execute=True, cert=c, trim_check=lambda doc: chk(doc, "trim"), start_recheck=lambda doc: chk(doc, "start"), stop_requested=Path(str(self.out) + ".STOP").exists, **kw)
        if self.ep is not None:
            self._safe(lambda: self.ep.stage(f"handoff_{name}", d, side, log["result"]))
        self.note("segment", name=name, side=side, result=log["result"], writes=log.get("writes"), folder=str(d)); self.notes_sent.append(name)
        if log["result"] not in ("HOLDING", "RETURNED_RELEASED"):
            raise RuntimeError(f"{name} ({side}): executor ended {log['result']}")
        log["folder"] = str(d); return log

    def _write_check(self, d, tag, c):
        d.mkdir(parents=True, exist_ok=True); (d / f"{tag}.CHECK.json").write_text(json.dumps(c, indent=1, default=str)); return c

    def _safe(self, fn):
        try:
            fn()
        except Exception as e:
            self.notes.append(dict(kind="recording_error", what=f"{type(e).__name__}: {e}"))

    def go(self, name, q12, *, arms=("left", "right"), grippers=()):
        """Bring the named arms (5 joints) and grippers to a canonical q12 through the executor, one arm at a time."""
        tk = self.p1.rc.q12_to_ticks(self.mp, np.array(q12, float)); logs = {}
        for side in arms:
            logs[side] = self.segment(name, side, target={n: float(round(tk[side][n])) for n in ARM})
        for side in grippers:
            logs[side + "_gripper"] = self.segment(name + "_gripper", side, target=dict(gripper=float(round(tk[side]["gripper"]))))
        return logs

    def result(self, label, q12, source):
        tg = codex()["receive_session"].tg
        return dict(label=label, result="HOLDING", q12=list(q12), independent=True, source_kind="REAL", provenance=tg.INDEPENDENT, source=source, session_id=self.session.session_id, executor_pid=os.getpid())

    def ack(self):
        """After a write to the right arm: its Goal_Position register read back, as the applied command."""
        tg = codex()["receive_session"].tg; raw = self._read(); q = self.p1.rc.ticks_to_q12(self.mp, dict(left=raw["goal"]["left"], right=raw["goal"]["right"]))
        self.pending["owner_ack"] = dict(accepted=True, independent=True, source="Goal_Position registers read back after the executor run", session_id=self.session.session_id, executor_pid=os.getpid(), q12=[float(v) for v in q])
        if getattr(self.session.applier, "relax", None) is not None:
            self._safe(lambda: self.session.applier.relax({s: raw["ticks"][s] for s in ("left", "right")}))      # the proposal generator starts its next proposal from where the executor left the arms

    def apply_dispatch(self, dispatch, observation):
        """The writer RECEIVE ReceiveManipulation calls with each SEND proposal. The proposal is recorded by the caller; what is
        SENT is the executor's own checked run to the target the proposal is heading for:
          COORDINATE     the planned waypoint the corrector is on (once), then the right close by the Jaw (once)
          LOAD_TRANSFER  the left gripper command of the proposal
          RELEASE        the left gripper / left arm target of the proposal
        The receipt says what was run; it is not feedback (the next observation is)."""
        s = self.session; phase = getattr(s, "phase", None); prop = s.proposals[-1]["q12"] if s.proposals else None
        if self.stopped is not None:
            raise RuntimeError("the owner is stopped")
        if phase == "COORDINATE":
            wp = self.waypoint()
            if wp["label"] not in self.done:
                if wp["label"] == "RIGHT_INSERT":
                    if not (self.insert or {}).get("ready"):
                        return dict(status="NOTHING_SENT", why="the right insert is not run: no insert re-planned from the observed block has passed its check", not_independent_feedback=True)
                    self.go("right_pregrasp_replanned", self.insert["pre_q12"], arms=("right",))      # the re-planned pre-grasp first (the approach that was run belonged to the estimate)
                self.near_handover = wp["label"] == "RIGHT_INSERT"
                moved = [side for side, sl in (("left", slice(0, 5)), ("right", slice(6, 11))) if not np.allclose(np.array(wp["q12"])[sl], np.array(observation["q12"])[sl], atol=2e-3)]
                grips = ["right"] if abs(wp["q12"][11] - observation["q12"][11]) > 2e-3 and wp["label"] == "RIGHT_OPEN" else []
                logs = self.go(wp["label"].lower(), wp["q12"], arms=moved, grippers=grips); self.done.add(wp["label"])
                rec = self.result(wp["label"], wp["q12"], "sweepick_move_a.run result HOLDING for this waypoint")
                self.pending["right_insert_confirmed" if wp["label"] == "RIGHT_INSERT" else "motion_result"] = rec
                if "right" in moved or grips:
                    self.ack()
                return dict(status="EXECUTOR_RAN", waypoint=wp["label"], runs={k: v["result"] for k, v in logs.items()}, not_independent_feedback=True)
            if getattr(s, "_right_close_started", False) and not self.right_closed:
                rj = self.refs["right_jaw"]
                if not rj["usable"]:
                    raise RuntimeError("the right gripper profile is not there")
                raw = self._read(); d = self.out / "handoff" / f"{self.k:02d}_right_close"; self.k += 1; cert = self._write_check(d, "sweep", self.close_sweep_check(raw))
                if cert.get("nominal_verdict") != "CLEAR":
                    raise RuntimeError(f"right close: the closing sweep touches something that is not a pad on the block: {cert.get('first_nominal_contact')}")
                code, log = self.grip("right", d, torque_joints=list(ALL), cert=cert, level="model", execute=True, jaw_profile=Path(rj["file"]), jaw_obj=s.right_jaw, stop_requested=Path(str(self.out) + ".STOP").exists, **self.run_kw)
                self.note("right_close", result=log["result"], grip=log.get("grip"))
                if log["result"] != "GRIP_CONTACT_HOLDING":
                    raise RuntimeError(f"right close: executor ended {log['result']}")
                self.right_closed = True; self.block_owner = "both"; self.ack()
                return dict(status="EXECUTOR_RAN", waypoint="RIGHT_CLOSE", result=log["result"], not_independent_feedback=True)
            return dict(status="NOTHING_SENT", why="the waypoint was already run; waiting for the session's next step", not_independent_feedback=True)
        tk = self.p1.rc.q12_to_ticks(self.mp, np.array(prop, float)); raw = self._read(); ran = {}
        if phase == "LOAD_TRANSFER":
            if "load_transfer" not in self.done and not np.allclose(np.array(prop)[6:11], np.array(observation["q12"])[6:11], atol=2e-3):
                self.pre_lift_right = {n: float(raw["ticks"]["right"][n]) for n in ARM}
                ran["right_lift"] = self.segment("load_transfer", "right", target={n: float(round(tk["right"][n])) for n in ARM})["result"]; self.done.add("load_transfer"); self.ack()
            return dict(status="EXECUTOR_RAN" if ran else "NOTHING_SENT", phase=phase, runs=ran, not_independent_feedback=True)
        if self.scope == "observe":
            raise RuntimeError("OBSERVE scope: a command of the left release reached the writer; it is not sent")
        lg = float(round(tk["left"]["gripper"]))
        if round(raw["goal"]["left"]["gripper"]) != lg:
            ran["left_gripper"] = self.segment("left_gripper", "left", target=dict(gripper=lg))["result"]
        if phase == "RELEASE" and not np.allclose(np.array(prop)[:5], np.array(observation["q12"])[:5], atol=2e-3):
            ran["left_arm"] = self.segment("left_retreat", "left", target={n: float(round(tk["left"][n])) for n in ARM})["result"]; self.done.add("left_retreat"); self.block_owner = "right"
        return dict(status="EXECUTOR_RAN" if ran else "NOTHING_SENT", phase=phase, runs=ran, not_independent_feedback=True)

    def lift_m(self):
        """The planned right-hand lift of the load transfer. OBSERVE scope: the value approved for this session (it is a motion to be run, not a judgement). FULL scope: the value the support reference was measured with."""
        if self.scope == "observe":
            return self.right_lift_m
        ref = self.refs["support"]["doc"]; return None if ref is None else float(ref["right_lift_m"])

    def observe_row(self, obs, raw, when):
        """OBSERVE scope: what the support reference will be made from. Raw registers of this cycle; no judgement."""
        Rl = float(self.mp["left"]["gripper"]["hi"] - self.mp["left"]["gripper"]["lo"]); Rr = float(self.mp["right"]["gripper"]["hi"] - self.mp["right"]["gripper"]["lo"]); l, r = obs["left"], obs["right"]
        self.support_rows.append(dict(when=when, seq=obs["seq"], t_bus_monotonic=obs["feedback_stamp"], left_load_raw=raw["load"]["left"], right_load_raw=raw["load"]["right"], left_gripper_residual_ticks=None if l.get("command") is None else abs(l["actual"] - l["command"]) * Rl,
                                      right_gripper_residual_ticks=None if r.get("command") is None else abs(r["actual"] - r["command"]) * Rr, left_ticks=raw["ticks"]["left"], right_ticks=raw["ticks"]["right"],
                                      frames={k: None if v is None else dict(n=v.get("n"), fresh=v.get("fresh"), recv_monotonic_s=v.get("recv_monotonic_s")) for k, v in self.last_images.items()}, object_at_right_tool=(obs.get("object_evidence") or {}).get("object_at_tool")))

    def finish_observation(self):
        """OBSERVE scope, the planned end: the right hand goes back down the lift, opens, goes back to its pre-grasp and home (released);
        the left hand, which never let go, carries the block back to the pose it was lifted to. Every move is a checked executor
        run. Then the session's own planned put-back is the way home. The left jaws are never opened here."""
        raw = self._read(); self.near_handover = True
        self.segment("observe_right_lower", "right", target={n: float(self.pre_lift_right[n]) for n in ARM})
        rlo, rhi = float(self.mp["right"]["gripper"]["lo"]), float(self.mp["right"]["gripper"]["hi"]); self.segment("observe_right_open", "right", target=dict(gripper=float(round(rlo + self.right_open_q * (rhi - rlo))))); self.block_owner = "left"
        back = (self.insert or {}).get("pre_q12") or next(w["q12"] for w in self.session.corrector.plan["waypoints"] if w["label"] == "RIGHT_APPROACH")
        self.go("observe_right_back", back, arms=("right",)); self.near_handover = False
        raw = self._read(); self.segment("observe_right_home", "right", rest={n: float(self.home["right"][n]) for n in ALL if raw["torque"]["right"][n]})
        self.segment("observe_left_back_to_lift", "left", target={n: float(self.left_at_lift[n]) for n in ARM})
        raw = self._read(); Rl = float(self.mp["left"]["gripper"]["hi"] - self.mp["left"]["gripper"]["lo"]); res = abs(raw["ticks"]["left"]["gripper"] - raw["goal"]["left"]["gripper"])
        at = all(abs(raw["ticks"]["left"][n] - self.left_at_lift[n]) <= 50 for n in ARM) and all(raw["torque"]["left"][n] for n in ALL)       # 50 = the progress profile's following lead: holding at the lift pose as a loaded arm does
        self.scope_result = dict(left_at_lift=bool(at), left_jaw_residual_ticks=float(res), left_still_loaded=bool(res > self.left_profile["floor"] * Rl), right_released=not any(raw["torque"]["right"].values()))
        doc = dict(schema="sweepick.support-observation.v1", session_id=getattr(self.ep, "id", None), scope="OBSERVE: right contact and load-transfer observation; the left hand was not released", right_lift_m=self.right_lift_m, rows=self.support_rows, end=self.scope_result,
                   note="raw observations for a person to read. A support reference (which left-arm joints' load fell, by how much) is written from this by a person as sweepick_support_reference.json with source_kind REAL and measurement_evidence naming this file; nothing here is a judgement and nothing is activated by itself")
        (self.out / "SUPPORT_OBSERVATION.json").write_text(json.dumps(_plain(doc), indent=1)); self.note("support_observation_done", **self.scope_result)

    def insertion_ready(self, observation):
        """RECEIVE gate before RIGHT_INSERT. Seeing the block is not enough. With the block's pose OBSERVED in the hand and applied
        to the planning scene (the session did that for this cycle), the right pre-grasp and insert are planned AGAIN by the
        teacher's own live re-plan (_plan_right_grasp: IK and its path checks with the block carried by the left hand); the
        old waypoint is not run. Then the numbers decide: the room the open right pads leave on each side of the block along
        their jaw axis must exceed what the observation does not resolve (its centre uncertainty plus the effect of the
        yaw that is only assumed). True only then; the record says why not otherwise."""
        held = (observation or {}).get("held_object") or {}; rec = dict(observed_in_hand=bool(self.held is not None and self.held.get("seen_tool_from_object") is not None), pose_role=held.get("pose_role"))
        try:
            if not rec["observed_in_hand"] or held.get("object_pose_model") is None or not held.get("final_insertion_ready"):
                rec.update(ready=False, why="the block has not been observed in the hand in this cycle's pose"); return False
            if self.insert is not None and self.insert.get("for_seen_seq") == self.held.get("seen_seq"):
                rec.update(self.insert["record"]); return bool(self.insert["ready"])
            import mujoco
            s = self.session; t = self.teacher; d = s.state["_data"]; m = s.state["_model"]; q = np.array(observation["q12"], float); t.cmd = self.gear(q); t.queue = []
            t._plan_right_grasp(d); pre = np.array(t.segment["goal"], float); grasp = np.array(t.queue[0][1], float); t.queue = []
            base = d.qpos.copy(); rel = t.site_block_rel(d, "left"); bq = t.plan.carried_block(base, grasp, ("left", rel)); pd = t.plan.pose(base, grasp, bq)
            pads = [pd.geom_xpos[m.geom(n).id].copy() for n in ("right_pgripper_pad_1", "right_pgripper_pad_2")]; c = pd.xpos[t.cm.block_body].copy(); rb = pd.xmat[t.cm.block_body].reshape(3, 3)
            jaw = pads[1] - pads[0]; gap = float(np.linalg.norm(jaw)); jaw /= gap; half_along = float(self.held_half * sum(abs(np.dot(rb[:, k], jaw)) for k in range(3)))
            off = float(np.dot(c - (pads[0] + pads[1]) / 2, jaw)); room = [gap / 2 - half_along + off, gap / 2 - half_along - off]
            unc = (self.held.get("seen") or {}).get("object_pose_uncertainty") or {}; yaw_deg = float(self.held["components"]["yaw"].get("uncertainty_deg", 3.0)); need = float(unc.get("centre_m", np.inf)) + self.held_half * np.sqrt(2) * np.radians(yaw_deg)
            ok = bool(min(room) > need)
            rec.update(ready=ok, planner="HandoverTeacher._plan_right_grasp on the observed block", right_depth_m=t.info.get("right_depth_m"), right_beta_deg=t.info.get("right_beta_deg"), pad_gap_m=gap, block_extent_along_jaw_m=2 * half_along, block_off_centre_m=off,
                       room_each_side_m=room, not_resolved_m=need, not_resolved_basis=f"centre uncertainty of the top-depth observation {unc.get('centre_m')} m + half the block's diagonal turned by the assumed-yaw allowance {yaw_deg} deg (pad centres; pad thickness is in the teacher's collision check, not in this number)",
                       why=None if ok else "the room beside the block is not larger than what the observation leaves open")
            self.insert = dict(for_seen_seq=self.held.get("seen_seq"), ready=ok, pre_q12=t.to_canonical(pre).tolist(), grasp_q12=t.to_canonical(grasp).tolist(), record=dict(rec))
            if ok:                                                              # the plan the state machine goes on with IS the re-planned one
                plan = s.corrector.plan; i = next(k for k, w in enumerate(plan["waypoints"]) if w["label"] == "RIGHT_INSERT"); old = plan["waypoints"][i]["q12"]
                new = list(self.insert["grasp_q12"]); new[5], new[11] = old[5], old[11]; plan["waypoints"][i] = dict(plan["waypoints"][i], q12=new, replanned_from="block observed in the hand", previous_q12=old); plan["_trajectories"][i] = None
                plan["right_insert_q12"] = new; s.state["right_insert_q12"] = new; s.corrector.trajectory = None; self.insert["grasp_q12"] = new
            return ok
        except Exception as e:
            rec.update(ready=False, why=f"the insert could not be re-planned from the observed block: {type(e).__name__}: {e}"); self.insert = dict(for_seen_seq=self.held.get("seen_seq") if self.held else None, ready=False, record=dict(rec)); return False
        finally:
            key = (rec.get("ready"), rec.get("why"), (self.held or {}).get("seen_seq"))
            if key != getattr(self, "_insert_noted", None):                     # recorded when it is computed or its answer changes, not once per cycle
                self._insert_noted = key; self.note("insertion_check", **rec)

    def close_sweep_check(self, raw):
        """The whole possible closing sweep of the RIGHT jaws at the pose they stand in, checked with the block in the scene by the
        same collision check as every segment (teacher's Planner.collisions): the pads of the two hands on the block are the
        intended contacts; any other pair is a contact and the close is not run. Nothing is returned without this computation."""
        from sweepick.control import sweepick_trajectory_profile as tt
        rj = self.refs["right_jaw"]["doc"]; lo = float(self.mp["right"]["gripper"]["lo"]); arm = raw["ticks"]["right"]; g0, g1 = float(arm["gripper"]), lo + float(rj["end_stop_ticks_above_closed"]["value"]); n = int(abs(g0 - g1)) + 1
        goals = {j: [float(arm[j])] * n for j in ARM}; goals["gripper"] = [float(v) for v in np.linspace(g0, g1, n)]
        doc = dict(side="right", trajectory=dict(start=dict(arm), target=dict(gripper=g1), moving=["gripper"], dt=0.02, steps=n, goals=goals)); was = self.near_handover; self.near_handover = True
        try:
            c = self._check(doc, "right", raw)
        finally:
            self.near_handover = was
        return dict(c, arm_ticks={n_: arm[n_] for n_ in ARM}, sweep=[g0, g1], checked="the closing sweep itself, every tick, with the block where this cycle's held pose puts it")

    def on_stop(self, dispatch, latest):
        """A STOP of the proposal generator or a failed executor run: nothing more is sent by this owner. The arms stay as the executor left them (holding); no release, no opening, no way back is run by itself."""
        self.stopped = dict(reasons=list(getattr(dispatch, "reasons", []) or []), t=self.clock()); self.note("owner_stop", **self.stopped)
        return dict(handled=True, sent_after_stop=0, state="holding as the executor left it; a person decides")

    def wait_next_cycle(self):
        """The next feedback cycle: registers and frames are read HERE, before the task manager takes its 'now', so that what it
        acts on was measured before it decided. Cycles in which nothing was sent and nothing changed are counted in time: when
        the Jaw's own confirmation time has passed many times over with no change, waiting longer cannot change anything."""
        self.sleep(0.05); step = (getattr(self.ep, "phase", None) or {}).get("step")
        if step in ("VERIFY_PLACE", "REOBSERVE_CLEAR") and self.area is not None and self.task is not None:
            self.area.take(step, self.task); self.idle_since = self.clock()
        fr_ = self._frames(); self.cycle = (self.raw(), fr_)                      # the frames that were already there when the registers are read (a frame's age is counted up to the bus read)
        if self.scope == "observe" and "load_transfer" in self.done:
            self.lift_seen_since = getattr(self, "lift_seen_since", None) or self.clock()
            if self.clock() - self.lift_seen_since >= self.observe_s and any(r["when"].startswith("after") for r in self.support_rows):
                self.finish_observation(); raise ScopeDone("OBSERVE scope: the observation is complete and the planned return was run")
        sig = (len(self.notes_sent), getattr(self.session, "phase", None), tuple(sorted(self.done)), self.right_closed)
        if sig != self.idle_sig:
            self.idle_sig, self.idle_since = sig, self.clock()
        elif self.clock() - self.idle_since > self.idle_limit_s:
            raise RuntimeError(f"nothing was sent and nothing changed for {self.idle_limit_s:.1f} s ({self.idle_basis}); the hand-over waits for something that is not coming")

    def take_right(self, plan):
        """The right arm joins this owner with the plan's own first right-hand command (RIGHT_OPEN): the executor opens the right
        jaws and powers the whole right arm where it stands. From then on the right Goal_Position registers are commands of this session."""
        raw = self._read(); wp = next(w for w in plan["waypoints"] if w["label"] == "RIGHT_OPEN"); tk = self.p1.rc.q12_to_ticks(self.mp, np.array(wp["q12"], float))
        log = self.segment("right_take_open", "right", target=dict(gripper=float(round(tk["right"]["gripper"]))), power=list(ALL)); self.took_right = True; return log

    def abandon(self):
        """The hand-over was not entered (nothing of it was commanded to the block): if this owner powered the right arm, it is put back to its start pose and released by the executor."""
        if not getattr(self, "took_right", False):
            return None
        raw = self._read(); on = [n for n in ALL if raw["torque"]["right"][n]]
        log = self.segment("right_release", "right", rest={n: float(self.home["right"][n]) for n in on}) if on else dict(result="RETURNED_RELEASED"); self.took_right = False; return log


class Downstream:
    """PLACE and STOW for RECEIVE ReceiveManipulation, with the same executor."""
    def __init__(self, owner):
        self.o, self.context = owner, None

    def set_receive_context(self, data):
        self.context = data

    def place(self, task, now):
        tm = codex()["receive_session"].tm; o = self.o; tgt = o.refs["place"]["doc"]
        if tgt is None:
            return tm.CommandResult(tm.Status.RUNNING, tm.Kind.REAL, "place target not confirmed on the real desk", now, dict(status="NOT_READY", missing=["confirmed place target"]))
        try:
            import mujoco
            obs = o.last_obs; t = o.teacher; m = t.m; d = mujoco.MjData(m); o.p1.sweepick_kin.set_q12(m, d, np.array(obs["q12"])); mujoco.mj_forward(m, d)
            T = np.array(self.context["object_pose_model"]); adr = t.plan.cm.block_qpos; quat = np.empty(4); mujoco.mju_mat2Quat(quat, np.ascontiguousarray(T[:3, :3]).ravel()); d.qpos[adr:adr + 7] = np.r_[T[:3, 3], quat]; mujoco.mj_forward(m, d)
            t.cmd = o.gear(obs["q12"]); rel = t.site_block_rel(d, "right"); t.plan.pose(d.qpos.copy(), t.cmd); ps, rot = t.plan.site_frame("right")
            B = np.eye(4); B[:3, :3] = rot.T @ T[:3, :3]; B[:3, 3] = rot.T @ (T[:3, 3] - ps); Binv = np.linalg.inv(B)       # the block in the right tool frame, from this cycle's held pose
            c = np.array(tgt["centre_xy_m"], float); z = float(tgt["surface_z_m"]) + float(o.held_half); up = float(tgt.get("approach_height_m", 0.05)); found = None; tried = []
            ups = [np.eye(3)] + [np.array(r, float) for r in ([[1, 0, 0], [0, 0, -1], [0, 1, 0]], [[1, 0, 0], [0, 0, 1], [0, -1, 0]], [[0, 0, 1], [0, 1, 0], [-1, 0, 0]], [[0, 0, -1], [0, 1, 0], [1, 0, 0]], [[1, 0, 0], [0, -1, 0], [0, 0, -1]])]
            for R0 in ups:                                                        # the block flat on the target with any of its faces down and any yaw: the first the right arm can reach and the teacher's path check passes
                for yaw in np.linspace(-np.pi, np.pi, 24, endpoint=False):
                    Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1.0]]); Tb = np.eye(4); Tb[:3, :3] = Rz @ R0; Tb[:3, 3] = [c[0], c[1], z]; Ts = Tb @ Binv; Ta = Ts.copy(); Ta[2, 3] += up
                    try:
                        t.cmd = o.gear(obs["q12"]); g1, i1 = t.plan_move(d, "right", Ta[:3, 3], Ta[:3, 0], [Ta[:3, 2]], carry=("right", rel), allow=("right",), label="PLACE above")
                        keep = t.cmd.copy(); t.cmd = g1[-1]; base2 = t.plan.pose(d.qpos.copy(), g1[-1], t.plan.carried_block(d.qpos.copy(), g1[-1], ("right", rel))).qpos.copy(); d2 = mujoco.MjData(m); d2.qpos[:] = base2; mujoco.mj_forward(m, d2)
                        g2, i2 = t.plan_move(d2, "right", Ts[:3, 3], Ts[:3, 0], [Ts[:3, 2]], carry=("right", rel), allow=("right",), label="PLACE down"); found = (g1, g2, i1, i2, float(yaw)); break
                    except Exception as e:
                        tried.append(str(e)[:60])
                if found:
                    break
            if not found:
                raise RuntimeError(f"no reachable, collision-free way to set the block flat on the place target ({len(tried)} candidates; last: {tried[-1] if tried else None})")
            goals, goals2, info, info2, yaw = found
            for g in goals:
                o.go("place_above", t.to_canonical(g).tolist(), arms=("right",))
            for g in goals2:
                o.go("place_down", t.to_canonical(g).tolist(), arms=("right",))
            o.block_owner = "placed"; q_open = float(o.right_open_q); raw = o._read(); tk = o.p1.rc.q12_to_ticks(o.mp, np.r_[obs["q12"][:11], q_open])
            o.segment("place_open", "right", target=dict(gripper=float(round(tk["right"]["gripper"]))))
            for g in reversed(goals):
                o.go("place_retreat", t.to_canonical(g).tolist(), arms=("right",))
            o.ack(); o.note("place_done", target=tgt.get("name"))
            return tm.CommandResult(tm.Status.SUCCEEDED, tm.Kind.REAL, "placed and opened by the executor; whether the block rests in the target is VERIFY_PLACE's question", now, dict(status="COMPLETE", target=tgt, ik=[str(info), str(info2)]))
        except Exception as e:
            o.on_stop(type("D", (), dict(reasons=[f"PLACE: {type(e).__name__}: {e}"]))(), None)
            return tm.CommandResult(tm.Status.FAILED, tm.Kind.REAL, f"PLACE: {type(e).__name__}: {e}", now, dict(status="STOP", requires_explicit_restart=True))

    def stow(self, now):
        tm = codex()["receive_session"].tm; o = self.o
        try:
            for side in ("right", "left"):
                raw = o._read(); on = [n for n in ALL if raw["torque"][side][n]]
                if on:
                    o.segment("stow", side, rest={n: float(o.home[side][n]) for n in on})
            o.note("stow_done")
            return tm.CommandResult(tm.Status.SUCCEEDED, tm.Kind.REAL, "both arms returned to their start poses and released by the executor", now, dict(status="COMPLETE"))
        except Exception as e:
            o.on_stop(type("D", (), dict(reasons=[f"STOW: {type(e).__name__}: {e}"]))(), None)
            return tm.CommandResult(tm.Status.FAILED, tm.Kind.REAL, f"STOW: {type(e).__name__}: {e}", now, dict(status="STOP", requires_explicit_restart=True))


class Area:
    """Perception for the TaskManager's VERIFY_PLACE and REOBSERVE_CLEAR: the session's own look (SDK top depth through the board) on a region. This is the ONE final area check and the one place a clean-resume request comes from."""
    def __init__(self, owner, out, look):
        self.o, self.out, self.look, self.kind, self.k = owner, Path(out), look, codex()["receive_session"].tm.Kind.REAL, 0
        self.records = []; self.taken = None

    def region(self, region, tag):
        from sweepick.integration.sweepick_observation_session import real_area
        self.k += 1; sc = self.look(self.out, f"{tag}_{self.k}")
        if sc.get("support_region_at") is not None:                              # a look that already evaluated this region (tests)
            reg = sc["support_region_at"](region)
        else:
            p1 = self.o.p1; board = (sc.get("board") or {}).get("T_camera_from_board")
            if board is None:
                reg = dict(valid=False, state="UNKNOWN", reason="board not found")
            else:                                                                 # as sweepick_pick01.support_region: the region centre into the printed board's frame
                depth = np.load(Path(sc["_out"]) / "looks" / sc["tag"] / "depth_mm.npy"); m = p1.model(); Tb = np.eye(4); Tb[:3, 3] = p1.DATUM_IN_MODEL_BASE; Tw = np.eye(4); Tw[:3, 3] = m.body("left_base").pos
                pb = np.linalg.inv(Tw @ Tb @ p1.T_DATUM_FROM_BOARD) @ np.r_[region.center_xy[0], region.center_xy[1], p1.BOARD_Z_IN_WORLD, 1.0]
                reg = real_area(depth, sc["K"], board, 0.0, (float(pb[0]), float(-pb[1])), region.radius_m)
        done = self.o.clock(); self.records.append(dict(tag=tag, region=region.region_id, result=reg, look_completed_monotonic_s=done, sensor_stamps=sc.get("stamps")))
        self.o.note("area_look", tag=tag, region=region.region_id, state=reg.get("state"), valid=reg.get("valid")); return reg, sc, done

    def take(self, step, task):
        """Called by the owner BEFORE the task manager's step for the two looking states, so that the look is finished when the manager takes its 'now'. The record's stamp is when the look was completed; the sensor's own stamps are kept beside it."""
        if step == "VERIFY_PLACE":
            self.taken = ("place", self.region(task.target_region, "05_place"))
        elif step == "REOBSERVE_CLEAR":
            self.taken = ("source", self.region(task.source_region, "06_area"))

    def _get(self, kind, region, tag):
        if self.taken is not None and self.taken[0] == kind:
            r, self.taken = self.taken[1], None; return r
        return self.region(region, tag)                                          # not taken beforehand: taken now, and then it is later than the manager's 'now' and the manager will call it not fresh

    def observe_object(self, task, now):
        return None

    def _validity(self, reg, now):
        tm = codex()["receive_session"].tm; import dataclasses
        names = [f.name for f in dataclasses.fields(tm.Validity)]; v = dict(valid=bool(reg.get("valid")), stamp_s=now, reason="" if reg.get("valid") else str(reg.get("reason") or reg.get("state") or "not validly observed"), source="sweepick look: SDK top depth through the printed board", frame_id="model")
        return tm.Validity(**{n: v[n] for n in names if n in v})

    def observe_area(self, region, now):
        tm = codex()["receive_session"].tm; reg, sc, done = self._get("source", region, "06_area")
        return tm.AreaObservation(region_id=region.region_id, clear=bool(reg.get("valid") and reg.get("state") == "CLEAR"), points_above_surface=int(reg.get("points_above_8mm") or 0), validity=self._validity(reg, done))

    def verify_place(self, task, now):
        """In the target region: validly observed and occupied (top depth). Supported: the right jaws were opened by the executor at the place pose and the right hand went back up (its runs ended HOLDING). Separated: the right gripper register stands at its open tick."""
        tm = codex()["receive_session"].tm; reg, sc, done_t = self._get("place", task.target_region, "05_place"); raw = self.o._read(); done = [n["name"] for n in self.o.notes if n.get("kind") == "segment" and n.get("result") == "HOLDING"]
        rlo, rhi = float(self.o.mp["right"]["gripper"]["lo"]), float(self.o.mp["right"]["gripper"]["hi"]); opened = (raw["ticks"]["right"]["gripper"] - rlo) / (rhi - rlo) >= self.o.right_open_q - 2 * self.o.left_profile["floor"]
        return tm.PlaceVerification(object_in_region=bool(reg.get("valid") and reg.get("state") == "OCCUPIED"), object_supported=bool("place_open" in done and "place_retreat" in done), gripper_separated=bool(opened and "place_retreat" in done),
                                    evidence=dict(region=reg, executor_runs=[n for n in done if n.startswith("place")], right_gripper_tick=raw["ticks"]["right"]["gripper"]), validity=self._validity(reg, done_t))


def _plain(o, depth=0):
    """A record without image arrays: an array becomes its shape (the frame itself is in the recorder's files under its number)."""
    if isinstance(o, np.ndarray):
        return o.tolist() if o.size <= 16 else f"<array {o.shape}>"
    if isinstance(o, dict):
        return {str(k): _plain(v, depth + 1) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_plain(v, depth + 1) for v in o]
    if isinstance(o, (np.floating, np.integer)):
        return o.item()
    return o if isinstance(o, (str, int, float, bool, type(None))) else str(o)[:300]


def _mk(cls, **k):
    """Build one of the TaskManager's records from the fields it has (its field names are its own)."""
    import dataclasses
    names = [f.name for f in dataclasses.fields(cls)]; return cls(**{n: k[n] for n in names if n in k})


NEEDS = dict(full=("right_jaw", "right_wrist", "support", "place"), observe=("right_jaw",))      # OBSERVE measures what the support reference AND the right wrist reference are made from, and places nothing: it asks for none of the three. The right close needs the right gripper's own profile in every scope


def build(out, episode, *, p1=None, run_kw=None, read=None, frames=None, look=None, check=None, refs=None, run=None, grip=None, clock=time.monotonic, sleep=time.sleep, applier=None, rig=None, scope="full", right_lift_m=None):
    """The hand-over callback for sweepick_pick01.run_session(full_chain=True): RECEIVE FullEpisodeHandoff with this process's owner injected. Returns (handoff, owner)."""
    if p1 is None:
        from sweepick.integration import sweepick_manipulation_session as p1
    cx = codex(); entry, sess, tm, tg = cx["receive_entry"], cx["receive_session"], cx["receive_session"].tm, cx["receive_session"].tg
    refs = refs if refs is not None else field_refs(); out = Path(out); codex_sha = {n: hashlib.sha256(Path(m.__file__).read_bytes()).hexdigest() for n, m in cx.items()}
    o = Owner(out, episode, refs, p1=p1, run=run, grip=grip, run_kw=run_kw, read=read, frames=frames, look=look, check=check, clock=clock, sleep=sleep)
    assert scope in NEEDS; o.scope, o.right_lift_m = scope, None if right_lift_m is None else float(right_lift_m)
    o.scene_look = o.last_held_pose = o.scene = None; o.area = o.task = None; o.near_handover = False; o.last_obs = None; o.held_half = 0.02; o.held_seen_fn = held_seen
    down = Downstream(o); area = Area(o, out, look or p1.look); ports = ("/dev/dapier/left_arm", "/dev/dapier/right_arm")
    from sweepick.control import sweepick_command_application as rx
    from sweepick.control import sweepick_trajectory_executor as mv
    from sweepick.control import sweepick_trajectory_profile as tt

    def right_profile():
        rj = refs["right_jaw"]
        if not rj["usable"]:
            return None
        d = rj["doc"]; lo, hi = float(o.mp["right"]["gripper"]["lo"]), float(o.mp["right"]["gripper"]["hi"])
        return dict(verified=True, side="right", source_kind="REAL", source=rj["file"], device_identity=d.get("device_identity"), measurement_evidence=d.get("measurement_evidence"), device=None)

    def session_kwargs(present):
        lo, hi = float(o.mp["left"]["gripper"]["lo"]), float(o.mp["left"]["gripper"]["hi"]); rp = right_profile()
        ap = applier or rx.Applier(o.mp, (run_kw or {}).get("profile") or tt.load_profile(), present, real_jaw_profile=None if rp is None else dict(left=str(mv.JAW_PROFILE), right=refs["right_jaw"]["file"]))                       # the same motion profile the executor uses
        prof = (run_kw or {}).get("profile") or tt.load_profile(); dt = float(prof["dt"])
        if rp is not None:
            rp["device"] = mv.jaw_device(float(o.mp["right"]["gripper"]["lo"]), float(o.mp["right"]["gripper"]["hi"]), prof, Path(refs["right_jaw"]["file"]))
        return dict(ports=ports, owner_pid=os.getpid(), port_owner_pids={p: os.getpid() for p in ports}, rig=rig if rig is not None else o, applier=ap, control_dt=dt, session_id=episode.id,
                    left_jaw=tg.Jaw(mv.jaw_device(lo, hi, prof), dt, provenance=tg.INDEPENDENT), right_jaw_profile=rp, right_jaw=None, support_confirmation=lambda ob: ob.get("support_confirmation"),
                    release_controller=Release(o), load_transfer_controller=LoadTransfer(o), insertion_alignment=o.insertion_ready)

    def task_of(state):
        c = (state.get("plan") or {}).get("object_centre_world") or json.loads((out / state["scene"]).read_text())["object"]["centre_world"]; tgt = refs["place"]["doc"] or {}
        return tm.Task(f"sweepick-{episode.id}", tm.Region("source", "model", (float(c[0]), float(c[1])), 0.04, float(p1.BOARD_Z_IN_WORLD)), tm.Region(tgt.get("name", "place"), "model", tuple(tgt.get("centre_xy_m", (0.0, 0.0))), float(tgt.get("radius_m", 0.04)), float(tgt.get("surface_z_m", p1.BOARD_Z_IN_WORLD))), needs_two_arm_coordination=True)

    def preflight(o_, state):
        """Before the first motor command: the field records, then RECEIVE own reference check on exactly the objects the hand-over will use."""
        items = {n: dict(present=r["present"], usable=r["usable"], file=r["file"], what=r["what"], blocks=r["blocks"]) for n, r in refs.items()}
        missing = [f"FIELD {n}: {r['what']} -> {r['file']}" for n, r in refs.items() if not r["usable"] and n in NEEDS[scope]]; items["scope"] = dict(scope=scope, needs=NEEDS[scope], left_release="NOT POSSIBLE in this scope" if scope == "observe" else "after a confirmed support record")
        if scope == "observe" and o.right_lift_m is None:
            missing.append("PLAN right_lift_m: the right-hand lift approved for this observation session (given when the session is started; a motion to be run, not a measured judgement)")
        handoff.task = o.task = task_of(state); o.home = dict(left=state["home_ticks"], right=state["right_ticks"])
        present = dict(left=state["home_ticks"], right=state["right_ticks"])
        try:
            rr = entry.receive_references_ready(session_kwargs(present), observation_source=o.observation, downstream=down, real_stow_profile=dict(kind="return to the start poses of this session", home=o.home), apply_dispatch=o.apply_dispatch, on_stop=o.on_stop, record_event=o.note)
            code_missing = [m for m in rr["missing"] if m not in ("right REAL Jaw profile",)]
        except Exception as e:
            code_missing = [f"reference check failed: {type(e).__name__}: {e}"]
        items["code_references"] = dict(missing=code_missing, checked_by="sweepick_261008_receive_entry.receive_references_ready")
        now_sha = {n: hashlib.sha256(Path(m.__file__).read_bytes()).hexdigest() for n, m in cx.items()}; items["codex_modules"] = dict(sha256=codex_sha, dir=str(CODEX))
        if now_sha != codex_sha:
            code_missing.append("RECEIVE receive modules changed on disk since this process imported them: " + ", ".join(n for n in cx if now_sha[n] != codex_sha[n]) + " (a session runs on the files it started with)")
        return dict(ready=not missing and not code_missing, missing=missing + [f"CODE {m}" for m in code_missing], items=items)

    def prepare(snapshot, scene, evidence):
        st = json.loads((out / "STATE.json").read_text()); close = (st["stages"].get("close") or {}); end = close.get("end") or {}
        raw = o._read(); o.scene_look = scene
        q_close = p1.rc.ticks_to_q12(o.mp, dict(left={n: float(v) for n, v in end["ticks"].items()}, right=raw["ticks"]["right"])); c, yaw = p1.state_object(out, st)
        h = held_pose(q_close, scene["q12"], c, yaw, model=p1.model(), pad_geoms=p1.PAD_GEOMS, source=f"session {out.name}: close readback + look {scene.get('tag')}"); h["grasp_evidence"] = (snapshot or {}).get("grasp_evidence"); o.held = h
        o.scene = scene_bindings(p1.model(), scene, p1.BOARD_Z_IN_WORLD, model_sha=hashlib.sha256(p1.SCENE.read_bytes()).hexdigest(), source_sha=hashlib.sha256((RUNTIME / "source/production_source_g000/integration_scenes.py").read_bytes()).hexdigest())
        b = dict(source_kind="REAL", object_pose_model=h["coarse_preview_pose"], object_pose_frame=o.scene["frame"], object_pose_source="COARSE PREVIEW ESTIMATE (see components): " + h["object_pose_source"], object_pose_uncertainty=dict(h["object_pose_uncertainty"], components=h["components"]),
                 q_mapping_verified=False, geometry_verified=False, right_jaw_profile=None, scene={k: v for k, v in o.scene.items() if k not in ("provenance", "geometry_verified")})
        (out / "HELD_POSE.json").write_text(json.dumps(h, indent=1, default=str))
        grip_log = (close.get("grip") or {}); onset = (grip_log.get("contact_onset") or {}).get("commanded"); lo, hi = float(o.mp["left"]["gripper"]["lo"]), float(o.mp["left"]["gripper"]["hi"])
        o.left_onset_command = None if onset is None else (float(onset) - lo) / (hi - lo); o.left_open_q = (float(st["home_ticks"]["gripper"]) - lo) / (hi - lo)
        rlo, rhi = float(o.mp["right"]["gripper"]["lo"]), float(o.mp["right"]["gripper"]["hi"]); o.right_open_q = (float(st["right_ticks"]["gripper"]) - rlo) / (rhi - rlo)
        o.last_held_pose = h["coarse_preview_pose"]; o.left_at_lift = {n: float(raw["ticks"]["left"][n]) for n in ALL}
        man = _prepare(entry, snapshot, b, session_kwargs(dict(left=raw["ticks"]["left"], right=raw["ticks"]["right"])), o, down)
        if not isinstance(man, dict):
            try:
                o.take_right(man.session.corrector.plan)                          # after the plan exists and before the first cycle; undone by abandon() if the hand-over is not entered
            except Exception:
                man.session.close(); raise                                        # the session made for this hand-over is ended with it
            o.session = man.session; o.teacher = man.session.state.get("_teacher") or getattr(man.session.corrector, "teacher", None)
            (out / "RECEIVE_PLAN.json").write_text(json.dumps(dict(waypoints=man.session.corrector.plan["waypoints"], planner_info=man.session.corrector.plan.get("planner_info"), pose_provenance="coarse preview estimate; the insert waits for the block observed in the hand"), indent=1, default=str))
        return man

    handoff = cx["full_episode"].FullEpisodeHandoff(prepare=prepare, preflight=preflight, task=None, perception=area, wait_next_cycle=o.wait_next_cycle, recorder=episode)
    handoff.task = object()                                                     # replaced by the task of this session in preflight (the source region is where the block was observed)
    handoff.owner, handoff.area, handoff.downstream, handoff.abandon = o, area, down, o.abandon; o.area = area
    return Scoped(handoff, o, scope), o


def _prepare(entry, snapshot, b, kw, o, down):
    """RECEIVE prepare_receive on this owner's references."""
    return entry.prepare_receive(snapshot, b, session_kwargs=kw, observation_source=o.observation, downstream=down, real_stow_profile=dict(kind="return to the start poses of this session", home=o.home),
                                 apply_dispatch=o.apply_dispatch, on_stop=o.on_stop, record_event=o.note)
