"""LEFT_CONTACT01: one supervised contact observation with the left gripper (motor 6) only.

One session: slow close with the existing executor -> whatever the executor decides (its result / exit code are kept
verbatim) -> a recorded dwell -> the planned, controlled opening back to this session's start tick -> torque release.
The opening is a step of THIS plan; it is not a "return after a successful out". It is run only after the executor ended
the close holding the gripper without a fault. After a Status fault, a bus loss, an operator stop or a refusal nothing
further is commanded.

This file commands nothing itself: every read and write goes through sweepick_move_a.run (the only owner of the motor port).
It adds the camera recording (existing WristCam reader) and the observation record. A stop of the executor is never
turned into "contact" or "grasp" here: contact_candidate stays unknown until independent evidence (a reviewed frame or
the person on site) is added with `annotate`. HOLDING after no_progress is the motor holding a position, not a grip force.

usage:  python -m sweepick.control.sweepick_gripper_contact_inspection plan OUT --close-target TICK
        python -m sweepick.control.sweepick_gripper_contact_inspection run  OUT --close-target TICK [--dwell-s S] --execute
        python -m sweepick.control.sweepick_gripper_contact_inspection annotate OUT --object-visible true|false|unknown --contact true|false|unknown --source FRAME_REVIEWED|USER_OBSERVED
                                      [--frames a.jpg,b.jpg] [--before TEXT] [--at-stop TEXT] [--after-relax TEXT] [--after-open TEXT]
"""
from sweepick.integration.sweepick_resource_paths import source_path
import argparse, hashlib, json, sys, threading, time
from pathlib import Path
import numpy as np
from sweepick.control import sweepick_trajectory_executor as mv

HERE = Path(__file__).resolve().parent
SIDE, JOINT = "left", "gripper"
OPEN_AFTER = ("HOLDING", "NO_PROGRESS_HOLDING", "TARGET_NOT_REACHED_HOLDING")      # the executor ended the close holding the gripper, no fault
BASELINE = Path.home() / "sweepick_261007_commission/c_run_1008/out/move.json"       # no-load close of the same joint, same profile (2026-10-08)
TRI = ("true", "false", "unknown")


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


class FrameRecorder:
    """Saves every new frame of an existing reader with the time it was received. Receive time is host time; it is not
    the exposure time, and it is a different clock reading from the feedback read times (both are host wall time)."""
    def __init__(self, cam, folder, phase):
        import cv2
        self.cv2, self.cam, self.folder, self.phase, self.stop, self.index, self.last = cv2, cam, Path(folder), phase, threading.Event(), [], 0
        self.folder.mkdir(parents=True, exist_ok=True)
        self.offset = time.time() - time.monotonic()                              # reader stamps are monotonic; rows of the executor are time.time()
        self.thread = threading.Thread(target=self._run, daemon=True); self.thread.start()

    def _run(self):
        while not self.stop.is_set():
            latest, _ = self.cam.get()
            if latest is not None and latest["n"] != self.last:
                self.last = latest["n"]; name = f"{self.cam.name}_{latest['n']:06d}.jpg"
                self.cv2.imwrite(str(self.folder / name), latest["frame"])
                self.index.append(dict(file=name, n=latest["n"], recv_host_s=latest["recv_monotonic_s"] + self.offset, phase=self.phase[0]))
            time.sleep(0.005)

    def close(self):
        self.stop.set(); self.thread.join(timeout=3)
        return self.index


def open_cameras():
    from sweepick.integration.sweepick_observation_session import WristCam
    return [WristCam("left_wrist", "/dev/dapier/left_wrist_rgb")]


def feedback_summary(log):
    """Numbers of one executor log, no judgement: where the command and the gripper were, and the raw channels."""
    rows = [r for r in log.get("rows", []) if JOINT in r.get("commanded", {})]
    if not rows:
        return None
    g = lambda k: np.array([r[k][JOINT] for r in rows], float)
    cmd, pos = np.array([r["commanded"][JOINT] for r in rows], float), g("present")
    d = -1.0 if log["trajectory"]["target"][JOINT] < log["trajectory"]["start"][JOINT] else 1.0
    lead = d * (np.round(cmd) - pos)
    moved = np.flatnonzero(d * np.diff(pos) > 0)
    last_motion = int(moved[-1] + 1) if len(moved) else 0
    t0 = rows[0]["t"]
    return dict(rows=len(rows), start_tick=float(pos[0]), deepest_commanded_tick=float(cmd.min() if d < 0 else cmd.max()), actual_stop_tick=float(pos[-1]),
                last_position_change=dict(row=last_motion, t_host_s=rows[last_motion]["t"], t_from_start_s=rows[last_motion]["t"] - t0, tick=float(pos[last_motion])),
                first_wait_row=next((dict(row=i, t_host_s=r["t"], t_from_start_s=r["t"] - t0) for i, r in enumerate(rows) if r.get("command_waiting")), None),
                lead_ticks=dict(max=float(lead.max()), at_end=float(lead[-1])),
                trace=[dict(t=r["t"] - t0, commanded=float(c), present=float(p), lead=float(l), velocity=r["velocity"][JOINT], load=r["load"][JOINT], current=r["current"][JOINT], moving=r["moving"][JOINT], status=r["status"][JOINT],
                            torque=r["torque"][JOINT], progress=(r.get("progress") or {}).get(JOINT), waiting=r.get("command_waiting"), phase=r.get("phase", "command")) for r, c, p, l in zip(rows, cmd, pos, lead)])


def compare_with_baseline(summary, baseline_log):
    """Same joint, direction, profile: raw load / current / lead of this close next to the no-load close, per 50-tick
    position bin. A table for a person to read; no number here decides contact."""
    base = feedback_summary(baseline_log)
    if not summary or not base:
        return None
    def bins(tr):
        out = {}
        for r in tr:
            if r["phase"] != "command": continue
            out.setdefault(int(r["present"] // 50 * 50), []).append(r)
        return out
    a, b = bins(summary["trace"]), bins(base["trace"])
    med = lambda rows, k: float(np.median([r[k] for r in rows]))
    return dict(baseline=str(BASELINE), note="raw register values; Load is about +40 while closing and +36 at rest with EMPTY jaws, so a non-zero load is not contact",
                per_50_tick_bin=[dict(tick_from=k, n=len(a[k]), load=med(a[k], "load"), current=med(a[k], "current"), lead=med(a[k], "lead"), velocity=med(a[k], "velocity"),
                                      baseline=(dict(n=len(b[k]), load=med(b[k], "load"), current=med(b[k], "current"), lead=med(b[k], "lead"), velocity=med(b[k], "velocity")) if k in b else None)) for k in sorted(a, reverse=True)])


def observation(close, opening, frames):
    """The observation record. Everything a camera or a person has to say starts as unknown / NOT_REVIEWED."""
    s = feedback_summary(close) if close else None
    o = (opening or {}).get("outcome") or {}
    unknown = [e for l in (close, opening) if l for e in l.get("events", []) if isinstance(e, dict) and "UNKNOWN" in json.dumps(e)]
    return dict(commanded_close=None if not s else s["deepest_commanded_tick"], actual_stop=None if not s else s["actual_stop_tick"],
                command_actual_residual=None if not s else s["actual_stop_tick"] - close["trajectory"]["target"][JOINT],
                controller_stop_reason=((close or {}).get("stopped_by") or (close or {}).get("refused") or {}).get("gate"),
                object_visible_in_jaw_region="unknown", independent_visual_contact_evidence=dict(status="NOT_REVIEWED", source=None, frames=[], note=None),
                contact_candidate="unknown", contact_candidate_basis="no independent evidence recorded yet; a stop of the executor alone does not set this",
                grasp_confirmed="NOT_TESTED", support_confirmed="NOT_TESTED",
                object_state=dict(before_close="unknown", at_stop="unknown", after_relaxation="unknown", after_opening="unknown"),
                holding_meaning="after no_progress the goal is set to the measured position: the motor holds that position. It is not a held grip force",
                opening_completed=None if opening is None else bool(o.get("return_confirmed")), final_torque_readback=None if opening is None else o.get("torque_released"),
                write_ack_unknown=len(unknown), frames_recorded=len(frames), frames_reviewed=0)


def session(out, close_target, *, dwell_s=5.0, execute=False, run=mv.run, cameras=open_cameras, sleep=time.sleep, run_kw=None):
    out = Path(out); out.mkdir(parents=True, exist_ok=True); run_kw = dict(run_kw or {})
    stop_file = Path(str(out) + ".STOP")
    doc = dict(schema="tjj.contact01.v1", name="LEFT_CONTACT01", side=SIDE, joint=JOINT, motor_id=6, execute=bool(execute), close_target_tick=close_target, dwell_s=dwell_s,
               sources={f: sha(source_path(f)) for f in ("sweepick_contact01.py", "sweepick_move_a.py", "sweepick_progress.py", "sweepick_motion_profile.json", "sweepick_progress_profile.json")},
               scope="left gripper only; the other five left joints and the right arm are neither powered nor commanded; the object stays on the bench and is not lifted",
               phases=[], stop_file=str(stop_file))
    if not execute:
        code, plan = run("plan", SIDE, out / "plan", target={JOINT: close_target}, torque_joints=[JOINT], **run_kw)
        doc.update(result="PLANNED" if code == 0 else plan["result"], controller=dict(plan=dict(result=plan["result"], exit_code=code, refused=plan.get("refused"), start=plan.get("start"), trajectory={k: v for k, v in (plan.get("trajectory") or {}).items() if k != "goals"})))
        (out / "CONTACT01_PLAN.json").write_text(json.dumps(doc, indent=1, default=str)); return code, doc
    phase, cams, recs = ["before_close"], [], []
    try:
        cams = cameras(); recs = [FrameRecorder(c, out / "frames", phase) for c in cams]
        doc["cameras"] = [getattr(c, "info", {}) for c in cams]
    except Exception as e:                                                        # no camera is not a reason to move blind: nothing is commanded
        doc.update(result="NOT_STARTED", why=f"camera could not be opened: {type(e).__name__}: {e}")
        for c in cams:
            c.close()
        (out / "CONTACT01_RESULT.json").write_text(json.dumps(doc, indent=1, default=str)); return 2, doc
    close = opening = None
    try:
        sleep(1.0); phase[0] = "close"
        code_c, close = run("out", SIDE, out / "close", target={JOINT: close_target}, execute=True, torque_joints=[JOINT], stop_requested=stop_file.exists, **run_kw)
        doc["phases"].append(dict(phase="close", result=close["result"], exit_code=code_c))
        start = (close.get("start") or {}).get("ticks", {}).get(JOINT)
        if close["result"] in OPEN_AFTER and start is not None:
            phase[0] = "dwell"; sleep(dwell_s)                                    # the gripper holds its position; the cameras keep recording
            doc["phases"].append(dict(phase="dwell", seconds=dwell_s, feedback="none: the executor has closed the port; the first rows of the opening show the state after the dwell"))
            phase[0] = "open"
            code_o, opening = run("return", SIDE, out / "open", rest={JOINT: start}, execute=True, torque_joints=[JOINT], stop_requested=stop_file.exists, **run_kw)
            doc["phases"].append(dict(phase="planned_open", to_tick=start, result=opening["result"], exit_code=code_o))
            phase[0] = "after_open"; sleep(1.0)
        else:
            doc["phases"].append(dict(phase="planned_open", result="NOT_RUN", why=f"the close ended as {close['result']}: nothing further is commanded; the executor's own handling of that state stands"))
    finally:
        frames = [f for r in recs for f in r.close()]
        doc["camera_close"] = [c.close() for c in cams]
    (out / "frames/INDEX.json").write_text(json.dumps(frames, indent=1)) if frames else None
    summ = feedback_summary(close)
    doc.update(controller=dict(close={k: close.get(k) for k in ("result", "exit_code", "outcome", "stopped_by", "refused", "writes", "command_waits", "reached", "goal_seed", "torque_scope", "needs_a_person", "recovery", "end")},
                               open=None if opening is None else {k: opening.get(k) for k in ("result", "exit_code", "outcome", "stopped_by", "refused", "writes", "command_waits", "reached", "release", "needs_a_person", "end")}),
               observation=observation(close, opening, frames),
               feedback=None if summ is None else {k: v for k, v in summ.items() if k != "trace"},
               feedback_vs_no_load=compare_with_baseline(summ, json.loads(BASELINE.read_text())) if BASELINE.exists() else None,
               frame_times=dict(clock="host wall time at receipt of the frame (not exposure)", first=frames[0]["recv_host_s"] if frames else None, last=frames[-1]["recv_host_s"] if frames else None,
                                per_phase={p: sum(1 for f in frames if f["phase"] == p) for p in ("before_close", "close", "dwell", "open", "after_open")}))
    final = (opening or close)["result"]
    doc["result"] = dict(close=close["result"], open=None if opening is None else opening["result"], session=final)
    if summ:
        (out / "close_trace.json").write_text(json.dumps(summ["trace"]))
    (out / "CONTACT01_RESULT.json").write_text(json.dumps(doc, indent=1, default=str))
    return (opening or close)["exit_code"] if opening is not None else close["exit_code"], doc


def annotate(out, *, object_visible, contact, source, frames=(), before=None, at_stop=None, after_relax=None, after_open=None, note=None):
    """Adds what a reviewed frame or the person on site says. This is the only place contact_candidate can leave 'unknown'."""
    assert object_visible in TRI and contact in TRI and source in ("FRAME_REVIEWED", "USER_OBSERVED")
    p = Path(out) / "CONTACT01_RESULT.json"; doc = json.loads(p.read_text()); o = doc["observation"]
    o["object_visible_in_jaw_region"] = object_visible
    o["independent_visual_contact_evidence"] = dict(status="RECORDED" if contact != "unknown" else "INCONCLUSIVE", source=source, frames=list(frames), note=note, says_contact=contact)
    o["contact_candidate"] = contact if object_visible == "true" or contact != "true" else "unknown"      # contact needs the object to have been seen between the jaws
    o["contact_candidate_basis"] = f"{source}: object visible in the jaw region = {object_visible}, contact = {contact}; controller stop reason = {o['controller_stop_reason']} (kept as recorded, not used as proof)"
    for k, v in (("before_close", before), ("at_stop", at_stop), ("after_relaxation", after_relax), ("after_opening", after_open)):
        if v:
            o["object_state"][k] = dict(text=v, source=source)
    o["frames_reviewed"] = len(frames)
    p.write_text(json.dumps(doc, indent=1, default=str)); return doc


def main(argv=None):
    ap = argparse.ArgumentParser(); ap.add_argument("mode", choices=("plan", "run", "annotate")); ap.add_argument("out")
    ap.add_argument("--close-target", type=float); ap.add_argument("--dwell-s", type=float, default=5.0); ap.add_argument("--execute", action="store_true")
    ap.add_argument("--object-visible", choices=TRI, default="unknown"); ap.add_argument("--contact", choices=TRI, default="unknown"); ap.add_argument("--source", choices=("FRAME_REVIEWED", "USER_OBSERVED"))
    ap.add_argument("--frames", default=""); ap.add_argument("--before"); ap.add_argument("--at-stop"); ap.add_argument("--after-relax"); ap.add_argument("--after-open"); ap.add_argument("--note")
    a = ap.parse_args(argv)
    if a.mode == "annotate":
        d = annotate(a.out, object_visible=a.object_visible, contact=a.contact, source=a.source, frames=[f for f in a.frames.split(",") if f], before=a.before, at_stop=a.at_stop, after_relax=a.after_relax, after_open=a.after_open, note=a.note)
        print(json.dumps(d["observation"], indent=1)); return 0
    code, doc = session(a.out, a.close_target, dwell_s=a.dwell_s, execute=(a.mode == "run" and a.execute))
    print(json.dumps({k: doc.get(k) for k in ("name", "execute", "result", "phases", "observation", "feedback", "frame_times", "why")}, indent=1, default=str)); return code


if __name__ == "__main__":
    sys.exit(main())
