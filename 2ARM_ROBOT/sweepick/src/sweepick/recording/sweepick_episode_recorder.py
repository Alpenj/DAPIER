"""One continuous record of one session: the three views (top, left wrist, right wrist), both arms' measured joints, and
where the commands of each stage are, under ONE session id.

It records; it does not control. The executor (sweepick_move_a) stays the only writer and the only owner of an arm's port
while a stage runs. This module
  * owns the camera readers for the whole session (one SDK top reader, two wrist readers) and lends them to the looks and to
    the stage recorders, so that no camera is opened twice;
  * reads an arm read-only ONLY while no stage holds that arm (`lease`); during a stage the arm's measured joints, the
    controller's target (`commanded`) and the goal actually sent (`goal`) are the executor's own rows, which the manifest
    points to. Measured and commanded are never copied into each other;
  * saves in its own threads. A slow disk or a failed save is counted and reported, never waited for by a stage.
Raw frames keep the stamps they came with (SDK stamp, helper receipt, this process's receipt: three different clocks).
A frame that was not received is missing; none is repeated or made up."""
import json
import os
import threading
import time
from pathlib import Path

import numpy as np

SCHEMA = "sweepick.episode.v1"
SKILLS = ("START", "CARRY", "RECEIVE", "RELEASE", "PLACE", "STOW", "PUT_BACK", "OBSERVE")
ACT_VIEW = dict(
    note="how a learning view is derived from this raw record; the raw record is not changed by it and no such view is written here",
    image_keys=["observation.images.left_wrist", "observation.images.right_wrist", "observation.images.top"],
    top="the SDK colour image of the FrameSet (the same channel conversion the model input uses is applied when the view is made, not at recording)",
    state="q12 from the MEASURED ticks of both arms through the mapping named in the manifest (left 5 rad + left gripper fraction + right 5 rad + right gripper fraction)",
    action="q12 from the controller's COMMAND of the same cycle (executor row 'commanded' for the arm, the Jaw's command for the gripper), NOT from the measured position. An arm no stage commanded has no action (None), not a hold made up afterwards",
    time="images are joined to a control row by their receive stamp: the newest frame not later than the row; its age is kept. Camera rate and control rate differ",
    relative="anchor-relative action views are computed from this absolute view and kept separate")


class Lent:
    """A resident reader lent to a stage recorder: closing the loan does not close the camera."""
    def __init__(self, reader):
        self.reader, self.name = reader, reader.name

    def get(self):
        return self.reader.get()

    def close(self):
        pass


class Episode:
    def __init__(self, out, *, wrists=None, top=None, open_arm=None, session_id=None, source_kind="REAL", mode="BASELINE_PICK_AND_PUT_BACK",
                 arm_period_s=0.1, top_depth_every=5, clock=time.time, save_image=None, identifiers=None):
        """wrists: {name: reader} with get() -> (latest, previous), latest = dict(n, recv_monotonic_s, frame).
        top: reader with get() -> dict(n, bgr, zd, sdk_color_stamp_s, sdk_depth_stamp_s, helper_receipt_monotonic_s, recv_monotonic_s) or None.
        open_arm(side) -> a read-only bus with sync_read(name, normalize=False) and disconnect()."""
        self.out = Path(out); self.dir = self.out / "episode"; self.dir.mkdir(parents=True, exist_ok=True)
        self.id = session_id or f"sweepick_{time.strftime('%y%m%d_%H%M%S')}_{os.getpid()}"
        self.wrists, self.top, self.open_arm, self.clock, self.kind, self.mode = dict(wrists or {}), top, open_arm, clock, source_kind, mode
        self.arm_period_s, self.top_depth_every, self.identifiers = arm_period_s, top_depth_every, identifiers or {}
        self.offset = time.time() - time.monotonic()                              # readers stamp with the monotonic clock; rows of the executor are wall time
        self.stop, self.lock = threading.Event(), threading.Lock()
        self.phase = dict(step="start", skill="OBSERVE", owner=None)
        self.counts, self.errors, self.gaps, self.stages, self.closed = {}, [], {}, [], None; self.event_errors = 0
        self.want = {s: threading.Event() for s in ("left", "right")}; self.free = {s: threading.Event() for s in ("left", "right")}; self.held = {s: 0 for s in ("left", "right")}
        for e in self.free.values():
            e.set()
        self.files = {n: open(self.dir / f"{n}.jsonl", "a", buffering=1) for n in ("events", "joints", "frames")}
        self._save = save_image or self._imwrite
        self.event("episode_start", session_id=self.id, mode=mode, source_kind=source_kind, pid=os.getpid())
        self.threads = [threading.Thread(target=self._wrist, args=(n, r), daemon=True) for n, r in self.wrists.items()]
        if top is not None:
            self.threads.append(threading.Thread(target=self._top, daemon=True))
        if open_arm is not None:
            self.threads.append(threading.Thread(target=self._arms, daemon=True))
        for t in self.threads:
            t.start()

    # -- what the session tells the record ------------------------------------------------------------------------
    def event(self, kind, **k):
        """Called on the control path. A record that cannot be written is counted as the record's defect (data quality);
        it is never raised into the caller, whose motion has its own outcome and its own way back."""
        try:
            with self.lock:
                self.files["events"].write(json.dumps(dict(t_host_s=self.clock(), kind=kind, **k), default=str) + "\n")
        except Exception as e:
            self.event_errors += 1; self.errors.append(dict(t_host_s=self.clock(), what=f"event '{kind}' not recorded: {type(e).__name__}: {e}"))

    def mark(self, step, skill=None, owner=None, **k):
        """The session's own boundary: which step runs now, which skill it belongs to, who owns the commands."""
        self.phase = dict(step=step, skill=skill or self.phase["skill"], owner=owner)
        self.event("phase", **self.phase, **k)

    def stage(self, name, folder, side, result):
        """Where the executor's own rows of a stage are: measured, controller target ('commanded'), goal sent, acknowledgement."""
        self.stages.append(dict(stage=name, folder=str(folder), side=side, result=result, t_host_s=self.clock())); self.event("stage_record", stage=name, folder=str(folder), side=side, result=result)

    def lend(self, name):
        return Lent(self.wrists[name])

    def lease(self, *sides):
        """`with episode.lease('left'):` — the recorder lets go of that arm's port before a stage (or a look's own read) uses it."""
        ep = self

        class _Lease:                                                           # counted: a look's own read inside a stage's lease does not end the stage's lease
            def __enter__(self_):
                with ep.lock:
                    for s in sides:
                        ep.held[s] += 1; ep.want[s].set()
                for s in sides:
                    if not ep.free[s].wait(timeout=3.0):
                        ep.errors.append(dict(t_host_s=ep.clock(), what=f"the recorder did not let go of the {s} arm within 3 s")); self_.__exit__(); raise RuntimeError(f"the recorder still holds the {s} arm's port")
                return self_

            def __exit__(self_, *a):
                with ep.lock:
                    for s in sides:
                        ep.held[s] -= 1
                        if ep.held[s] <= 0:
                            ep.held[s] = 0; ep.want[s].clear()
        return _Lease()

    # -- the recorder's own threads ---------------------------------------------------------------------------------
    def _imwrite(self, path, image):
        import cv2
        return bool(cv2.imwrite(str(path), image))

    def _count(self, name, t):
        last = self.counts.get(name, (0, None)); self.counts[name] = (last[0] + 1, t)
        if last[1] is not None:
            g = self.gaps.setdefault(name, dict(max_s=0.0, sum_s=0.0)); g["max_s"] = max(g["max_s"], t - last[1]); g["sum_s"] += t - last[1]

    def _frame(self, row):
        with self.lock:
            self.files["frames"].write(json.dumps(dict(row, **self.phase), default=str) + "\n")

    def _wrist(self, name, reader):
        folder = self.dir / name; folder.mkdir(exist_ok=True); last = 0
        while not self.stop.is_set():
            try:
                latest, _ = reader.get()
                if latest is not None and latest["n"] != last:
                    missed = latest["n"] - last - 1 if last else 0; last = latest["n"]; f = f"{name}_{last:06d}.jpg"; ok = self._save(folder / f, latest["frame"])
                    self._count(name, latest["recv_monotonic_s"])
                    self._frame(dict(camera=name, file=f"{name}/{f}" if ok else None, n=last, recv_monotonic_s=latest["recv_monotonic_s"], recv_host_s=latest["recv_monotonic_s"] + self.offset, not_saved_before_this=missed))
            except Exception as e:
                self.errors.append(dict(t_host_s=self.clock(), what=f"{name}: {type(e).__name__}: {e}"))
            time.sleep(0.004)

    def _top(self):
        folder = self.dir / "top"; folder.mkdir(exist_ok=True); last = 0
        while not self.stop.is_set():
            try:
                fs = self.top.get()
                if fs is not None and fs["n"] != last:
                    missed = fs["n"] - last - 1 if last else 0; last = fs["n"]; row = dict(camera="top", n=last, not_saved_before_this=missed, **{k: fs[k] for k in ("sdk_color_stamp_s", "sdk_depth_stamp_s", "helper_receipt_monotonic_s", "recv_monotonic_s") if k in fs})
                    row["recv_host_s"] = fs["recv_monotonic_s"] + self.offset
                    if fs.get("bgr") is not None:
                        row["file"] = f"top/top_{last:06d}.jpg" if self._save(folder / f"top_{last:06d}.jpg", fs["bgr"]) else None
                    if fs.get("zd") is not None and last % self.top_depth_every == 0:      # every FrameSet's image is kept; the depth of every n-th one (stated in the manifest)
                        row["depth_file"] = f"top/depth_{last:06d}.png" if self._save(folder / f"depth_{last:06d}.png", np.asarray(fs["zd"], np.uint16)) else None
                    self._count("top", fs["recv_monotonic_s"]); self._frame(row)
            except Exception as e:
                self.errors.append(dict(t_host_s=self.clock(), what=f"top: {type(e).__name__}: {e}"))
            time.sleep(0.004)

    def _arms(self):
        bus = {}
        def drop(s):
            b = bus.pop(s, None)
            if b is not None:
                try:
                    b.disconnect()
                except Exception:
                    pass
            self.free[s].set()
        try:
            while not self.stop.is_set():
                for s in ("left", "right"):
                    if self.want[s].is_set():
                        drop(s); continue
                    try:
                        if s not in bus:
                            self.free[s].clear()
                            if self.want[s].is_set():                              # a lease came in between: let it have the port
                                self.free[s].set(); continue
                            bus[s] = self.open_arm(s)
                        t0 = self.clock(); b = bus[s]
                        row = dict(t_host_s=t0, side=s, ticks={n: float(v) for n, v in b.sync_read("Present_Position", normalize=False).items()}, torque={n: int(v) for n, v in b.sync_read("Torque_Enable", normalize=False).items()},
                                   read_s=self.clock() - t0, source="recorder, read-only, no stage owns this arm now", kind="MEASURED", **self.phase)
                        with self.lock:
                            self.files["joints"].write(json.dumps(row) + "\n")
                        self._count(f"joints_{s}", t0)
                    except Exception as e:
                        self.errors.append(dict(t_host_s=self.clock(), what=f"{s} arm read: {type(e).__name__}: {e}")); drop(s)
                self.stop.wait(self.arm_period_s)
        finally:
            for s in list(bus):
                drop(s)

    # -- the end ----------------------------------------------------------------------------------------------------
    def close(self, *, task_outcome="UNKNOWN", stage_outcomes=None, human_intervention="none recorded by the tool; a person's report overrides this", stopped_by=None, notes=None):
        """Ends the record. task outcome, per-stage outcomes, human intervention and data quality are separate fields;
        an automatic check that could not decide stays UNKNOWN."""
        if self.closed is not None:
            return self.closed
        self.event("episode_end", task_outcome=task_outcome, stopped_by=stopped_by); self.stop.set()
        for t in self.threads:
            t.join(timeout=5)
        for f in self.files.values():
            try:
                f.close()
            except Exception as e:
                self.errors.append(dict(t_host_s=self.clock(), what=f"closing a record file: {type(e).__name__}: {e}"))
        cams = {n: dict(frames=self.counts.get(n, (0, None))[0], longest_gap_s=self.gaps.get(n, {}).get("max_s"), mean_interval_s=(self.gaps[n]["sum_s"] / max(self.counts[n][0] - 1, 1)) if n in self.gaps else None) for n in list(self.wrists) + (["top"] if self.top is not None else [])}
        expected = ["top", "left_wrist", "right_wrist"]; missing_views = [n for n in expected if cams.get(n, {}).get("frames", 0) == 0]
        quality = dict(three_views_recorded=not missing_views, missing_views=missing_views, recorder_errors=len(self.errors), source_kind=self.kind,
                       events_not_recorded=self.event_errors,
                       training_eligibility=("NOT_ELIGIBLE_AS_IS: events of this session were lost; the control traces of the stages are separate files" if self.event_errors else "NOT_DECIDED") if self.kind == "REAL" else "NOT_ELIGIBLE (synthetic readers: a wiring check, not a real episode)",
                       note="eligibility is decided when a learning view is made; a failed or stopped session is still an episode, and it is not a success demonstration")
        self.closed = dict(schema=SCHEMA, session_id=self.id, mode=self.mode, source_kind=self.kind, task_outcome=task_outcome, stage_outcomes=stage_outcomes or [], human_intervention=human_intervention, stopped_by=stopped_by, data_quality=quality,
                           cameras=cams, joints={s: self.counts.get(f"joints_{s}", (0, None))[0] for s in ("left", "right")}, top_depth_saved_every_n_framesets=self.top_depth_every,
                           stages=self.stages, files=dict(events="episode/events.jsonl", frames="episode/frames.jsonl", joints="episode/joints.jsonl", executor_rows="<stage folder>/run*/move.json rows: t, present (MEASURED), commanded (controller target), goal (sent), status"),
                           clocks=dict(recv_monotonic_s="this process's monotonic clock at receipt", recv_host_s="the same plus a wall-clock offset read once at start", sdk_stamps="the camera SDK's own stamps", t_host_s="wall time (executor rows use the same clock)"),
                           errors=self.errors[:200], identifiers=self.identifiers, act_view=ACT_VIEW, notes=notes or [])
        try:
            (self.dir / "EPISODE.json").write_text(json.dumps(self.closed, indent=1, default=str))
        except Exception as e:
            self.closed["manifest_not_written"] = f"{type(e).__name__}: {e}"
        return self.closed


def alignment(folder, rows_of=None):
    """For every executor row of every recorded stage: the newest frame of each camera not later than the row, and its age.
    A row with no earlier frame of a camera has None there. Reads the record; changes nothing."""
    folder = Path(folder); ep = json.loads((folder / "episode/EPISODE.json").read_text()); frames = {}
    for line in (folder / "episode/frames.jsonl").read_text().splitlines():
        r = json.loads(line)
        if r.get("file"):
            frames.setdefault(r["camera"], []).append((r["recv_host_s"], r["file"]))
    for v in frames.values():
        v.sort()
    out = dict(session_id=ep["session_id"], stages=[])
    for st in ep["stages"]:
        rows = (rows_of or _rows)(Path(st["folder"])); ages = {c: [] for c in frames}; none = {c: 0 for c in frames}
        for r in rows:
            for c, fr in frames.items():
                i = int(np.searchsorted([f[0] for f in fr], r["t"], side="right")) - 1
                if i < 0:
                    none[c] += 1
                else:
                    ages[c].append(r["t"] - fr[i][0])
        out["stages"].append(dict(stage=st["stage"], rows=len(rows), cameras={c: dict(rows_without_an_earlier_frame=none[c], oldest_frame_used_s=max(a) if a else None, median_age_s=float(np.median(a)) if a else None) for c, a in ages.items()},
                                  has_command=bool(rows) and all("commanded" in r or "goal" in r for r in rows)))
    return out


def _rows(stage_folder):
    rows = []
    for f in sorted(Path(stage_folder).glob("run*/move.json")):
        rows += json.loads(f.read_text()).get("rows") or []
    return rows


def real(out, *, mode, right_wrist=True, top_framesets=30 * 60 * 20, identifiers=None):
    """The episode recorder on the real devices: one SDK top reader for the whole session and the two wrist readers.
    Opens cameras and read-only arm ports; sends nothing to a motor."""
    from sweepick.integration.sweepick_observation_session import SdkTop, WristCam, open_readonly_bus
    out = Path(out); opened = []
    try:
        wr = dict(left_wrist=WristCam("left_wrist", "/dev/dapier/left_wrist_rgb")); opened.append(wr["left_wrist"])
        if right_wrist:
            wr["right_wrist"] = WristCam("right_wrist", "/dev/dapier/right_wrist_rgb"); opened.append(wr["right_wrist"])
        top = SdkTop(out / f"episode/sdk_{int(time.time())}", top_framesets); opened.append(top)
    except Exception:
        for o in opened:
            try:
                o.close()
            except Exception:
                pass
        raise
    ep = Episode(out, wrists=wr, top=top, open_arm=open_readonly_bus, mode=mode, identifiers=identifiers); ep.owned = opened
    return ep


def release(ep, **k):
    """Close the record, then the readers this module opened."""
    doc = ep.close(**k); doc["readers_closed"] = {}
    for o in getattr(ep, "owned", []):
        try:
            doc["readers_closed"][getattr(o, "name", type(o).__name__)] = o.close() or "closed"
        except Exception as e:
            doc["readers_closed"][getattr(o, "name", type(o).__name__)] = f"{type(e).__name__}: {e}"
    (ep.dir / "EPISODE.json").write_text(json.dumps(doc, indent=1, default=str))
    return doc


def chain_record(folder):
    """One episode read as a chain, without changing it: for the START part every stage's executor rows (measured, controller
    target, goal sent); for the part after the lift every cycle's observation -> proposal -> what the executor ran -> the
    acknowledgement read back, joined by the cycle's sequence number and order in the event log. A link that is not there is
    reported as missing; nothing is filled in. This says what the record contains, not that it is enough to learn from."""
    folder = Path(folder); ep = json.loads((folder / "episode/EPISODE.json").read_text()); ev = [json.loads(l) for l in (folder / "episode/events.jsonl").read_text().splitlines()]
    stages = []
    for st in ep["stages"]:
        rows = _rows(Path(st["folder"])); stages.append(dict(stage=st["stage"], side=st["side"], result=st["result"], rows=len(rows), measured=bool(rows) and all("present" in r for r in rows), controller_target=bool(rows) and all("commanded" in r for r in rows), goal_sent=bool(rows) and all("goal" in r for r in rows),
                                                             part="AFTER_LIFT" if st["stage"].startswith("handoff_") else "START"))
    cycles, cur = [], None
    for e in ev:
        k = e["kind"]
        if k == "receive_observation":
            ob = e.get("observation") or {}; cur = dict(seq=ob.get("seq"), stage=e.get("stage"), t_host_s=e["t_host_s"], observation=True, measured_q12=ob.get("q12") is not None, proposal=None, dispatch_status=None, executor=None, runs=None, ack=bool(ob.get("owner_ack"))); cycles.append(cur)
        elif cur is not None and k == "receive_dispatch":
            cur.update(proposal=e.get("proposal_q12") is not None, raw_target=e.get("raw_target") is not None, retimed_goal=e.get("retimed_goal") is not None, dispatch_status=e.get("dispatch_status"), owner=e.get("owner"))
        elif cur is not None and k == "receive_writer_receipt":
            r = e.get("receipt") or {}; cur.update(executor=r.get("status"), runs=r.get("runs") or ({"waypoint": r.get("waypoint")} if r.get("waypoint") else None))
    sent = [c for c in cycles if c["executor"] == "EXECUTOR_RAN"]; after = [s for s in stages if s["part"] == "AFTER_LIFT"]
    missing = []
    if any(c["proposal"] is None and c["executor"] is not None for c in cycles):
        missing.append("a writer receipt without its proposal")
    if len(sent) and not after:
        missing.append("executor runs after the lift have no stage record (rows) in this episode")
    if any(not s["rows"] for s in after if s["result"] not in (None,)) and after:
        missing.append("a stage after the lift without executor rows: " + ", ".join(s["stage"] for s in after if not s["rows"]))
    return dict(session_id=ep["session_id"], start_stages=[s for s in stages if s["part"] == "START"], after_lift_stages=after, cycles=len(cycles), cycles_with_proposal=sum(1 for c in cycles if c["proposal"]), cycles_that_sent=len(sent),
                acknowledgements=sum(1 for c in cycles if c["ack"]), events_not_recorded=ep["data_quality"].get("events_not_recorded"), missing_links=missing, cameras=ep["cameras"],
                note="proposal = the state machine's command of that cycle; what was SENT is in the stage's executor rows ('goal'), which follow the executor's own checked trajectory to the proposal's target; ack = Goal_Position read back in the next observation")
