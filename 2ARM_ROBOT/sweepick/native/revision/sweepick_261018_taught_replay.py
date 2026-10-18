"""sweepick 2026-10-18 - TAUGHT REPLAY of one grasp on the arm that was hand-guided (torque off) to three poses over the block:
pregrasp (jaw tips 2-3 cm over the top), just above the top, and the grasp (jaws around the block). Replays them through the
EXISTING executor (sweepick_move_a.run / grip), each move planned first and checked for collisions in a scene with the block
where the head camera sees it now (only <side> pad - block contact allowed; carried after the close):

  start -> pregrasp -> above -> grasp -> close (Jaw) -> lift (= pregrasp pose) -> lower (= grasp pose) -> open -> pregrasp -> start, release

Purpose: the first real check of the right close / lift / lower chain, independent of the model's forward kinematics (cm off
on this setup). It is NOT a general pick: the poses belong to this block position. A wrist image is saved at every stop.
Any controller result other than holding ends the run where it is (nothing further is commanded); STOP file: <out>.STOP

usage: sweepick_261018_taught_replay.py OUT --refs DIR [--side right] [--execute]
"""
import argparse, json, sys, time
from pathlib import Path

import numpy as np

import sweepick_pick01 as pk
import sweepick_move_a as mv

ARM, ALL = pk.ARM, pk.ALL


def main(argv=None):
    ap = argparse.ArgumentParser(); ap.add_argument("out"); ap.add_argument("--refs", required=True); ap.add_argument("--side", default="right"); ap.add_argument("--execute", action="store_true"); ap.add_argument("--carry-hold-s", type=float, default=5.0, help="seconds held in the carry pose (the base would drive here), gripper watched"); ap.add_argument("--fast-v", type=float, default=0.6, help="rad/s for the moves away from the block"); ap.add_argument("--fast-a", type=float, default=3.0); ap.add_argument("--lift-to", default="above", choices=("above", "pregrasp"), help="lift target: the taught just-above pose (about 2 cm; 2026-10-18 right: the full pregrasp lift stalled 65 ticks short on shoulder_lift under the block load) or the pregrasp pose"); ap.add_argument("--record", action="store_true", help="3-view + arm record (sweepick_episode) for the whole replay")
    a = ap.parse_args(argv); out = Path(a.out).resolve(); out.mkdir(parents=True, exist_ok=False); refs = Path(a.refs).resolve(); side = a.side
    pk.set_side(side); other = pk.other(side)
    pk.KEEP_TOP = True                                                          # one head reader for the whole replay
    log = dict(schema="sweepick.taught-replay.v1", side=side, execute=a.execute, refs=str(refs), steps=[], started=time.strftime("%Y-%m-%dT%H:%M:%S"))
    def save():
        (out / "REPLAY.json").write_text(json.dumps(log, indent=1, default=str))
    taught = {k: json.loads((refs / d / "ticks.json").read_text())[side]["ticks"] for k, d in (("pregrasp", "H6_pregrasp"), ("above", "H2_above_c"), ("grasp", "H0_grasp"))}
    if (refs / "H9_carry/ticks.json").exists():                                # the taught carry pose (block held, arm folded over the base)
        taught["carry"] = json.loads((refs / "H9_carry/ticks.json").read_text())[side]["ticks"]
    log["carry_pose"] = "carry" in taught
    sp = json.loads(pk.START_POSE.read_text())[side]; open_tick = float(sp["gripper_open_tick"])
    release_tick = float(pk.mapping()[side]["gripper"]["hi"]) - 5.0                  # release: as wide as the calibrated range allows (user 2026-10-18: open fully when putting down)
    mech_ = json.loads((pk.HERE / "sweepick_start_pose.json").read_text())[side].get("gripper_mechanical_open_tick")
    if mech_ is not None:                                                        # 2026-10-18 measured: the jaws stop at 3364 (left) / 2998 (right), below the servo limit; hi-5 pushed the left 62 ticks into the stop
        release_tick = min(release_tick, float(mech_) - 3.0)
    ep = None
    if a.record:
        import contextlib, hashlib, sweepick_episode
        ep = sweepick_episode.real(out / "episode", mode="BASELINE_PICK_AND_PUT_BACK", identifiers=dict(run="taught_replay", sources={f.name: hashlib.sha256(f.read_bytes()).hexdigest() for f in sorted(pk.HERE.glob("sweepick_*.py"))}))
        pk.EPISODE = ep
    if a.execute:                                                              # the arm starts from the start pose (gripper open) whatever a person left it at (user 2026-10-18)
        def leased_run(*args, **kw):
            with (ep.lease("left", "right") if ep is not None else __import__("contextlib").nullcontext()):
                return mv.run(*args, **kw)
        log["prepare"] = {}
        t_obs = time.time(); now0 = pk.read_ticks(); fs0 = pk.top_frame(str(out / "top")); sc0 = pk.analyse(fs0["bgr"], fs0["depth_mm"], fs0["K"], {s: now0[s]["ticks"] for s in ("left", "right")})
        log["start_observation_s"] = round(time.time() - t_obs, 1); shared_scene = lambda folder, now_: sc0      # ONE head observation for both prepares and the replay (start-up time, 2026-10-18)
        for s_ in (other, side):                                               # both arms: the other one first, so it is folded away before this one moves
            pr = pk.prepare(out / f"prepare_{s_}", s_, execute=True, run=leased_run, scene_of=shared_scene, stop_file=Path(str(out) + ".STOP"))
            log["prepare"][s_] = {k: pr.get(k) for k in ("result", "why", "target", "controller_result", "at_start_pose", "needs_a_person")}; save()
            if pr["result"] not in ("ALREADY_AT_START",) and not (pr["result"] == "RAN" and pr.get("at_start_pose")):
                log["result"] = f"NOT_RUN: prepare {s_} {pr['result']} {pr.get('why') or pr.get('needs_a_person')}"; save(); print(log["result"])
                if ep is not None:
                    sweepick_episode.release(ep, task_outcome="NOT_STARTED: prepare"); pk.EPISODE = None
                return 2
        pk.set_side(side)                                                      # prepare() sets the session arm; this replay's arm again
    now = pk.read_ticks()
    if any(v for s in now.values() for v in s["torque"].values()):
        log["result"] = "NOT_RUN: torque is on somewhere"; save(); print(log["result"]); return 2
    home = dict(now[side]["ticks"]); other_ticks = now[other]["ticks"]; log["home_ticks"] = home
    if a.execute and "sc0" in locals():
        sc = sc0                                                               # the start observation (the block has not been touched since)
    else:
        fs = pk.top_frame(str(out / "top")); sc = pk.analyse(fs["bgr"], fs["depth_mm"], fs["K"], {s: now[s]["ticks"] for s in ("left", "right")})
    (out / "SCENE_start.json").write_text(json.dumps(sc, default=lambda o: o.tolist() if hasattr(o, "tolist") else str))
    o = sc.get("object")
    if not sc.get("ok") or o is None:
        log["result"] = f"NOT_RUN: no block in the head observation ({sc.get('why')})"; save(); print(log["result"]); return 2
    block = (np.array(o["centre_world"], float), float(o.get("yaw_rad") or 0.0)); log["block_seen"] = o["centre_world"]
    grasp_arm = {n: float(taught["grasp"][n]) for n in ARM}

    def check(doc, folder, tag, carried):
        folder.mkdir(parents=True, exist_ok=True)
        c = pk.object_check(doc, block, carried=carried, right_ticks=other_ticks, grasp_arm_ticks=dict(grasp_arm, gripper=float(doc["trajectory"]["start"].get("gripper", open_tick))) if carried else None)
        (folder / f"{tag}.CHECK.json").write_text(json.dumps(c, indent=1, default=str)); return c

    def other_read():
        from sweepick_local_loop import open_readonly_bus
        b = open_readonly_bus(other)
        try:
            return {n: float(v) for n, v in b.sync_read("Present_Position", normalize=False).items()}
        finally:
            b.disconnect()

    def wrist(tag):
        if ep is not None:                                                     # the recorder owns the wrist cameras: its frames are the record
            return
        try:
            import cv2
            cam = cv2.VideoCapture(f"/dev/dapier/{side}_wrist_rgb", cv2.CAP_V4L2); img = None
            for _ in range(6):
                ok_, f_ = cam.read()
                img = f_ if ok_ else img
            cam.release()
            if img is not None:
                cv2.imwrite(str(out / f"wrist_{tag}.jpg"), img)
        except Exception as e:
            log.setdefault("wrist_errors", []).append(f"{tag}: {type(e).__name__}: {e}")

    import sweepick_trajectory as tt, hashlib as _h
    base_prof = tt.load_profile()
    FAST = dict(base_prof, v=a.fast_v, a=a.fast_a); FAST["sha256"] = _h.sha256(json.dumps({k: FAST[k] for k in ("v", "a", "j", "dt")}, sort_keys=True).encode()).hexdigest()
    FAST_STEPS = ("pregrasp", "retreat", "home")                                 # away from the block (user 2026-10-18: not so conservative; STS3215 no-load about 4.4 rad/s at 6 V)
    log["profiles"] = dict(near=dict(v=base_prof["v"], a=base_prof["a"]), far=dict(v=FAST["v"], a=FAST["a"], steps=list(FAST_STEPS)))
    TRIMMED = ("pregrasp", "above", "lift", "carry", "back_over_place", "retreat", "home")             # as the session's TRIM: a loaded joint that settles short is trimmed onto its target (checked); never the final descent beside the block

    def move(name, mode, target, powered, carried=False):
        sdir = out / name
        kw = dict(level="model", torque_joints=powered, other_arm_ticks=other_ticks, profile=FAST if name in FAST_STEPS else base_prof)
        lease = lambda: ep.lease("left", "right") if ep is not None else __import__("contextlib").nullcontext()     # the recorder lets go of the arm ports for every executor read
        if mode == "return":
            kw["rest"] = target
            with lease():
                _, plan = mv.run("return", side, sdir / "plan", **kw)
        else:
            kw["target"] = target
            with lease():
                _, plan = mv.run("plan", side, sdir / "plan", **kw)
        step = dict(step=name, mode=mode, target=target)
        if plan["result"] != "PLANNED":
            step.update(result="NOT_RUN", why=f"planning read refused: {plan.get('refused')}"); log["steps"].append(step); save(); return False
        c = check(json.loads((sdir / "plan/TRAJECTORY.json").read_text()), sdir / "plan", "path", carried)
        step["collision"] = dict(verdict=c.get("nominal_verdict"), first_contact=c.get("first_nominal_contact"), allowed=c.get("allowed"))
        if c.get("nominal_verdict") != "CLEAR":
            step.update(result="NOT_RUN", why=f"collision: {c.get('first_nominal_contact')}"); log["steps"].append(step); save(); return False
        if not a.execute:
            step["result"] = "PLANNED"; log["steps"].append(step); save(); return True
        if ep is not None:
            ep.mark(name, skill="taught_replay", owner="sweepick_move_a")
        with (ep.lease("left", "right") if ep is not None else __import__("contextlib").nullcontext()):
            n_trim = [0]
            def trim_check(doc):
                n_trim[0] += 1; return check(doc, sdir / "run", f"trim{n_trim[0]}", carried)
            code, rl = mv.run(mode, side, sdir / "run", execute=True, cert=c, start_recheck=lambda doc: check(doc, sdir / "run", "start", carried),
                              settle_trim=ARM if name in TRIMMED else [], trim_check=trim_check,
                              stop_requested=Path(str(out) + ".STOP").exists, other_arm_read=other_read, **kw)
        step.update(result=rl["result"], writes=rl.get("writes"), reached=rl.get("reached"), stopped_by=rl.get("stopped_by"), refused=rl.get("refused"), needs_a_person=rl.get("needs_a_person"))
        log["steps"].append(step); save(); wrist(name)
        return rl["result"] in ("HOLDING", "TARGET_NOT_REACHED_HOLDING", "RETURNED_RELEASED")

    def grip():
        sdir = out / "close"; jd = json.loads(pk.JAW_PROFILES[side].read_text()); lo = float(pk.mapping()[side]["gripper"]["lo"])
        end_tick = lo + float(jd["end_stop_ticks_above_closed"]["value"])
        arm_now = {n: float(v) for n, v in pk.read_ticks()[side]["ticks"].items()} if a.execute else dict(grasp_arm, gripper=open_tick)
        def closing(actual, tag):
            c = check(pk.sweep_doc(actual, actual["gripper"], end_tick), sdir, tag, False); c.update(arm_ticks={n: actual[n] for n in ARM}, sweep=[actual["gripper"], end_tick]); return c
        c = closing(arm_now, "sweep"); step = dict(step="close", collision=dict(verdict=c.get("nominal_verdict"), first_contact=c.get("first_nominal_contact")))
        if c.get("nominal_verdict") != "CLEAR":
            step.update(result="NOT_RUN", why=f"closing sweep touches: {c.get('first_nominal_contact')}"); log["steps"].append(step); save(); return False
        if not a.execute:
            step["result"] = "PLANNED"; log["steps"].append(step); save(); return True
        if ep is not None:
            ep.mark("close", skill="taught_replay", owner="sweepick_grasp.Jaw through sweepick_move_a.grip")
        with (ep.lease("left", "right") if ep is not None else __import__("contextlib").nullcontext()):
            code, gl = mv.grip(side, sdir / "run", torque_joints=ALL, cert=c, level="model", execute=True, stop_requested=Path(str(out) + ".STOP").exists,
                               jaw_profile=pk.JAW_PROFILES[side], start_recheck=lambda actual: closing(actual, "sweep_recheck"))
        step.update(result=gl["result"], writes=gl.get("writes"), grip=gl.get("grip"), outcome=gl.get("outcome"), needs_a_person=gl.get("needs_a_person")); log["steps"].append(step); save(); wrist("close")
        return gl["result"] == "GRIP_CONTACT_HOLDING"

    import cv2

    def blob(img):
        if img is None:
            return None
        if img.shape[1] != 320:
            img = cv2.resize(img, (320, 240), interpolation=cv2.INTER_AREA)
        _, lab, st = pk._wrist_red_components(img)
        if len(st) < 2:
            return None
        k = 1 + int(np.argmax(st[1:, 4])); ys, xs = np.nonzero(lab == k); return dict(cx=float(xs.mean()), cy=float(ys.mean()), area=int(len(xs)))

    def wrist_now():
        if ep is not None:
            latest, _ = ep.wrists[f"{side}_wrist"].get(); return None if latest is None else latest["frame"]
        cam = cv2.VideoCapture(f"/dev/dapier/{side}_wrist_rgb", cv2.CAP_V4L2); cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV")); cam.set(cv2.CAP_PROP_FRAME_WIDTH, 320); cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 240); f = None
        for _ in range(6):
            ok_, x = cam.read(); f = x if ok_ else f
        cam.release(); return f

    def at_taught_place():
        """Before the descent: the block in the wrist image at 'above' must sit where it sat when 'above' was taught
        (2026-10-18 162330_right: the block 2.2 cm away, the jaws closed on nothing). Allowed: 20 % of the block's side
        in px (about 8 mm), area within 0.7..1.4. Only a reference taken in the recorder's 320x240 mode is compared."""
        ref_img = refs / "H2_above_c" / f"{side}_wrist_rgb.jpg"; ref = cv2.imread(str(ref_img)) if ref_img.exists() else None
        if ref is None or ref.shape[1] != 320:
            return dict(ok=True, checked=False, why="the taught image is not in the 320x240 recorder mode: not compared (re-teach to enable)")
        a_, b_ = blob(ref), blob(wrist_now())
        if a_ is None or b_ is None:
            return dict(ok=False, checked=True, why="no block in the taught or the current wrist image")
        side_px = a_["area"] ** 0.5; tol = 0.2 * side_px; dx, dy, ar = b_["cx"] - a_["cx"], b_["cy"] - a_["cy"], b_["area"] / a_["area"]
        ok = abs(dx) <= tol and abs(dy) <= tol and 0.7 <= ar <= 1.4
        return dict(ok=bool(ok), checked=True, dx_px=dx, dy_px=dy, area_ratio=ar, tol_px=tol, why=None if ok else f"블록이 가르친 위치에서 벗어남(dx {dx:+.0f}, dy {dy:+.0f} px, 허용 {tol:.0f}; 면적비 {ar:.2f}) - 블록을 그 자리로 옮기거나 기준 자세를 다시 가르치세요")

    def watch_grip(seconds):
        """Hold in the carry pose and watch the jaws: a block slipping out lets the held gripper close further (present
        drops) and its load fall. review 2026-10-19: a STATUS as in the session's watch - HELD / DROPPED / FAULT / STOP /
        UNKNOWN (no reference read, too few valid reads or a long gap); a failed read is counted, never read as 'held'."""
        from sweepick_local_loop import open_readonly_bus
        rows, bad, start, valid, gap_max = [], 0, None, 0, 0.0; last_ok = time.monotonic(); t_end = time.monotonic() + seconds
        def res(status, **kw):
            return dict(status=status, dropped=status == "DROPPED", start=start, rows=rows, valid_reads=valid, error_reads=len(rows) - valid, longest_gap_s=gap_max, **kw)
        while time.monotonic() < t_end:
            if Path(str(out) + ".STOP").exists():
                return res("STOP")
            try:
                with (ep.lease("left", "right") if ep is not None else __import__("contextlib").nullcontext()):
                    b = open_readonly_bus(side)
                    try:
                        pos = float(b.sync_read("Present_Position", normalize=False)["gripper"]); load = float(b.sync_read("Present_Load", normalize=False)["gripper"])
                        tq = int(b.sync_read("Torque_Enable", normalize=False)["gripper"]); stv = int(b.sync_read("Status", normalize=False)["gripper"])
                    finally:
                        b.disconnect()
            except Exception as e:
                rows.append(dict(t=time.time(), error=f"{type(e).__name__}: {e}")); gap_max = max(gap_max, time.monotonic() - last_ok); time.sleep(0.2); continue
            gap_max = max(gap_max, time.monotonic() - last_ok); last_ok = time.monotonic(); valid += 1
            rows.append(dict(t=time.time(), pos=pos, load=load, torque=tq, status=stv))
            if tq != 1 or stv != 0:
                return res("FAULT", why=f"gripper torque {tq}, status {stv}")
            start = start or dict(pos=pos, load=abs(load) or 1.0)
            slip = pos < start["pos"] - 25 or abs(load) < 0.4 * start["load"]; bad = bad + 1 if slip else 0
            if bad >= 2:
                return res("DROPPED")
            time.sleep(0.2)
        gap_max = max(gap_max, time.monotonic() - last_ok)
        if start is None or valid < pk.GRIP_WATCH_MIN_VALID or gap_max > pk.GRIP_WATCH_MAX_GAP_S:
            return res("UNKNOWN", why=f"valid reads {valid}, longest gap {gap_max:.2f} s")
        return res("HELD")

    tgt = lambda k: {n: float(taught[k][n]) for n in ARM}
    ok = (move("pregrasp", "out", dict(tgt("pregrasp"), gripper=open_tick), ALL)
          and move("above", "next", tgt("above"), ALL))
    place = at_taught_place() if (ok and a.execute) else dict(ok=True, checked=False, why="plan only")
    log["taught_place_check"] = place; save()
    if ok and not place["ok"]:
        log["steps"].append(dict(step="place_check", result="NOT_AT_TAUGHT_PLACE", why=place["why"])); save()
        ok2 = move("retreat", "next", tgt("pregrasp"), ALL) and move("home", "return", dict({n: float(home[n]) for n in ARM}, gripper=release_tick), ALL)
        ok = False
    ok = ok and move("grasp", "next", tgt("grasp"), ALL)
    gripped = ok and grip()
    if gripped:
        ok = move("lift", "next", tgt(a.lift_to), ALL, carried=True)
        if ok and "carry" in taught:                                           # fold the held block over the base, hold (the base would drive), come back over the place
            ok = move("carry", "next", tgt("carry"), ALL, carried=True)
            if ok and a.execute:
                w = watch_grip(a.carry_hold_s); log["carry_watch"] = {k: w.get(k) for k in ("status", "start", "valid_reads", "error_reads", "longest_gap_s", "why")}; log["carry_watch"]["last"] = w["rows"][-1] if w["rows"] else None; save()
                log["steps"].append(dict(step="carry_hold", result=w["status"], why=w.get("why"))); save()
                if w["status"] != "HELD":                                      # review 2026-10-19: DROPPED / UNKNOWN / FAULT / STOP end the run where it is (nothing further is commanded)
                    ok = False; log["needs_a_person"] = f"carry hold {w['status']}: the arm holds where it is"
            ok = ok and move("back_over_place", "next", tgt(a.lift_to), ALL, carried=True)
        ok = ok and move("lower", "next", tgt("grasp"), ALL, carried=True)
    if ok and (gripped or not a.execute):
        ok = move("open", "next", dict(gripper=release_tick), ALL) and move("retreat", "next", tgt("pregrasp"), ALL)
    elif ok and a.execute and not gripped:
        ok = move("open", "next", dict(gripper=release_tick), ALL) and move("retreat", "next", tgt("pregrasp"), ALL)
    if ok:
        move("home", "return", dict({n: float(home[n]) for n in ARM}, gripper=release_tick), ALL)       # back to the start with the jaws left fully open (user 2026-10-18)
    if ep is not None:
        try:
            doc = sweepick_episode.release(ep, task_outcome=f"TAUGHT_REPLAY gripped={gripped}"); log["record"] = dict(session_id=doc.get("session_id"), data_quality=doc.get("data_quality"))
        except Exception as e:
            log["record_error"] = f"{type(e).__name__}: {e}"
        pk.EPISODE = None
    last = log["steps"][-1] if log["steps"] else {}
    log["result"] = ("PLANNED_ALL" if not a.execute and all(s["result"] == "PLANNED" for s in log["steps"]) else
                     f"ENDED_AFTER_{last.get('step')}_{last.get('result')}"); log["gripped"] = gripped; save()
    print(json.dumps(dict(result=log["result"], gripped=gripped, steps=[(s["step"], s["result"], (s.get("collision") or {}).get("verdict"), s.get("why")) for s in log["steps"]]), indent=1, default=str))
    return 0


if __name__ == "__main__":
    sys.exit(main())
