"""Move joints of ONE arm from where they are NOW to a target, and bring them back.

Size of the move: unlimited. What can refuse or stop it is the HARD list of sweepick_HARD_GATE_ALLOWLIST.json and nothing else:
malformed command, the servo's own Min / Max, device != calibration, bus loss, the servo's own fault report (Status
register), operator stop, a write based on a stale position, port / torque ownership.
  --level device  (default) device-level commissioning in ticks. No model is consulted.
  --level model   a model-dependent move: the PROVISIONAL gates apply too (model joint range and collision of the commanded
                  path through the CANDIDATE mapping). They are model results, not physical limits.
Tracking error, current, load, how steady the arm was at rest: recorded as diagnostics, never a reason to stop. A joint
that keeps moving the right way is not stopped for being behind. After the last command the outcome is classified from
the readback (reached / behind / no motion observed).

  plan     read the device, build the trajectory, write TRAJECTORY.json. Writes nothing to the device.
  out      (--execute) hold the measured pose with torque on, run the trajectory, STAY at the target with torque on.
  return   (torque already on) trajectory back to --rest joint=tick,..., then release with a confirmed read.

After a hard stop: operator stop -> freeze and hold. Servo fault -> stop commanding the reported joints; the recovery
heuristics (not gates) decide between holding and releasing them. Bus lost -> nothing more can be commanded or known.
No automatic return.

usage: python -m sweepick.control.sweepick_trajectory_executor plan|out|return SIDE OUT_DIR --target joint=tick[,...] | --clear-range-end JOINT:TICKS
                     [--level device|model] [--cert FILE] [--rest joint=tick,...] [--other-arm joint=tick,...] [--execute]"""
from sweepick.integration.sweepick_resource_paths import source_path
import argparse, hashlib, json, sys, time
from pathlib import Path
import numpy as np
from sweepick.control import sweepick_joint_command_mapping as rc
from sweepick.control import sweepick_trajectory_profile as tt
from sweepick.control import sweepick_execution_gates as G
from sweepick.control import sweepick_motion_progress as PG
from sweepick.control.sweepick_servo_access import IN, CAL, READS, RELEASE_TRIES, real_bus, port_owner

# RECOVERY heuristics: used only AFTER a hard stop to choose between holding and releasing. They never start, allow or
# stop a normal move.
SETTLE_CYCLES, STEADY_TICKS = 15, 3
POST_READS = 15         # fewest reads after the last command; reading goes on while a joint still advances (progress_profile start_delay_s without progress ends it)
# exit 0 only when the measured state is what the mode promises: HOLDING = target reached and holding; RETURNED_RELEASED =
# return target reached AND torque release confirmed.
EXIT = dict(PLANNED=0, HOLDING=0, GRIP_CONTACT_HOLDING=0, GRIP_NO_CONTACT_HOLDING=9, GRIP_UNAVAILABLE_HOLDING=9, RETURNED_RELEASED=0, REFUSED=2, RELEASE_UNCONFIRMED=5, TARGET_NOT_REACHED_HOLDING=6, RETURN_NOT_CONFIRMED_HOLDING=7, NO_PROGRESS_HOLDING=8, STOPPED_HOLDING=10, FAULT_HOLDING=13, FAULT_JOINTS_RELEASED=14, BUS_LOST=15)


class Stop(Exception):
    def __init__(self, gate):
        super().__init__(gate.get("detail") or gate["gate"]); self.gate = gate


def parse_pairs(text):
    return {k: float(v) for k, v in (x.split("=") for x in text.split(","))} if text else {}


def following_state(direction, moved, moving_flag, velocity, error):
    """DIAGNOSTIC label of one joint in one cycle from the servo's own readback. No threshold: the facts as read."""
    if direction == 0:
        return "HELD"
    if moving_flag or velocity:
        return "MOVING" + ("" if moved * direction >= 0 else " (against the command)")
    return "AT_COMMAND" if error == 0 else "NOT_MOVING_THIS_CYCLE"


def run(mode, side, out, *, target=None, clear_range_end=None, rest=None, cert=None, level="device", execute=False, other_arm_ticks=None, open_bus=real_bus, owner=port_owner, mapping=None, mapping_sha="test", profile=None,
        progress=None, torque_joints=None, settle_trim=(), trim_check=None, start_recheck=None, hold_s=0.0, sleep=time.sleep, clock=time.time, stop_requested=lambda: False):
    """torque_joints: the joints whose torque this run switches on (out) / expects on (return). Default: ONLY the joints
    that move. No other joint of this arm, and nothing of the other arm, is powered implicitly; holding more joints (an arm
    move against gravity) must be asked for explicitly by naming them."""
    out = Path(out); out.mkdir(parents=True, exist_ok=False)
    port = f"/dev/dapier/{side}_arm"
    prof = profile or tt.load_profile()
    pg = progress or PG.load()
    log = dict(schema="tjj.move.v3", mode=mode, side=side, level=level, port=port, execute=bool(execute), started=time.strftime("%Y-%m-%dT%H:%M:%S%z"), writes=dict(Goal_Position=0, Torque_Enable=0), rows=[], events=[], profile=prof, total_travel_limit=None,
               mapping="CANDIDATE (verified=false)", diagnostics=[], progress_profile=pg, outcome=dict(command_completed=False, target_reached=None, return_confirmed=None, torque_released=None))

    def finish(result):
        log.update(result=result, exit_code=EXIT[result], finished=time.strftime("%Y-%m-%dT%H:%M:%S%z"))
        (out / "move.json").write_text(json.dumps(log, indent=1, default=str))
        return EXIT[result], log

    def refuse(gate):
        log["refused"] = gate
        return gate

    who = owner(port)
    if who:
        refuse(G.hard("port_owned", f"{port} is held by process {who}: not opened")); return finish("REFUSED")
    if mapping is None:
        mapping = rc.load_mapping(IN / "CANDIDATE_Q12_CONFIG.json", CAL, IN / "ASSEMBLED_ZERO_AND_PGRIPPER.json")
        mapping_sha = hashlib.sha256((IN / "CANDIDATE_Q12_CONFIG.json").read_bytes()).hexdigest()
    mp = mapping[side]
    try:
        bus = open_bus(port); bus.connect()
    except Exception as e:
        refuse(G.hard("bus_lost", f"the bus could not be opened: {type(e).__name__}: {str(e).splitlines()[0]}")); return finish("REFUSED")

    def read():
        """One fresh readback. Any failure of the driver (its own packet timeout included) is a bus loss."""
        t0 = clock(); r = {}
        try:
            for k, reg in READS + (("status", "Status"), ("moving", "Moving")):
                r[k] = {n: float(v) for n, v in bus.sync_read(reg, normalize=False).items()}
        except Exception as e:
            raise Stop(G.hard("bus_lost", f"read failed: {type(e).__name__}: {str(e).splitlines()[0]}"))
        r["t"], r["read_s"] = t0, clock() - t0
        return r

    def goal(n, tick):
        try:
            bus.write("Goal_Position", n, int(round(tick)), normalize=False); log["writes"]["Goal_Position"] += 1
        except Exception as e:
            raise Stop(G.hard("bus_lost", f"write failed: {type(e).__name__}: {str(e).splitlines()[0]}"))

    def torque(n, v):
        # A lost acknowledgement may happen after the servo applied the write.
        # Reuse the existing BUS_LOST handling; never claim torque is OFF/ON here.
        try:
            bus.write("Torque_Enable", n, int(v), normalize=False)
            log["writes"]["Torque_Enable"] += 1
        except Exception as e:
            log["events"].append({"torque_write_acknowledgement": "UNKNOWN", "joint": n,
                                  "requested": int(v), "error": f"{type(e).__name__}: {e}"})
            raise Stop(G.hard("bus_lost", f"torque write failed: {type(e).__name__}: {str(e).splitlines()[0]}")) from e

    def release(names):                                                       # RECOVERY: per motor, retried, confirmed by a read
        rec = dict(attempted=list(names), failures=[], confirmed=None)
        for n in names:
            for k in range(RELEASE_TRIES):
                try:
                    torque(n, 0); break
                except BaseException as e:
                    rec["failures"].append(dict(joint=n, attempt=k + 1, error=f"{type(e).__name__}: {e}")); sleep(0.05)
        try:
            t = read()["torque"]; rec["confirmed"] = all(t[n] == 0 for n in names); rec["torque_now"] = t
        except BaseException as e:
            rec["confirmed"] = False; rec["error"] = f"{type(e).__name__}: {e}"
        return rec

    def close(result):
        try:
            end = read()
            log["end"] = dict(ticks=end["present"], torque=end["torque"], status=end["status"], goal_register=end["goal"], torque_on=[n for n in rc.JOINTS if end["torque"][n]])
        except BaseException as e:
            log["end"] = dict(error=f"the end state could not be read: {type(e).__name__}: {e}")
        try:
            bus.disconnect(disable_torque=False)
        except BaseException as e:
            log["events"].append(f"disconnect: {type(e).__name__}: {e}")
        return finish(result)

    # ---- preflight: reads only; a refusal here writes nothing -------------------------------------------------------
    try:
        try:
            lo = {n: float(v) for n, v in bus.sync_read("Min_Position_Limit", normalize=False).items()}; hi = {n: float(v) for n, v in bus.sync_read("Max_Position_Limit", normalize=False).items()}
        except Exception as e:
            raise Stop(G.hard("bus_lost", f"read failed: {type(e).__name__}: {str(e).splitlines()[0]}"))
        bad = [n for n in rc.JOINTS if (lo[n], hi[n]) != (mp[n]["lo"], mp[n]["hi"])]
        if bad:
            raise Stop(G.hard("device_calibration_mismatch", f"device position limits differ from the calibration snapshot on {bad}"))
        pre = [read() for _ in range(5)]
        fresh = pre[-1]                                                       # the start state is the LAST read, taken just now
        now = dict(fresh["present"])
        P = np.array([[r["present"][n] for n in rc.JOINTS] for r in pre])
        log["diagnostics"].append(G.diagnostic("start_spread", "spread of the joint readings over the preflight reads (recorded; not a gate)", ticks=dict(zip(rc.JOINTS, np.ptp(P, axis=0).tolist()))))
        faults = {n: v for n, v in fresh["status"].items() if v}
        if faults:
            raise Stop(G.hard("servo_fault", f"Status register non-zero before the move: {faults}"))
        on = [n for n in rc.JOINTS if fresh["torque"][n]]
        if mode in ("plan", "out", "next"):
            if mode == "next" and execute and (not torque_joints or not set(on) <= set(torque_joints)):
                raise Stop(G.hard("torque_state", f"'next' continues a held arm: it needs --torque-joints and torque on only among them; on: {on}"))
            if on and mode == "out" and execute:
                raise Stop(G.hard("torque_state", f"torque is already on ({on}): another controller may own the arm; left as it is"))
            if clear_range_end:
                j, c = clear_range_end
                m = mp[j]; near_hi = (m["hi"] - now[j]) < (now[j] - m["lo"]); t = (m["hi"] - c) if near_hi else (m["lo"] + c)
                target = {} if (near_hi and t >= now[j]) or (not near_hi and t <= now[j]) else {j: float(t)}
            if target is None:
                raise Stop(G.hard("command_malformed", "no target given"))
        else:
            if not rest:
                raise Stop(G.hard("command_malformed", "'return' needs --rest (the ticks read before the 'out' move)"))
            target = {n: float(v) for n, v in rest.items() if v != now[n]}
            need = set(torque_joints or rest)
            if set(on) != need:
                raise Stop(G.hard("torque_state", f"'return' expects torque on exactly {sorted(need)}; on: {on}"))
        traj = tt.make(now, target, prof)
        if not traj["moving"] and mode in ("out", "next"):                      # the arm already stands on the target ticks: there is no path to check and nothing to send
            if mode == "next" and execute and set(torque_joints) <= set(on):
                log.update(start=dict(ticks=now, torque_on=on, goal_register=fresh["goal"], status=fresh["status"]), trajectory={k: v for k, v in traj.items() if k != "goals"}, torque_scope=dict(joints=list(torque_joints)))
                log["events"].append("already at the target: nothing was written"); log["outcome"].update(command_completed=True, target_reached=True, torque_released=False)
                log["reached"] = {}; return close("HOLDING")
            raise Stop(G.hard("command_malformed", "the target equals the measured position: nothing to move"))
        powered = [n for n in rc.JOINTS if n in (torque_joints or traj["moving"])] if mode != "return" else list(on)
        log["torque_scope"] = dict(joints=powered, not_powered=[n for n in rc.JOINTS if n not in powered], other_arm="not opened by this run", rule="only these joints are switched on / commanded; the others keep the torque state they have")
        check = tt.validate(side, mp, traj, prof, mapping_sha=mapping_sha, level=level, cert=cert, other_arm_ticks=other_arm_ticks)
        log.update(start=dict(ticks=now, torque_on=on, goal_register=fresh["goal"], status=fresh["status"]), trajectory={k: v for k, v in traj.items() if k != "goals"}, validation=check)
        (out / "TRAJECTORY.json").write_text(json.dumps(dict(side=side, level=level, mapping_sha256=mapping_sha, profile=prof, identity=check["identity"], other_arm_ticks=other_arm_ticks, trajectory=traj), indent=1))
        if mode == "plan" or not execute:
            log["events"].append("the device was read, nothing was written")
            return close("PLANNED")
        if check["hard"]:
            raise Stop(check["hard"][0])
        if check["provisional"]:
            raise Stop(check["provisional"][0])
    except Stop as e:
        refuse(e.gate); return close("REFUSED") if e.gate["gate"] != "bus_lost" else finish("REFUSED")
    assert log["writes"] == dict(Goal_Position=0, Torque_Enable=0)

    moving = traj["moving"]
    direction = {n: (int(np.sign(traj["target"][n] - now[n])) if n in moving else 0) for n in rc.JOINTS}
    try:
        try:
            if mode in ("out", "next"):                                      # next: the joints already held stay as they are; only newly powered joints are seeded
                new = [n for n in powered if n not in on]
                hold = read()                                                 # read again immediately before the first write
                if hold["present"] != now:                                    # the start is whatever is measured NOW, not the planning read
                    moved_since_plan = {n: hold["present"][n] - now[n] for n in rc.JOINTS if hold["present"][n] != now[n]}
                    now = dict(hold["present"]); traj = tt.make(now, {n: v for n, v in traj["target"].items()}, prof)
                    if level == "model":                                      # a held arm's reading changes by a tick: neither refused for that nor waved through - the path from where the arm IS now is checked again
                        cert2 = None if start_recheck is None else start_recheck(dict(side=side, level=level, mapping_sha256=mapping_sha, profile=prof, identity=tt.identity(side, traj, mapping_sha), other_arm_ticks=other_arm_ticks, trajectory=traj))
                        recheck = tt.validate(side, mp, traj, prof, mapping_sha=mapping_sha, level=level, cert=cert2, other_arm_ticks=other_arm_ticks)
                        log["events"].append(dict(start_changed_since_plan=moved_since_plan, rechecked=cert2 is not None, collision=None if cert2 is None else cert2.get("nominal_verdict")))
                        if cert2 is None:
                            raise Stop(G.provisional("certificate_binding", f"the arm is not where the collision result was computed for ({moved_since_plan}) and no re-check of the path from here was given"))
                    else:
                        recheck = tt.validate(side, mp, traj, prof, mapping_sha=mapping_sha, level=level)
                    log["events"].append("the trajectory was rebuilt from the read taken immediately before the first write")
                    log.update(start=dict(log["start"], ticks=now), trajectory={k: v for k, v in traj.items() if k != "goals"}, validation=recheck)
                    if recheck["hard"]:
                        raise Stop(recheck["hard"][0])
                    if recheck["provisional"]:
                        raise Stop(recheck["provisional"][0])
                    direction = {n: (int(np.sign(traj["target"][n] - now[n])) if n in moving else 0) for n in rc.JOINTS}
                for n in new:                                                 # Goal_Position is 0 after a power cycle: seed it with the position just read
                    goal(n, now[n])
                back = read()
                if any(back["goal"][n] != round(now[n]) for n in new):
                    for n in new:
                        goal(n, log["start"]["goal_register"][n])
                    refuse(G.hard("bus_lost", "the Goal_Position read-back differs from what was written; goal register restored, torque never switched on"))
                    return close("REFUSED")
                log["goal_seed"] = dict(joints=new, already_held=[n for n in powered if n in on], from_read_at=hold["t"], readback={n: back["goal"][n] for n in new})
                log["drive_enable"] = dict(note="on this device a Goal_Position write has been seen to switch torque on by itself (2026-10-08: 5 goal writes acknowledged, no Torque_Enable write, torque read 1 on exactly those 5 joints; "
                                                "the driver's write() sends only address 42, so this is the servo, not the wrapper). The FIRST goal write of a joint is therefore treated as the write that may power it",
                                           torque_read_after_the_goal_write={n: back["torque"][n] for n in new}, explicit_torque_writes_follow=True)
                for n in new:
                    torque(n, 1)
                on_reads = [read() for _ in range(5)]
                log["diagnostics"].append(G.diagnostic("torque_on_drift", "movement of the joints when torque was switched on (recorded)", ticks={n: max(abs(r["present"][n] - now[n]) for r in on_reads) for n in rc.JOINTS}))
            cmd, prev, k, waits = dict(now), None, 0, 0
            mon = {n: PG.Joint(pg, now[n], direction[n], clock()) for n in moving}
            while k < traj["steps"]:
                t0 = clock()
                if stop_requested():
                    raise Stop(G.hard("operator_stop", "stop requested"))
                waiting = [n for n in moving if mon[n].state == PG.WAIT]
                if not waiting:                                               # the command advances only while every moving joint is progressing or inside its normal start delay
                    for n in moving:
                        cmd[n] = traj["goals"][n][k]; goal(n, cmd[n])
                    k += 1
                else:
                    waits += 1                                                # nothing further is accumulated while a joint is not advancing
                r = read()
                faults = {n: v for n, v in r["status"].items() if v}
                if faults:
                    raise Stop(G.hard("servo_fault", f"Status register non-zero: {faults}", joints=sorted(faults)))
                err = {n: r["present"][n] - round(cmd[n]) for n in rc.JOINTS}
                state = {n: mon[n].update(r["t"] + r["read_s"], r["present"][n], direction[n] * (round(cmd[n]) - r["present"][n])) for n in moving}
                r.update(k=k, commanded={n: cmd[n] for n in moving}, tracking_error=err, progress=state, command_waiting=bool(waiting),
                         following={n: following_state(direction[n], 0 if prev is None else r["present"][n] - prev["present"][n], r["moving"][n], r["velocity"][n], err[n]) for n in rc.JOINTS})
                log["rows"].append(r); prev = r
                stuck = [n for n in moving if state[n] == PG.NO_PROGRESS]
                if stuck:
                    raise Stop(G.contract("no_progress", f"{stuck} stayed more than {pg['follow']:.0f} ticks behind the command for {2 * pg['start_delay']:.2f} s without catching up; the command had been waiting since the first {pg['start_delay']:.2f} s", joints=stuck,
                                          commanded={n: round(cmd[n]) for n in stuck}, measured={n: r["present"][n] for n in stuck}, target={n: traj["target"][n] for n in stuck}))
                sleep(max(0.0, prof["dt"] - (clock() - t0)))
            log["outcome"]["command_completed"] = True
            log["command_waits"] = waits
            def settle():
                """after a command: read while a joint is still advancing, then return the reads"""
                post = []
                best, t_best = {n: None for n in moving}, clock()
                while True:
                    r = read(); post.append(r)
                    for n in moving:
                        if best[n] is None or direction[n] * (r["present"][n] - best[n]) >= pg["resolution"]:      # further in the direction of travel; a reading that flickers back and forth by a tick is not 'still advancing'
                            t_best = clock() if best[n] is not None else t_best; best[n] = r["present"][n]
                    faults = {n: v for n, v in r["status"].items() if v}
                    if faults:
                        raise Stop(G.hard("servo_fault", f"Status register non-zero: {faults}", joints=sorted(faults)))
                    r.update(k=k, commanded={n: cmd[n] for n in moving}, tracking_error={n: r["present"][n] - round(cmd[n]) for n in rc.JOINTS}, phase="post")
                    log["rows"].append(r)
                    if len(post) >= POST_READS and clock() - t_best > pg["start_delay"]:
                        return post
                    if stop_requested():
                        raise Stop(G.hard("operator_stop", "stop requested"))
                    sleep(prof["dt"])
            post = settle()
            log["settle_reads"] = len(post)
            trims = []
            while settle_trim:                                                # integral trim of a loaded arm joint that settled short (sweepick_progress_profile.json: settle_trim)
                here = {n: float(np.median([r["present"][n] for r in post[-5:]])) for n in moving}
                short = {n: traj["target"][n] - here[n] for n in moving if n in settle_trim and not PG.reached(pg, here[n], traj["target"][n])}
                stuck = bool(trims) and all(abs(short[n]) >= abs(trims[-1]["short"].get(n, np.inf)) for n in short)
                lead_after = {n: abs(cmd[n] + e - here[n]) for n, e in short.items()}
                if not short or (trims and not trims[-1].get("applied")) or (stuck and max(lead_after.values()) > pg["follow"]):
                    break                                                     # within tolerance; or the last trim could not be applied; or the joint has not moved and one more trim would put the goal further ahead than a loaded joint has ever needed to follow (progress profile: following lead)
                seg_target = {n: float(round(cmd[n] + e)) for n, e in short.items()}
                seg = tt.make({n: float(round(cmd[n])) for n in rc.JOINTS}, seg_target, prof)          # the trim is a commanded move like any other: generated, retimed and checked
                verdict = None if trim_check is None else trim_check(dict(side=side, level=level, mapping_sha256=mapping_sha, profile=prof, identity=tt.identity(side, seg, mapping_sha), other_arm_ticks=other_arm_ticks, trajectory=seg))
                chk = tt.validate(side, mp, seg, prof, mapping_sha=mapping_sha, level=level, cert=verdict, other_arm_ticks=other_arm_ticks)     # the SAME validation as the stage's own path: servo range, profile, and at model level the model joint range and the collision result of exactly this segment
                rec = dict(short={n: float(v) for n, v in short.items()}, goal=seg_target, steps=seg["steps"], hard=chk["hard"], provisional=chk["provisional"], collision=None if verdict is None else verdict.get("nominal_verdict"))
                if chk["hard"] or chk["provisional"]:
                    rec["applied"] = False; trims.append(rec)                 # not checked or not clear: the joint stays short and is reported as it stands
                    break
                for i in range(seg["steps"]):
                    for n in seg["moving"]:
                        cmd[n] = seg["goals"][n][i]; goal(n, cmd[n])
                    r = read()
                    faults = {n: v for n, v in r["status"].items() if v}
                    if faults:
                        raise Stop(G.hard("servo_fault", f"Status register non-zero: {faults}", joints=sorted(faults)))
                    r.update(k=k, commanded={n: cmd[n] for n in moving}, tracking_error={n: r["present"][n] - round(cmd[n]) for n in rc.JOINTS}, phase="trim")
                    log["rows"].append(r)
                    if stop_requested():
                        raise Stop(G.hard("operator_stop", "stop requested"))
                    sleep(max(0.0, prof["dt"]))
                rec["applied"] = True
                trims.append(rec)
                post = settle()
            if trims:
                log["settle_trim"] = trims
            t_hold = clock()
            while hold_s and clock() - t_hold < hold_s:                       # the port owner keeps recording while the arm holds (no other reader is opened)
                r = read()
                faults = {n: v for n, v in r["status"].items() if v}
                if faults:
                    raise Stop(G.hard("servo_fault", f"Status register non-zero: {faults}", joints=sorted(faults)))
                r.update(k=k, commanded={n: cmd[n] for n in moving}, tracking_error={n: r["present"][n] - round(cmd[n]) for n in rc.JOINTS}, phase="hold")
                log["rows"].append(r); post.append(r)
                if stop_requested():
                    raise Stop(G.hard("operator_stop", "stop requested"))
                sleep(prof["dt"])
            last = post[-5:]
            reached = {n: float(np.median([r["present"][n] for r in last])) for n in rc.JOINTS}
            settled = {n: bool(max(r["present"][n] for r in last) - min(r["present"][n] for r in last) <= pg["resolution"] and not any(r["moving"][n] for r in last)) for n in moving}      # within the encoder's resolution: a held joint's reading flickers by one tick
            ok = {n: bool(PG.reached(pg, reached[n], traj["target"][n]) and settled[n]) for n in moving}
            log["reached"] = {n: dict(tick=reached[n], target=traj["target"][n], minus_target=reached[n] - traj["target"][n], travelled=reached[n] - now[n], within_tolerance=PG.reached(pg, reached[n], traj["target"][n]), settled=settled[n], reached=ok[n],
                                      load=float(np.median([r["load"][n] for r in last])), current=float(np.median([r["current"][n] for r in last]))) for n in moving}
            errs = [abs(v) for r in log["rows"] for v in r["tracking_error"].values()]
            log["diagnostics"].append(G.diagnostic("tracking_error", "largest |measured - commanded| during the move (recorded)", max_ticks=max(errs) if errs else 0.0))
            log["outcome"]["target_reached"] = all(ok.values())
            if mode == "return":
                log["outcome"]["return_confirmed"] = all(ok.values())
                if not all(ok.values()):                                      # not back: torque is NOT released on that ground and this is not a success
                    log["outcome"]["torque_released"] = False
                    log["needs_a_person"] = "the return target was not reached; the powered joints still hold. Nothing was released"
                    log["stopped_by"] = G.contract("target_not_reached", f"return not confirmed: {({n: log['reached'][n]['minus_target'] for n in moving if not ok[n]})}")
                    return close("RETURN_NOT_CONFIRMED_HOLDING")
                rel = release(powered); log["release"] = rel
                log["outcome"]["torque_released"] = True if rel["confirmed"] else ("UNKNOWN" if "error" in rel else False)   # read back as off / no read-back possible / read back as still on
                return close("RETURNED_RELEASED" if rel["confirmed"] else "RELEASE_UNCONFIRMED")
            log["outcome"]["torque_released"] = False
            log["state_left"] = f"HOLDING with torque on {powered}. 'return' (or a release by hand) ends it"
            if not all(ok.values()):
                log["stopped_by"] = G.contract("target_not_reached", f"target not reached: {({n: log['reached'][n]['minus_target'] for n in moving if not ok[n]})}")
                return close("TARGET_NOT_REACHED_HOLDING")
            return close("HOLDING")
        except Stop:
            raise
        except KeyboardInterrupt:
            raise Stop(G.hard("operator_stop", "interrupted by the operator"))
    except Stop as p:
        log["stopped_by"] = p.gate
        kind = p.gate["gate"]
        if kind != "bus_lost" and not any(log["writes"].values()):
            refuse(p.gate); return close("REFUSED")                           # stopped before the first write of this call: nothing was sent, whatever was held before is held as it was
        if kind in ("certificate_binding", "port_owned", "torque_state", "command_malformed", "servo_range", "device_calibration_mismatch", "profile_violation") and not log["writes"]["Torque_Enable"]:
            refuse(p.gate); return close("REFUSED")                           # nothing was switched on
        if kind == "bus_lost":
            log["needs_a_person"] = ("the bus does not answer: the torque state is NOT known (powered servos keep their last goal). Support the arm and cut motor power by hand"
                                     + (". Goal_Position was written before the loss: on this device that alone can leave the joints powered, whatever the count of torque writes says" if log["writes"]["Goal_Position"] else ""))
            try:
                bus.disconnect(disable_torque=False)
            except BaseException:
                pass
            log["end"] = dict(error="no read possible", torque="UNKNOWN")
            log["outcome"]["torque_released"] = "UNKNOWN"                     # a write whose answer was lost may have been applied
            return finish("BUS_LOST")
        # ---- RECOVERY (heuristics; not gates) -----------------------------------------------------------------------
        try:
            cur = read()
            for n in moving:
                goal(n, cur["present"][n])                                    # stop commanding further motion
            settle = []
            for _ in range(SETTLE_CYCLES):
                sleep(prof["dt"]); settle.append(read())
            spread = {n: float(np.ptp([r["present"][n] for r in settle[-8:]])) for n in rc.JOINTS}
            log["outcome"].update(target_reached=False, torque_released=False)
            faulted = sorted({n for r in settle[-3:] for n, v in r["status"].items() if v})
            log["recovery"] = dict(heuristics=dict(SETTLE_CYCLES=SETTLE_CYCLES, STEADY_TICKS=STEADY_TICKS, RELEASE_TRIES=RELEASE_TRIES), spread_ticks=spread, steady=all(v <= STEADY_TICKS for v in spread.values()), still_faulted=faulted)
            if kind == "servo_fault":
                if faulted:
                    log["release"] = release([n for n in faulted if n in powered]); log["needs_a_person"] = f"{faulted} released: the servo still reports a fault; the other powered joints hold"
                    return close("FAULT_JOINTS_RELEASED" if log["release"]["confirmed"] else "RELEASE_UNCONFIRMED")
                log["needs_a_person"] = "holding at the measured position after a servo fault that cleared; no automatic return"
                return close("FAULT_HOLDING")
            if kind == "no_progress":
                log["needs_a_person"] = ("the joint did not advance: the command was relaxed to the measured position and the powered joints hold. No automatic return and no automatic release. "
                                         "For an arm joint this is a stall; for a gripper it may be a contact and needs other evidence")
                return close("NO_PROGRESS_HOLDING")
            log["needs_a_person"] = "stopped by the operator and holding at the measured position; no automatic return"
            return close("STOPPED_HOLDING")
        except BaseException as e:
            log["needs_a_person"] = f"the bus failed during the stop handling ({type(e).__name__}: {e}): the torque state is NOT known"
            log["end"] = dict(error="no read possible")
            return finish("BUS_LOST")


JAW_PROFILE = source_path("sweepick_jaw_profile_left.json")


def jaw_device(lo, hi, prof, path=JAW_PROFILE):
    """sweepick_grasp.JawDevice of the real left gripper from its profile file, on the primitive's opening scale (0 = calibrated closed, 1 = calibrated open)."""
    from sweepick.control import sweepick_gripper_contact_hold as tg
    d = json.loads(Path(path).read_text()); R = float(hi - lo); v = lambda k: float(d[k]["value"])
    return tg.JawDevice(residual_floor=v("residual_floor_ticks") / R, preload_residual=v("preload_residual_ticks") / R, creep_step=v("creep_step_ticks") / R, creep_period_s=v("creep_period_s"), close_rate_per_s=prof["v"] / tt.TICK_RAD / R,
                        end_stop_opening=v("end_stop_ticks_above_closed") / R, rest_step=v("rest_step_ticks") / R, max_feedback_age_s=v("max_feedback_age_s"), confirm_s=v("confirm_s"), source=f"{d['name']} sha256 {hashlib.sha256(Path(path).read_bytes()).hexdigest()[:16]}")


def grip(side, out, *, torque_joints, cert=None, level="device", execute=False, hold_s=1.0, open_bus=real_bus, owner=port_owner, mapping=None, mapping_sha="test", profile=None, sleep=time.sleep, clock=time.time, stop_requested=lambda: False, jaw_profile=None, jaw_obj=None):
    """Close the gripper on whatever is between the jaws with the EXISTING grasp primitive (sweepick_grasp.Jaw): the closing
    command runs at the profile speed until the residual leaves the free stroke (contact onset), then squeezes in single
    ticks to the device preload and holds there. Nothing is accumulated after the onset and nothing is relaxed afterwards:
    the held command stays a little ahead of the jaws, so a lost object shows as the residual falling to the no-load floor.
    The arm joints named in torque_joints must already be held; none of them is commanded here.
    cert (model level): dict(nominal_verdict, arm_ticks, sweep=[from_tick, to_tick]) of a check of the WHOLE possible closing
    sweep at this arm pose, with the object in the scene and only the intended contacts allowed."""
    from sweepick.control import sweepick_gripper_contact_hold as tg
    out = Path(out); out.mkdir(parents=True, exist_ok=True); port = f"/dev/dapier/{side}_arm"; prof = profile or tt.load_profile(); J = "gripper"
    log = dict(schema="tjj.grip.v1", mode="grip", side=side, level=level, port=port, execute=bool(execute), started=time.strftime("%Y-%m-%dT%H:%M:%S%z"), writes=dict(Goal_Position=0, Torque_Enable=0), rows=[], events=[], profile=prof,
               owner="sweepick_grasp.Jaw", outcome=dict(command_completed=False, contact_candidate=None, torque_released=False), mapping="CANDIDATE (verified=false)")
    def finish(result):
        log.update(result=result, exit_code=EXIT[result], finished=time.strftime("%Y-%m-%dT%H:%M:%S%z")); (out / "move.json").write_text(json.dumps(log, indent=1, default=str)); return EXIT[result], log
    if owner(port):
        log["refused"] = G.hard("port_owned", f"{port} is held by another process: not opened"); return finish("REFUSED")
    if mapping is None:
        mapping = rc.load_mapping(IN / "CANDIDATE_Q12_CONFIG.json", CAL, IN / "ASSEMBLED_ZERO_AND_PGRIPPER.json")
    mp = mapping[side]
    try:
        bus = open_bus(port); bus.connect()
    except Exception as e:
        log["refused"] = G.hard("bus_lost", f"the bus could not be opened: {type(e).__name__}: {str(e).splitlines()[0]}"); return finish("REFUSED")
    def read():
        t0 = clock(); r = {}
        try:
            for k, reg in READS + (("status", "Status"), ("moving", "Moving")):
                r[k] = {n: float(v) for n, v in bus.sync_read(reg, normalize=False).items()}
        except Exception as e:
            raise Stop(G.hard("bus_lost", f"read failed: {type(e).__name__}: {str(e).splitlines()[0]}"))
        r["t"], r["read_s"] = t0, clock() - t0; return r
    def write(reg, v):
        try:
            bus.write(reg, J, int(round(v)), normalize=False); log["writes"][reg] += 1
        except Exception as e:
            log["events"].append({f"{reg}_write_acknowledgement": "UNKNOWN", "requested": int(round(v)), "error": f"{type(e).__name__}: {e}"})
            raise Stop(G.hard("bus_lost", f"write failed: {type(e).__name__}: {str(e).splitlines()[0]}")) from e
    def done(result):
        try:
            e = read(); log["end"] = dict(ticks=e["present"], torque=e["torque"], status=e["status"], goal_register=e["goal"], torque_on=[n for n in rc.JOINTS if e["torque"][n]])
        except BaseException as x:
            log["end"] = dict(error=f"{type(x).__name__}: {x}")
        try:
            bus.disconnect(disable_torque=False)
        except BaseException as x:
            log["events"].append(f"disconnect: {type(x).__name__}: {x}")
        return finish(result)
    cmd = None
    try:
        lo, hi = float(mp[J]["lo"]), float(mp[J]["hi"])
        dev_lo, dev_hi = (float(bus.sync_read(k, normalize=False)[J]) for k in ("Min_Position_Limit", "Max_Position_Limit"))
        if (dev_lo, dev_hi) != (lo, hi):
            raise Stop(G.hard("device_calibration_mismatch", "device position limits of the gripper differ from the calibration snapshot"))
        now = read(); on = [n for n in rc.JOINTS if now["torque"][n]]
        if any(now["status"].values()):
            raise Stop(G.hard("servo_fault", f"Status register non-zero before the grip: {now['status']}"))
        if not set(on) <= set(torque_joints) or J not in torque_joints:
            raise Stop(G.hard("torque_state", f"the grip continues a held arm: torque may be on only among {sorted(torque_joints)}; on: {on}"))
        if side != "left" and jaw_profile is None:
            raise ValueError("the device profile of this gripper was not given: the left gripper's profile is the left device's and is not used for another gripper")
        dev = jaw_device(lo, hi, prof, jaw_profile or JAW_PROFILE); R = hi - lo; op = lambda t: (t - lo) / R; tk = lambda o: lo + o * R
        log.update(start=dict(ticks=now["present"], torque_on=on), jaw_device=dev.__dict__ if hasattr(dev, "__dict__") else str(dev), sweep=[now["present"][J], tk(dev.end_stop_opening)])
        if level == "model":
            ok = cert is not None and cert.get("nominal_verdict") == "CLEAR" and all(abs(cert["arm_ticks"][n] - now["present"][n]) <= 1 for n in rc.JOINTS[:5]) and cert["sweep"][0] >= now["present"][J] - 1 and cert["sweep"][1] <= tk(dev.end_stop_opening) + 1
            if not ok:
                raise Stop(G.provisional("certificate_binding" if cert is not None and cert.get("nominal_verdict") == "CLEAR" else "collision_candidate_frame", "the closing sweep at this arm pose has no CLEAR collision result bound to it"))
        if not execute:
            log["events"].append("the device was read, nothing was written"); return done("PLANNED")
    except Stop as e:
        log["refused"] = e.gate
        return done("REFUSED") if e.gate["gate"] != "bus_lost" else finish("REFUSED")
    if jaw_obj is not None and jaw_obj.dev != dev:
        raise ValueError("the Jaw handed in was built for another device profile than this gripper's")
    jaw = jaw_obj if jaw_obj is not None else tg.Jaw(dev, prof["dt"], tg.INDEPENDENT); vmax, amax = prof["v"] / tt.TICK_RAD * prof["dt"], prof["a"] / tt.TICK_RAD * prof["dt"] ** 2
    try:
        try:
            cmd = float(now["present"][J])
            if J not in on:
                write("Goal_Position", cmd); back = read()
                if back["goal"][J] != round(cmd):
                    raise Stop(G.hard("bus_lost", "the Goal_Position read-back differs from what was written"))
                write("Torque_Enable", 1)
            jaw.start_close(); step, t_hold, contact = 0.0, None, None
            while True:
                t0 = clock()
                if stop_requested():
                    raise Stop(G.hard("operator_stop", "stop requested"))
                r = read()
                if any(r["status"].values()):
                    raise Stop(G.hard("servo_fault", f"Status register non-zero: {({n: v for n, v in r['status'].items() if v})}"))
                state = jaw.update(r["t"] + r["read_s"], op(cmd), op(r["present"][J]), torque_enabled=bool(r["torque"][J]))
                if state == tg.CONTACT_CANDIDATE and contact is None:
                    contact = dict(t=r["t"], commanded=cmd, present=r["present"][J], residual_ticks=r["present"][J] - cmd, load_raw=r["load"][J], current_raw=r["current"][J])
                want = tk(jaw.next_command(op(cmd)))                          # the primitive decides: close / squeeze / hold
                if jaw.mode == "close":
                    step = min(cmd - want, vmax, step + amax); want = cmd - max(step, 0.0)       # within the selected profile (speed and its rate of change)
                else:
                    step = 0.0
                if want != cmd:
                    cmd = want; write("Goal_Position", cmd)
                r.update(commanded={J: cmd}, residual_ticks=r["present"][J] - cmd, jaw_state=state, jaw_mode=jaw.mode, phase="grip")
                log["rows"].append(r)
                if jaw.mode == "hold":
                    t_hold = t_hold or clock()
                    if clock() - t_hold >= hold_s and jaw.closed_and_resting():
                        break
                sleep(max(0.0, prof["dt"] - (clock() - t0)))
            log["outcome"]["command_completed"] = True
        except KeyboardInterrupt:
            raise Stop(G.hard("operator_stop", "interrupted by the operator"))
        last = log["rows"][-10:]
        log["grip"] = dict(state=jaw.state, loaded=jaw.loaded, dropped=jaw.dropped, contact_onset=contact, hold_command_tick=cmd, at_rest=dict(present=float(np.median([x["present"][J] for x in last])), residual_ticks=float(np.median([x["residual_ticks"] for x in last])),
                           load_raw=float(np.median([x["load"][J] for x in last])), current_raw=float(np.median([x["current"][J] for x in last]))), log=jaw.log[-12:], samples=jaw.samples, gaps=jaw.gaps,
                           meaning="CONTACT_CANDIDATE = something resists the closing; it is not known from this to be the object and it is not a grasp")
        log["outcome"]["contact_candidate"] = jaw.state == tg.CONTACT_CANDIDATE
        return done("GRIP_CONTACT_HOLDING" if jaw.state == tg.CONTACT_CANDIDATE else ("GRIP_NO_CONTACT_HOLDING" if jaw.state in (tg.NO_CONTACT, tg.EMPTY_END_STOP) else "GRIP_UNAVAILABLE_HOLDING"))
    except Stop as p:
        log["stopped_by"] = p.gate; kind = p.gate["gate"]; log["grip"] = dict(state=jaw.state, log=jaw.log[-12:])
        if kind == "bus_lost":
            log["needs_a_person"] = "the bus does not answer: the torque state is NOT known. Support the arm and cut motor power by hand"
            try:
                bus.disconnect(disable_torque=False)
            except BaseException:
                pass
            log["end"] = dict(error="no read possible", torque="UNKNOWN"); log["outcome"]["torque_released"] = "UNKNOWN"; return finish("BUS_LOST")
        try:
            cur = read(); write("Goal_Position", cur["present"][J])            # stop closing: goal = measured; nothing is opened, released or retried
            log["needs_a_person"] = "the grip was stopped; the gripper holds its measured position and the arm holds. No automatic opening"
            return done("FAULT_HOLDING" if kind == "servo_fault" else ("STOPPED_HOLDING" if kind == "operator_stop" else "REFUSED"))
        except BaseException as e:
            log["needs_a_person"] = f"the bus failed during the stop handling ({type(e).__name__}): the torque state is NOT known"; log["end"] = dict(error="no read possible"); return finish("BUS_LOST")


def main(argv=None):
    ap = argparse.ArgumentParser(); ap.add_argument("mode", choices=("plan", "out", "next", "return")); ap.add_argument("side", choices=("left", "right")); ap.add_argument("out")
    ap.add_argument("--target"); ap.add_argument("--clear-range-end"); ap.add_argument("--rest"); ap.add_argument("--cert"); ap.add_argument("--other-arm"); ap.add_argument("--level", choices=("device", "model"), default="device"); ap.add_argument("--execute", action="store_true"); ap.add_argument("--torque-joints", help="joints to power, comma separated (default: only the joints that move)")
    a = ap.parse_args(argv)
    cre = None
    if a.clear_range_end:
        j, c = a.clear_range_end.split(":"); cre = (j, float(c))
    cert = json.loads(Path(a.cert).read_text()) if a.cert else None
    code, log = run(a.mode, a.side, a.out, target=parse_pairs(a.target) or None, clear_range_end=cre, rest=parse_pairs(a.rest) or None, cert=cert, level=a.level, execute=a.execute, other_arm_ticks=parse_pairs(a.other_arm) or None, torque_joints=a.torque_joints.split(",") if a.torque_joints else None,
                    stop_requested=Path(str(a.out) + ".STOP").exists)
    print(json.dumps({k: log.get(k) for k in ("mode", "side", "level", "execute", "result", "exit_code", "outcome", "torque_scope", "goal_seed", "refused", "stopped_by", "command_waits", "trajectory", "validation", "recovery", "release", "reached", "diagnostics", "state_left", "needs_a_person", "writes", "end")}, indent=1, default=str))
    return code


if __name__ == "__main__":
    sys.exit(main())
