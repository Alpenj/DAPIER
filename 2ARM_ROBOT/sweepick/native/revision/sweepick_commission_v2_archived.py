"""ARCHIVED (2026-10-07): the fixed-delta commissioning run of this morning. Kept as the record of what ran and for its
preflight / release tests. It contains a fixed per-move tick bound and is NOT a tool any more: real moves go through
sweepick_move_a.py, where a move is gated by the trajectory checks of sweepick_trajectory.py and by nothing else. main() refuses.

Supervised commissioning of ONE arm (v2): torque on at the present pose, a small move of one joint at a time toward
the middle of its calibrated range and back, torque off.

Registers written: Goal_Position and Torque_Enable. Nothing else (no Lock, no limits, no PID, no calibration, no mode).

v2 (after the review of the first run):
  * a refusal BEFORE the first write leaves the device exactly as it was: no torque write, no goal write
  * goals written before torque-on are put back if the run is refused at that point
  * torque release: every motor on its own, with retries, confirmed by a read; a failure on one motor does not stop the
    release of the others; an unconfirmed release is reported as such and never as 'off'
  * after an error the walk back to the start pose is checked (limits on every step, final distance), and the result is
    recorded; 'returned' is a measured fact, not an assumption
  * exit code: 0 COMPLETED, 2 REFUSED (state preserved), 3 ABORTED (returned, torque off confirmed),
               4 ABORTED (NOT returned, torque off confirmed), 5 TORQUE RELEASE UNCONFIRMED (needs a person)

usage: sweepick_commission.py SIDE OUT_DIR [--delta 20] [--joints a,b,...]"""
import argparse, json, subprocess, sys, time
from pathlib import Path
import numpy as np
import sweepick_real_command as rc

IN = Path.home() / "sweepick_261007_commission/inputs"
CAL = Path.home() / ".config/dapier/lerobot-calibration"
PERIOD = 0.02
TRACK_ABORT, DRIFT_ABORT, HOLD_DRIFT = 40, 15, 8          # ticks. Commissioning safety bounds for a <= 25-tick move, not task thresholds
RETURN_TOLERANCE = 15                                     # ticks from the start pose that still count as 'returned' (the first run rested 1..11 ticks away)
RELEASE_TRIES = 3
READS = (("goal", "Goal_Position"), ("present", "Present_Position"), ("velocity", "Present_Velocity"), ("load", "Present_Load"), ("current", "Present_Current"), ("torque", "Torque_Enable"))
EXIT = dict(COMPLETED=0, REFUSED=2, ABORTED_RETURNED=3, ABORTED_NOT_RETURNED=4, TORQUE_RELEASE_UNCONFIRMED=5)


class Refuse(Exception):
    """Raised before anything was written."""


class Abort(Exception):
    pass


def real_bus(port):
    from lerobot.motors import Motor, MotorNormMode
    from lerobot.motors.feetech import FeetechMotorsBus
    return FeetechMotorsBus(port=port, motors={n: Motor(i + 1, "sts3215", MotorNormMode.RANGE_0_100 if n == "gripper" else MotorNormMode.DEGREES) for i, n in enumerate(rc.JOINTS)})


def port_owner(port):
    return subprocess.run(["fuser", port], capture_output=True, text=True).stdout.strip()


def run(side, out, *, delta=20, joints=rc.JOINTS, open_bus=real_bus, owner=port_owner, mapping=None, sleep=time.sleep):
    out = Path(out); out.mkdir(parents=True, exist_ok=False)
    port = f"/dev/dapier/{side}_arm"
    log = dict(schema="tjj.commission.v2", side=side, port=port, started=time.strftime("%Y-%m-%dT%H:%M:%S%z"), delta_ticks=delta, registers_written=["Goal_Position", "Torque_Enable"], writes=dict(Goal_Position=0, Torque_Enable=0), write_failures=[],
               mapping="CANDIDATE (verified=false); used for limits only", bounds=dict(track_abort_ticks=TRACK_ABORT, drift_abort_ticks=DRIFT_ABORT, hold_drift_ticks=HOLD_DRIFT, return_tolerance_ticks=RETURN_TOLERANCE, current="0.5 x the motor's own Protection_Current register"),
               rows=[], joints={}, events=[], result=None)

    def finish(result):
        log["result"], log["exit_code"], log["finished"] = result, EXIT[result], time.strftime("%Y-%m-%dT%H:%M:%S%z")
        (out / "commission.json").write_text(json.dumps(log, indent=1, default=str))
        return EXIT[result], log

    who = owner(port)
    if who:
        log["refused"] = f"{port} is held by process {who}: not opened"
        return finish("REFUSED")
    mp = (mapping or rc.load_mapping(IN / "CANDIDATE_Q12_CONFIG.json", CAL, IN / "ASSEMBLED_ZERO_AND_PGRIPPER.json"))[side]
    try:
        bus = open_bus(port)
        bus.connect()
    except Exception as e:
        log["refused"] = f"the bus could not be opened: {type(e).__name__}: {e}"
        return finish("REFUSED")
    enabled, goal_before, start = [], None, None

    def read():
        r = dict(t=time.time())
        for k, reg in READS:
            r[k] = {n: float(v) for n, v in bus.sync_read(reg, normalize=False).items()}
        return r

    def goal(n, tick):
        bus.write("Goal_Position", n, int(round(tick)), normalize=False); log["writes"]["Goal_Position"] += 1

    def torque(n, v):
        bus.write("Torque_Enable", n, int(v), normalize=False); log["writes"]["Torque_Enable"] += 1

    result = None
    try:
        # ---- checks before the first write: a refusal here changes nothing on the device --------------------------
        try:
            prot = {n: float(v) for n, v in bus.sync_read("Protection_Current", normalize=False).items()}
            log["device"] = dict(Protection_Current=prot, Min_Position_Limit={n: float(v) for n, v in bus.sync_read("Min_Position_Limit", normalize=False).items()}, Max_Position_Limit={n: float(v) for n, v in bus.sync_read("Max_Position_Limit", normalize=False).items()})
            for n in rc.JOINTS:
                if (log["device"]["Min_Position_Limit"][n], log["device"]["Max_Position_Limit"][n]) != (mp[n]["lo"], mp[n]["hi"]):
                    raise Refuse(f"{n}: device position limits differ from the calibration file")
            pre = [read() for _ in range(15)]
            P = np.array([[r["present"][n] for n in rc.JOINTS] for r in pre])
            if any(r["torque"][n] != 0 for r in pre for n in rc.JOINTS):
                raise Refuse("torque is already on before commissioning: left as it is")
            if np.ptp(P, axis=0).max() > 1:
                raise Refuse("the arm is not at rest before commissioning")
            start = dict(zip(rc.JOINTS, np.median(P, axis=0).tolist()))
            goal_before = dict(pre[-1]["goal"])
            for n in joints:
                d = 1 if (mp[n]["lo"] + mp[n]["hi"]) / 2 > start[n] else -1
                why = rc.check_command({side: mp}, side, n, start[n] + d * delta, start[n], margin_ticks=0, max_step_ticks=delta)
                if why:
                    raise Refuse(f"{n}: the planned move is not allowed: {why}")
            log["start"] = dict(ticks=start, goal_register_before=goal_before, q_candidate={n: rc.tick_to_model(n, mp[n], start[n]) for n in rc.JOINTS})
        except Refuse:
            raise
        except Exception as e:
            raise Refuse(f"the state could not be read before the first write: {type(e).__name__}: {e}")
        assert log["writes"] == dict(Goal_Position=0, Torque_Enable=0)
        # ---- 1. goal = present while torque is still off, read back, then torque on --------------------------------
        for n in rc.JOINTS:
            goal(n, start[n])
        back = read()
        if any(back["goal"][n] != round(start[n]) for n in rc.JOINTS):
            raise Abort(f"goal read-back differs from the present position: {back['goal']}")
        for n in rc.JOINTS:
            enabled.append(n)                                                 # counted before the write: an uncertain write is treated as 'may be on'
            torque(n, 1)
        hold = []
        for _ in range(24):
            hold.append(read()); sleep(PERIOD)
        H = np.array([[r["present"][n] for n in rc.JOINTS] for r in hold])
        log["hold_after_torque_on"] = dict(samples=len(hold), max_abs_drift_ticks=dict(zip(rc.JOINTS, np.abs(H - np.array([start[n] for n in rc.JOINTS])).max(axis=0).tolist())))
        if np.abs(H - np.array([start[n] for n in rc.JOINTS])).max() > HOLD_DRIFT:
            raise Abort("the arm moved when torque was switched on")
        log["events"].append("torque on at the present pose; pose held")

        def cycle(n, g, phase):
            why = rc.check_command({side: mp}, side, n, g, start[n], margin_ticks=0, max_step_ticks=delta)
            if why:
                raise Abort(f"{n}: command refused: {why}")
            t0 = time.time(); goal(n, g); r = read(); r.update(joint=n, phase=phase, commanded=g); log["rows"].append(r)
            if abs(r["present"][n] - g) > TRACK_ABORT:
                raise Abort(f"{n}: tracking error {r['present'][n] - g:+.0f} ticks")
            for o in rc.JOINTS:
                if o != n and abs(r["present"][o] - start[o]) > DRIFT_ABORT:
                    raise Abort(f"{o} drifted {r['present'][o] - start[o]:+.0f} ticks while {n} moved")
                if r["current"][o] > 0.5 * prot[o]:
                    raise Abort(f"{o}: current {r['current'][o]:.0f} above half of its protection current {prot[o]:.0f}")
            sleep(max(0.0, PERIOD - (time.time() - t0)))
            return r

        for n in joints:
            d = 1 if (mp[n]["lo"] + mp[n]["hi"]) / 2 > start[n] else -1
            i0 = len(log["rows"])
            for k in range(1, delta + 1):
                cycle(n, start[n] + d * k, "out")
            for _ in range(20):
                cycle(n, start[n] + d * delta, "hold_out")
            for k in range(delta - 1, -1, -1):
                cycle(n, start[n] + d * k, "back")
            for _ in range(15):
                cycle(n, start[n], "hold_start")
            rows = log["rows"][i0:]
            ho = [r for r in rows if r["phase"] == "hold_out"][-8:]; hs = [r for r in rows if r["phase"] == "hold_start"][-8:]
            err = np.array([r["present"][n] - r["commanded"] for r in rows])
            log["joints"][n] = dict(direction_ticks=d, commanded_out=start[n] + d * delta, reached_out=float(np.median([r["present"][n] for r in ho])), returned=float(np.median([r["present"][n] for r in hs])), start=start[n], moved_ticks=float(np.median([r["present"][n] for r in ho]) - start[n]),
                                    tracking_error_ticks=dict(min=float(err.min()), max=float(err.max()), at_hold_out=float(np.median([r["present"][n] - r["commanded"] for r in ho])), at_hold_start=float(np.median([r["present"][n] - r["commanded"] for r in hs]))),
                                    current_max=float(max(r["current"][n] for r in rows)), load=dict(min=float(min(r["load"][n] for r in rows)), max=float(max(r["load"][n] for r in rows)), hold_out=float(np.median([r["load"][n] for r in ho]))),
                                    others_max_abs_drift=float(max(abs(r["present"][o] - start[o]) for r in rows for o in rc.JOINTS if o != n)), cycles=len(rows))
        result = "COMPLETED"
    except Refuse as e:
        log["refused"] = str(e)
        result = "REFUSED"
    except BaseException as e:                                                # Abort, bus errors, KeyboardInterrupt: stop procedure
        log["abort_reason"] = f"{type(e).__name__}: {e}"
        rec = dict(held=False, steps=0, refused_steps=[], error=None)
        try:
            if enabled:
                now = read()
                for n in enabled:
                    goal(n, now["present"][n])                                # hold where it is
                rec["held"] = True
                for _ in range(80):                                           # walk back at 1 tick per cycle, bounded, every step checked
                    now = read()
                    left = {n: start[n] - now["goal"][n] for n in enabled}
                    if all(abs(v) < 1 for v in left.values()):
                        break
                    for n, v in left.items():
                        if abs(v) >= 1:
                            g = now["goal"][n] + float(np.sign(v))
                            why = rc.check_command({side: mp}, side, n, g, now["present"][n], margin_ticks=0, max_step_ticks=TRACK_ABORT)
                            if why:
                                rec["refused_steps"].append(dict(joint=n, goal=g, reasons=why)); continue
                            goal(n, g)
                    rec["steps"] += 1
                    sleep(PERIOD)
                for _ in range(10):
                    sleep(PERIOD)
            elif goal_before is not None and log["writes"]["Goal_Position"]:
                for n in rc.JOINTS:                                           # refused after the goal writes, before torque: put the goal register back
                    goal(n, goal_before[n])
                rec["goal_register_restored"] = True
        except BaseException as e2:
            rec["error"] = f"{type(e2).__name__}: {e2}"
        log["stop_procedure"] = rec
        result = "ABORTED"
    # ---- release: only what this run may have switched on; each motor on its own; confirmed by a read -----------------
    release = dict(attempted=list(enabled), failures=[], confirmed=None)
    for n in enabled:
        for k in range(RELEASE_TRIES):
            try:
                torque(n, 0)
                break
            except BaseException as e:
                release["failures"].append(dict(joint=n, attempt=k + 1, error=f"{type(e).__name__}: {e}"))
                sleep(0.05)
    try:
        sleep(0.3 if enabled else 0.0)
        end = [read() for _ in range(10)]
        release["confirmed"] = all(v == 0 for v in end[-1]["torque"].values()) if enabled or result != "REFUSED" else None
        log["end"] = dict(torque=end[-1]["torque"], ticks={n: float(np.median([r["present"][n] for r in end])) for n in rc.JOINTS}, goal_register=end[-1]["goal"])
        if start is not None:
            log["end"]["minus_start"] = {n: log["end"]["ticks"][n] - start[n] for n in rc.JOINTS}
            log["end"]["returned"] = bool(max(abs(v) for v in log["end"]["minus_start"].values()) <= RETURN_TOLERANCE)
        if result == "REFUSED":
            log["end"]["state_preserved"] = bool(log["writes"] == dict(Goal_Position=0, Torque_Enable=0) and (goal_before is None or end[-1]["goal"] == goal_before))
    except BaseException as e:
        log["end"] = dict(error=f"the end state could not be read: {type(e).__name__}: {e}")
        release["confirmed"] = False if enabled else None
    log["release"] = release
    try:
        bus.disconnect(disable_torque=False)
    except BaseException as e:
        log["events"].append(f"disconnect: {type(e).__name__}: {e}")
    if enabled and release["confirmed"] is not True:
        log["needs_a_person"] = "torque release was NOT confirmed on every motor: check the arm and power before anything else"
        return finish("TORQUE_RELEASE_UNCONFIRMED")
    if result == "ABORTED":
        return finish("ABORTED_RETURNED" if log.get("end", {}).get("returned") else "ABORTED_NOT_RETURNED")
    return finish(result)


def main(argv=None):
    ap = argparse.ArgumentParser(); ap.add_argument("side", choices=("left", "right")); ap.add_argument("out"); ap.add_argument("--delta", type=int, default=20); ap.add_argument("--joints", default=",".join(rc.JOINTS)); a = ap.parse_args(argv)
    sys.exit("archived: use sweepick_move_a.py (trajectory-validated). This module is kept for its record and tests only")
    code, log = run(a.side, a.out, delta=a.delta, joints=a.joints.split(","))
    print(json.dumps({k: log.get(k) for k in ("side", "result", "exit_code", "refused", "abort_reason", "writes", "stop_procedure", "release", "hold_after_torque_on", "joints", "end", "events", "needs_a_person")}, indent=1, default=str))
    return code


if __name__ == "__main__":
    sys.exit(main())
