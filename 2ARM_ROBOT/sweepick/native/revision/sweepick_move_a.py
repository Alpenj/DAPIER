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
heuristics (not gates) decide between holding and releasing them. Bus lost -> no further actuator command; one ordered
read-only look records whatever state can still be confirmed before the port closes.
No automatic return.

usage: sweepick_move_a.py plan|out|return SIDE OUT_DIR --target joint=tick[,...] | --clear-range-end JOINT:TICKS
                     [--level device|model] [--cert FILE] [--rest joint=tick,...] [--other-arm joint=tick,...] [--execute]"""
import argparse, hashlib, json, sys, time
from pathlib import Path
import numpy as np
import sweepick_real_command as rc, sweepick_trajectory as tt, sweepick_gates as G, sweepick_progress as PG
from sweepick_commission import IN, CAL, READS, RELEASE_TRIES, real_bus, port_owner

# RECOVERY heuristics: used only AFTER a hard stop to choose between holding and releasing. They never start, allow or
# stop a normal move.
SETTLE_CYCLES, STEADY_TICKS = 15, 3
POST_READS = 15         # fewest reads after the last command; reading goes on while a joint still advances (progress_profile start_delay_s without progress ends it)
POSTWRITE_READS = (("goal", "Goal_Position"), ("torque", "Torque_Enable"), ("status", "Status"), ("moving", "Moving"),
                   ("velocity", "Present_Velocity"), ("load", "Present_Load"), ("current", "Present_Current"), ("present", "Present_Position"))
# exit 0 only when the measured state is what the mode promises: HOLDING = target reached and holding; RETURNED_RELEASED =
# return target reached AND torque release confirmed.
EXIT = dict(EXECUTOR_EXCEPTION=3, PLANNED=0, HOLDING=0, GRIP_CONTACT_HOLDING=0, GRIP_NO_CONTACT_HOLDING=9, GRIP_UNAVAILABLE_HOLDING=9, RETURNED_RELEASED=0, REFUSED=2, RELEASE_UNCONFIRMED=5, TARGET_NOT_REACHED_HOLDING=6, RETURN_NOT_CONFIRMED_HOLDING=7, NO_PROGRESS_HOLDING=8, STOPPED_HOLDING=10, FAULT_HOLDING=13, FAULT_JOINTS_RELEASED=14, BUS_LOST=15)
EXIT['STOPPED_STATE_RECORDED'] = 16


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


def _fresh_arm_goal_matches(feedback, writes_after_read, joint, requested):
    return (joint != "gripper" and joint not in writes_after_read and feedback is not None
            and feedback["torque"][joint] == 1 and feedback["goal"][joint] == requested)


def write_goal_position(bus, log, joint, requested):
    """The existing unicast Goal write and ACK contract, also used by the seed-only field entry."""
    log["last_goal_write_attempt"] = dict(joint=joint, requested_tick=requested, acknowledgement="UNKNOWN")
    try:
        bus.write("Goal_Position", joint, requested, normalize=False)
        log["writes"]["Goal_Position"] += 1
        log["last_goal_write_attempt"]["acknowledgement"] = "ACK"
        log.setdefault("last_acknowledged_goal", {})[joint] = requested
    except Exception as e:
        raise Stop(G.hard("bus_lost", f"write failed: {type(e).__name__}: {str(e).splitlines()[0]}")) from e


def read_feedback_once(bus, reads=POSTWRITE_READS):
    """One ordered cycle; retain completed registers when a later read fails. No retry."""
    record = dict(answered=False, reads=[], attempted_reads=[], state={}, started_monotonic_s=time.monotonic())
    for key, register in reads:
        record["attempted_reads"].append(register)
        try:
            record["state"][key] = {n: float(v) for n, v in bus.sync_read(register, normalize=False).items()}
            if set(record["state"][key]) != set(rc.JOINTS):
                raise ValueError(f"{register}: incomplete six-axis response")
            if not all(np.isfinite(v) for v in record["state"][key].values()):
                raise ValueError(f"{register}: non-finite response")
            record["reads"].append(register)
        except BaseException as e:
            record.update(error=f"{type(e).__name__}: {str(e).splitlines()[0] if str(e) else ''}", failed_register=register)
            break
    else:
        record["answered"] = True
    record["finished_monotonic_s"] = time.monotonic()
    return record


def record_read_after_loss(bus, log):
    """Post-loss termination record: observe once without changing BUS_LOST or sending recovery commands."""
    record = read_feedback_once(bus)
    state = record["state"]
    torque = state.get("torque", {})
    torque_known = set(torque) == set(rc.JOINTS) and all(np.isfinite(v) for v in torque.values())
    on_now = [n for n in rc.JOINTS if torque.get(n)] if torque_known else None
    record["torque_on"] = on_now if torque_known else "UNKNOWN"
    last = log.get("last_goal_write_attempt") or {}
    if last.get("acknowledgement") == "UNKNOWN":
        observed = state.get("goal", {}).get(last["joint"], "UNKNOWN")
        record["unanswered_goal_write"] = dict(joint=last["joint"], requested_tick=last["requested_tick"],
            goal_register_after=observed, register_equals_request=None if observed == "UNKNOWN" else observed == last["requested_tick"])
    log["read_after_loss"] = record
    log["end"] = dict(error="an answer was lost; state from one read-only look after the loss", torque=torque if torque else "UNKNOWN") if state else dict(error="no read possible", torque="UNKNOWN")
    for key, end_key in (("goal", "goal_register"), ("present", "ticks"), ("status", "status"), ("moving", "moving"), ("velocity", "velocity")):
        if key in state:
            log["end"][end_key] = state[key]
    if torque_known:
        log["end"]["torque_on"] = on_now
        log["needs_a_person"] = (f"an answer was lost; torque is ON at {on_now} (read, not assumed). "
                                 "Nothing was retransmitted or released; support the arm and decide the release by hand")
    else:
        log["needs_a_person"] = "an answer was lost; torque is NOT known. Nothing was retransmitted or released; support the arm and cut motor power by hand"
    log["outcome"]["torque_released"] = not on_now if torque_known else "UNKNOWN"
    return record


def no_progress_gate(stuck, cmd, row, traj, monitors, profile, rows):
    """Record the elapsed clocks that actually produced NO_PROGRESS; this does not alter its judgement."""
    update_s = row["t"] + row["read_s"]
    timing = {}
    for name in stuck:
        mark_row = next((sample for sample in reversed(rows) if sample["t"] + sample["read_s"] == monitors[name].t_mark), None)
        timing[name] = dict(
            last_monitor_reset_read_started_s=None if mark_row is None else mark_row["t"],
            no_progress_read_started_s=row["t"],
            elapsed_read_started_s=None if mark_row is None else row["t"] - mark_row["t"],
            monitor_t_mark_s=monitors[name].t_mark,
            monitor_waited_since_s=monitors[name].waited_since,
            no_progress_monitor_update_s=update_s,
            elapsed_since_t_mark_s=update_s - monitors[name].t_mark,
            elapsed_since_waited_since_s=None if monitors[name].waited_since is None else update_s - monitors[name].waited_since,
        )
    read_elapsed = {name: None if timing[name]["elapsed_read_started_s"] is None else round(timing[name]["elapsed_read_started_s"], 6) for name in stuck}
    monitor_elapsed = {name: round(timing[name]["elapsed_since_t_mark_s"], 6) for name in stuck}
    return G.contract(
        "no_progress",
        f"{stuck} stayed more than {profile['follow']:.0f} ticks behind the command when NO_PROGRESS was reported; "
        f"actual elapsed since the last monitor reset (read-start timestamps): {read_elapsed} s; "
        f"monitor-update elapsed since t_mark: {monitor_elapsed} s",
        joints=stuck,
        commanded={name: round(cmd[name]) for name in stuck},
        measured={name: row["present"][name] for name in stuck},
        target={name: traj["target"][name] for name in stuck},
        timing=timing,
    )


def run(mode, side, out, *, target=None, clear_range_end=None, rest=None, cert=None, level="device", execute=False, other_arm_ticks=None, open_bus=real_bus, owner=port_owner, mapping=None, mapping_sha="test", profile=None,
        progress=None, torque_joints=None, settle_trim=(), trim_check=None, start_recheck=None, hold_s=0.0, sleep=time.sleep, clock=time.time, stop_requested=lambda: False,
        contact_observer=None, observation_view=False, other_arm_read=None, range_entry=True, read_only_fault_recovery=False):
    """torque_joints: the joints whose torque this run switches on (out) / expects on (return). Default: ONLY the joints
    that move. No other joint of this arm, and nothing of the other arm, is powered implicitly; holding more joints (an arm
    move against gravity) must be asked for explicitly by naming them."""
    out = Path(out); out.mkdir(parents=True, exist_ok=False)
    port = f"/dev/dapier/{side}_arm"
    prof = profile or tt.load_profile()
    pg = progress or PG.load()
    log = dict(schema="tjj.move.v3", mode=mode, side=side, level=level, port=port, execute=bool(execute), started=time.strftime("%Y-%m-%dT%H:%M:%S%z"), writes=dict(Goal_Position=0, Torque_Enable=0), unchanged_arm_goal_writes_avoided=0, rows=[], events=[], profile=prof, total_travel_limit=None,
               mapping="CANDIDATE (verified=false)", diagnostics=[], progress_profile=pg, outcome=dict(command_completed=False, target_reached=None, return_confirmed=None, torque_released=None))
    observer_scope = None

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
    def make_trajectory(start, target):
        command_start = dict(start)
        if range_entry:
            for name in set(target) | (set(torque_joints or target) - set(on)):
                if start[name] < mp[name]["lo"] or start[name] > mp[name]["hi"]:
                    if name in on:
                        applied = last_feedback["goal"][name]
                        if applied < mp[name]["lo"] or applied > mp[name]["hi"]:
                            raise Stop(G.hard("command_malformed", f"the powered joint {name} has an out-of-range current Goal_Position; no goal is inferred"))
                        command_start[name] = applied
                    else:
                        command_start[name] = mp[name]["lo"] if start[name] < mp[name]["lo"] else mp[name]["hi"]
        return tt.make(start, target, prof, **({"command_start": command_start} if command_start != start else {}))
    try:
        bus = open_bus(port); bus.connect()
    except Exception as e:
        refuse(G.hard("bus_lost", f"the bus could not be opened: {type(e).__name__}: {str(e).splitlines()[0]}")); return finish("REFUSED")

    last_feedback = None
    writes_after_read = set()

    def read():
        nonlocal last_feedback
        """One fresh readback. Any failure of the driver (its own packet timeout included) is a bus loss."""
        t0 = clock(); r = {}
        try:
            reads = POSTWRITE_READS if writes_after_read else READS + (("status", "Status"), ("moving", "Moving"))
            for k, reg in reads:
                r[k] = {n: float(v) for n, v in bus.sync_read(reg, normalize=False).items()}
        except Exception as e:
            raise Stop(G.hard("bus_lost", f"read failed: {type(e).__name__}: {str(e).splitlines()[0]}"))
        r["t"], r["read_s"] = t0, clock() - t0
        last_feedback = r
        writes_after_read.clear()
        return r

    def goal(n, tick, stop_hold=False):
        if stop_requested() and not stop_hold:
            raise Stop(G.hard("operator_stop", "stop requested before Goal_Position write"))
        requested = int(round(tick))
        if _fresh_arm_goal_matches(last_feedback, writes_after_read, n, requested):
            log["unchanged_arm_goal_writes_avoided"] += 1
            return
        writes_after_read.add(n)
        write_goal_position(bus, log, n, requested)

    def torque(n, v):
        if stop_requested():
            raise Stop(G.hard("operator_stop", "stop requested before Torque_Enable write"))
        # A lost acknowledgement may happen after the servo applied the write.
        # Reuse the existing BUS_LOST handling; never claim torque is OFF/ON here.
        writes_after_read.add(n)
        try:
            bus.write("Torque_Enable", n, int(v), normalize=False)
            log["writes"]["Torque_Enable"] += 1
        except Exception as e:
            log["events"].append({"torque_write_acknowledgement": "UNKNOWN", "joint": n,
                                  "requested": int(v), "error": f"{type(e).__name__}: {e}"})
            raise Stop(G.hard("bus_lost", f"torque write failed: {type(e).__name__}: {str(e).splitlines()[0]}")) from e

    def release(names):                                                       # RECOVERY: confirmed by a read; lost write ACK aborts
        rec = dict(attempted=list(names), failures=[], confirmed=None)
        for n in names:
            for k in range(RELEASE_TRIES):
                try:
                    torque(n, 0); break
                except BaseException as e:
                    rec["failures"].append(dict(joint=n, attempt=k + 1, error=f"{type(e).__name__}: {e}"))
                    if isinstance(e, Stop) and e.gate.get("gate") in ("bus_lost", "operator_stop"):
                        log["release"] = rec
                        raise
                    sleep(0.05)
        try:
            t = read()["torque"]; rec["confirmed"] = all(t[n] == 0 for n in names); rec["torque_now"] = t
        except BaseException as e:
            rec["confirmed"] = False; rec["error"] = f"{type(e).__name__}: {e}"
        return rec

    def close(result):
        try:
            end = read()
            log["end"] = dict(ticks=end["present"], torque=end["torque"], status=end["status"], moving=end["moving"], velocity=end["velocity"], goal_register=end["goal"], torque_on=[n for n in rc.JOINTS if end["torque"][n]])
        except BaseException as e:
            log["end"] = dict(error=f"the end state could not be read: {type(e).__name__}: {e}")
        try:
            bus.disconnect(disable_torque=False)
        except BaseException as e:
            log["events"].append(f"disconnect: {type(e).__name__}: {e}")
        return finish(result)

    def trajectory_doc(traj, check, certificate=None):
        doc = dict(side=side, level=level, mapping_sha256=mapping_sha, profile=prof, identity=check["identity"], other_arm_ticks=other_arm_ticks, trajectory=traj, current_feedback=last_feedback)
        if certificate is not None:
            doc["certificate"] = certificate
        return doc

    def save_trajectory(traj, check, certificate=None):
        try:
            (out / "TRAJECTORY.json").write_text(json.dumps(trajectory_doc(traj, check, certificate), indent=1))
        except (TypeError, ValueError) as exc:
            raise Stop(G.provisional("certificate_binding", f"the trajectory certificate cannot be recorded: {type(exc).__name__}: {exc}")) from exc

    def recheck_trajectory(traj, why):
        nonlocal other_arm_ticks
        if other_arm_read is not None:
            try:
                other_arm_ticks = other_arm_read()
            except Stop:
                raise
            except Exception as exc:
                raise Stop(G.hard("bus_lost", f"other-arm actual recheck failed: {type(exc).__name__}: {exc}")) from exc
        unchecked = tt.validate(side, mp, traj, prof, mapping_sha=mapping_sha, level=level, other_arm_ticks=other_arm_ticks)
        if unchecked["hard"]:
            raise Stop(unchecked["hard"][0])
        blocking = [gate for gate in unchecked["provisional"] if gate["gate"] != "collision_candidate_frame"]
        if blocking:
            raise Stop(blocking[0])
        try:
            checked_cert = None if start_recheck is None else start_recheck(trajectory_doc(traj, unchecked))
        except Exception as exc:
            raise Stop(G.provisional("certificate_binding", f"{why}; path re-check failed: {type(exc).__name__}: {exc}")) from exc
        if not isinstance(checked_cert, dict):
            raise Stop(G.provisional("certificate_binding", f"{why}; no valid re-check result for this trajectory was given"))
        try:
            checked = tt.validate(side, mp, traj, prof, mapping_sha=mapping_sha, level=level, cert=checked_cert, other_arm_ticks=other_arm_ticks)
        except Exception as exc:
            raise Stop(G.provisional("certificate_binding", f"{why}; the re-check result is malformed: {type(exc).__name__}: {exc}")) from exc
        save_trajectory(traj, checked, checked_cert)
        if checked["hard"]:
            raise Stop(checked["hard"][0])
        if checked["provisional"]:
            raise Stop(checked["provisional"][0])
        return checked, checked_cert

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
        traj = make_trajectory(now, target)
        if not traj["moving"] and mode in ("out", "next"):                      # the arm already stands on the target ticks: there is no path to check and nothing to send
            if mode == "next" and execute and set(torque_joints) <= set(on):
                log.update(start=dict(ticks=now, torque_on=on, goal_register=fresh["goal"], status=fresh["status"]), trajectory={k: v for k, v in traj.items() if k != "goals"}, torque_scope=dict(joints=list(torque_joints)))
                log["events"].append("already at the target: nothing was written"); log["outcome"].update(command_completed=True, target_reached=True, torque_released=False)
                log["reached"] = {}
                if observation_view:
                    settled = all(max(r["present"][n] for r in pre) - min(r["present"][n] for r in pre) <= pg["resolution"]
                                  and not any(r["moving"][n] or r["velocity"][n] for r in pre) for n in target)
                    log["observation_view"] = dict(nominal_target_reached=True, command_completed=True, actual_settled=settled,
                        command_state_resolved=all(abs(fresh["goal"][n] - now[n]) <= pg["follow"] for n in target),
                        current_support=None, fresh_observation_acquired=False, insertion_ready=False, actual_ticks=now,
                        nominal_target=target, endpoint_contract_unchanged=True, trim_policy="no_integral_trim")
                return close("HOLDING")
            raise Stop(G.hard("command_malformed", "the target equals the measured position: nothing to move"))
        powered = [n for n in rc.JOINTS if n in (torque_joints or traj["moving"])] if mode != "return" else list(on)
        log["torque_scope"] = dict(joints=powered, not_powered=[n for n in rc.JOINTS if n not in powered], other_arm="not opened by this run", rule="only these joints are switched on / commanded; the others keep the torque state they have")
        check = tt.validate(side, mp, traj, prof, mapping_sha=mapping_sha, level=level, cert=cert, other_arm_ticks=other_arm_ticks)
        log.update(start=dict(ticks=now, torque_on=on, goal_register=fresh["goal"], status=fresh["status"]), trajectory={k: v for k, v in traj.items() if k != "goals"}, validation=check)
        if contact_observer is not None:
            observer_scope = getattr(contact_observer, "scope", None)
            required = ("side", "joint", "direction", "target_tick", "observe_s", "baseline_file", "baseline_sha256", "baseline_max_lead_ticks", "profile_sha256", "mapping_sha256")
            malformed = [] if callable(contact_observer) and isinstance(observer_scope, dict) else ["callable contact_observer with a dict .scope"]
            malformed += [k for k in required if not isinstance(observer_scope, dict) or k not in observer_scope]
            numeric = ("direction", "target_tick", "observe_s", "baseline_max_lead_ticks")
            malformed += [k for k in numeric if isinstance(observer_scope, dict) and (isinstance(observer_scope.get(k), bool) or not isinstance(observer_scope.get(k), (int, float)) or not np.isfinite(observer_scope[k]))]
            actual_direction = int(np.sign(traj["target"].get("gripper", now["gripper"]) - now["gripper"]))
            if (malformed or mode not in ("out", "next") or side != "right" or traj["moving"] != ["gripper"]
                    or observer_scope.get("side") != side or observer_scope.get("joint") != "gripper"
                    or observer_scope.get("direction") != actual_direction or actual_direction != -1
                    or observer_scope.get("target_tick") != traj["target"].get("gripper")
                    or observer_scope.get("observe_s", 0) <= 0 or observer_scope.get("baseline_max_lead_ticks", -1) < 0
                    or observer_scope.get("profile_sha256") != prof.get("sha256")
                    or observer_scope.get("mapping_sha256") != mapping_sha):
                raise Stop(G.hard("command_malformed", "contact_observer is valid only for the bound right-gripper-only closing run with the same target, direction, motion profile and mapping as its measured baseline"
                                  + (f"; malformed: {sorted(set(malformed))}" if malformed else "")))
            log["contact_observer"] = dict(observer_scope, classification="PROVISIONAL measured free-closing lead; not a HARD gate", support_confirmed=False, grasp_confirmed=False)
        save_trajectory(traj, check)
        if mode == "plan" or not execute:
            if cert is not None:
                save_trajectory(traj, check, cert)
            log["events"].append("the device was read, nothing was written")
            return close("PLANNED")
        if check["hard"]:
            raise Stop(check["hard"][0])
        blocking = [gate for gate in check["provisional"] if gate["gate"] != "certificate_binding"]
        if blocking:
            raise Stop(blocking[0])
        if check["provisional"]:
            check, cert = recheck_trajectory(traj, "the measured start differs from the trajectory certificate")
            log["events"].append(dict(start_certificate_rechecked=True, collision=cert.get("nominal_verdict")))
            log["validation"] = check
        else:
            save_trajectory(traj, check, cert)
    except Stop as e:
        refuse(e.gate); return close("REFUSED") if e.gate["gate"] != "bus_lost" else finish("REFUSED")
    assert log["writes"] == dict(Goal_Position=0, Torque_Enable=0)

    moving = traj["moving"]
    direction = {n: (int(np.sign(traj["target"][n] - now[n])) if n in moving else 0) for n in rc.JOINTS}
    if stop_requested():
        refuse(G.hard("operator_stop", "stop requested before the first actuator write"))
        return close("REFUSED")
    try:
        try:
            new = [n for n in powered if n not in on] if mode in ("out", "next") else []
            recheck_started, recheck_cost_s, recheck_calls = clock(), 0.0, 0
            # Software work budget: reuse Owner's existing idle budget; each check costs at least one profile cycle.
            recheck_budget_s = None
            while True:
                hold = read()                                                 # read again immediately before the first write
                faults = {n: v for n, v in hold["status"].items() if v}
                if faults:
                    raise Stop(G.hard("servo_fault", f"Status register non-zero before the first actuator write: {faults}", joints=sorted(faults)))
                current_on = [n for n in rc.JOINTS if hold["torque"][n]]
                if current_on != on:
                    raise Stop(G.hard("torque_state", "torque ownership changed before the first actuator write; no seed or trajectory is applied"))
                updated = make_trajectory(hold["present"], {n: v for n, v in traj["target"].items()})
                if hold["present"] == now and updated.get("command_start") == traj.get("command_start"):
                    break
                moved_since_plan = {n: hold["present"][n] - now[n] for n in rc.JOINTS if hold["present"][n] != now[n]}
                previous_seed = traj.get("command_start")
                now = dict(hold["present"]); traj = updated
                if previous_seed != traj.get("command_start"):
                    log["events"].append(dict(applied_goal_seed_changed=True, previous=previous_seed, current=traj.get("command_start")))
                if level == "model":                                          # a held arm's reading changes: the exact path from where the arm IS now is checked again before any write
                    if recheck_budget_s is None:
                        recheck_budget_s = 100 * float(json.loads(JAW_PROFILE.read_text())["confirm_s"]["value"])
                    spent = max(clock() - recheck_started, recheck_cost_s)
                    log["prewrite_recheck_work"] = dict(budget_s=recheck_budget_s, spent_s=spent, calls=recheck_calls,
                        cost_basis="measured checker time, at least one selected profile dt per call; existing Owner idle budget=100*Jaw confirm_s",
                        classification="SOFTWARE_OPERATION_BUDGET; no physical/tick tolerance", latest_actual=dict(now))
                    if spent >= recheck_budget_s:
                        log["prewrite_recheck_work"]["cancelled"] = True
                        raise Stop(G.provisional("certificate_binding", "current start did not stay bound within the software recheck work budget; no actuator write or automatic OFF"))
                    check_started = clock()
                    recheck, cert = recheck_trajectory(traj, f"the arm is not where the collision result was computed for ({moved_since_plan})")
                    recheck_calls += 1
                    recheck_cost_s += max(clock() - check_started, float(prof["dt"]))
                    log["prewrite_recheck_work"].update(calls=recheck_calls, spent_s=max(clock() - recheck_started, recheck_cost_s))
                    log["events"].append(dict(start_changed_since_plan=moved_since_plan, rechecked=True, collision=cert.get("nominal_verdict")))
                else:
                    recheck = tt.validate(side, mp, traj, prof, mapping_sha=mapping_sha, level=level)
                    save_trajectory(traj, recheck)
                log["events"].append("the trajectory was rebuilt from the read taken immediately before the first write")
                log.update(start=dict(log["start"], ticks=now), trajectory={k: v for k, v in traj.items() if k != "goals"}, validation=recheck)
                if recheck["hard"]:
                    raise Stop(recheck["hard"][0])
                if recheck["provisional"]:
                    raise Stop(recheck["provisional"][0])
                direction = {n: (int(np.sign(traj["target"][n] - now[n])) if n in moving else 0) for n in rc.JOINTS}
                if stop_requested():
                    raise Stop(G.hard("operator_stop", "stop requested before the first actuator write"))
                if level != "model":
                    break
            if stop_requested():
                raise Stop(G.hard("operator_stop", "stop requested before the first actuator write"))
            if mode in ("out", "next"):                                      # next: the joints already held stay as they are; only newly powered joints are seeded
                seed = traj.get("command_start", now)
                for n in new:                                                 # A seed write may power the servo; use only its checked, range-valid command.
                    goal(n, seed[n])
                back = read()
                faults = {n: v for n, v in back["status"].items() if v}
                if faults:
                    raise Stop(G.hard("servo_fault", f"Status register non-zero during drive initialization: {faults}", joints=sorted(faults)))
                if any(back["goal"][n] != round(seed[n]) for n in new):
                    raise Stop(G.hard("bus_lost", "the first Goal_Position read-back differs; that write may already have powered the servo; no old goal is restored"))
                log["goal_seed"] = dict(joints=new, already_held=[n for n in powered if n in on], from_read_at=hold["t"], measured_start=dict(now), command_seed=dict(seed), readback={n: back["goal"][n] for n in new})
                log["drive_enable"] = dict(note="on this device a Goal_Position write has been seen to switch torque on by itself (2026-10-08: 5 goal writes acknowledged, no Torque_Enable write, torque read 1 on exactly those 5 joints; "
                                                "the driver's write() sends only address 42, so this is the servo, not the wrapper). The FIRST goal write of a joint is therefore treated as the write that may power it",
                                           torque_read_after_the_goal_write={n: back["torque"][n] for n in new}, explicit_torque_writes_follow=True)
                for n in new:
                    torque(n, 1)
                on_reads = [read() for _ in range(5)]
                log["drive_enable"]["actual_reads"] = on_reads
                faults = {n: v for r in on_reads for n, v in r["status"].items() if v}
                if faults:
                    raise Stop(G.hard("servo_fault", f"Status register non-zero after drive initialization: {faults}", joints=sorted(faults)))
                log["diagnostics"].append(G.diagnostic("torque_on_drift", "movement of the joints when torque was switched on (recorded)", ticks={n: max(abs(r["present"][n] - now[n]) for r in on_reads) for n in rc.JOINTS}))
            cmd, prev, k, waits = dict(traj.get("command_start", now)), None, 0, 0
            mon = {n: PG.Joint(pg, now[n], direction[n], clock()) for n in moving}
            while k < traj["steps"]:
                t0 = clock()
                waiting = [n for n in moving if mon[n].state == PG.WAIT]
                if not waiting:                                               # the command advances only while every moving joint is progressing or inside its normal start delay
                    for n in moving:
                        cmd[n] = traj["goals"][n][k]; goal(n, cmd[n])
                    k += 1
                else:
                    if stop_requested():
                        raise Stop(G.hard("operator_stop", "stop requested"))
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
                    raise Stop(no_progress_gate(stuck, cmd, r, traj, mon, pg, log["rows"]))
                if contact_observer is not None:
                    sample = dict(side=side, joint="gripper", actual_tick=float(r["present"]["gripper"]), requested_goal_tick=float(cmd["gripper"]),
                                  closing_lead_ticks=float(r["present"]["gripper"] - cmd["gripper"]), direction=direction["gripper"], raw=r)
                    try:
                        observed = contact_observer(sample)
                    except Exception as exc:
                        raise Stop(G.hard("command_malformed", f"contact observer failed: {type(exc).__name__}: {exc}")) from exc
                    if not isinstance(observed, bool):
                        raise Stop(G.hard("command_malformed", "contact observer returned a non-boolean result"))
                    if observed:
                        requested, actual0 = float(cmd["gripper"]), float(r["present"]["gripper"])
                        goal("gripper", actual0)                              # normal early collection: stop adding closing goal; hold the measured position
                        cmd["gripper"] = actual0
                        onset = dict(classification="CONTACT_CANDIDATE", level="PROVISIONAL", support_confirmed=False, grasp_confirmed=False,
                                     actual0_tick=actual0, requested_goal_tick=requested, owner_written_goal_tick=actual0,
                                     closing_lead_ticks=float(sample["closing_lead_ticks"]), baseline_max_lead_ticks=float(observer_scope["baseline_max_lead_ticks"]),
                                     target_reached=False, meaning="closing resistance exceeded the measured free-closing lead; this is neither verified object contact nor support")
                        t_observe, observed_rows = clock(), 0
                        while clock() - t_observe < observer_scope["observe_s"]:
                            if stop_requested():
                                raise Stop(G.hard("operator_stop", "stop requested"))
                            rr = read(); faults = {n: v for n, v in rr["status"].items() if v}
                            if faults:
                                raise Stop(G.hard("servo_fault", f"Status register non-zero: {faults}", joints=sorted(faults)))
                            rr.update(k=k, commanded={"gripper": actual0}, tracking_error={n: rr["present"][n] - round(cmd[n]) for n in rc.JOINTS},
                                      phase="contact_observation", fresh_raw=True, contact_candidate=True)
                            log["rows"].append(rr); observed_rows += 1
                            sleep(max(0.0, prof["dt"] - (clock() - rr["t"])))
                        onset.update(observe_s=float(clock() - t_observe), fresh_raw_rows=observed_rows)
                        log["contact_candidate"] = onset
                        log["outcome"].update(command_completed=False, target_reached=False, torque_released=False,
                                              contact_candidate=True, contact_collection_completed=True)
                        log["state_left"] = f"HOLDING at the measured contact-candidate position with torque on {powered}; only a planned normal return may open/release it"
                        return close("GRIP_CONTACT_HOLDING")
                sleep(max(0.0, prof["dt"] - (clock() - t0)))
            log["outcome"]["command_completed"] = True
            log["command_waits"] = waits
            def settle(monitors=None, monitor_directions=None):
                """after a command: read while a joint is still advancing, then return the reads"""
                settle_directions = {**direction, **(monitor_directions or {})}
                post = []
                best, t_best = {n: None for n in moving}, clock()
                while True:
                    r = read(); post.append(r)
                    for n in moving:
                        if best[n] is None or settle_directions[n] * (r["present"][n] - best[n]) >= pg["resolution"]:      # further in the active segment's direction, including a reversed trim
                            t_best = clock() if best[n] is not None else t_best; best[n] = r["present"][n]
                    faults = {n: v for n, v in r["status"].items() if v}
                    if faults:
                        raise Stop(G.hard("servo_fault", f"Status register non-zero: {faults}", joints=sorted(faults)))
                    r.update(k=k, commanded={n: cmd[n] for n in moving}, tracking_error={n: r["present"][n] - round(cmd[n]) for n in rc.JOINTS}, phase="post")
                    if monitors:
                        state = {n: monitors[n].update(r["t"] + r["read_s"], r["present"][n], monitor_directions[n] * (round(cmd[n]) - r["present"][n])) for n in monitors}
                        r["progress"] = state
                    log["rows"].append(r)
                    if monitors:
                        stuck = [n for n, state_n in state.items() if state_n == PG.NO_PROGRESS]
                        if stuck:
                            raise Stop(no_progress_gate(stuck, cmd, r, traj, monitors, pg, log["rows"]))
                    if stop_requested():
                        raise Stop(G.hard("operator_stop", "stop requested"))
                    if len(post) >= POST_READS and clock() - t_best > pg["start_delay"]:
                        return post
                    sleep(prof["dt"])
            post = settle(mon if settle_trim and not observation_view else None, direction)
            log["settle_reads"] = len(post)
            trims = []; log["settle_trim"] = trims
            while settle_trim and not observation_view:                                                # integral trim of a loaded arm joint that settled short (sweepick_progress_profile.json: settle_trim)
                here = {n: float(np.median([r["present"][n] for r in post[-5:]])) for n in moving}
                short = {n: traj["target"][n] - here[n] for n in moving if n in settle_trim and not PG.reached(pg, here[n], traj["target"][n])}
                stuck = bool(trims) and all(abs(short[n]) >= abs(trims[-1]["short"].get(n, np.inf)) for n in short)
                lead_after = {n: abs(cmd[n] + e - here[n]) for n, e in short.items()}
                if not short or (trims and not trims[-1].get("applied")) or (stuck and max(lead_after.values()) > pg["follow"]):
                    break                                                     # within tolerance; or the last trim could not be applied; or the joint has not moved and one more trim would put the goal further ahead than a loaded joint has ever needed to follow (progress profile: following lead)
                fresh_trim = read()
                faults = {n: v for n, v in fresh_trim["status"].items() if v}
                if faults:
                    raise Stop(G.hard("servo_fault", f"Status register non-zero before trim: {faults}", joints=sorted(faults)))
                if stop_requested():
                    raise Stop(G.hard("operator_stop", "stop requested before trim"))
                ordinal = len(trims) + 1; folder = out / f"trim_{ordinal:03d}"; folder.mkdir()
                seg_target = {n: float(round(cmd[n] + e)) for n, e in short.items()}
                seg = tt.make({n: float(round(cmd[n])) for n in rc.JOINTS}, seg_target, prof)
                actual_connection = tt.make(fresh_trim["present"], seg_target, prof)
                rec = dict(ordinal=ordinal, short={n: float(v) for n, v in short.items()}, goal=seg_target,
                           actual_start_ticks=dict(fresh_trim["present"]), command_start_ticks=dict(seg["start"]),
                           command_offset_ticks={n: seg["start"][n] - fresh_trim["present"][n] for n in rc.JOINTS},
                           steps=seg["steps"], applied=False, folder=str(folder), feedback_start=len(log["rows"]))
                trims.append(rec)
                (folder / "APPLICATION.json").write_text(json.dumps(rec, indent=1))
                try:
                    current_other = other_arm_ticks if other_arm_read is None else other_arm_read()
                except Stop:
                    raise
                except Exception as exc:
                    raise Stop(G.hard("bus_lost", f"other-arm actual read before trim failed: {type(exc).__name__}: {exc}")) from exc
                try:
                    if level == "model" and other_arm_read is None:
                        raise ValueError("fresh other-arm actual reader is required for the trim connection")
                    for label, path in (("ACTUAL_CONNECTION", actual_connection), ("TRAJECTORY", seg)):
                        doc = dict(side=side, level=level, mapping_sha256=mapping_sha, profile=prof,
                                   identity=tt.identity(side, path, mapping_sha), other_arm_ticks=current_other,
                                   trajectory=path, current_feedback=fresh_trim, trim_ordinal=ordinal, trim_path_role=label)
                        (folder / f"{label}.json").write_text(json.dumps(doc, indent=1))
                        verdict = None if trim_check is None else trim_check(doc)
                        validation = tt.validate(side, mp, path, prof, mapping_sha=mapping_sha, level=level,
                                                 cert=verdict, other_arm_ticks=current_other)
                        doc["certificate"] = verdict
                        (folder / f"{label}.json").write_text(json.dumps(doc, indent=1))
                        (folder / f"{label}.CHECK.json").write_text(json.dumps(dict(certificate=verdict, validation=validation), indent=1))
                        rec.update(hard=validation["hard"], provisional=validation["provisional"],
                                   collision=None if verdict is None else verdict.get("nominal_verdict"))
                        if validation["hard"] or validation["provisional"]:
                            break
                    else:
                        rec["checked"] = True
                except Exception as exc:
                    rec.update(check_error=f"{type(exc).__name__}: {exc}", checked=False)
                if not rec.get("checked"):
                    (folder / "APPLICATION.json").write_text(json.dumps(rec, indent=1)); break
                before_write = read()
                if any(before_write["status"].values()):
                    raise Stop(G.hard("servo_fault", f"Status register non-zero before trim write: {before_write['status']}"))
                if stop_requested():
                    raise Stop(G.hard("operator_stop", "stop requested before trim write"))
                try:
                    latest_other = current_other if other_arm_read is None else other_arm_read()
                except Stop:
                    raise
                except Exception as exc:
                    raise Stop(G.hard("bus_lost", f"other-arm actual read before trim write failed: {type(exc).__name__}: {exc}")) from exc
                rec["prewrite_other_actual"] = latest_other
                if before_write["present"] != fresh_trim["present"] or latest_other != current_other:
                    rec.update(checked=False, check_error="active/other actual changed during trim check; compensation cancelled without a write")
                    (folder / "APPLICATION.json").write_text(json.dumps(rec, indent=1)); break
                trim_direction = {n: int(np.sign(seg_target[n] - fresh_trim["present"][n])) for n in seg["moving"]}
                for n in seg["moving"]:
                    if mon[n].dir != trim_direction[n]:
                        mon[n] = PG.Joint(pg, fresh_trim["present"][n], trim_direction[n], fresh_trim["t"] + fresh_trim["read_s"])
                trim_mon = {n: mon[n] for n in seg["moving"]}
                i = 0
                try:
                    while i < seg["steps"]:
                        t0 = clock()
                        waiting = [n for n in trim_mon if trim_mon[n].state == PG.WAIT]
                        if not waiting:
                            for n in seg["moving"]:
                                cmd[n] = seg["goals"][n][i]; goal(n, cmd[n])
                            rec["applied"] = True; i += 1
                        elif stop_requested():
                            raise Stop(G.hard("operator_stop", "stop requested during trim"))
                        r = read()
                        faults = {n: v for n, v in r["status"].items() if v}
                        if faults:
                            raise Stop(G.hard("servo_fault", f"Status register non-zero: {faults}", joints=sorted(faults)))
                        state = {n: trim_mon[n].update(r["t"] + r["read_s"], r["present"][n], trim_direction[n] * (round(cmd[n]) - r["present"][n])) for n in trim_mon}
                        r.update(k=k, commanded={n: cmd[n] for n in moving}, tracking_error={n: r["present"][n] - round(cmd[n]) for n in rc.JOINTS},
                                 phase="trim", trim_ordinal=ordinal, trim_step=i, progress=state, command_waiting=bool(waiting))
                        log["rows"].append(r)
                        stuck = [n for n, state_n in state.items() if state_n == PG.NO_PROGRESS]
                        if stuck:
                            raise Stop(no_progress_gate(stuck, cmd, r, seg, trim_mon, pg, log["rows"]))
                        sleep(max(0.0, prof["dt"] - (clock() - t0)))
                    rec["command_completed"] = True
                    post = settle(trim_mon, trim_direction)
                finally:
                    rec.update(feedback_end=len(log["rows"]), last_actual_ticks=dict(log["rows"][-1]["present"]) if log["rows"] else None,
                               last_applied_goal={n: round(cmd[n]) for n in rc.JOINTS})
                    (folder / "APPLICATION.json").write_text(json.dumps(rec, indent=1))
                    (folder / "FEEDBACK.json").write_text(json.dumps(log["rows"][rec["feedback_start"]:rec["feedback_end"]], indent=1))
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
            if observation_view:
                drive_state = {n: PG.Joint(pg, now[n], direction[n], log["rows"][0]["t"]) for n in moving}
                for sample in log["rows"]:
                    for n in moving:
                        drive_state[n].update(sample["t"] + sample["read_s"], sample["present"][n], direction[n] * (round(sample["commanded"].get(n, cmd[n])) - sample["present"][n]))
                log["observation_view"] = dict(nominal_target_reached=all(ok.values()), command_completed=True,
                    actual_settled=all(settled.values()) and not any(r["velocity"][n] for r in last for n in moving),
                    command_state_resolved=all(m.state == PG.OK for m in drive_state.values()) and not trims and all(r["goal"][n] == round(cmd[n]) and abs(r["goal"][n] - r["present"][n]) <= pg["follow"] for r in last for n in moving),
                    drive_progress={n: m.state for n, m in drive_state.items()}, current_support=None,
                    fresh_observation_acquired=False, insertion_ready=False,
                    actual_ticks=reached, nominal_target=dict(traj["target"]),
                    goal_actual_gap_ticks={n: last[-1]["goal"][n] - last[-1]["present"][n] for n in moving},
                    endpoint_contract_unchanged=True, trim_policy="observe_actual_settled_pose_without_integral_trim")

            if contact_observer is not None:
                log["outcome"].update(contact_candidate=False, contact_collection_completed=True)
            if stop_requested():
                raise Stop(G.hard("operator_stop", "stop requested"))
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
        except Exception as exc:
            log["executor_error"] = f"{type(exc).__name__}: {exc}"
            log["needs_a_person"] = "executor code/recording exception; no automatic goal/open/reverse/OFF was sent"
            exc.executor_log = log
            return close("EXECUTOR_EXCEPTION")
    except Stop as p:
        log["stopped_by"] = p.gate
        kind = p.gate["gate"]
        if kind != "bus_lost" and not any(log["writes"].values()) and not log["unchanged_arm_goal_writes_avoided"]:
            refuse(p.gate); return close("REFUSED")                           # stopped before the first write of this call: nothing was sent, whatever was held before is held as it was
        if kind in ("certificate_binding", "port_owned", "torque_state", "command_malformed", "servo_range", "device_calibration_mismatch", "profile_violation") and not log["writes"]["Torque_Enable"]:
            refuse(p.gate); return close("REFUSED")                           # nothing was switched on
        if kind == "bus_lost":
            record_read_after_loss(bus, log)
            try:
                bus.disconnect(disable_torque=False)
            except BaseException:
                pass
            return finish("BUS_LOST")
        if read_only_fault_recovery:
            # Supervised direct-joint trial: no hold/return/OFF goals after a stop; the operator supports/cuts power.
            record = read_feedback_once(bus)
            log['read_after_stop'] = record
            state = record['state']
            log['end'] = {key: state.get(source, 'UNKNOWN') for key, source in
                          (('ticks', 'present'), ('goal_register', 'goal'), ('torque', 'torque'), ('status', 'status'), ('moving', 'moving'), ('velocity', 'velocity'))}
            torque_now = state.get('torque', {})
            log['outcome']['torque_released'] = all(torque_now[n] == 0 for n in powered) if all(n in torque_now for n in powered) else 'UNKNOWN'
            log['needs_a_person'] = 'Direct trial stopped; no subsequent Goal/return/OFF was sent. Support the arm and use the agreed manual power cut; unread torque remains UNKNOWN.'
            try:
                bus.disconnect(disable_torque=False)
            except BaseException as exc:
                log['events'].append(f'disconnect: {type(exc).__name__}: {exc}')
            return finish('STOPPED_STATE_RECORDED')
        # ---- RECOVERY (heuristics; not gates) -----------------------------------------------------------------------
        try:
            cur = read()
            if traj.get("command_start") and any(cur["present"][n] < mp[n]["lo"] or cur["present"][n] > mp[n]["hi"] for n in moving):
                log["recovery"] = dict(range_entry_interrupted=True, actual=cur["present"], goal_register=cur["goal"], additional_goal_writes=0)
                log["needs_a_person"] = "range-entry interrupted before actual entered the servo range; retain the last valid command, no out-of-range actual hold, automatic open/reverse/OFF or retry"
                log["outcome"].update(target_reached=False, torque_released=False)
                return close("FAULT_HOLDING" if kind == "servo_fault" else "STOPPED_HOLDING" if kind == "operator_stop" else "NO_PROGRESS_HOLDING")
            for n in moving:
                if kind != "operator_stop" or (cur["torque"][n] and not any(cur["status"].values())):
                    goal(n, cur["present"][n], stop_hold=kind == "operator_stop")  # STOP hold only for healthy joints still powered
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


JAW_PROFILE = Path(__file__).resolve().parent / "sweepick_jaw_profile_left.json"


def jaw_device(lo, hi, prof, path=JAW_PROFILE):
    """sweepick_grasp.JawDevice of the real left gripper from its profile file, on the primitive's opening scale (0 = calibrated closed, 1 = calibrated open)."""
    import sweepick_grasp as tg
    d = json.loads(Path(path).read_text()); R = float(hi - lo); v = lambda k: float(d[k]["value"])
    return tg.JawDevice(residual_floor=v("residual_floor_ticks") / R, preload_residual=v("preload_residual_ticks") / R, creep_step=v("creep_step_ticks") / R, creep_period_s=v("creep_period_s"), close_rate_per_s=prof["v"] / tt.TICK_RAD / R,
                        end_stop_opening=v("end_stop_ticks_above_closed") / R, rest_step=v("rest_step_ticks") / R, max_feedback_age_s=v("max_feedback_age_s"), confirm_s=v("confirm_s"), source=f"{d['name']} sha256 {hashlib.sha256(Path(path).read_bytes()).hexdigest()[:16]}")


def grip(side, out, *, torque_joints, cert=None, level="device", execute=False, hold_s=1.0, open_bus=real_bus, owner=port_owner, mapping=None, mapping_sha="test", profile=None, sleep=time.sleep, clock=time.time, stop_requested=lambda: False, jaw_profile=None, jaw_obj=None, start_recheck=None):
    """Close the gripper on whatever is between the jaws with the EXISTING grasp primitive (sweepick_grasp.Jaw): the closing
    command runs at the profile speed until the residual leaves the free stroke (contact onset), then squeezes in single
    ticks to the device preload and holds there. Nothing is accumulated after the onset and nothing is relaxed afterwards:
    the held command stays a little ahead of the jaws, so a lost object shows as the residual falling to the no-load floor.
    The arm joints named in torque_joints must already be held; none of them is commanded here.
    cert (model level): dict(nominal_verdict, arm_ticks, sweep=[from_tick, to_tick]) of a check of the WHOLE possible closing
    sweep at this arm pose, with the object in the scene and only the intended contacts allowed."""
    import sweepick_grasp as tg
    out = Path(out); out.mkdir(parents=True, exist_ok=True); port = f"/dev/dapier/{side}_arm"; prof = profile or tt.load_profile(); J = "gripper"
    log = dict(schema="tjj.grip.v1", mode="grip", side=side, level=level, port=port, execute=bool(execute), started=time.strftime("%Y-%m-%dT%H:%M:%S%z"), writes=dict(Goal_Position=0, Torque_Enable=0), rows=[], events=[], profile=prof,
               owner="sweepick_grasp.Jaw", outcome=dict(command_completed=False, contact_candidate=None, torque_released=False), mapping="CANDIDATE (verified=false)", write_attempts=dict(Goal_Position=0, Torque_Enable=0))
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
    def write(reg, v, stop_hold=False):
        if stop_requested() and not stop_hold:
            raise Stop(G.hard("operator_stop", f"stop requested before {reg} write"))
        log["write_attempts"][reg] += 1
        log["last_write"] = dict(register=reg, requested=int(round(v)), acknowledgement="UNKNOWN")
        try:
            bus.write(reg, J, int(round(v)), normalize=False); log["writes"][reg] += 1
            log["last_write"]["acknowledgement"] = "ACK"
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
        log.update(start=dict(ticks=now["present"], torque_on=on, torque=now["torque"], status=now["status"], goal_register=now["goal"], t=now["t"], read_s=now["read_s"]), jaw_device=dev.__dict__ if hasattr(dev, "__dict__") else str(dev), sweep=[now["present"][J], tk(dev.end_stop_opening)])
        if level == "model":
            def bound():
                try:
                    delta = {n: now["present"][n] - cert["arm_ticks"][n] for n in rc.JOINTS[:5]}
                    ok = cert.get("nominal_verdict") == "CLEAR" and all(abs(v) <= 1 for v in delta.values()) and cert["sweep"][0] >= now["present"][J] - 1 and cert["sweep"][1] <= tk(dev.end_stop_opening) + 1
                    log["certificate_binding"].update(matched=ok, arm_delta_ticks=delta, certificate=cert, actual=now["present"], feedback_time_s=now["t"], read_s=now["read_s"])
                    return ok
                except (KeyError, TypeError, AttributeError, ValueError):
                    log["certificate_binding"].update(matched=False, certificate=cert, actual=now["present"])
                    return False
            log["certificate_binding"] = dict(rechecked=False)
            ok = bound()
            if not ok and start_recheck is not None:
                if stop_requested():
                    raise Stop(G.hard("operator_stop", "stop requested before closing-sweep recheck"))
                log["certificate_binding"]["initial"] = dict(log["certificate_binding"])
                try:
                    cert = start_recheck(dict(now["present"]))
                except Exception as exc:
                    raise Stop(G.provisional("certificate_binding", f"closing-sweep recheck failed: {type(exc).__name__}: {exc}")) from exc
                log["certificate_binding"]["rechecked"] = True
                now = read()
                if any(now["status"].values()):
                    raise Stop(G.hard("servo_fault", f"Status register non-zero after closing-sweep recheck: {now['status']}"))
                if [n for n in rc.JOINTS if now["torque"][n]] != on:
                    raise Stop(G.hard("torque_state", "torque ownership changed during closing-sweep recheck"))
                ok = bound()
            if not ok:
                raise Stop(G.provisional("certificate_binding" if isinstance(cert, dict) and cert.get("nominal_verdict") == "CLEAR" else "collision_candidate_frame", "the closing sweep at this arm pose has no CLEAR collision result bound to it"))
        if not execute:
            log["events"].append("the device was read, nothing was written"); return done("PLANNED")
    except KeyboardInterrupt:
        log["refused"] = G.hard("operator_stop", "interrupted before the first grip write")
        return done("REFUSED")
    except Stop as e:
        log["refused"] = e.gate
        if e.gate["gate"] != "bus_lost":
            return done("REFUSED")
        log["end"] = dict(error="bus read failed before the first write; current state not confirmed", torque="UNKNOWN")
        log["outcome"]["torque_released"] = "UNKNOWN"
        log["needs_a_person"] = "the bus does not answer; no actuator write or automatic OFF; current torque state is UNKNOWN"
        try:
            bus.disconnect(disable_torque=False)
        except BaseException as exc:
            log["events"].append(f"disconnect: {type(exc).__name__}: {exc}")
        return finish("REFUSED")
    if jaw_obj is not None and jaw_obj.dev != dev:
        raise ValueError("the Jaw handed in was built for another device profile than this gripper's")
    jaw = jaw_obj if jaw_obj is not None else tg.Jaw(dev, prof["dt"], tg.INDEPENDENT); vmax, amax = prof["v"] / tt.TICK_RAD * prof["dt"], prof["a"] / tt.TICK_RAD * prof["dt"] ** 2
    try:
        try:
            actual0 = float(now["present"][J]); cmd = min(max(actual0, lo), hi)
            if cmd != actual0:
                log["goal_seed_normalization"] = dict(actual_tick=actual0, command_tick=cmd, configured_range=[lo, hi], outside_by_ticks=actual0 - cmd,
                                                      source="fresh Present_Position plus the calibrated servo range; preflight confirmed the EEPROM Min/Max matches it",
                                                      basis="an initial Goal_Position write must be inside the actual configured range; the nearest endpoint is used with no margin")
            if J not in on or now['goal'][J] != round(cmd):
                write("Goal_Position", cmd); back = read()
                if back["goal"][J] != round(cmd):
                    raise Stop(G.hard("bus_lost", "the Goal_Position read-back differs from what was written"))
                if any(back['status'].values()):
                    raise Stop(G.hard('servo_fault', f'Status register non-zero after initial grip Goal: {back["status"]}'))
            if J not in on:
                write("Torque_Enable", 1)
            jaw.start_close(); step, t_hold, contact = 0.0, None, None
            pg = PG.load()
            close_start_actual, close_start_mon, close_start_best = None, None, None
            close_started = False
            entry = cmd != actual0
            if entry:
                # Range entry is a free positioning move; the arm's following lead is not a gripper contact baseline.
                entry_mon = PG.Joint(dict(pg, follow=pg['lead']), actual0, float(np.sign(cmd - actual0)), clock())
                entry_best_lead = abs(actual0 - cmd)
                log['range_entry'] = dict(status='WAITING_FOR_ACTUAL', goal_tick=cmd, actual_tick=actual0,
                    progress_profile_sha256=pg['sha256'], contact_samples_used=0,
                    basis='Hold the acknowledged range-valid seed; start Jaw only inside its existing no-load residual floor.')
            while True:
                t0 = clock()
                if stop_requested():
                    raise Stop(G.hard("operator_stop", "stop requested"))
                r = read()
                if any(r["status"].values()):
                    raise Stop(G.hard("servo_fault", f"Status register non-zero: {({n: v for n, v in r['status'].items() if v})}"))
                if entry:
                    residual = r['present'][J] - cmd
                    entry_lead = abs(residual)
                    entry_best_lead = min(entry_best_lead, entry_lead)
                    progress = entry_mon.update(r['t'] + r['read_s'], r['present'][J], entry_best_lead)
                    if r['goal'][J] != round(cmd):
                        raise Stop(G.hard('bus_lost', 'The initial grip Goal readback changed before actual range entry was confirmed.'))
                    if r['torque'][J] and entry_lead <= dev.residual_floor * R:
                        if {n for n in rc.JOINTS if r['torque'][n]} != set(on) | {J}:
                            raise Stop(G.hard('torque_state', 'Torque ownership changed during initial grip range entry.'))
                        now = r
                        if level == 'model' and not bound():
                            raise Stop(G.provisional('certificate_binding', 'The closing-sweep certificate no longer matches after initial grip range entry.'))
                        if residual < 0:
                            previous = cmd
                            cmd = r['present'][J]
                            write('Goal_Position', cmd)
                            back = read()
                            if back['goal'][J] != round(cmd):
                                raise Stop(G.hard('bus_lost', 'The fresh actual grip seed was not read back after range-entry overshoot.'))
                            if any(back['status'].values()):
                                raise Stop(G.hard('servo_fault', f'Status register non-zero after fresh actual grip seed: {back["status"]}'))
                            log['range_entry']['fresh_hold_reseed'] = dict(previous_goal=previous, goal_tick=cmd,
                                actual_tick=r['present'][J], readback_tick=back['goal'][J], acknowledgement='ACK')
                            r = back
                            now = r
                            if {n for n in rc.JOINTS if r['torque'][n]} != set(on) | {J}:
                                raise Stop(G.hard('torque_state', 'Torque ownership changed during fresh actual grip seeding.'))
                            if level == 'model' and not bound():
                                raise Stop(G.provisional('certificate_binding', 'The closing-sweep certificate changed during fresh actual grip seeding.'))
                        entry = False
                        log['range_entry'].update(status='ACTUAL_CONFIRMED', confirmed_at=r['t'], actual_tick=r['present'][J],
                            current_goal_tick=cmd, residual_ticks=r['present'][J] - cmd)
                    else:
                        r.update(commanded={J: cmd}, residual_ticks=residual, jaw_state=tg.UNAVAILABLE,
                                 jaw_mode='range_entry', phase='range_entry', progress=progress)
                        log['rows'].append(r)
                        if not r['torque'][J] or progress == PG.NO_PROGRESS:
                            log['range_entry']['status'] = 'UNCONFIRMED_HOLDING'
                            log['needs_a_person'] = 'Range entry was not confirmed; no closing, contact claim, automatic retry, return or OFF.'
                            return done('GRIP_UNAVAILABLE_HOLDING')
                        sleep(max(0.0, prof['dt'] - (clock() - t0)))
                        continue
                freeze_start = False
                if not close_started:
                    if close_start_actual is None:
                        close_start_actual = r['present'][J]
                        close_start_mon = PG.Joint(dict(pg, follow=pg['lead']), close_start_actual, -1, clock())
                    closing_residual = r['present'][J] - cmd
                    advanced = close_start_actual - r['present'][J] >= pg['resolution']
                    close_started = (advanced and 0 <= closing_residual <= dev.residual_floor * R) or cmd <= tk(dev.end_stop_opening)
                    if close_started:
                        log['close_start'] = dict(actual_confirmed=True, t=r['t'], actual_tick=r['present'][J], goal_tick=cmd)
                    else:
                        freeze_start = closing_residual > dev.residual_floor * R
                        close_start_best = min(close_start_best if close_start_best is not None else closing_residual,
                                               closing_residual) if freeze_start else None
                        progress = close_start_mon.update(r['t'] + r['read_s'], r['present'][J],
                            close_start_best if freeze_start else max(0., closing_residual))
                        if not r['torque'][J] or progress == PG.NO_PROGRESS:
                            log['close_start'] = dict(actual_confirmed=False, actual_tick=r['present'][J], goal_tick=cmd)
                            log['needs_a_person'] = 'Initial closing motion was not confirmed; no contact claim, retry, return or OFF.'
                            return done('GRIP_UNAVAILABLE_HOLDING')
                state = jaw.update(r["t"] + r["read_s"], op(cmd), op(r["present"][J]), torque_enabled=bool(r["torque"][J])) if close_started else tg.UNAVAILABLE
                if state == tg.CONTACT_CANDIDATE and contact is None:
                    contact = dict(t=r["t"], commanded=cmd, present=r["present"][J], residual_ticks=r["present"][J] - cmd, load_raw=r["load"][J], current_raw=r["current"][J])
                want = cmd if freeze_start else tk(jaw.next_command(op(cmd)))  # the primitive decides: close / squeeze / hold
                if jaw.mode == "close":
                    step = min(cmd - want, vmax, step + amax); want = cmd - max(step, 0.0)       # within the selected profile (speed and its rate of change)
                else:
                    step = 0.0
                if want != cmd:
                    cmd = want; write("Goal_Position", cmd)
                r.update(commanded={J: cmd}, residual_ticks=r["present"][J] - cmd, jaw_state=state, jaw_mode=jaw.mode,
                         phase="grip" if close_started else 'close_start')
                log["rows"].append(r)
                if jaw.mode == "hold":
                    t_hold = t_hold or clock()
                    if clock() - t_hold >= hold_s and jaw.closed_and_resting():
                        break
                sleep(max(0.0, prof["dt"] - (clock() - t0)))
            if stop_requested():
                raise Stop(G.hard("operator_stop", "stop requested after the grip loop"))
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
            cur = read()
            if not lo <= cur['present'][J] <= hi:
                log['recovery'] = dict(range_entry_interrupted=True, actual=cur['present'], goal_register=cur['goal'],
                    moving=cur['moving'], additional_goal_writes=0, motion_stop_confirmed='UNKNOWN')
                log['needs_a_person'] = 'Range entry interrupted: retain the last valid Goal; no out-of-range hold, automatic opening/OFF or retry. Physical stop is not confirmed.'
                return done('FAULT_HOLDING' if kind == 'servo_fault' else 'STOPPED_STATE_RECORDED')
            if kind != "operator_stop" or (cur["torque"][J] and not any(cur["status"].values())):
                write("Goal_Position", cur["present"][J], stop_hold=kind == "operator_stop")  # stop an already-powered healthy jaw; never re-enable an OFF jaw
            log["needs_a_person"] = ("the grip was stopped; an already-powered healthy gripper holds fresh Present_Position, with no automatic opening or OFF"
                                          if kind == "operator_stop" else "the grip was stopped; the gripper holds its measured position and the arm holds. No automatic opening")
            return done("FAULT_HOLDING" if kind == "servo_fault" else ("STOPPED_HOLDING" if kind == "operator_stop" else "REFUSED"))
        except BaseException as e:
            log["needs_a_person"] = f"the bus failed during the stop handling ({type(e).__name__}): the torque state is NOT known"; log["end"] = dict(error="no read possible")
            try:
                bus.disconnect(disable_torque=False)
            except BaseException:
                pass
            return finish("BUS_LOST")


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
    print(json.dumps({k: log.get(k) for k in ("mode", "side", "level", "execute", "result", "exit_code", "outcome", "torque_scope", "goal_seed", "refused", "stopped_by", "command_waits", "trajectory", "validation", "recovery", "release", "reached", "diagnostics", "unchanged_arm_goal_writes_avoided", "state_left", "needs_a_person", "writes", "end")}, indent=1, default=str))
    return code


if __name__ == "__main__":
    sys.exit(main())
