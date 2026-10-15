#!/usr/bin/env python3
"""R5 result reconciliation: re-aggregate stored student-only SIM evaluations.

Reads what sim_data_factory.student_eval.run actually writes (checker.valid,
checker.handover_success, checker.pick_lift_success, abort, completed_transitions,
actual_sim_time, ...). There is no top-level `valid` / `task_success` in that contract.

Rules kept separate on purpose:
- checker validity, horizon completion, events, terminal task success and abort are
  five different facts and are never folded into one bool.
- A missing required key or file is an aggregation error / "unavailable", never False/0.
- Events at or after the first fault are post-fault diagnostics, not normal-run events.
- Contact metrics use the stored checker config (e.g. grip_force_min_N); no new threshold.

No physics, model, checker or checkpoint is touched here; this module imports neither
torch nor mujoco.
"""
import argparse
import json
from pathlib import Path

UNAVAILABLE = "unavailable"
EVENT_ORDER = ("reach", "left_grasp", "pick_lift", "right_grasp", "handover")
STAGE_EVENTS = ("reach", "pick_lift", "handover")  # task_success.evaluate_states stage rule
REQUIRED_REPORT_KEYS = ("checker", "abort", "completed_transitions", "actual_sim_time", "stopped", "completed")
REQUIRED_CHECKER_KEYS = ("valid", "handover_success", "pick_lift_success", "events", "failures", "stage")

FACTORY_ROOT = Path("/home/robot/DAPIER/.local-artifacts/sim-data-factory-20261002")
LEGACY_VAL = {
    "model_a": ("Model_A_Orig_Continued", "evaluation/r5-orig-continued-val"),
    "model_b": ("Model_B_Augmented_Continued", "evaluation/r5-augmented-continued-val"),
}


class AggregationError(ValueError):
    """Stored evaluation does not satisfy the evaluator output contract."""


def _require(mapping, keys, where):
    if not isinstance(mapping, dict):
        raise AggregationError(f"{where} is not an object")
    missing = [k for k in keys if k not in mapping]
    if missing:
        raise AggregationError(f"{where} missing required keys: {missing}")


def first_fault(report, diagnostic_fault=None):
    """Earliest fault of a run: terminal abort and/or static forbidden-contact rows.

    `time_limit` is the SIM time from which states are no longer a fault-free prefix:
    a substep fault uses its contact_configuration_time, a static failure its state time.
    An action rejection stops before any physics, so the state at that time is still valid
    (`inclusive`=True).
    """
    candidates = []
    abort = report["abort"]
    if abort:
        _require(abort, ("reason", "time"), "abort")
        rejected = abort["reason"] == "student_action_rejected"
        candidates.append(dict(
            source="terminal_abort", reason=abort["reason"], sim_time=float(abort["time"]),
            time_limit=float(abort["time"] if rejected else abort.get("contact_configuration_time", abort["time"])),
            inclusive=rejected, pairs=abort.get("forbidden_contacts", UNAVAILABLE if not rejected else []),
            max_penetration_m=abort.get("max_penetration_m", UNAVAILABLE), substep=abort.get("substep", UNAVAILABLE),
            detail=abort.get("detail")))
    if diagnostic_fault:  # located afterwards by replaying a run that was executed without abort
        _require(diagnostic_fault, ("reason", "time"), "diagnostic_fault")
        candidates.append(dict(
            source="substep_replay_of_nonabort_run", reason=diagnostic_fault["reason"], sim_time=float(diagnostic_fault["time"]),
            time_limit=float(diagnostic_fault.get("contact_configuration_time", diagnostic_fault["time"])), inclusive=False,
            pairs=diagnostic_fault.get("forbidden_contacts", UNAVAILABLE),
            max_penetration_m=diagnostic_fault.get("max_penetration_m", UNAVAILABLE),
            substep=diagnostic_fault.get("substep", UNAVAILABLE)))
    for failure in report["checker"]["failures"]:
        if failure is abort or failure == abort or "time" not in failure:
            continue  # terminal duplicate, or whole-run worst-case entries without a time
        candidates.append(dict(
            source="static_state_reconstruction", reason=failure.get("reason", UNAVAILABLE),
            sim_time=float(failure["time"]), time_limit=float(failure["time"]), inclusive=False,
            pairs=failure.get("detail", UNAVAILABLE), step=failure.get("step", UNAVAILABLE)))
    if not candidates:
        return None
    return min(candidates, key=lambda c: (c["time_limit"], c["source"] == "static_state_reconstruction"))


def split_events(events, fault):
    """(events inside the fault-free prefix, events at/after the first fault)."""
    if fault is None:
        return dict(events), {}
    keep = (lambda t: t <= fault["time_limit"]) if fault["inclusive"] else (lambda t: t < fault["time_limit"])
    return ({k: v for k, v in events.items() if keep(v)}, {k: v for k, v in events.items() if not keep(v)})


def stage_of(events):
    stage = "none"
    for name in STAGE_EVENTS:
        if name in events:
            stage = name
    return stage


def contact_metrics(rows, config, control_dt, fault=None):
    """Grasp/lift metrics from stored checker rows using the checker's own definitions.

    Scope: states at completed control transitions (static reconstruction), restricted to
    the fault-free prefix. Sub-step contacts between those states are not in the rows.
    """
    if not rows or config is None or control_dt is None:
        return dict(status=UNAVAILABLE, reason="no stored checker rows / config / control_dt")
    for key in ("grip_force_min_N", "release_force_max_N", "lift_clearance_m"):
        if key not in config:
            return dict(status=UNAVAILABLE, reason=f"checker config lacks {key}")
    try:
        if fault is not None:
            limit = fault["time_limit"]
            used = [r for r in rows if (r["time"] <= limit if fault["inclusive"] else r["time"] < limit)]
        else:
            used = list(rows)
        if not used:
            return dict(status=UNAVAILABLE, reason="no fault-free state rows", rows_total=len(rows))
        grip = config["grip_force_min_N"]
        hold = lift = best_hold = best_lift = touch = best_touch = right_hold = best_right_hold = 0.0
        confirm = config.get("grip_confirm_s")
        grasped = False
        for row in used:
            left, right = row["pad_force_N"]["left"], row["pad_force_N"]["right"]
            hold = hold + control_dt if min(left) >= grip else 0.0
            best_hold = max(best_hold, hold)
            touch = touch + control_dt if min(left) > 0.0 else 0.0   # descriptive only, not a grasp criterion
            best_touch = max(best_touch, touch)
            right_hold = right_hold + control_dt if min(right) >= grip else 0.0
            best_right_hold = max(best_right_hold, right_hold)
            if confirm is not None and hold >= confirm:
                grasped = True
            lifted = (grasped and min(left) >= grip and max(right) < config["release_force_max_N"]
                      and row["table_support_N"] == 0.0 and row["block_bottom_clearance_m"] >= config["lift_clearance_m"])
            lift = lift + control_dt if lifted else 0.0
            best_lift = max(best_lift, lift)
        support = [row["table_support_N"] for row in used]
        return dict(
            status="ok", scope="fault-free prefix, completed-transition states only",
            rows_used=len(used), rows_total=len(rows), grip_force_min_N=grip,
            left_both_pad_hold_max_s=round(best_hold, 4),
            left_only_lifted_hold_max_s=round(best_lift, 4) if confirm is not None else UNAVAILABLE,
            right_both_pad_hold_max_s=round(best_right_hold, 4),
            left_both_pad_touch_max_s=round(best_touch, 4),
            left_both_pad_touch_note="both left pads with any contact force > 0; descriptive, not the checker's grasp condition",
            max_left_weaker_pad_force_N=max(min(row["pad_force_N"]["left"]) for row in used),
            max_right_weaker_pad_force_N=max(min(row["pad_force_N"]["right"]) for row in used),
            left_pad_contact_states=dict(both=sum(min(row["pad_force_N"]["left"]) > 0 for row in used),
                                         any=sum(max(row["pad_force_N"]["left"]) > 0 for row in used)),
            max_block_bottom_clearance_m=max(row["block_bottom_clearance_m"] for row in used),
            max_block_z_body_m=max(row["block_pos"][2] for row in used),
            table_support_N=dict(first=support[0], last=support[-1], min=min(support), max=max(support),
                                 zero_support_states=sum(s == 0.0 for s in support)),
            max_left_pad_force_N=max(max(row["pad_force_N"]["left"]) for row in used))
    except (KeyError, TypeError, IndexError) as error:
        return dict(status=UNAVAILABLE, reason=f"checker row missing field: {error!r}")


def action_rejection_count(report, action_trace):
    """Rejections are commands the range check refused; they never reach episode.npz q_cmd."""
    abort = report["abort"]
    by_abort = int(bool(abort) and abort.get("reason") == "student_action_rejected")
    if action_trace is None:
        return dict(count=by_abort if by_abort else UNAVAILABLE, source="abort only; action_trace unavailable")
    try:
        by_trace = sum(1 for row in action_trace if row["ctrl_validated"] is None)
    except (KeyError, TypeError) as error:
        return dict(count=by_abort if by_abort else UNAVAILABLE, source=f"abort only; action_trace unreadable: {error!r}")
    if by_trace != by_abort:
        raise AggregationError(f"action rejection mismatch: action_trace={by_trace}, abort={by_abort}")
    return dict(count=by_trace, source="action_trace ctrl_validated is null, cross-checked with abort")


def rejected_command_detail(abort, limits):
    """Channels of a rejected command that lie outside the decoder limits, with the command and the limit.

    `limits` = dict(names, lower, upper) for the 12 canonical channels, written by the runner from the
    decoder. Without it the per-joint detail is unavailable; it is never guessed.
    """
    if not abort or abort.get("reason") != "student_action_rejected":
        return None
    command = abort.get("command")
    if command is None or limits is None:
        return dict(status=UNAVAILABLE, reason="rejected command or canonical limits not available",
                    command=command if command is not None else UNAVAILABLE, detail=abort.get("detail"))
    if not (len(command) == len(limits["names"]) == len(limits["lower"]) == len(limits["upper"])):
        raise AggregationError("rejected command and canonical limits have different lengths")
    joints = [dict(joint=name, command=value, lower=lower, upper=upper, excess=max(lower - value, value - upper))
              for name, value, lower, upper in zip(limits["names"], command, limits["lower"], limits["upper"])
              if not (lower <= value <= upper)]
    return dict(status="ok", sim_time=abort.get("time"), detail=abort.get("detail"), command=command,
                joints_outside_limits=joints)


TASK_CONDITIONS = dict(
    reach="both left pads press the block with at least grip_force_min_N in the same state",
    left_grasp="the reach condition held continuously for grip_confirm_s",
    pick_lift="left-only grasp: right pads below release_force_max_N, zero table support, block bottom clearance "
              "at least lift_clearance_m, held for lift_confirm_s",
    right_grasp="after pick_lift, both right pads at least grip_force_min_N for grip_confirm_s",
    handover="recipient-only hold for hold_s: right grasp, left pads released and clear by donor_clearance_m, "
             "no table support, no non-pad block contact, clearance and slip limits met",
)


def first_unmet_condition(prefix_events, termination):
    """First task milestone the run did not reach inside its fault-free prefix (checker definitions, unchanged)."""
    for name in EVENT_ORDER:
        if name not in prefix_events:
            return dict(event=name, checker_condition=TASK_CONDITIONS[name], run_ended_by=termination)
    return None


def record_from_report(report, *, action_trace=None, scoring_rows=None, control_dt=None,
                       sim_time_start=None, stored_transitions=None, wall_time_s=None, horizon_transitions=None,
                       diagnostic_fault=None, limits=None):
    """One case record from an evaluator report dict. Raises AggregationError on contract violation."""
    _require(report, REQUIRED_REPORT_KEYS, "evaluation report")
    checker = report["checker"]
    _require(checker, REQUIRED_CHECKER_KEYS, "evaluation report checker")
    transitions = int(report["completed_transitions"])
    if stored_transitions is not None and stored_transitions != transitions:
        raise AggregationError(f"transition count mismatch: report={transitions}, episode.npz={stored_transitions}")
    abort = report["abort"]
    if bool(abort) != bool(report["stopped"]) or bool(abort) == bool(report["completed"]):
        raise AggregationError("abort / stopped / completed disagree")
    if checker["handover_success"] and not checker["valid"]:
        raise AggregationError("handover_success with invalid checker")
    if horizon_transitions is not None and (not abort) != (transitions == horizon_transitions):
        raise AggregationError(f"horizon {horizon_transitions} vs completed {transitions} disagrees with abort state")
    fault = first_fault(report, diagnostic_fault)
    events = dict(checker["events"])
    prefix_events, post_events = split_events(events, fault)
    sim_end = float(report["actual_sim_time"])
    return dict(
        status="ok",
        completed_transitions=transitions,
        horizon_transitions=horizon_transitions if horizon_transitions is not None else UNAVAILABLE,
        horizon_completed=not abort,  # evaluator loop ran to its end without a terminal fault
        sim_time_start=sim_time_start if sim_time_start is not None else UNAVAILABLE,
        sim_time_end=sim_end,
        sim_elapsed_s=round(sim_end - sim_time_start, 6) if sim_time_start is not None else UNAVAILABLE,
        wall_time_s=wall_time_s if wall_time_s is not None else UNAVAILABLE,
        inference_wall_s=report.get("inference_wall_s", UNAVAILABLE),
        checker_valid=bool(checker["valid"]),
        checker_failure_count=checker.get("failure_count", len(checker["failures"])),
        checker_failure_reasons=sorted({f.get("reason", UNAVAILABLE) for f in checker["failures"]}),
        termination="horizon_completed" if not abort else abort.get("reason", UNAVAILABLE),
        first_fault=fault,
        events_all=events,
        events_valid_prefix=prefix_events,
        events_post_fault_diagnostic=post_events,
        stage_checker=checker["stage"],
        stage_valid_prefix=stage_of(prefix_events),
        terminal_pick_lift_success=bool(checker["pick_lift_success"]),
        terminal_handover_success=bool(checker["handover_success"]),
        action_rejections=action_rejection_count(report, action_trace),
        rejected_command=rejected_command_detail(abort, limits),
        first_unmet_task_condition=first_unmet_condition(prefix_events, "horizon_completed" if not abort else abort.get("reason", UNAVAILABLE)),
        contact_metrics=contact_metrics(scoring_rows, checker.get("config"), control_dt, fault),
        worst=checker.get("worst", UNAVAILABLE),
    )


def _load_json(path):
    return json.loads(Path(path).read_text())


def read_case(case_dir, *, wall_time_s=None, horizon_transitions=None, limits=None):
    """Re-aggregate one stored case directory. Never raises: errors become a record."""
    case_dir = Path(case_dir)
    base = dict(case=case_dir.name, source_dir=str(case_dir))
    sources = {name: str(case_dir / name) for name in
               ("evaluation.json", "scoring_trace.json", "action_trace.json", "episode.npz", "metadata.json")
               if (case_dir / name).exists()}
    try:
        if "evaluation.json" not in sources:
            raise AggregationError("evaluation.json missing")
        report = _load_json(case_dir / "evaluation.json")
        action_trace = _load_json(case_dir / "action_trace.json") if "action_trace.json" in sources else None
        rows = _load_json(case_dir / "scoring_trace.json").get("contacts") if "scoring_trace.json" in sources else None
        control_dt = sim_start = stored = None
        if "metadata.json" in sources:
            metadata = _load_json(case_dir / "metadata.json")
            control_dt = metadata.get("control_dt")
            stored = metadata.get("transitions")
            sim_start = (metadata.get("actual_initial_state") or {}).get("sim_time")
        if "episode.npz" in sources:
            import numpy as np
            with np.load(case_dir / "episode.npz", allow_pickle=False) as arrays:
                stored_npz = int(len(arrays["q_cmd"]))
                sim_start = float(arrays["sim_time"][0])
            if stored is not None and stored != stored_npz:
                raise AggregationError(f"metadata transitions {stored} != episode.npz {stored_npz}")
            stored = stored_npz
        if control_dt is None and rows and len(rows) > 1:
            control_dt = round(rows[1]["time"] - rows[0]["time"], 9)
        aggregation = UNAVAILABLE   # how commands were formed from the chunks; written by the evaluator
        if (case_dir / "execution_config.json").exists():
            aggregation = "off" if _load_json(case_dir / "execution_config.json").get("temporal_ensemble") is None else "temporal_ensemble"
        identity_path = case_dir / "run_identity.json"   # sidecar written by the runner right after the run
        if wall_time_s is None and identity_path.exists():
            wall_time_s = _load_json(identity_path).get("wall_time_s")
        record = record_from_report(report, action_trace=action_trace, scoring_rows=rows, control_dt=control_dt,
                                    sim_time_start=sim_start, stored_transitions=stored, wall_time_s=wall_time_s,
                                    horizon_transitions=horizon_transitions, limits=limits)
        return dict(base, seed=report.get("seed", UNAVAILABLE), execution_aggregation=aggregation, sources=sources, **record)
    except (AggregationError, json.JSONDecodeError, OSError, KeyError, TypeError, ValueError) as error:
        return dict(base, status="aggregation_error", error=f"{type(error).__name__}: {error}", sources=sources)


def _count(records, predicate):
    """Count over readable cases only; unreadable cases stay visible instead of becoming 0."""
    ok = [r for r in records if r["status"] == "ok"]
    return dict(count=sum(1 for r in ok if predicate(r)), of_available=len(ok), unavailable=len(records) - len(ok))


def summarize(records):
    ok = [r for r in records if r["status"] == "ok"]
    terminations = {}
    for r in ok:
        terminations[r["termination"]] = terminations.get(r["termination"], 0) + 1
    return dict(
        total_cases=len(records),
        aggregation_errors=[dict(case=r["case"], error=r["error"]) for r in records if r["status"] != "ok"],
        checker_valid=_count(records, lambda r: r["checker_valid"]),
        horizon_completed=_count(records, lambda r: r["horizon_completed"]),
        termination=terminations,
        action_rejection_cases=_count(records, lambda r: r["action_rejections"]["count"] not in (0, UNAVAILABLE)),
        events_valid_prefix={e: _count(records, lambda r, e=e: e in r["events_valid_prefix"]) for e in EVENT_ORDER},
        events_post_fault_diagnostic={e: _count(records, lambda r, e=e: e in r["events_post_fault_diagnostic"]) for e in EVENT_ORDER},
        terminal_pick_lift_success=_count(records, lambda r: r["terminal_pick_lift_success"]),
        terminal_handover_success=_count(records, lambda r: r["terminal_handover_success"]),
        first_unmet_task_condition={e: sum(1 for r in ok if (r.get("first_unmet_task_condition") or {}).get("event") == e)
                                    for e in EVENT_ORDER},
    )


def reconcile_val(val_dir, model_name, *, horizon_transitions=None):
    """All case_* dirs of one stored Val run, plus the diff against its legacy summary."""
    val_dir = Path(val_dir)
    limits_path = val_dir / "canonical_limits.json"
    limits = _load_json(limits_path) if limits_path.exists() else None
    legacy_path = val_dir / "val_summary_report.json"
    legacy = _load_json(legacy_path) if legacy_path.exists() else None
    legacy_cases = {}
    if legacy:
        for row in legacy.get("cases", []):
            legacy_cases[f"case_{row['case_index']:02d}_{row['generation']}_s{row['seed']}"] = row
    records = []
    for case_dir in sorted(p for p in val_dir.iterdir() if p.is_dir() and p.name.startswith("case_")):
        old = legacy_cases.get(case_dir.name)
        # Legacy elapsed_s is a time.time() difference around the whole call: wall clock, not SIM time.
        record = read_case(case_dir, wall_time_s=old.get("elapsed_s") if old else None, horizon_transitions=horizon_transitions,
                           limits=limits)
        if record["status"] == "ok":
            record["wall_time_source"] = ("legacy val_summary_report elapsed_s (REPORTED, wall clock incl. model load)" if old
                                          else "run_identity.json (wall clock around the evaluator call, incl. model load)"
                                          if (case_dir / "run_identity.json").exists() else UNAVAILABLE)
        record["legacy_summary_row"] = old if old else UNAVAILABLE
        records.append(record)
    summary = summarize(records)
    return dict(model_name=model_name, source_dir=str(val_dir), legacy_summary=str(legacy_path) if legacy else UNAVAILABLE,
                summary=summary, legacy_differences=legacy_differences(legacy, records, summary), cases=records)


def legacy_differences(legacy, records, summary):
    if not legacy:
        return UNAVAILABLE
    diffs = []
    def add(field, old, new, cause):
        if old != new:
            diffs.append(dict(field=field, legacy=old, corrected=new, cause=cause))
    add("valid_cases", legacy.get("valid_cases"), summary["checker_valid"]["count"],
        "legacy read report['valid'] (absent in evaluator output) with default False; correct key is checker.valid")
    add("handover_count", legacy.get("handover_count"), summary["terminal_handover_success"]["count"],
        "legacy read report['task_success'] (absent); correct key is checker.handover_success")
    add("pick_lift_count", legacy.get("pick_lift_count"), summary["terminal_pick_lift_success"]["count"],
        "checker.pick_lift_success")
    ok = [r for r in records if r["status"] == "ok"]
    old_rows = [r["legacy_summary_row"] for r in ok if r["legacy_summary_row"] != UNAVAILABLE]
    add("cases with action_rejections>0", sum(1 for r in old_rows if r.get("action_rejections")),
        summary["action_rejection_cases"]["count"],
        "legacy only tested stored q_cmd for NaN; rejected commands are never stored in episode.npz")
    add("cases labelled 'Contact or kinematics fault'", sum(1 for r in old_rows if r.get("contact_fault")),
        summary["termination"].get("physics_substep_fault", 0),
        "legacy labelled every checker-invalid case as contact fault; corrected value counts terminal physics_substep_fault only")
    return diffs


def _cell(value):
    return UNAVAILABLE if value == UNAVAILABLE else value


def markdown_table(result):
    lines = ["| case | 완료 transition | SIM 경과(s) | wall(s) | checker.valid | 종료 원인 | 첫 fault SIM시각 / 대상 | 유효 prefix 이벤트 | fault 이후 진단 이벤트 | pick_lift | handover | 거부 명령 | 좌 양패드 유지 최대(s) | 블록 바닥 이격 최대(m) | 테이블 지지력 last(N) |",
             "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in result["cases"]:
        if r["status"] != "ok":
            lines.append(f"| {r['case']} | AGGREGATION ERROR: {r['error']} |" + " unavailable |" * 13)
            continue
        fault = r["first_fault"]
        fault_text = "-" if fault is None else f"{fault['sim_time']:.3f} / {fault['pairs'] if fault['pairs'] else fault['reason']}"
        cm = r["contact_metrics"]
        ok = cm["status"] == "ok"
        lines.append("| " + " | ".join(str(x) for x in (
            r["case"], r["completed_transitions"], _cell(r["sim_elapsed_s"]), _cell(r["wall_time_s"]), r["checker_valid"],
            r["termination"], fault_text,
            ", ".join(f"{k}@{v:.2f}" for k, v in r["events_valid_prefix"].items()) or "none",
            ", ".join(f"{k}@{v:.2f}" for k, v in r["events_post_fault_diagnostic"].items()) or "none",
            r["terminal_pick_lift_success"], r["terminal_handover_success"], r["action_rejections"]["count"],
            cm["left_both_pad_hold_max_s"] if ok else UNAVAILABLE,
            round(cm["max_block_bottom_clearance_m"], 4) if ok else UNAVAILABLE,
            round(cm["table_support_N"]["last"], 3) if ok else UNAVAILABLE)) + " |")
    return "\n".join(lines)


def summary_row(label, summary):
    def c(entry):
        text = f"{entry['count']}/{entry['of_available']}"
        return text + (f" (+{entry['unavailable']} unavailable)" if entry["unavailable"] else "")
    ev = summary["events_valid_prefix"]
    term = summary["termination"]
    return (f"| {label} | {summary['total_cases']} | {c(summary['checker_valid'])} | {c(summary['horizon_completed'])} | "
            f"{term.get('physics_substep_fault', 0)} | {term.get('student_action_rejected', 0)} | {c(ev['reach'])} | "
            f"{c(ev['left_grasp'])} | {c(ev['pick_lift'])} | {c(summary['terminal_pick_lift_success'])} | "
            f"{c(summary['terminal_handover_success'])} |")


SUMMARY_HEADER = ("| 모델 | cases | checker.valid | horizon 완료 | 접촉 substep fault 종료 | 명령 범위 거부 종료 | reach | left_grasp | "
                  "pick_lift 이벤트 | terminal pick_lift | terminal handover |\n|---|---|---|---|---|---|---|---|---|---|---|")


def pregrasp_markdown(pregrasp):
    """Non-abort post-hoc diagnostic and abort-contract run are reported as different things."""
    lines = ["", "## Pregrasp 진단 (seed 5000 step 431 복원 상태, Val과 별개)"]
    legacy, strict = pregrasp["legacy_nonabort_reanalysis.json"], pregrasp["abort_contract_summary.json"]
    if legacy == UNAVAILABLE and strict == UNAVAILABLE:
        return lines + ["- unavailable (이 run-id에서 pregrasp 재분석/재평가를 실행하지 않음)"]
    lines += ["", "| 모델 | 실행 계약 | 완료 transition | 첫 fault SIM시각 / 대상 | 유효 prefix 이벤트 | fault 이후 진단 이벤트 | checker.valid | terminal pick_lift |",
              "|---|---|---|---|---|---|---|---|"]
    def row(label, contract, r):
        fault = r["first_fault"]
        fault_text = "-" if fault is None else f"{fault['sim_time']:.3f} / {fault['pairs'] if fault['pairs'] else fault['reason']}"
        fmt = lambda ev: ", ".join(f"{k}@{v:.2f}" for k, v in ev.items()) or "none"
        return (f"| {label} | {contract} | {r['completed_transitions']} | {fault_text} | {fmt(r['events_valid_prefix'])} | "
                f"{fmt(r['events_post_fault_diagnostic'])} | {r['checker_valid']} | {r['terminal_pick_lift_success']} |")
    for source, contract in ((legacy, "비중단 사후검사 (기존 저장 rollout 재분석)"), (strict, "substep fault 중단 (재평가 1회)")):
        if source == UNAVAILABLE:
            lines.append(f"| - | {contract} | unavailable | | | | | |")
            continue
        for key in sorted(source["models"]):
            lines.append(row(key, contract, source["models"][key]))
    return lines


def main():
    parser = argparse.ArgumentParser(description="Re-aggregate stored R5 A/B Val results into a new run directory")
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--root", type=Path, default=FACTORY_ROOT)
    args = parser.parse_args()
    out = args.root / "evaluation" / f"r5-reconciled-{args.run_id}"
    val_out = out / "val"
    val_out.mkdir(parents=True, exist_ok=False)  # never overwrite an earlier reconciliation
    results = {}
    for key, (name, rel) in LEGACY_VAL.items():
        # student_eval.run has a fixed range(2000) control loop (40.0 s at control_dt 0.02).
        results[key] = reconcile_val(args.root / rel, name, horizon_transitions=2000)
        (val_out / f"{key}.json").write_text(json.dumps(results[key], indent=2))
    pregrasp = {}
    for name in ("legacy_nonabort_reanalysis.json", "abort_contract_summary.json"):
        path = out / "pregrasp" / name
        pregrasp[name] = _load_json(path) if path.exists() else UNAVAILABLE
    comparison = dict(
        run_id=args.run_id, scope="re-aggregation of stored runs; no new Val rollout, no training",
        models={k: dict(model_name=v["model_name"], source_dir=v["source_dir"], summary=v["summary"],
                        legacy_differences=v["legacy_differences"]) for k, v in results.items()},
        pregrasp=pregrasp)
    (out / "comparison.json").write_text(json.dumps(comparison, indent=2))
    text = ["# R5 A/B Val 정정 집계 (run-id: %s)" % args.run_id, "",
            "저장된 기존 실행 원본(evaluation.json / scoring_trace.json / action_trace.json / episode.npz)의 재집계다. 새 rollout·학습 없음.", "",
            SUMMARY_HEADER]
    text += [summary_row(v["model_name"], v["summary"]) for v in results.values()]
    for v in results.values():
        text += ["", f"## {v['model_name']}", f"원본: `{v['source_dir']}`", "", markdown_table(v), "", "종전 summary와 달라진 값:"]
        text += [f"- {d['field']}: {d['legacy']} → {d['corrected']} ({d['cause']})" for d in v["legacy_differences"]] or ["- 없음"]
    text += pregrasp_markdown(pregrasp)
    (out / "comparison.md").write_text("\n".join(text) + "\n")
    print("\n".join(text))
    print(f"\n[written] {out}")


if __name__ == "__main__":
    main()
