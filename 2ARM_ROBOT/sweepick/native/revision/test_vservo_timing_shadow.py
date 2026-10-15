"""2026-10-15: monotonic timing of the visual loop and the shadow-only input record. Recording only: the same fake
session sends exactly the same bus writes with and without these records (compared against the stored command log
of the unmodified source, produced by the same fixture)."""
import json, os
from pathlib import Path
import pytest
import sweepick_pick01 as p1
from test_sweepick_pick02_actual import base, cell, S, trail   # noqa: F401  (fixture)


def _session(tmp_path, monkeypatch, base, **world):
    out, f, stage_fn, run = cell(tmp_path, monkeypatch, base, block=3075, **world)
    r = run(); return out, f, r


@pytest.mark.parametrize("world", [dict(), dict(delta=4.0, k=0.8)])
def test_visual_rows_carry_monotonic_timing_and_shadow_lines(tmp_path, monkeypatch, base, world):
    out, f, r = _session(tmp_path, monkeypatch, base, **world)
    rows = trail(out, "visual"); assert rows
    for row in rows:
        m = row["timing_monotonic_s"]
        assert m["look_called"] <= m["look_returned"] <= m["judged"]
        if "planned" in m:
            assert m["judged"] <= m["correction_computed"] <= m["planned"]
    corrected = [row for row in rows if "correction" in row]
    for later in rows:
        if later.get("previous_align_stage_monotonic_s"):
            st = later["previous_align_stage_monotonic_s"]; assert st["stage_called"] <= st["stage_returned"] <= later["timing_monotonic_s"]["look_called"]
    lines = [json.loads(l) for l in (out / "VSERVO_SHADOW_INPUT.jsonl").read_text().splitlines()]
    assert lines and all(l["schema"] == p1.VSERVO_SHADOW_SCHEMA and l["executable"] is False and l["motor_write_authorized"] is False and l["role"] == "SHADOW_INPUT_ONLY" for l in lines)
    assert len(lines) == len([row for row in rows if row.get("ok") or "correction" in row])
    assert all(l["timing_monotonic_s"]["look_called"] <= l["timing_monotonic_s"]["judged"] for l in lines)
    if corrected:
        assert any(l["a_correction"] and l["a_correction"]["tool_move_along_jaw_mm"] == c["correction"]["tool_move_along_jaw_mm"] for l in lines for c in corrected)
    record = Path(os.environ.get("VSERVO_COMMAND_LOG", tmp_path)) / f"commands_{'corrected' if world else 'aligned'}.json"
    record.write_text(json.dumps(dict(order=S(out)["order"], writes=f.writes, result=r["result"])))
