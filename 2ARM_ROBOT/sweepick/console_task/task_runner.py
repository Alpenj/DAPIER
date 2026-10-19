"""Task runner for the operator page (2026-10-18): one button per arm runs the taught pick-and-place replay of the floor
candidate native, and the three taught poses can be recorded from here (read-only joint + wrist capture). Runs pile up in
RUNS; analysis and the move to work6 are requested from Claude afterwards. One job at a time, never next to the A owner."""
import json
import os
import shutil
import signal
import subprocess
import threading
import time
from pathlib import Path

HOME = Path.home()
NATIVE = HOME / "sweepick_261016_floor_native/revision"
PYTHON = HOME / "DAPIER/so101_imitation_learning/.venv/bin/python"
REPLAY = NATIVE / "sweepick_261018_taught_replay.py"
SESSION = NATIVE / "sweepick_pick01.py"                                         # auto: head finds the block, the arm with the better chance is chosen, the wrist view is aligned to the taught one
GRAB = HOME / "sweepick_261018_right_r0/handguide/grab.py"
REFS = {"right": HOME / "sweepick_261018_right_r0/refs", "left": HOME / "sweepick_261018_right_r0/refs_left"}
POSES = {"grasp": "H0_grasp", "above": "H2_above_c", "pregrasp": "H6_pregrasp"}
OPTIONAL_POSES = {"carry": "H9_carry"}                                             # block held, arm folded over the base for the drive (2026-10-18)
RUNS = HOME / "sweepick_task_runs"
SET_IR = HOME / "Desktop/sweepick_261018_os30a_depth_probe/set_ir2.sh"           # OS30A IR projector falls back to 0 (off) whenever the camera loses power (2026-10-19 hub move: depth 9.5%)
LOG_LINES = 60


class TaskError(Exception):
    pass


class TaskRunner:
    def __init__(self, owner_busy=lambda: False):
        self.owner_busy = owner_busy; self.lock = threading.Lock(); self.job = None; RUNS.mkdir(parents=True, exist_ok=True)

    # -- jobs ------------------------------------------------------------------------------------------------------------
    def _alive(self):
        return self.job is not None and self.job["proc"].poll() is None

    def _refuse_if_busy(self):
        if self._alive():
            raise TaskError(f"다른 작업이 실행 중입니다: {self.job['kind']} {self.job['side']}")
        if self.owner_busy():
            raise TaskError("기존 세션(owner)이 실행 중입니다. 끝난 뒤 다시 누르세요.")

    @staticmethod
    def _ensure_ir():
        """Set the OS30A IR projector to 2 before every run (the camera only; the script refuses while a reader holds it)."""
        try:
            r = subprocess.run([str(SET_IR)], capture_output=True, text=True, timeout=90)
        except (OSError, subprocess.TimeoutExpired) as e:
            raise TaskError(f"OS30A IR 설정 실패: {type(e).__name__}: {e}")
        text = (r.stdout or "") + (r.stderr or "")
        if r.returncode != 0 or '"ir_value":2' not in text.replace(" ", ""):
            raise TaskError(f"OS30A IR 설정 실패 (exit {r.returncode}): {text.strip()[-300:]}")
        return text.strip()

    def start(self, side, record=True, lift_to="above"):
        if side not in ("left", "right", "auto"):
            raise TaskError("알 수 없는 실행")
        with self.lock:
            self._refuse_if_busy()
            ir = self._ensure_ir()
            if side == "auto":
                missing = [f"{s_}:{p}" for s_ in REFS for p in ("H6_pregrasp", "H2_above_c") if not (REFS[s_] / p / f"{s_}_wrist_rgb.jpg").exists()]
                if missing:
                    raise TaskError(f"영상 정렬 기준 화면이 없습니다: {missing}")
                out = RUNS / time.strftime("%Y%m%d_%H%M%S_auto")
                cmd = [str(PYTHON), str(SESSION), "session", str(out), "--execute"] + (["--record"] if record else [])
                log = open(str(out) + ".log", "w"); log.write(f"[task_runner] OS30A IR: {ir}\n"); log.flush()
                proc = subprocess.Popen(cmd, cwd=NATIVE, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                self.job = dict(kind="session", side="auto", out=str(out), proc=proc, started=time.time(), record=record, cmd=cmd)
                return self.status()
            missing = [p for p in POSES.values() if not (REFS[side] / p / "ticks.json").exists()]
            if missing:
                raise TaskError(f"{side} 기준 자세가 없습니다: {missing}. 아래 '기준 자세 기록'으로 먼저 기록하세요.")
            out = RUNS / time.strftime(f"%Y%m%d_%H%M%S_{side}")
            cmd = [str(PYTHON), str(REPLAY), str(out), "--refs", str(REFS[side]), "--side", side, "--execute", "--lift-to", lift_to] + (["--record"] if record else [])
            log = open(str(out) + ".log", "w"); log.write(f"[task_runner] OS30A IR: {ir}\n"); log.flush()
            proc = subprocess.Popen(cmd, cwd=NATIVE, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            self.job = dict(kind="task", side=side, out=str(out), proc=proc, started=time.time(), record=record, cmd=cmd)
            return self.status()

    def teach(self, side, pose):
        if side not in ("left", "right") or pose not in {**POSES, **OPTIONAL_POSES}:
            raise TaskError("알 수 없는 팔/자세")
        with self.lock:
            self._refuse_if_busy()
            d = REFS[side] / {**POSES, **OPTIONAL_POSES}[pose]
            if d.exists():
                shutil.move(str(d), str(d) + time.strftime("_old_%H%M%S"))
            d.parent.mkdir(parents=True, exist_ok=True)
            r = subprocess.run([str(PYTHON), str(GRAB), str(d), "--no-top"], cwd=GRAB.parent, capture_output=True, text=True, timeout=120)
            t = d / "ticks.json"
            if not t.exists():
                raise TaskError(f"기록 실패: {r.stdout[-300:]} {r.stderr[-300:]}")
            ticks = json.loads(t.read_text())
            img = d / f"{side}_wrist_rgb.jpg"
            if pose in POSES and not img.exists():                              # the taught view is used by the wrist alignment: a pose without its image is not a reference
                shutil.move(str(d), str(d) + time.strftime("_noimage_%H%M%S"))
                raise TaskError(f"{side} 손목 카메라 사진이 찍히지 않았습니다(카메라 응답 없음). 같은 자세로 한 번 더 눌러 주세요.")
            if any(ticks[side]["torque"].values()):
                raise TaskError("토크가 켜져 있습니다. 손으로 잡는 기록은 토크가 꺼진 상태에서만 합니다.")
            return dict(side=side, pose=pose, ticks=ticks[side]["ticks"], folder=str(d))

    def stop(self):
        if not self._alive():
            raise TaskError("실행 중인 작업이 없습니다.")
        Path(self.job["out"] + ".STOP").write_text("operator stop from the console")       # the executor's stop_requested: it stops and holds
        return self.status()

    # -- views -----------------------------------------------------------------------------------------------------------
    @staticmethod
    def _summary(out):
        f = Path(out) / "REPLAY.json"
        if not f.exists() and (Path(out) / "SESSION.json").exists():          # an auto (session) run: its trail
            try:
                t = json.loads((Path(out) / "SESSION.json").read_text())
            except Exception:
                return dict(steps=[], result=None)
            steps = [dict(step=(x.get("step") or "") + (f" {x.get('height')}#{x.get('k')}" if x.get("step") == "visual" else ""), result=x.get("controller_result") or x.get("result") or (("OK" if x.get("ok") else "NOT_READY") if "ok" in x else None),
                          collision=None, note=str(x.get("end") or x.get("why") or "")[:200]) for x in t.get("trail", [])]
            close = [x for x in t.get("trail", []) if x.get("step") == "close"]
            return dict(steps=steps, result=t.get("result"), gripped=bool(close and close[-1].get("controller_result") == "GRIP_CONTACT_HOLDING"), prepare=None, record=None)
        try:
            d = json.loads(f.read_text())
        except Exception:
            return dict(steps=[], result=None)
        steps = [dict(step=s["step"], result=s.get("result"), collision=(s.get("collision") or {}).get("verdict"),
                      note=(s.get("why") or s.get("needs_a_person") or (s.get("stopped_by") or {}).get("detail") or "")[:200]) for s in d.get("steps", [])]
        return dict(steps=steps, result=d.get("result"), gripped=d.get("gripped"), prepare=d.get("prepare"), record=(d.get("record") or {}).get("session_id"))

    def status(self):
        j = self.job
        if j is None:
            return dict(running=False, job=None)
        alive = self._alive(); log = Path(j["out"] + ".log")
        tail = log.read_text(errors="replace").splitlines()[-LOG_LINES:] if log.exists() else []
        tail = [l for l in tail if "Warning" not in l and "nanmedian" not in l]
        return dict(running=alive, job=dict(side=j["side"], out=Path(j["out"]).name, started=j["started"], record=j["record"], elapsed_s=time.time() - j["started"],
                                            returncode=j["proc"].poll()), **self._summary(j["out"]), log=tail)

    def runs(self):
        rows = []
        for d in sorted((p for p in RUNS.iterdir() if p.is_dir()), reverse=True)[:40]:
            s = self._summary(d); size = sum(f.stat().st_size for f in d.rglob("*") if f.is_file())
            rows.append(dict(name=d.name, result=s["result"], gripped=s.get("gripped"), size_mb=round(size / 1e6, 1),
                             offloaded=(d / "OFFLOADED.json").exists(), analyzed=(d / "ANALYSIS.json").exists()))
        du = shutil.disk_usage(HOME)
        return dict(runs=rows, disk=dict(free_gb=round(du.free / 1e9, 1), used_pct=round(100 * du.used / du.total)),
                    refs={s: {k: (REFS[s] / v / "ticks.json").exists() for k, v in {**POSES, **OPTIONAL_POSES}.items()} for s in REFS})
