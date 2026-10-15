#!/usr/bin/env python3
"""TJJ MVP R2 / work6: the two 3-view ACT candidates, one training run each (SIM data only; no Val / Test; no hardware).

  START_3V    START_R1_E weights + its stored AdamW state, 5,000 updates on the Train69 pick stretch [0, T) exactly as
              START_R1_E sampled it (the first 5,000 updates of its own plan), now with the top view
  RECEIVE_3V  RECEIVE_BRIDGE_CTRL weights + its stored AdamW state, 5,000 updates on the EXPAND DATA plan (six slots of the
              stored replay + two slots of the 15 parent-balanced G corrections), now with the top view

Input of both: left wrist RGB + right wrist RGB + OS30A top RGB (SIM gray3, 160 x 120) + q12. Loss, targets, normalization,
chunk and optimizer settings are those of the 2-view runs. No extension, no checkpoint selection, no default is changed.
Execution code of the run identity (with tjj_mvp_r2.py).
"""
import argparse
import json
import os
import shutil
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import train_rollout_diagnostic as td   # noqa: E402
import receive_candidate as rc   # noqa: E402
import receive_r3_recheck as rk   # noqa: E402
import start_pick_d1 as d1   # noqa: E402
import start_pick_r1 as p1   # noqa: E402
import constraint_causal_c0 as c0   # noqa: E402
import receive_expand_r1 as ex   # noqa: E402
import tjj_alignment_r1 as al   # noqa: E402
import server_s0 as s0   # noqa: E402

FACTORY_ROOT = td.FACTORY_ROOT
RUN = "tjj-mvp-r2-work6-20261006-01"
UPDATES, BATCH, SEED = 5000, 8, 4100
CHECKPOINT = "step-005000"
NAMES = dict(start="START_3V", receive="RECEIVE_3V")
TOP, TOP_KEY = al.TOP, al.TOP_KEY

load, sha = rk.load, rk.sha


def run_dir():
    return FACTORY_ROOT / "evaluation" / RUN


def training_dir(role):
    return FACTORY_ROOT / "training" / f"{RUN}-{role}3v"


def checkpoint(role):
    return training_dir(role) / CHECKPOINT


def sources():
    m = c0.bridge_models()
    return dict(start=Path(m["START_R1_E"]["path"]), receive=Path(m["RECEIVE_BRIDGE_CTRL"]["path"]))


def demo_tops(view):
    """Top frames of the demonstrations' own files for the rows of a Train69 view."""
    man, out = d1.manifest(), {}
    for r in view.rows:
        a, b = r["receive"]
        with np.load(Path(man[r["manifest_index"]]["path"]) / "cameras" / f"{TOP}.npz", allow_pickle=False) as c:
            out[r["view_key"]] = c["rgb"][a:b]
    return out


def sidecar_tops(view):
    side = {e["view_key"]: e for e in load(al.run_dir() / "sidecar_manifest.json")["entries"] if e.get("sidecar")}
    out = {}
    for r in view.rows:
        e = side[r["view_key"]]
        if sha(e["sidecar"]) != e["sidecar_sha256"]:
            raise RuntimeError("a sidecar changed")
        out[r["view_key"]] = np.load(e["sidecar"])["rgb"]
    return out


def data(role):
    """(stats, {view name: ThreeView}, index {key: (view name, offset)}, keys, timesteps, description)."""
    if role == "start":
        table = load(p1.run_dir() / "prefix_manifest.json")
        st, replay = p1.replay_set()
        view, offset = p1.pick_view(st, replay, table)
        with np.load(p1.run_dir() / "sample_plan.npz", allow_pickle=False) as a:
            keys, ts = p1.plan_rows({k: a[k] for k in a.files}, "e")
        with np.load(p1.training_dir("e") / "consumed_samples.npz", allow_pickle=False) as a:
            if not (np.array_equal(a["episode"], keys) and np.array_equal(a["timestep"], ts)):
                raise RuntimeError("the START-PICK R1 plan is not what START_R1_E consumed")
        views = dict(pick=al.ThreeView(view, demo_tops(view)))
        index = {k: ("pick", o) for k, o in offset.items()}
        what = dict(view="Train69 pick stretch [0, T) of START-PICK R1 (69 parents)", plan="the first 5,000 updates of START_R1_E's own consumed plan (2 slots focus 3 + 6 slots other 66 per update)",
                    plan_file=str(p1.run_dir() / "sample_plan.npz"), plan_sha256=sha(p1.run_dir() / "sample_plan.npz"), top="the demonstrations' own os30a frames of the same rows")
        return st, views, index, keys[:UPDATES], ts[:UPDATES], what
    st, v2, idx, _ = ex.datasets()
    with np.load(ex.run_dir() / "sample_plan.npz", allow_pickle=False) as a:
        keys, ts = a["data_key"].copy(), a["data_t"].copy()
    used = {idx[int(k)][0] for k in np.unique(keys)}
    views = {}
    for name in used:
        views[name] = al.ThreeView(v2[name], demo_tops(v2[name]) if name == "train" else sidecar_tops(v2[name]))
    index = {k: (name, off) for k, (name, off, _) in idx.items() if name in used}
    what = dict(view="RECEIVE replay views (Train69 receiving, earlier corrections, R2 actual-entry, R3 onset) + the 15 G corrections", plan="the EXPAND DATA plan as stored: slots 0-5 = RECEIVE_BRIDGE_CTRL's own "
                "replay samples, slots 6-7 = G corrections, parent first (15 parents, 666-667 each)", plan_file=str(ex.run_dir() / "sample_plan.npz"), plan_sha256=sha(ex.run_dir() / "sample_plan.npz"),
                top="demonstrations' own os30a frames for Train69; top-view sidecars rendered from the stored states for every correction")
    return st, views, index, keys, ts, what


def trained(role):
    out = training_dir(role)
    try:
        run = load(out / "run.json")
    except (FileNotFoundError, json.JSONDecodeError):
        return None
    ok = (run.get("run_id") == RUN and run.get("steps") == UPDATES and run.get("consumed_equals_plan") and run.get("execution_source_sha256") == sha(__file__)
          and (out / CHECKPOINT / "model.safetensors").exists() and run.get("model_sha256") == sha(out / CHECKPOINT / "model.safetensors"))
    return run if ok else None


def train(args):
    import warnings
    import arm_output_trial as trial
    import single_demo_trial as sd
    import torch
    from torch.profiler import profile, ProfilerActivity
    from torch.utils.data import default_collate
    role = args.role
    if trained(role) is not None:
        print(f"[stored training] {training_dir(role)}")
        return
    out = training_dir(role)
    if out.exists():
        kept = out.with_name(out.name + time.strftime(".unfinished-%Y%m%dT%H%M%S"))
        out.rename(kept)
        print(f"[unfinished attempt kept] {kept}")
    load_report = trial.gpu_other_load()
    trial.apply_reproducibility_settings()
    before = c0.bridge_models()
    source = sources()[role]
    st, views, index, keys, ts, what = data(role)
    two, policy, migration = al.three_view_policy(source)
    del two
    policy = policy.cuda()
    policy.check_normalization(st)
    optimizer = torch.optim.AdamW(policy.get_optim_params(), lr=1e-4, weight_decay=1e-4)
    state = torch.load(source / "training_state.pt", map_location="cpu", weights_only=False)
    optimizer.load_state_dict(state["optimizer"])                             # parameter names, shapes and order are those of the 2-view model: a one-to-one state
    groups = [dict(lr=g["lr"], weight_decay=g["weight_decay"], tensors=len(g["params"])) for g in optimizer.param_groups]
    adam = lambda: sorted({int(v["step"]) for v in optimizer.state_dict()["state"].values() if "step" in v})
    adam_at_load = adam()
    with np.load(rc.HORIZON_WEIGHTS, allow_pickle=False) as a:
        r = torch.tensor(a["r"], dtype=torch.float32, device="cuda")
    out.mkdir(parents=True, exist_ok=False)
    shutil.copy2(source.parent / "normalization.npz", out / "normalization.npz")
    consumed_e, consumed_t = np.full((UPDATES, BATCH), -1, dtype=np.int64), np.full((UPDATES, BATCH), -1, dtype=np.int64)
    valid_targets, cuda_info, prof, backbone_inputs = 0, {}, None, []
    hook = policy.model.backbone.register_forward_hook(lambda module, a, o: backbone_inputs.append(1))
    torch.manual_seed(SEED)
    torch.cuda.reset_peak_memory_stats()
    started_wall, started = time.strftime("%Y-%m-%dT%H:%M:%S%z"), time.monotonic()
    with (out / "metrics.jsonl").open("x") as log, warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        for step in range(1, UPDATES + 1):
            u = step - 1
            items = []
            for k, t in zip(keys[u], ts[u]):
                name, offset = index[int(k)]
                items.append(views[name][offset + int(t)])
            batch, meta = trial.split_meta(default_collate(items))
            consumed_e[u], consumed_t[u] = meta["meta_episode"].numpy(), meta["meta_t"].numpy()
            valid_targets += int((~batch["action_is_pad"]).sum())
            batch = {k: v.cuda(non_blocking=True) for k, v in batch.items()}
            if step == 11:                                                  # three ordinary updates inside the profiler; nothing extra is computed
                prof = profile(activities=[ProfilerActivity.CPU, ProfilerActivity.CUDA])
                prof.__enter__()
                cuda_info = dict(parameter_devices=sorted({str(p.device) for p in policy.parameters()}), buffer_devices=sorted({str(b.device) for b in policy.buffers()}),
                                 batch={k: dict(shape=list(v.shape), device=str(v.device)) for k, v in batch.items()}, image_features=list(policy.config.image_features))
            policy.train()
            loss, parts = sd.loss_terms(policy, batch, r)
            if not torch.isfinite(loss):
                raise RuntimeError(f"non-finite loss at step {step}")
            optimizer.zero_grad(set_to_none=True)
            loss.backward()
            grad = torch.nn.utils.clip_grad_norm_(policy.parameters(), 1.0, error_if_nonfinite=True)
            optimizer.step()
            if step == 11:
                cuda_info["loss_device"] = str(loss.device)
            if step == 13 and prof is not None:
                torch.cuda.synchronize()
                prof.__exit__(None, None, None)
                rows = sorted(((float(getattr(e, "device_time_total", None) or getattr(e, "cuda_time_total", 0) or 0), e.key, int(e.count)) for e in prof.key_averages()), reverse=True)
                rows = [x for x in rows if x[0] > 0]
                cuda_info["profiled_updates"] = dict(steps=[11, 12, 13], cuda_kernel_events=len(rows), top=[dict(name=k[:60], device_time_us=round(t, 1), calls=c) for t, k, c in rows[:10]])
                prof = None
            if step == 1 or step % 100 == 0:
                entry = dict(step=step, elapsed_s=round(time.monotonic() - started, 2), loss=loss.item(), gradient_norm=grad.item(), **parts)
                log.write(json.dumps(entry) + "\n")
                log.flush()
                if step % 1000 == 0 or step == 1:
                    print(f"[{NAMES[role]}][{step:05d}] loss={entry['loss']:.4f} arm={parts['arm_l1_term']:.4f} grip={parts['gripper_l1_term']:.4f} grad={entry['gradient_norm']:.3f} t={entry['elapsed_s']:.0f}s", flush=True)
        name = out / CHECKPOINT
        policy.save_pretrained(name)
        torch.save(dict(optimizer=optimizer.state_dict(), step=UPDATES, inherited_step=int(state["step"]), cpu_rng=torch.get_rng_state(), cuda_rng=torch.cuda.get_rng_state()), name / "training_state.pt")
    hook.remove()
    seconds = round(time.monotonic() - started, 2)
    if not (np.array_equal(consumed_e, keys) and np.array_equal(consumed_t, ts)):
        raise RuntimeError("consumed samples differ from the plan")
    np.savez_compressed(out / "consumed_samples.npz", episode=consumed_e, timestep=consumed_t)
    cuda_info.update(pid=os.getpid(), gpu=s0.host().get("gpu"), peak_memory_mib=round(torch.cuda.max_memory_allocated() / 2**20, 1), torch=torch.__version__, cuda=torch.version.cuda,
                     backbone_inputs_per_update=len(backbone_inputs) / UPDATES)
    per_view = {}
    for k in consumed_e.ravel():
        per_view[index[int(k)][0]] = per_view.get(index[int(k)][0], 0) + 1
    run = dict(kind="DAPIER ACT TJJ MVP R2 3-view candidate (SIM only)", run_id=RUN, role=role, name=NAMES[role], policy_variant=policy.config.type, input_features=list(policy.config.input_features),
               image_features=list(policy.config.image_features), top_view="os30a_UNVERIFIED gray3 160x120 (SIM_ONLY / REAL_MODALITY_UNVERIFIED)", data=what, hardware_access=False, val_test_used=False,
               seed=SEED, batch_size=BATCH, steps=UPDATES, warm_start=str(source), warm_start_model_sha256=sha(source / "model.safetensors"), migration=migration,
               optimizer_state_loaded=True, optimizer_state_sha256=sha(source / "training_state.pt"), optimizer_step_at_load=int(state["step"]), adam_step_counters_at_load=adam_at_load,
               adam_step_counters_at_end=adam(), optimizer_param_groups=groups, normalization_sha256=sha(out / "normalization.npz"), consumed_equals_plan=True, samples=int(consumed_e.size),
               samples_per_view=per_view, valid_action_targets=valid_targets, precision="float32, no autocast", model_sha256=sha(name / "model.safetensors"), config_sha256=sha(name / "config.json"),
               execution_source_sha256=sha(__file__), stored_models_unchanged_after=c0.bridge_models() == before, started=started_wall, training_seconds=seconds, updates_per_second=round(UPDATES / seconds, 1),
               cuda=cuda_info, environment=trial.environment_record(), host=s0.host(), gpu_other_load_at_start=load_report,
               deterministic_warnings=sorted({str(w.message).split("\n")[0][:200] for w in caught}), status="candidate only; no default model is replaced")
    (out / "run.json").write_text(json.dumps(run, indent=2, default=str))
    print(json.dumps(dict(role=role, seconds=seconds, model=run["model_sha256"][:12], devices=cuda_info.get("parameter_devices"), kernels=(cuda_info.get("profiled_updates") or {}).get("cuda_kernel_events"),
                          backbone_inputs_per_update=cuda_info["backbone_inputs_per_update"], per_view=per_view)))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("role", choices=("start", "receive"))
    args = parser.parse_args(argv)
    import arm_output_trial  # noqa: F401
    train(args)


if __name__ == "__main__":
    main()
