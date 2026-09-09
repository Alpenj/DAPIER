"""문제 2 장치 없는 CPU 검사: python test_hand.py.

--smoke-training은 임시 합성 이미지로만 1 Epoch 학습·평가·보고서 흐름을
추가 검사한다. 해당 옵션에서만 CUDA가 있으면 사용하며 실제 성능을 측정하지 않는다.
실제 촬영 폴더와 8766 서버를 사용하지 않는다. 추가 테스트 프레임워크는 필요 없다.
"""
import argparse
import base64
import io
import json
from pathlib import Path
import sys
import tempfile
import threading
from types import SimpleNamespace
from unittest.mock import patch
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import numpy as np
from PIL import Image
import torch
from torch import nn
from torch.utils.data import DataLoader, TensorDataset

import hand_exam as app


def expect_error(call, kind=ValueError):
    try:
        call()
    except kind:
        return
    raise AssertionError(f"{kind.__name__} was not raised")


def jpeg(size=(224, 224), format="JPEG"):
    buffer = io.BytesIO()
    Image.new("RGB", size, "teal").save(buffer, format=format)
    return buffer.getvalue()


def check_teacher_and_datasets(root):
    rows = []
    for label, name in enumerate(app.CLASSES):
        path = f"{name}.jpg"
        (root/path).write_bytes(jpeg())
        rows.append({"path": path, "label": label, "session": f"session-{label}"})
    episodes = app.make_episodes(rows, seed=42)
    repeated = app.make_episodes(rows, seed=42)
    assert np.count_nonzero(episodes[0]["states"][0]) == 0, "UI open start missing from teacher"
    assert np.all(episodes[1]["states"][0] > 0), "random starts must also remain"
    for e, again in zip(episodes, repeated):
        assert e["states"].shape == (app.EPISODE_STEPS+1, app.JOINTS)
        assert e["actions"].shape == (app.EPISODE_STEPS, app.JOINTS)
        np.testing.assert_array_equal(e["states"], again["states"])
        for key in ("states", "actions", "memory", "memory_target"):
            assert np.isfinite(e[key]).all()
            assert (e[key] >= 0).all() and (e[key] <= app.MAX_RAD).all()
        np.testing.assert_allclose(e["states"][1:], e["states"][:-1]+app.LAG*(e["actions"]-e["states"][:-1]), atol=1e-7)
        name = app.CLASSES[e["label"]]
        if name == "none":
            assert app.counter_pose(name) is None
            np.testing.assert_allclose(e["states"], np.broadcast_to(e["states"][0], e["states"].shape), atol=1e-7)
            np.testing.assert_allclose(e["actions"], np.broadcast_to(e["states"][0], e["actions"].shape), atol=1e-7)
        else:
            np.testing.assert_allclose(e["actions"][-1], app.counter_pose(name), atol=1e-6)
        # 초반 cue를 포함한 32시점과 cue가 전혀 없는 마지막 8시점을 구분한다.
        np.testing.assert_array_equal(e["memory"][:4], np.broadcast_to(e["memory"][0], (4, app.JOINTS)))
        np.testing.assert_array_equal(e["memory"][4:], np.full((app.HISTORY-4, app.JOINTS), app.MAX_RAD/2, dtype=np.float32))
        np.testing.assert_allclose(e["memory_target"], e["memory"][-1]+.2*(e["memory"][0]-e["memory"][-1]), atol=1e-7)
    np.testing.assert_allclose(app.counter_pose("scissors"), [1.05, .95]*5)
    np.testing.assert_allclose(app.counter_pose("rock"), [.05, .05]*5)
    np.testing.assert_allclose(app.counter_pose("paper"), [1.05, .95, .05, .05, .05, .05, 1.05, .95, 1.05, .95])
    assert not np.array_equal(episodes[0]["memory_target"], episodes[1]["memory_target"])
    np.testing.assert_array_equal(episodes[0]["memory"][-8:], episodes[1]["memory"][-8:])

    sequence = app.SequenceDataset(episodes)
    assert len(sequence) == len(episodes)*6
    for index, (episode_id, t) in enumerate(sequence.samples):
        history, future, memory = sequence[index]
        e = episodes[episode_id]
        assert history.shape == (app.HISTORY, app.JOINTS) and future.shape == (app.JOINTS,)
        if t == -1:
            assert memory
            np.testing.assert_array_equal(history.numpy(), e["memory"])
            np.testing.assert_array_equal(future.numpy(), e["memory_target"])
        else:
            assert not memory
            np.testing.assert_array_equal(history.numpy(), e["states"][t-app.HISTORY+1:t+1])
            np.testing.assert_array_equal(future.numpy(), e["states"][t+1])
    before = sequence[0][0].clone()
    original = episodes[0]["states"][app.HISTORY:].copy()
    episodes[0]["states"][app.HISTORY:] = 999
    assert torch.equal(sequence[0][0], before), "future states leaked into history"
    episodes[0]["states"][app.HISTORY:] = original

    actions = app.ActionDataset(root, episodes)
    for index, (episode_id, t) in enumerate(actions.samples):
        image, current, chunk, label = actions[index]
        assert image.shape == (3, 64, 64)
        assert chunk.shape == (app.CHUNK, app.JOINTS)
        np.testing.assert_array_equal(current.numpy(), episodes[episode_id]["states"][t])
        np.testing.assert_array_equal(chunk.numpy(), episodes[episode_id]["actions"][t:t+app.CHUNK])
        assert label == episodes[episode_id]["label"]
    escaped = app.ActionDataset(root, [{**episodes[0], "image": "../outside.jpg"}])
    expect_error(lambda: escaped[0])
    print("PASS: teacher bounds/counter/none/LAG, history-next alignment, action alignment, delayed cue, path guard")


def check_models():
    # 단위 행렬 projection으로 패치의 행 우선 순서와 내부 C/H/W 평탄화를 직접 비교한다.
    embedding = app.PatchEmbedding(patch=2, dimension=12)
    with torch.no_grad():
        embedding.projection.weight.copy_(torch.eye(12))
        embedding.projection.bias.zero_()
    pixels = torch.arange(2*3*4*6, dtype=torch.float32).reshape(2, 3, 4, 6)
    expected = torch.stack([pixels[:, :, y:y+2, x:x+2].flatten(1) for y in (0, 2) for x in (0, 2, 4)], dim=1)
    assert torch.equal(embedding(pixels), expected)
    assert isinstance(embedding.projection, nn.Linear)
    assert not any(isinstance(layer, nn.Conv2d) for layer in embedding.modules())
    expect_error(lambda: embedding(torch.zeros(1, 3, 5, 6)))

    image, joints = torch.randn(2, 3, 64, 64), torch.rand(2, app.JOINTS)*app.MAX_RAD
    vit = app.TinyViT(dimension=32).eval()
    encoder_inputs = []
    hook = vit.encoder.register_forward_pre_hook(lambda _model, inputs: encoder_inputs.append(inputs[0].detach().clone()))
    with torch.inference_mode():
        features, logits = vit(image)
        assert features.shape == (2, 65, 32) and logits.shape == (2, 4)
        assert torch.equal(logits, vit.classifier(features[:, 0]))
    hook.remove()
    assert encoder_inputs[0].shape == (2, 65, 32)
    torch.testing.assert_close(encoder_inputs[0][:, 0], (vit.cls+vit.position[:, :1]).expand(2, -1, -1)[:, 0])

    for name in app.SEQUENCE_MODELS:
        model = app.StatePredictor(name, dimension=32).eval()
        for length in (app.HISTORY, 8):
            with torch.inference_mode():
                result = model(torch.rand(2, length, app.JOINTS)*app.MAX_RAD)
            assert result.shape == (2, app.JOINTS)
            assert torch.isfinite(result).all() and (result >= 0).all() and (result <= app.MAX_RAD).all()
    expect_error(lambda: app.StatePredictor("unknown"))

    act = app.ActionCVAE(dimension=32, latent=4)
    target = torch.rand(2, app.CHUNK, app.JOINTS)*app.MAX_RAD
    predicted, mu, logvar, logits = act(image, joints, actions=target)
    assert predicted.shape == (2, app.CHUNK, app.JOINTS) and logits.shape == (2, 4)
    assert mu.shape == logvar.shape == (2, 4)
    assert (predicted >= 0).all() and (predicted <= app.MAX_RAD).all()
    mu.retain_grad(); logvar.retain_grad()
    # KL 자체가 아니라 재구성 경로만으로 μ/logvar에 gradient가 전달되어야 한다.
    nn.functional.mse_loss(predicted, target).backward()
    for value in (mu, logvar):
        assert value.grad is not None and torch.isfinite(value.grad).all() and value.grad.abs().sum() > 0
    for layer in (act.mu, act.logvar, act.z_embed):
        assert layer.weight.grad is not None and layer.weight.grad.abs().sum() > 0
    kl = -.5*(1+logvar-mu.square()-logvar.exp()).mean()
    assert torch.isfinite(kl) and kl >= 0

    act.eval()
    def forbid_posterior(_model, _inputs):
        raise AssertionError("evaluation accessed future action posterior")
    hook = act.posterior.register_forward_pre_hook(forbid_posterior)
    with torch.inference_mode():
        zero1 = act(image, joints)
        zero2 = act(image, joints)
        assert zero1[1] is zero1[2] is None
        assert torch.equal(zero1[0], zero2[0]), "z=0 evaluation must be deterministic"
        torch.manual_seed(11); sample1 = act(image, joints, sample=True)[0]
        torch.manual_seed(12); sample2 = act(image, joints, sample=True)[0]
        assert not torch.allclose(sample1, sample2, atol=1e-7, rtol=0), "sampled z did not affect actions"
    # 평가 target을 크게 바꿔도 예측은 같아야 한다. target은 손실 계산에만 쓴다.
    labels = torch.tensor([0, 1])
    predictions = []
    output_hook = act.register_forward_hook(lambda _m, _args, result: predictions.append(result[0].detach().clone()))
    for targets in (torch.zeros_like(target), torch.full_like(target, app.MAX_RAD)):
        loader = DataLoader(TensorDataset(image, joints, targets, labels), batch_size=2)
        scores = app.action_pass(act, loader, torch.device("cpu"))
        assert all(np.isfinite(value) for value in scores.values())
    assert torch.equal(predictions[0], predictions[1]), "future targets changed test predictions"
    output_hook.remove(); hook.remove()
    print("PASS: Linear patch order, ViT CLS65, 3 state predictors, CVAE reparameterization gradients, prior-only evaluation")


def check_parser():
    with patch.object(sys, "argv", ["hand_exam.py", "train", "--run", "/tmp/not-executed-hand-test"]), patch.object(app, "train") as train:
        app.main()
        args = train.call_args.args[0]
        assert args.epochs == 10 and args.batch_size == 64 and args.seed == 42 and not args.smoke
        assert args.data == app.vision.ROOT/"data"
    with patch.object(sys, "argv", ["hand_exam.py", "train", "--run", "/tmp/not-executed-hand-test", "--smoke", "--epochs", "1", "--batch-size", "4", "--seed", "7"]), patch.object(app, "train") as train:
        app.main()
        args = train.call_args.args[0]
        assert args.smoke and args.epochs == 1 and args.batch_size == 4 and args.seed == 7
    expect_error(lambda: app.train(SimpleNamespace(epochs=0, batch_size=2)))
    expect_error(lambda: app.train(SimpleNamespace(epochs=10, batch_size=0)))
    print("PASS: CLI defaults/smoke parsing and argument range validation; no training launched")


def check_api(root):
    # 합성 미학습 체크포인트는 이 임시 서버의 경계 검사 전용이다. 실제 실행 폴더에는 쓰지 않는다.
    server = app.HandStudio(("127.0.0.1", 0), root/"source", root/"vision-runs", root/"hand-runs")
    assert server.device.type == "cpu"
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base = f"http://127.0.0.1:{server.server_port}"

    def request(path, payload=None, status=200, headers=None):
        body = json.dumps(payload).encode() if payload is not None else None
        req = Request(base+path, data=body, headers={"Content-Type": "application/json", **(headers or {})})
        try:
            response = urlopen(req, timeout=10)
        except HTTPError as error:
            response = error
        with response:
            data = json.load(response) if "application/json" in response.headers.get("Content-Type", "") else response.read().decode()
            assert response.code == status, (path, response.code, status, data)
        if status >= 400:
            assert isinstance(data, dict) and data.get("error")
        return data

    try:
        initial = request("/api/hand/status")
        assert not initial["ready_to_train"] and initial["source_sessions"] == 0 and initial["models"] == []
        assert "가상" in request("/hand")
        assert request("/api/status")["complete_sessions"] == 0
        with patch.object(app.subprocess, "Popen", side_effect=AssertionError("API test must not launch training")):
            request("/api/hand/train", {"epochs": 10}, 400)
            request("/api/hand/train", {"epochs": True}, 400)
            request("/api/hand/train", {"epochs": 101}, 400)
        request("/api/hand/predict?run=../../outside", {}, 400)
        request("/api/hand/predict", [], 400)
        request("/api/hand/status", status=400, headers={"Origin": "https://untrusted.example"})
        request("/hand-results/../../outside", status=400)

        identifier = "a"*32
        run = server.hand_runs/identifier
        run.mkdir()
        metadata = {"complete": True, "smoke": False}
        app.vision.write_json(run/"meta.json", metadata)
        model = app.ActionCVAE()
        app.save_checkpoint(run, "act", model, metadata)
        checkpoint = torch.load(run/"act.pt", map_location="cpu", weights_only=True)
        assert checkpoint["classes"] == list(app.CLASSES)
        assert checkpoint["chunk"] == app.CHUNK and checkpoint["joints"] == app.JOINTS and checkpoint["max_rad"] == app.MAX_RAD
        assert not (run/"act.pt.tmp").exists()
        url = f"/api/hand/predict?run={identifier}"
        payload = {"image": base64.b64encode(jpeg()).decode(), "joints": [0.]*10, "sample": False}
        for joints in ([], [0.]*9, [0.]*11, [True]*10, [None]*10, ["0"]*10, [-.01]*10, [1.401]*10, [float("nan")]*10, [float("inf")]*10):
            request(url, {**payload, "joints": joints}, 400)
        request(url, {**payload, "sample": 0}, 400)
        for image in (None, {}, "", "%%%%", base64.b64encode(b"not an image").decode(), base64.b64encode(jpeg((64, 64))).decode(), base64.b64encode(jpeg(format="PNG")).decode()):
            request(url, {**payload, "image": image}, 400)
        result = request(url, payload)
        assert np.asarray(result["actions"]).shape == (app.CHUNK, app.JOINTS)
        assert (np.asarray(result["actions"]) >= 0).all() and (np.asarray(result["actions"]) <= app.MAX_RAD).all()
        assert result["z_mode"] == "zero" and result["label"] in app.CLASSES
        assert len(result["probabilities"]) == 4 and abs(sum(result["probabilities"])-1) < 1e-6
        assert result["latency_ms"] >= 0 and np.isfinite(result["latency_ms"])
        repeated = request(url, {**payload, "image": "data:image/jpeg;base64,"+payload["image"]})
        np.testing.assert_array_equal(result["actions"], repeated["actions"])
        sampled = request(url, {**payload, "sample": True})
        assert sampled["z_mode"] == "sample"
        assert not np.allclose(result["actions"], sampled["actions"], atol=1e-7, rtol=0)

        # 상위 metadata와 모델 자체의 호환성을 각각 검사한다. 캐시 우회도 함께 막아야 한다.
        for key, value in (("complete", False), ("smoke", True)):
            app.vision.write_json(run/"meta.json", {**metadata, key: value})
            request(url, payload, 400)
        app.vision.write_json(run/"meta.json", metadata)
        for key, value in (("classes", list(reversed(app.CLASSES))), ("chunk", 9), ("joints", 11), ("smoke", True), ("max_rad", 2.0)):
            torch.save({**checkpoint, key: value}, run/"act.pt")
            server.hand_run = None
            request(url, payload, 400)
        torch.save(checkpoint, run/"act.pt")
        server.hand_run = None
        smoke_run = server.hand_runs/("b"*32)
        app.vision.write_json(smoke_run/"meta.json", {"complete": True, "smoke": True})
        assert request("/api/hand/status")["models"] == [{"run": identifier, "teacher_version": "random-start-v1"}]
        server.process = SimpleNamespace(poll=lambda: None)
        assert request("/api/hand/status")["training"]["running"]
        request(url, payload, 400)
        request("/api/hand/train", {"epochs": 10}, 400)
        request("/api/session", {}, 400)
        server.process = None
        print("PASS: ephemeral CPU API, invalid image/joint/origin rejection, checkpoint metadata, smoke gate, shared training slot")
    finally:
        server.process = None
        server.shutdown(); server.server_close(); thread.join(timeout=5)


def smoke_training(root):
    from test_rps import synthetic_sessions
    data, run = root/"synthetic-data", root/"synthetic-smoke-run"
    synthetic_sessions(data)
    app.train(argparse.Namespace(data=data, run=run, epochs=1, batch_size=64, seed=42, smoke=True))
    meta = app.vision.read_json(run/"meta.json")
    assert meta["complete"] and meta["smoke"] and meta["epochs"] == 1
    manifest = app.vision.read_json(run/"source_manifest.json")
    assert meta["source_fingerprint"] == manifest["fingerprint"]
    groups = {key: set(value) for key, value in manifest["session_groups"].items()}
    assert not groups["train"] & groups["val"] and not groups["train"] & groups["test"] and not groups["val"] & groups["test"]
    seq = app.vision.read_json(run/"sequence_metrics.json")
    assert [row["model"] for row in seq] == list(app.SEQUENCE_MODELS)
    assert all(np.isfinite(row[key]) and row[key] >= 0 for row in seq for key in ("mse", "memory_long_mse", "memory_short_mse"))
    act = app.vision.read_json(run/"act_metrics.json")
    assert all(np.isfinite(act[key]) and act[key] >= 0 for key in ("chunk_mse", "prior_sample_mse", "val_mse"))
    assert 0 <= act["vision_accuracy"] <= 1
    for name in (*app.SEQUENCE_MODELS, "act"):
        checkpoint = torch.load(run/f"{name}.pt", weights_only=True, map_location="cpu")
        assert checkpoint["smoke"] and checkpoint["max_rad"] == app.MAX_RAD
        assert len(app.vision.read_json(run/f"{name}_history.json")) == 1
    for file in ("report.html", "RESULTS.md", "learning_curves.png", "memory_comparison.png", "action_chunks.png", "vit_patches.png"):
        assert (run/file).is_file() and (run/file).stat().st_size > 0
    assert "제출 성능 아님" in (run/"RESULTS.md").read_text()
    print("PASS: SYNTHETIC ONLY — 4 one-epoch training/evaluation/checkpoint/report flows; no real accuracy measured")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--smoke-training", action="store_true")
    args = parser.parse_args()
    torch.set_num_threads(2)
    torch.manual_seed(42)
    with tempfile.TemporaryDirectory(prefix="hand-exam-check-") as directory:
        root = Path(directory)
        # 기본 검사는 GPU 여유와 관계없이 항상 CPU에서만 실행한다.
        with patch.object(torch.cuda, "is_available", return_value=False):
            check_teacher_and_datasets(root)
            check_models()
            check_parser()
            check_api(root)
        if args.smoke_training:
            smoke_training(root)
    print("ALL CHECKS PASSED — temporary synthetic fixtures only; no camera or physical robot opened")


if __name__ == "__main__":
    main()
