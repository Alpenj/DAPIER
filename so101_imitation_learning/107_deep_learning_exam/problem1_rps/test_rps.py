"""장치 없는 실행 검사. 합성 이미지는 테스트 임시 폴더에서만 사용한다.

실행: python test_rps.py
--smoke-training: 네 모델의 학습·저장·평가·보고서 전체 흐름도 1 Epoch로 검사.
이 결과는 실제 손 모양 인식 성능이 아니다.
"""
import argparse
import io
import json
from pathlib import Path
import tempfile
import threading
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import numpy as np
from PIL import Image
import torch

import rps_exam as app


def expect_error(call, kind=ValueError):
    try:
        call()
    except kind:
        return
    raise AssertionError(f"{kind.__name__} was not raised")


def synthetic_sessions(data):
    rng = np.random.default_rng(3)
    for s in range(10):
        session = data/"sessions"/f"{s:032x}"
        app.write_json(session/"session.json", {"created": f"synthetic-{s:02d}"})
        for label in app.CLASSES:
            folder = session/label
            folder.mkdir()
            for n in range(app.FRAMES_PER_CLASS):
                pixels = rng.integers(0, 256, (32, 32, 3), dtype=np.uint8)
                Image.fromarray(pixels).resize((224, 224)).save(folder/f"{n+1:04d}.jpg")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--smoke-training", action="store_true")
    args = parser.parse_args()
    torch.set_num_threads(2)
    torch.manual_seed(42)
    with tempfile.TemporaryDirectory(prefix="rps-exam-check-") as temp:
        root = Path(temp)
        data = root/"dataset"
        synthetic_sessions(data)
        manifest = app.make_manifest(data)
        assert manifest == app.make_manifest(data)
        assert [len(manifest["splits"][s]) for s in ("train", "val", "test")] == [840, 240, 120]
        groups = {s: set(v) for s, v in manifest["session_groups"].items()}
        assert not groups["train"] & groups["val"]
        assert not groups["train"] & groups["test"]
        assert not groups["val"] & groups["test"]
        ds = app.HandDataset(data, manifest["splits"]["train"])
        assert ds[0][0].shape == (3, app.IMAGE_SIZE, app.IMAGE_SIZE)
        assert torch.equal(ds[0][0], ds[0][0])
        bad = app.HandDataset(data, [{"path": "../outside.jpg", "label": 0}])
        expect_error(lambda: bad[0])
        # 같은 이미지가 분할을 넘나들면 검출하는지 확인한다.
        first = data/manifest["splits"]["train"][0]["path"]
        other = data/manifest["splits"]["test"][0]["path"]
        original = other.read_bytes()
        other.write_bytes(first.read_bytes())
        expect_error(lambda: app.make_manifest(data))
        other.write_bytes(original)
        print("PASS: deterministic 7:2:1 grouped split, leakage guard, Custom Dataset")

        x, y = torch.randn(4, 3, 32, 32), torch.tensor([0, 1, 2, 3])
        for name in app.MODEL_NAMES:
            model = app.build_model(name, pretrained=False, size=32)
            if name == "cnn":
                assert sum(isinstance(m, torch.nn.Conv2d) for m in model.modules()) >= 3
            before = {k: p.clone() for k, p in model.state_dict().items()} if name == "resnet_frozen" else {}
            opt = app.make_optimizer(model, name)
            app.train_mode(model, name)
            out = model(x)
            assert out.shape == (4, 4)
            loss = torch.nn.functional.cross_entropy(out, y)
            loss.backward(); opt.step()
            if name == "resnet_frozen":
                after = model.state_dict()
                assert all(torch.equal(value, after[k]) for k, value in before.items() if not k.startswith("fc."))
                assert not torch.equal(before["fc.weight"], after["fc.weight"])
            if name == "resnet_finetune":
                assert model.conv1.weight.grad is not None and model.conv1.weight.grad.abs().sum() > 0
        print("PASS: 4 model outputs, >=3 Conv, frozen backbone/BN, fine-tuning gradients")

        server = app.Studio(("127.0.0.1", 0), root/"capture", root/"runs")
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        base = f"http://127.0.0.1:{server.server_port}"

        def request(path, body=None, extra=None):
            req = Request(base+path, data=body, headers=extra or {})
            with urlopen(req, timeout=10) as response:
                return json.load(response)

        try:
            assert request("/api/status")["complete_sessions"] == 0
            assert request("/api/status")["models"] == []
            result = request("/api/session", b"{}")
            sid = result["session"]["id"]
            expect_error(lambda: request("/api/session", b"{}"), HTTPError)
            buffer = io.BytesIO()
            Image.new("RGB", (224, 224), "gray").save(buffer, format="JPEG")
            unfinished = root/"capture"/"sessions"/sid/"none"/"0001.tmp"
            unfinished.parent.mkdir()
            unfinished.write_bytes(b"interrupted previous save")
            captured = request(f"/api/capture?session={sid}&label=none", buffer.getvalue())
            assert captured["count"] == 1
            assert not unfinished.exists()
            expect_error(lambda: request(f"/api/capture?session={sid}&label=none", b"not jpeg"), HTTPError)
            expect_error(lambda: request("/api/capture?session=../../bad&label=none", buffer.getvalue()), HTTPError)
            expect_error(lambda: request("/api/train", b'{"epochs":10}'), HTTPError)
            expect_error(lambda: request("/api/train", b'{"epochs":1}'), HTTPError)
            expect_error(lambda: request("/api/session", b"{}", {"Origin": "https://untrusted.example"}), HTTPError)
            expect_error(lambda: request("/data/sessions/"), HTTPError)
            assert request("/api/status")["total_counts"]["none"] == 1
        finally:
            server.shutdown(); server.server_close(); thread.join(timeout=5)
        print("PASS: local capture API, count persistence, invalid image/path/origin rejection, no raw data serving")

        if args.smoke_training:
            run = root/"synthetic-smoke-run"
            app.train(argparse.Namespace(data=data, run=run, seed=42, epochs=1, batch_size=4, smoke=True))
            meta = app.read_json(run/"meta.json")
            metrics = app.read_json(run/"metrics.json")
            assert meta["complete"] and meta["smoke"]
            assert len(metrics) == 4
            assert all(np.isfinite(m["test_loss"]) and m["fps"] > 0 for m in metrics)
            assert (run/"report.html").is_file() and (run/"learning_curves.png").is_file()
            assert (run/"sample_predictions.png").is_file() and (run/"augmentation.png").is_file()
            assert "제출 성능 아님" in (run/"RESULTS.md").read_text()
            expect_error(lambda: app.load_checkpoint(run/"mlp.pt", torch.device("cpu")))
            print("PASS: SYNTHETIC ONLY — all 4 one-epoch training/evaluation/checkpoint/report flows; smoke game blocked")
    print("ALL CHECKS PASSED — no real webcam classification accuracy measured")


if __name__ == "__main__":
    main()
