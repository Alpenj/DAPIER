"""고정 테스트 이미지로 ACT 폐루프를 64시점 평가한다. 실물 로봇 평가가 아니다.

python evaluate_rollout.py --run runs/<완료한 실행 ID>
출력: rollout_metrics.json, rollout_trajectories.png (원본 이미지 제외).
"""
import argparse
from datetime import datetime, timezone
import hashlib
import io
from pathlib import Path

import numpy as np
from PIL import Image
import torch

import hand_exam as app

CHUNKS, EXECUTED, CONFIDENCE, CRITERION = 16, 4, .7, .2


@torch.inference_mode()
def rollout(model, images):
    """정답 라벨·목표 자세를 받지 않는다. UI와 같은 prior z=0 관찰/명령 경로다."""
    model.eval()
    q = images.new_zeros((len(images), app.JOINTS))
    trajectory, responses = [q.clone()], images.new_zeros(len(images))
    first = None
    for _ in range(CHUNKS):
        actions, _, _, logits = model(images, q, sample=False)
        if actions.shape != (len(images), app.CHUNK, app.JOINTS) or not torch.isfinite(actions).all():
            raise ValueError("예측은 유한한 [B,8,10] 명령이어야 합니다.")
        if (actions < 0).any() or (actions > app.MAX_RAD).any() or not torch.isfinite(logits).all():
            raise ValueError("예측 관절 범위 또는 분류 출력이 잘못되었습니다.")
        confidence, label = logits.softmax(1).max(1)
        if first is None:
            first = {"predicted": label.cpu(), "confidence": confidence.cpu()}
        move = (confidence >= CONFIDENCE) & (label != app.CLASSES.index("none"))
        responses += move
        for t in range(EXECUTED):
            q = torch.where(move[:, None], q+app.LAG*(actions[:, t]-q), q)
            trajectory.append(q.clone())
    return {**first, "trajectory": torch.stack(trajectory, 1).cpu(), "response_fraction": (responses/CHUNKS).cpu()}


def summarize(rows):
    if not rows:
        return {"images": 0}
    return {"images": len(rows), "raw_vit_accuracy": float(np.mean([r["correct"] for r in rows])),
            "response_fraction": float(np.mean([r["response_fraction"] for r in rows])),
            "mean_initial_joint_rmse_rad": float(np.mean([r["initial_joint_rmse_rad"] for r in rows])),
            "mean_final_joint_rmse_rad": float(np.mean([r["final_joint_rmse_rad"] for r in rows])),
            "initial_within_criterion_fraction": float(np.mean([r["initial_joint_rmse_rad"] <= CRITERION for r in rows])),
            "within_criterion_fraction": float(np.mean([r["final_joint_rmse_rad"] <= CRITERION for r in rows]))}


def plot_trajectories(path, examples):
    fig, axes = app.plt.subplots(2, 2, figsize=(13, 9))
    seconds = np.arange(CHUNKS*EXECUTED+1)/10
    for label, ax in zip(app.CLASSES, axes.flat):
        trajectory, goal = examples[label]
        for joint in range(app.JOINTS):
            line, = ax.plot(seconds, trajectory[:, joint], label=f"q{joint}", linewidth=1.2)
            ax.axhline(goal[joint], color=line.get_color(), linestyle="--", alpha=.5, linewidth=.9)
        error = np.sqrt(np.mean((trajectory[-1]-goal)**2))
        ax.set(title=f"True opponent: {label} | final RMSE {error:.3f} rad", xlabel="Kinematic time (s)",
               ylabel="Joint position (rad)", ylim=(-.04, app.MAX_RAD+.04))
        ax.grid(alpha=.2)
    axes[0, 0].legend(fontsize=7, ncol=5)
    fig.suptitle("First held-out image per class | solid: model state; dashed: evaluation goal\n64 ticks, 16 observations; fixed image; virtual kinematics only")
    fig.tight_layout(); fig.savefig(path, dpi=150); app.plt.close(fig)


def evaluate(args):
    if not 1 <= args.batch_size <= 32:
        raise ValueError("평가 batch는 1~32로 지정하세요.")
    run, data = args.run.resolve(), args.data.resolve()
    meta = app.vision.read_json(run/"meta.json")
    manifest = app.vision.read_json(run/"source_manifest.json")
    checkpoint = torch.load(run/"act.pt", map_location="cpu", weights_only=True)
    if not meta.get("complete"):
        raise ValueError("학습을 완료한 실행만 평가할 수 있습니다.")
    smoke = bool(meta.get("smoke") or checkpoint.get("smoke"))
    if smoke and not args.allow_smoke:
        raise ValueError("합성 smoke 모델입니다. 실제 성능 평가에 사용할 수 없습니다.")
    if (checkpoint.get("classes") != list(app.CLASSES) or checkpoint.get("joints") != app.JOINTS
            or checkpoint.get("chunk") != app.CHUNK or checkpoint.get("max_rad") != app.MAX_RAD):
        raise ValueError("가상 손과 호환되지 않는 체크포인트입니다.")
    if meta.get("source_fingerprint") != manifest.get("fingerprint"):
        raise ValueError("학습 기록과 source manifest가 다릅니다.")
    groups = {key: set(value) for key, value in manifest["session_groups"].items()}
    if groups["train"] & groups["test"] or groups["val"] & groups["test"] or groups["train"] & groups["val"]:
        raise ValueError("촬영 세션 분할이 겹칩니다.")
    rows = manifest["splits"]["test"]
    if {r["label"] for r in rows} != set(range(4)) or any(r["session"] not in groups["test"] for r in rows):
        raise ValueError("네 클래스가 있는 held-out 테스트 세션이 필요합니다.")
    device = torch.device("cuda" if args.device == "auto" and torch.cuda.is_available() else "cpu" if args.device == "auto" else args.device)
    torch.set_num_threads(2)
    model = app.ActionCVAE().to(device).eval()
    model.load_state_dict(checkpoint["state_dict"])
    transform = app.vision.image_transform(False, 64)
    records, examples = [], {}
    for offset in range(0, len(rows), args.batch_size):
        batch = rows[offset:offset+args.batch_size]
        images = []
        for row in batch:
            path = (data/row["path"]).resolve()
            if not path.is_relative_to(data):
                raise ValueError("데이터 폴더 밖의 이미지입니다.")
            raw = path.read_bytes()
            if hashlib.sha256(raw).hexdigest() != row["sha256"]:
                raise ValueError("학습 당시와 다른 테스트 이미지입니다: "+row["path"])
            with Image.open(io.BytesIO(raw)) as image:
                images.append(transform(image.convert("RGB")))
        result = rollout(model, torch.stack(images).to(device))
        for i, row in enumerate(batch):
            label = app.CLASSES[row["label"]]
            trajectory = result["trajectory"][i].numpy()
            goal = app.counter_pose(label)
            if goal is None:
                goal = np.zeros(app.JOINTS, dtype=np.float32)
            predicted = app.CLASSES[result["predicted"][i].item()]
            final_rmse = float(np.sqrt(np.mean((trajectory[-1]-goal)**2)))
            records.append({"path": row["path"], "session": row["session"], "label": label, "predicted_label": predicted,
                            "confidence": result["confidence"][i].item(), "correct": predicted == label,
                            "response_fraction": result["response_fraction"][i].item(),
                            "initial_joint_rmse_rad": float(np.sqrt(np.mean(goal**2))), "final_joint_rmse_rad": final_rmse,
                            "within_criterion": final_rmse <= CRITERION, "final_joints_rad": trajectory[-1].tolist()})
            examples.setdefault(label, (trajectory, goal))
        print(f"Rollout {min(offset+args.batch_size, len(rows))}/{len(rows)} test images", flush=True)
    none = [r for r in records if r["label"] == "none"]
    report = {"created": datetime.now(timezone.utc).isoformat(), "smoke": smoke,
              "scope": "고정 held-out 웹캠 이미지에 대한 가상 운동학 폐루프 평가. 실물·새 웹캠 장면·게임 승률 평가 아님.",
              "device": str(device), "source_fingerprint": manifest["fingerprint"],
              "checkpoint_sha256": hashlib.sha256((run/"act.pt").read_bytes()).hexdigest(),
              "settings": {"chunks": CHUNKS, "predicted_steps_per_chunk": app.CHUNK, "executed_steps_per_chunk": EXECUTED,
                           "ticks": CHUNKS*EXECUTED, "kinematic_seconds": CHUNKS*EXECUTED/10, "hz": 10, "lag": app.LAG,
                           "confidence_threshold": CONFIDENCE, "z": "zero", "initial_joints_rad": [0.]*app.JOINTS,
                           "criterion_rmse_rad": CRITERION, "criterion_note": "0.2 rad RMSE는 이 평가에서 정한 기준이며 시험·실물 성공 기준이 아니다.",
                           "response_fraction_note": "신뢰도 게이트를 통과해 명령을 실행한 관찰 청크 비율. 정답 여부와 별개.",
                           "representative_selection": "source manifest 순서에서 클래스별 첫 번째 테스트 이미지"},
              "overall": summarize(records), "per_class": {label: summarize([r for r in records if r["label"] == label]) for label in app.CLASSES},
              "none_hold": {"images": len(none), "no_response_fraction": float(np.mean([r["response_fraction"] == 0 for r in none])),
                            "pose_hold_fraction": float(np.mean([r["final_joint_rmse_rad"] <= 1e-6 for r in none]))},
              "per_image": records}
    plot_trajectories(run/"rollout_trajectories.png", examples)
    app.vision.write_json(run/"rollout_metrics.json", report)
    print("SYNTHETIC ONLY" if smoke else "VIRTUAL KINEMATIC EVALUATION — no physical robot")
    for label, metrics in report["per_class"].items():
        print(f"{label}: raw ViT={metrics['raw_vit_accuracy']:.1%}, response={metrics['response_fraction']:.1%}, "
              f"RMSE={metrics['mean_initial_joint_rmse_rad']:.3f}→{metrics['mean_final_joint_rmse_rad']:.3f} rad, "
              f"within .2={metrics['within_criterion_fraction']:.1%}")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--data", type=Path, default=app.vision.ROOT/"data")
    parser.add_argument("--batch-size", type=int, default=24)
    parser.add_argument("--device", choices=("auto", "cpu", "cuda"), default="auto")
    parser.add_argument("--allow-smoke", action="store_true", help="합성 모델의 흐름 검사에만 사용")
    evaluate(parser.parse_args())


if __name__ == "__main__":
    main()
