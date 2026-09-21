"""완성된 바위·보·가위 자세에서 새 상대 패로 전환하는 가상 폐루프 진단.

python evaluate_transitions.py --run runs/<ID>
고정 테스트 사진 120장 × 출발 자세 3개. 원시 사진은 출력하지 않는다.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import io
from pathlib import Path

import numpy as np
from PIL import Image
import torch

import evaluate_rollout as base

app = base.app
POSES = {"rock": app.counter_pose("scissors"), "paper": app.counter_pose("rock"), "scissors": app.counter_pose("paper")}


def mean(rows, key):
    return float(np.mean([r[key] for r in rows])) if rows else None


def summarize(rows):
    correct = [r for r in rows if r["correct"]]
    responding = [r for r in rows if r["response_fraction"] > 0]
    eligible = [r for r in rows if r["initial_outside_criterion"]]
    return {"transitions": len(rows), "unique_images": len({r["path"] for r in rows}),
            "raw_classification_fraction": mean(rows, "correct"), "response_fraction": mean(rows, "response_fraction"),
            "mean_initial_joint_rmse_rad": mean(rows, "initial_rmse_rad"), "mean_final_joint_rmse_rad": mean(rows, "final_rmse_rad"),
            "correct_classified_count": len(correct), "correct_classified_mean_final_rmse_rad": mean(correct, "final_rmse_rad"),
            "correct_and_responding_mean_final_rmse_rad": mean([r for r in correct if r["response_fraction"] > 0], "final_rmse_rad"),
            "predicted_goal_mean_final_rmse_rad": mean(rows, "predicted_goal_rmse_rad"),
            "responding_predicted_goal_mean_final_rmse_rad": mean(responding, "predicted_goal_rmse_rad"),
            "within_final_criterion_fraction": mean(rows, "within_final_criterion"),
            "eligible_2s_count": len(eligible), "reached_within_2s_fraction_excluding_initial": mean(eligible, "reached_within_2s")}


def transition_record(row, initial_name, predicted, confidence, response, trajectory):
    label = app.CLASSES[row["label"]]
    initial = POSES[initial_name]
    goal, predicted_goal = app.counter_pose(label), app.counter_pose(predicted)
    goal = initial if goal is None else goal
    predicted_goal = initial if predicted_goal is None else predicted_goal
    errors = np.sqrt(np.mean((trajectory-goal)**2, axis=1))
    hits = np.flatnonzero(errors <= base.CRITERION)
    first_tick = int(hits[0]) if len(hits) else None
    return {"path": row["path"], "label": label, "initial_pose": initial_name, "predicted_label": predicted,
            "confidence": confidence, "correct": label == predicted, "response_fraction": response,
            "initial_rmse_rad": float(errors[0]), "final_rmse_rad": float(errors[-1]),
            "predicted_goal_rmse_rad": float(np.sqrt(np.mean((trajectory[-1]-predicted_goal)**2))),
            "initial_outside_criterion": bool(errors[0] > base.CRITERION), "within_final_criterion": bool(errors[-1] <= base.CRITERION),
            "first_reach_seconds": first_tick/10 if first_tick is not None else None,
            "reached_within_2s": first_tick is not None and first_tick <= 20,
            "final_joints_rad": trajectory[-1].tolist()}


def plot(path, groups):
    fig, axes = app.plt.subplots(1, 2, figsize=(12, 5))
    for ax, key, title, limit in zip(axes,
            ("mean_final_joint_rmse_rad", "reached_within_2s_fraction_excluding_initial"),
            ("Final RMSE to true goal (rad)", "Reached by 2 s (initially close excluded)"), (app.MAX_RAD, 1)):
        values = np.array([[groups[label][pose][key] if groups[label][pose][key] is not None else np.nan for pose in POSES] for label in app.CLASSES])
        im = ax.imshow(values, vmin=0, vmax=limit, cmap="viridis")
        ax.set(xticks=range(3), xticklabels=list(POSES), yticks=range(4), yticklabels=app.CLASSES,
               xlabel="Initial virtual-hand pose", ylabel="True opponent image", title=title)
        for y, x in np.ndindex(values.shape):
            value = values[y, x]
            ax.text(x, y, "N/A" if np.isnan(value) else f"{value:.3f}", ha="center", va="center", color="black" if np.isnan(value) or value > limit*.55 else "white")
        fig.colorbar(im, ax=ax, shrink=.7)
    fig.suptitle("Fixed held-out images × 3 canonical starts | virtual kinematics only")
    fig.tight_layout(); fig.savefig(path, dpi=150); app.plt.close(fig)


def evaluate(args):
    if not 1 <= args.batch_size <= 32:
        raise ValueError("batch는 1~32여야 합니다.")
    run, data = args.run.resolve(), args.data.resolve()
    meta = app.vision.read_json(run/"meta.json")
    manifest = app.vision.read_json(run/"source_manifest.json")
    checkpoint = torch.load(run/"act.pt", map_location="cpu", weights_only=True)
    if not meta.get("complete") or meta.get("smoke") or checkpoint.get("smoke"):
        raise ValueError("실제 촬영 데이터로 완료한 실행만 평가합니다.")
    if (checkpoint.get("classes") != list(app.CLASSES) or checkpoint.get("chunk") != app.CHUNK
            or checkpoint.get("joints") != app.JOINTS or checkpoint.get("max_rad") != app.MAX_RAD):
        raise ValueError("가상 손과 호환되지 않는 체크포인트입니다.")
    if meta.get("source_fingerprint") != manifest.get("fingerprint"):
        raise ValueError("source manifest가 학습 기록과 다릅니다.")
    groups = {key: set(value) for key, value in manifest["session_groups"].items()}
    if groups["train"] & groups["test"] or groups["val"] & groups["test"] or groups["train"] & groups["val"]:
        raise ValueError("세션 분할이 겹칩니다.")
    rows = manifest["splits"]["test"]
    if {r["label"] for r in rows} != set(range(4)) or any(r["session"] not in groups["test"] for r in rows):
        raise ValueError("네 클래스가 포함된 held-out 회차가 필요합니다.")
    torch.set_num_threads(2)
    device = torch.device("cuda" if args.device == "auto" and torch.cuda.is_available() else "cpu" if args.device == "auto" else args.device)
    model = app.ActionCVAE().to(device).eval(); model.load_state_dict(checkpoint["state_dict"])
    transform = app.vision.image_transform(False, 64)
    inputs = []
    for row in rows:
        path = (data/row["path"]).resolve()
        if not path.is_relative_to(data):
            raise ValueError("데이터 폴더 밖의 이미지입니다.")
        raw = path.read_bytes()
        if hashlib.sha256(raw).hexdigest() != row["sha256"]:
            raise ValueError("학습 당시와 다른 테스트 이미지입니다.")
        with Image.open(io.BytesIO(raw)) as image:
            tensor = transform(image.convert("RGB"))
        inputs.extend((row, pose, tensor) for pose in POSES)
    records = []
    for offset in range(0, len(inputs), args.batch_size):
        batch = inputs[offset:offset+args.batch_size]
        images = torch.stack([x[2] for x in batch]).to(device)
        initial = torch.from_numpy(np.stack([POSES[x[1]] for x in batch])).to(device)
        result = base.rollout(model, images, initial)
        for i, (row, pose, _) in enumerate(batch):
            records.append(transition_record(row, pose, app.CLASSES[result["predicted"][i].item()],
                result["confidence"][i].item(), result["response_fraction"][i].item(), result["trajectory"][i].numpy()))
        print(f"Transitions {min(offset+len(batch), len(inputs))}/{len(inputs)}", flush=True)
    grouped = {label: {pose: summarize([r for r in records if r["label"] == label and r["initial_pose"] == pose]) for pose in POSES} for label in app.CLASSES}
    report = {"created": datetime.now(timezone.utc).isoformat(), "scope": "고정 테스트 사진에서 세 완성 자세 간 전환을 반복 추론한 가상 운동학 진단. 실물·실시간 웹캠 성공률이 아니다.",
              "source_fingerprint": manifest["fingerprint"], "checkpoint_sha256": hashlib.sha256((run/"act.pt").read_bytes()).hexdigest(),
              "teacher_version": meta.get("teacher_version", "legacy"), "device": str(device),
              "settings": {"chunks": base.CHUNKS, "executed_steps": base.EXECUTED, "ticks": 64, "kinematic_seconds": 6.4,
                           "lag": app.LAG, "confidence_threshold": base.CONFIDENCE, "z": "zero", "criterion_rmse_rad": base.CRITERION,
                           "initial_poses_rad": {key: value.tolist() for key, value in POSES.items()},
                           "reach_note": "20틱 이내 처음 RMSE≤0.2에 도달한 비율. 처음부터 기준 이내인 전환은 분모에서 제외. 임의 진단 기준.",
                           "predicted_goal_note": "모델이 분류한 패의 반대 자세와 실제 출력 자세의 자기일관성. 정답 정확도와 별개; 예측 none이면 시작 자세 유지가 기준."},
              "overall": summarize(records), "per_class": {label: summarize([r for r in records if r["label"] == label]) for label in app.CLASSES},
              "per_class_initial_pose": grouped, "per_transition": records}
    plot(run/"transition_summary.png", grouped)
    app.vision.write_json(run/"transition_metrics.json", report)
    for label, metrics in report["per_class"].items():
        print(f"{label}: final RMSE={metrics['mean_final_joint_rmse_rad']:.3f}, correct-subset RMSE={metrics['correct_classified_mean_final_rmse_rad']}, "
              f"predicted-goal RMSE={metrics['predicted_goal_mean_final_rmse_rad']:.3f}, reach2s={metrics['reached_within_2s_fraction_excluding_initial']}")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--data", type=Path, default=app.vision.ROOT/"data")
    parser.add_argument("--batch-size", type=int, default=24)
    parser.add_argument("--device", choices=("auto", "cpu", "cuda"), default="auto")
    evaluate(parser.parse_args())


if __name__ == "__main__":
    main()
