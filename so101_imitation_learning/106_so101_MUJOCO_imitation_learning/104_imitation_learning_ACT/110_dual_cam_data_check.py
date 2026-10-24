import cv2
import torch
import numpy as np
from lerobot.datasets.lerobot_dataset import LeRobotDataset

def to_bgr(img_tensor):
    """(C, H, W) 텐서를 (H, W, 3) BGR 배열로 변환"""
    img_np = img_tensor.permute(1, 2, 0).numpy()
    img_np = (img_np * 255).astype(np.uint8)
    return cv2.cvtColor(img_np, cv2.COLOR_RGB2BGR)

def visualize_episode_zero(ds: LeRobotDataset, delay_ms: int = 33):
    print("\n================ VISUALIZING EPISODE 0 (Dual Cam) ================")

    indices = [i for i, ep_idx in enumerate(ds.hf_dataset["episode_index"]) if ep_idx == 0]

    if not indices:
        print("에피소드 0 데이터를 찾을 수 없습니다.")
        return

    print(f"총 {len(indices)} 프레임 재생 중...")

    for i in indices:
        frame_data = ds[i]
        
        # 1. 상태 및 행동 출력
        state = frame_data.get("observation.state", torch.tensor([])).tolist()
        action = frame_data.get("action", torch.tensor([])).tolist()
        print(f"▶ Frame {i-indices[0]:03d} | State: {np.round(state, 2)}", end="\r")

        # 2. 메인/손목 이미지 처리
        main_img = frame_data.get("observation.images.main")
        wrist_img = frame_data.get("observation.images.wrist")

        if main_img is not None and wrist_img is not None:
            # 두 이미지 변환
            main_bgr = to_bgr(main_img)
            wrist_bgr = to_bgr(wrist_img)

            # 3. 옆으로 나란히 붙이기 (hconcat)
            combined_img = cv2.hconcat([main_bgr, wrist_bgr])
            
            # 텍스트 추가 (어느 쪽이 main인지 wrist인지 표시)
            cv2.putText(combined_img, "MAIN", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(combined_img, "WRIST", (main_bgr.shape[1] + 20, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

            cv2.imshow("LeRobot Dual-Cam Visualization", combined_img)
            
            if cv2.waitKey(delay_ms) & 0xFF == ord('q'):
                break
        else:
            print(f"\n[Frame {i}] 이미지 데이터를 찾을 수 없습니다.")

    cv2.destroyAllWindows()
    print("\n================ 재생 완료 ================")

def print_dataset_summary(dataset: LeRobotDataset):
    meta = dataset.meta
    print("\n================ DATASET SUMMARY ================")
    print(f"Features found:")
    feats = getattr(meta, "features", {})
    for k, v in feats.items():
        shape = v.get("shape", "?")
        print(f" - {k}: shape={shape}")
    
    episodes = getattr(meta, "episodes", None)
    if episodes is not None:
        print(f"Total Episodes: {len(episodes)}")
        for i in range(min(5, len(episodes))):
            print(f" - Ep {i}: length={episodes[i].get('length')}, task={episodes[i].get('tasks', ['N/A'])[0]}")

# 사용 예시
repo_id = "mujoco_pick_place_dualcam" # 본인의 repo_id 확인
dataset = LeRobotDataset(repo_id=repo_id, video_backend="pyav")

print_dataset_summary(dataset)
visualize_episode_zero(dataset, delay_ms=33) # 30fps 기준