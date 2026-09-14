
import cv2
import torch
import numpy as np
from lerobot.datasets.lerobot_dataset import LeRobotDataset

def visualize_episode_zero(ds: LeRobotDataset, delay_ms: int = 100):
    print("\n================ VISUALIZING EPISODE 0 ================")

    # episode_index 컬럼에서 0번 에피소드 인덱스 추출
    indices = [i for i, ep_idx in enumerate(ds.hf_dataset["episode_index"]) if ep_idx == 0]

    if not indices:
        print("에피소드 0 데이터를 찾을 수 없습니다.")
        return

    print(f"총 {len(indices)} 프레임을 재생합니다. (딜레이: {delay_ms}ms)")
    print("⚠️  반드시 '이미지 창'을 클릭한 상태에서 'q'를 눌러야 종료됩니다.")

    # 창 이름 설정
    win_name = "LeRobot Visualization"
    
    for i in indices:
        frame_data = ds[i]
        
        # 데이터 출력
        state = frame_data.get("observation.state", torch.tensor([])).tolist()
        action = frame_data.get("action", torch.tensor([])).tolist()
        print(f"▶ 재생 중: Frame {i-indices[0]:04d} / {len(indices)-1}", end="\r") # 한 줄에서 갱신
        print(action, state)
        # 이미지 처리
        img_keys = [k for k in frame_data.keys() if "observation.image" in k]
        if img_keys:
            img_tensor = frame_data[img_keys[0]]
            # (C, H, W) -> (H, W, C) & RGB -> BGR
            img_np = img_tensor.permute(1, 2, 0).numpy()
            img_np = (img_np * 255).astype(np.uint8)
            img_bgr = cv2.cvtColor(img_np, cv2.COLOR_RGB2BGR)

            cv2.imshow('win', img_bgr)
            
            # 여기서 멈춘다면 창을 클릭하고 아무 키나 눌러보세요.
            # waitKey(0)은 키 입력 전까지 무한 대기, 1 이상은 ms만큼 대기입니다.
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("\n사용자에 의해 중단되었습니다.")
                break
        else:
            print(f"\n[Frame {i}] 이미지가 없습니다.")
            

    cv2.destroyAllWindows()
    # 일부 환경에서 창이 즉시 닫히지 않을 때를 대비
    for _ in range(5): cv2.waitKey(1)
    print("\n================ 재생 완료 ================")

    cv2.destroyAllWindows()
    print("=======================================================\n")

def print_dataset_summary(dataset: LeRobotDataset, show_episodes: int = 50):
    meta = dataset.meta
    print("\n================ DATASET SUMMARY ================")
    print(f"repo_id: {getattr(meta, 'repo_id', '(unknown)')}")
    print(f"fps: {getattr(meta, 'fps', '(unknown)')}")
    print(f"total_episodes(meta): {getattr(meta, 'total_episodes', '(unknown)')}")
    print(f"total_frames(meta): {getattr(meta, 'total_frames', '(unknown)')}")
    print("-------------------------------------------------")
    print("features keys:")
    feats = getattr(meta, "features", {})
    for k, v in feats.items():
        dtype = v.get("dtype", "?") if isinstance(v, dict) else "?"
        shape = v.get("shape", "?") if isinstance(v, dict) else "?"
        print(f" - {k}: dtype={dtype}, shape={shape}")
    print("-------------------------------------------------")
    episodes = getattr(meta, "episodes", None)
    
    if episodes is None:
        print("No episode metadata found.")
    else:
        # episodes가 Hugging Face Dataset 객체인 경우 길이를 바로 구함
        num_episodes = len(episodes)
        print(f"episodes found in meta: {num_episodes}")
        print(f"showing first {min(show_episodes, num_episodes)} episodes:")
        
        for i in range(min(show_episodes, num_episodes)):
            ep_data = episodes[i]  # 인덱스로 접근
            length = ep_data.get("length", "?")
            # tasks가 리스트 형태일 수 있으므로 처리
            tasks = ep_data.get("tasks", "N/A")
            if isinstance(tasks, list) and len(tasks) > 0:
                task_str = tasks[0]
            else:
                task_str = str(tasks)
            print(f" - ep {i:04d}: length={length}, task={task_str}")
   
    print("=================================================\n")


repo_id = "my_robot_task"
dataset = LeRobotDataset(repo_id=repo_id, video_backend="pyav")

# 1. 요약 정보 출력 (이전 질문의 함수)
# print_dataset_summary(dataset)

# 2. 에피소드 0 시각화 및 데이터 출력
visualize_episode_zero(dataset, delay_ms=100)