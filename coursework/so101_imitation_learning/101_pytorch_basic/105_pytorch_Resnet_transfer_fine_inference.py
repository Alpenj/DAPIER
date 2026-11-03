import cv2
import torch
import torch.nn as nn
from torchvision import models, transforms
import torch.nn.functional as F

# 1. 모델 및 장치 설정
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# 학습 시 폴더 순서에 맞춰 클래스 이름을 적어주세요.
# 예: ['can', 'cup', 'pet']
class_names = ['can', 'cup', 'pet'] 
num_classes = len(class_names)

def load_resnet18_model(model_path):
    # 학습 코드와 동일한 구조의 ResNet18 생성
    model = models.resnet18()
    num_ftrs = model.fc.in_features
    model.fc = nn.Linear(num_ftrs, num_classes) # 학습 시 수정했던 부분과 동일하게 설정
    
    # 저장된 가중치(.pth) 로드
    try:
        model.load_state_dict(torch.load(model_path, map_location=device))
        print(f"성공: '{model_path}' 가중치를 로드했습니다.")
    except FileNotFoundError:
        print(f"에러: '{model_path}' 파일을 찾을 수 없습니다.")
        exit()
        
    model = model.to(device)
    model.eval() # 추론 모드 (Dropout, BatchNorm 고정)
    return model

# 2. 전처리 (ResNet/ImageNet 표준 규격)
transform = transforms.Compose([
    transforms.ToPILImage(),
    transforms.Resize((224, 224)),
    transforms.ToTensor(),
    transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
])

# 모델 로드
MODEL_FILE = "resnet18_final.pth"
model = load_resnet18_model(MODEL_FILE)

# 3. 웹캠 루프 시작
cap = cv2.VideoCapture(0)

print("웹캠 추론을 시작합니다. 'q'를 누르면 종료합니다.")

while True:
    ret, frame = cap.read()
    if not ret: break

    # 화면 중앙에 ROI(관심 영역) 설정 - 400x400 크기
    h, w, _ = frame.shape
    size = 400
    x1, y1 = (w - size) // 2, (h - size) // 2
    x2, y2 = x1 + size, y1 + size
    
    roi = frame[y1:y2, x1:x2]
    
    # 이미지 전처리 (OpenCV BGR -> RGB 변환 필수)
    img_rgb = cv2.cvtColor(roi, cv2.COLOR_BGR2RGB)
    input_tensor = transform(img_rgb).unsqueeze(0).to(device)

    # 모델 추론
    with torch.no_grad():
        outputs = model(input_tensor)
        probs = F.softmax(outputs, dim=1)
        conf, pred = torch.max(probs, 1)
        
        label = class_names[pred.item()]
        score = conf.item()

    # 결과 화면 시각화
    # 확신도(score)가 80% 이상일 때만 초록색, 아니면 빨간색 표시
    color = (0, 255, 0) if score > 0.8 else (0, 0, 255)
    display_text = f"{label} ({score*100:.1f}%)"
    
    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 3)
    cv2.putText(frame, display_text, (x1, y1-15), 
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2)

    cv2.imshow("ResNet18 Live Inference", frame)
    
    # 'q' 키를 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()