import cv2
import torch
import torch.nn as nn
from torchvision import transforms
import numpy as np

# 1. 모델 구조 정의 (학습 코드의 MiniVGGNet과 완전히 동일해야 함)
class MiniVGGNet(nn.Module):
    def __init__(self, num_classes=3): # num_classes는 본인의 학습 클래스 개수에 맞게 수정
        super(MiniVGGNet, self).__init__()
        self.block1 = nn.Sequential(
            nn.Conv2d(3, 32, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.BatchNorm2d(32),
            nn.Conv2d(32, 32, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.BatchNorm2d(32),
            nn.MaxPool2d(kernel_size=2, stride=2),
            nn.Dropout(0.3)
        )
        self.block2 = nn.Sequential(
            nn.Conv2d(32, 64, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.BatchNorm2d(64),
            nn.Conv2d(64, 64, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.BatchNorm2d(64),
            nn.MaxPool2d(kernel_size=2, stride=2),
            nn.Dropout(0.4)
        )
        # 64x64 입력 기준, MaxPool 2번 거치면 16x16이 됩니다.
        self.fc_layers = nn.Sequential(
            nn.Flatten(),
            nn.Linear(64 * 16 * 16, 512),
            nn.ReLU(),
            nn.BatchNorm1d(512),
            nn.Dropout(0.5),
            nn.Linear(512, num_classes)
        )

    def forward(self, x):
        x = self.block1(x)
        x = self.block2(x)
        x = self.fc_layers(x)
        return x

# 2. 설정 및 모델 로드
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# 본인의 클래스 이름을 학습 순서(알파벳순)대로 입력하세요.
# 예: ['cat', 'dog', 'rabbit']
class_names = ['CAN', 'CUP', 'PET'] 

model = MiniVGGNet(num_classes=len(class_names)).to(device)

model_path = "miniVGGnet.pth"
try:
    model.load_state_dict(torch.load(model_path, map_location=device))
    print(f"모델 로드 완료: {model_path}")
except FileNotFoundError:
    print(f"에러: {model_path} 파일을 찾을 수 없습니다.")
    exit()

model.eval()

# 3. 전처리 (학습 시 사용한 Normalize와 Resize 유지)
transform = transforms.Compose([
    transforms.ToPILImage(),
    transforms.Resize((64, 64)),
    transforms.ToTensor(),
    transforms.Normalize((0.5, 0.5, 0.5), (0.5, 0.5, 0.5))
])

# 4. 웹캠 루프
cap = cv2.VideoCapture(0)

print("웹캠 추론 시작... 'q'를 누르면 종료")

while True:
    ret, frame = cap.read()
    if not ret: break

    # 화면 중앙 ROI 설정
    h, w, _ = frame.shape
    roi_size = 300
    x1, y1 = (w - roi_size) // 2, (h - roi_size) // 2
    x2, y2 = x1 + roi_size, y1 + roi_size
    
    roi = frame[y1:y2, x1:x2]
    
    # 전처리 및 추론
    img_rgb = cv2.cvtColor(roi, cv2.COLOR_BGR2RGB)
    input_tensor = transform(img_rgb).unsqueeze(0).to(device)
    
    with torch.no_grad():
        outputs = model(input_tensor)
        probs = torch.nn.functional.softmax(outputs, dim=1)
        conf, pred = torch.max(probs, 1)
        
        label = class_names[pred.item()]
        score = conf.item()

    # 결과 화면 출력
    color = (0, 255, 0) if score > 0.7 else (0, 0, 255) # 확신도 70% 미만이면 빨간색
    display_text = f"{label} ({score*100:.1f}%)"
    
    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
    cv2.putText(frame, display_text, (x1, y1-10), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

    cv2.imshow("MiniVGGNet Real-time", frame)
    cv2.imshow("Model Input (64x64)", cv2.resize(roi, (128, 128)))

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()