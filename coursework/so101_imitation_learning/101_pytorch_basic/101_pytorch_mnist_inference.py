import cv2
import torch
import numpy as np
from torchvision import transforms
import torch.nn as nn

class MultiLayerPerceptron(nn.Module):
    def __init__(self):
        super(MultiLayerPerceptron, self).__init__()
        self.flatten = nn.Flatten()
        self.linear_relu_stack = nn.Sequential(
            nn.Linear(28*28, 512),
            nn.ReLU(),
            nn.Linear(512, 256),
            nn.ReLU(),
            nn.Linear(256, 10)
        )

    def forward(self, x):
        x = self.flatten(x)
        logits = self.linear_relu_stack(x)
        return logits

# 1. 모델 로드 (위에서 정의한 클래스가 있어야 함)
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
model = MultiLayerPerceptron().to(device)
model.load_state_dict(torch.load("mnist_mlp.pth", map_location=device))
model.eval()

# 2. MNIST 전처리 함수 (학습 때와 동일하게)
transform = transforms.Compose([
    transforms.ToPILImage(),
    transforms.Resize((28, 28)),
    transforms.Grayscale(),
    transforms.ToTensor(),
    transforms.Normalize((0.1307,), (0.3081,))
])

# 3. 웹캠 시작
cap = cv2.VideoCapture(0)

print("웹캠을 시작합니다. 'q'를 누르면 종료합니다.")

while True:
    ret, frame = cap.read()
    if not ret: break

    # 화면 중앙에 관심 영역(ROI) 설정 (숫자를 쓸 사각형 영역)
    height, width, _ = frame.shape
    x1, y1, x2, y2 = width//2-100, height//2-100, width//2+100, height//2+100
    roi = frame[y1:y2, x1:x2]

    # 이미지 전처리 (흑백 전환 및 반전)
    # MNIST는 배경이 검은색, 글씨가 흰색이므로 필요에 따라 반전시켜야 함
    gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    _, thresh_roi = cv2.threshold(gray_roi, 128, 255, cv2.THRESH_BINARY_INV) # 배경 검게, 글씨 희게

    # 모델 입력을 위한 텐서 변환
    input_tensor = transform(thresh_roi).unsqueeze(0).to(device)

    # 추론
    with torch.no_grad():
        output = model(input_tensor)
        prediction = output.argmax(dim=1).item()
        confidence = torch.nn.functional.softmax(output, dim=1).max().item()

    # 결과 화면 출력
    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
    cv2.putText(frame, f"Predict: {prediction} ({confidence*100:.1f}%)", 
                (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
    
    cv2.imshow("MNIST Real-time", frame)
    cv2.imshow("Input to Model", thresh_roi) # 모델이 실제로 보는 이미지

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()