import cv2
import torch
import torch.nn as nn
import torch.nn.functional as F
from torchvision import transforms

# 1. 모델 구조 정의 (학습 코드의 ImprovedCNN과 반드시 동일해야 함)
class ImprovedCNN(nn.Module):
    def __init__(self):
        super(ImprovedCNN, self).__init__()
        self.conv1 = nn.Conv2d(3, 32, 3, padding=1)
        self.bn1 = nn.BatchNorm2d(32)
        self.conv2 = nn.Conv2d(32, 32, 3, padding=1)
        self.bn2 = nn.BatchNorm2d(32)
        
        self.conv3 = nn.Conv2d(32, 64, 3, padding=1)
        self.bn3 = nn.BatchNorm2d(64)
        self.conv4 = nn.Conv2d(64, 64, 3, padding=1)
        self.bn4 = nn.BatchNorm2d(64)
        
        self.pool = nn.MaxPool2d(2, 2)
        self.dropout_conv = nn.Dropout(0.3)
        self.dropout_fc = nn.Dropout(0.5)
        
        self.fc1 = nn.Linear(64 * 8 * 8, 512)
        self.bn5 = nn.BatchNorm1d(512)
        self.fc2 = nn.Linear(512, 10)

    def forward(self, x):
        x = F.relu(self.bn1(self.conv1(x)))
        x = F.relu(self.bn2(self.conv2(x)))
        x = self.pool(x)
        x = self.dropout_conv(x)
        
        x = F.relu(self.bn3(self.conv3(x)))
        x = F.relu(self.bn4(self.conv4(x)))
        x = self.pool(x)
        x = self.dropout_conv(x)
        
        x = x.view(-1, 64 * 8 * 8)
        x = F.relu(self.bn5(self.fc1(x)))
        x = self.dropout_fc(x)
        x = self.fc2(x)
        return x

# 2. CIFAR-10 클래스 이름 (결과 출력용)
classes = ('plane', 'car', 'bird', 'cat', 'deer', 'dog', 'frog', 'horse', 'ship', 'truck')

# 3. 모델 로드 및 설정
device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
model = ImprovedCNN().to(device)

# 저장된 가중치 불러오기
model_path = "cifar10_cnn.pth"
try:
    model.load_state_dict(torch.load(model_path, map_location=device))
    print(f"모델 로드 성공: {model_path}")
except FileNotFoundError:
    print(f"에러: '{model_path}' 파일이 없습니다. 학습을 먼저 완료해주세요.")
    exit()

model.eval() # 추론 모드 (Dropout, BatchNorm 비활성화)

# 4. 이미지 전처리 (학습할 때 사용한 transform과 일치시켜야 함)
# 주의: OpenCV는 BGR 형식이므로 ToPILImage() 사용 시 RGB로 변환됨
transform = transforms.Compose([
    transforms.ToPILImage(),
    transforms.Resize((32, 32)),
    transforms.ToTensor(),
    transforms.Normalize((0.5, 0.5, 0.5), (0.5, 0.5, 0.5))
])

# 5. 실시간 웹캠 추론
cap = cv2.VideoCapture(0)

print("웹캠 추론을 시작합니다. 'q'를 누르면 종료합니다.")

while True:
    ret, frame = cap.read()
    if not ret: break

    # 화면 중앙에 정사각형 ROI 설정 (32x32로 리사이즈될 영역)
    h, w, _ = frame.shape
    size = 300
    x1, y1 = (w - size) // 2, (h - size) // 2
    x2, y2 = x1 + size, y1 + size
    
    roi = frame[y1:y2, x1:x2]
    # OpenCV(BGR)를 RGB로 변환하여 전처리
    rgb_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2RGB)
    input_tensor = transform(rgb_roi).unsqueeze(0).to(device)

    # 추론
    with torch.no_grad():
        outputs = model(input_tensor)
        probabilities = F.softmax(outputs, dim=1)
        conf, predicted = torch.max(probabilities, 1)
        
        class_idx = predicted.item()
        confidence = conf.item()

    # 화면 표시
    label = f"{classes[class_idx]} ({confidence*100:.1f}%)"
    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
    cv2.putText(frame, label, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    cv2.imshow("CIFAR-10 Live Inference", frame)
    cv2.imshow("Input ROI", cv2.resize(roi, (200, 200))) # 모델이 보는 영역 확대 확인용

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()