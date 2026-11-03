import cv2
import torch
import timm
from torchvision import transforms
import torch.nn.functional as F

# 1. 환경 설정 및 모델 로드
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# CIFAR-10 클래스 정의
classes = ('plane', 'car', 'bird', 'cat', 'deer', 'dog', 'frog', 'horse', 'ship', 'truck')

def load_vit_model(model_path):
    # 학습 시 사용한 모델과 동일한 구조 생성
    model = timm.create_model('vit_tiny_patch16_224', pretrained=False, num_classes=10)
    
    # 저장된 가중치 로드
    try:
        model.load_state_dict(torch.load(model_path, map_location=device))
        print(f"모델 로드 완료: {model_path}")
    except FileNotFoundError:
        print(f"에러: {model_path} 파일을 찾을 수 없습니다.")
        exit()
        
    model = model.to(device)
    model.eval() # 추론 모드 설정
    return model

# 2. ViT 전용 전처리 (224x224 리사이즈 필수)
transform = transforms.Compose([
    transforms.ToPILImage(),
    transforms.Resize((224, 224)),
    transforms.ToTensor(),
    transforms.Normalize((0.5, 0.5, 0.5), (0.5, 0.5, 0.5))
])

# 모델 로드
MODEL_FILE = "vit_cifar10.pth"
model = load_vit_model(MODEL_FILE)

# 3. 실시간 웹캠 추론 시작
cap = cv2.VideoCapture(0)

print("ViT 웹캠 추론을 시작합니다. 'q'를 누르면 종료합니다.")

while True:
    ret, frame = cap.read()
    if not ret: break

    # 화면 중앙에 정중앙 ROI 설정 (정사각형 영역 추출)
    h, w, _ = frame.shape
    roi_size = 400
    x1, y1 = (w - roi_size) // 2, (h - roi_size) // 2
    x2, y2 = x1 + roi_size, y1 + roi_size
    roi = frame[y1:y2, x1:x2]

    # 추론 준비 (BGR -> RGB 변환 후 텐서화)
    img_rgb = cv2.cvtColor(roi, cv2.COLOR_BGR2RGB)
    input_tensor = transform(img_rgb).unsqueeze(0).to(device)

    # 예측
    with torch.no_grad():
        outputs = model(input_tensor)
        probs = F.softmax(outputs, dim=1)
        conf, pred = torch.max(probs, 1)
        
        label = classes[pred.item()]
        score = conf.item()

    # 결과 표시
    color = (255, 0, 255) # 보라색 (ViT의 느낌을 살려봤습니다)
    display_text = f"ViT Pred: {label} ({score*100:.1f}%)"
    
    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 3)
    cv2.putText(frame, display_text, (x1, y1-15), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)

    cv2.imshow("Vision Transformer Real-time", frame)
    
    # 'q' 키를 누르면 루프 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()