import cv2
import torch
import torch.nn as nn
from torchvision import models, transforms
import torch.nn.functional as F

# 1. 모델 설정 및 가중치 로드
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# 학습 시 사용한 클래스 이름을 순서대로 적어주세요 (알파벳 순서)
# 예: ['can', 'cup', 'pet']
class_names = ['can', 'cup', 'pet'] 
num_classes = len(class_names)

def load_vgg16_model(model_path):
    # 학습 코드와 동일한 구조 생성
    model = models.vgg16()
    num_ftrs = model.classifier[6].in_features
    model.classifier[6] = nn.Linear(num_ftrs, num_classes)
    
    # 저장된 가중치 불러오기
    model.load_state_dict(torch.load(model_path, map_location=device))
    model = model.to(device)
    model.eval() # 추론 모드 고정
    return model

# 2. 이미지 전처리 설정 (학습 시 사용한 ImageNet 정규화 포함)
# VGG16은 224x224 사이즈를 선호합니다.
transform = transforms.Compose([
    transforms.ToPILImage(),
    transforms.Resize((224, 224)),
    transforms.ToTensor(),
    transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
])

# 모델 로드
MODEL_FILE = "vgg16_transfer_model.pth"
model = load_vgg16_model(MODEL_FILE)
print(f"모델 '{MODEL_FILE}' 로드 완료!")

# 3. 웹캠 실행
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret: break

    # 화면 중앙에 400x400 ROI(관심 영역) 설정
    h, w, _ = frame.shape
    roi_size = 400
    x1, y1 = (w - roi_size) // 2, (h - roi_size) // 2
    x2, y2 = x1 + roi_size, y1 + roi_size
    
    roi = frame[y1:y2, x1:x2]
    
    # 추론 준비 (BGR -> RGB 변환 필수)
    img_rgb = cv2.cvtColor(roi, cv2.COLOR_BGR2RGB)
    input_tensor = transform(img_rgb).unsqueeze(0).to(device)

    # 예측
    with torch.no_grad():
        outputs = model(input_tensor)
        #print(outputs)   tensor([[-4.9541, -9.6104, 18.0320]], device='cuda:0')
        probs = F.softmax(outputs, dim=1)
        #print(probs)   tensor([[1.0405e-10, 9.8862e-13, 1.0000e+00]], device='cuda:0')
        conf, pred = torch.max(probs, 1)
        #print(conf, pred)   tensor([1.], device='cuda:0') tensor([2], device='cuda:0')
        label = class_names[pred.item()]
        score = conf.item()
        #print(label, score)  pet 1.0

    # 화면 결과 텍스트 처리
    color = (0, 255, 0) if score > 0.7 else (0, 0, 255)
    result_text = f"{label} ({score*100:.1f}%)"
    
    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 3)
    cv2.putText(frame, result_text, (x1, y1-15), 
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2)

    cv2.imshow("VGG16 Real-time Classification", frame)
    cv2.imshow("What Model Sees (ROI)", roi) # 모델에 들어가는 실제 영역 확인

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()