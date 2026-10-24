import cv2
import torch
import torch.nn as nn
import torchvision.transforms as transforms
import numpy as np

# 1. 환경 설정 및 하이퍼파라미터
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
IMG_SIZE = 224

# CIFAR-10 클래스 정의
classes = ('plane', 'car', 'bird', 'cat', 'deer', 'dog', 'frog', 'horse', 'ship', 'truck')

# 2. 순정 PyTorch Vision Transformer 아키텍처 정의 (가중치 로드용)
class VisionTransformer(nn.Module):
    def __init__(self, img_size=224, patch_size=16, in_channels=3, num_classes=10, d_model=192, nhead=3, num_layers=12):
        super().__init__()
        self.num_patches = (img_size // patch_size) ** 2
        self.patch_embed = nn.Conv2d(in_channels, d_model, kernel_size=patch_size, stride=patch_size)
        self.cls_token = nn.Parameter(torch.zeros(1, 1, d_model))
        self.pos_embed = nn.Parameter(torch.zeros(1, self.num_patches + 1, d_model))
        
        encoder_layer = nn.TransformerEncoderLayer(d_model=d_model, nhead=nhead, dim_feedforward=d_model*4, batch_first=True, activation='gelu')
        self.transformer_encoder = nn.TransformerEncoder(encoder_layer, num_layers=num_layers)
        self.mlp_head = nn.Linear(d_model, num_classes)
        
    def forward(self, x):
        x = self.patch_embed(x) 
        x = x.flatten(2) 
        x = x.transpose(1, 2) 
        cls_tokens = self.cls_token.expand(x.size(0), -1, -1) 
        x = torch.cat((cls_tokens, x), dim=1) 
        x = x + self.pos_embed 
        x = self.transformer_encoder(x)
        cls_output = x[:, 0]
        output = self.mlp_head(cls_output)
        return output

# 3. 모델 생성 및 학습된 가중치(Weights) 로드
model = VisionTransformer(img_size=224, patch_size=16, d_model=192, nhead=3, num_layers=12)
model.load_state_dict(torch.load("pure_vit_cifar10.pth", map_location=device))
model = model.to(device)
model.eval() # 추론 모드 고정

# 4. 실시간 카메라 프레임 전처리용 Transform 정의
# OpenCV는 이미지를 BGR 배열로 읽으므로, 파이토치 이식을 위해 전처리가 필요합니다.
transform = transforms.Compose([
    transforms.ToPILImage(),                    # OpenCV numpy 배열을 PIL 이미지로 변환
    transforms.Resize((IMG_SIZE, IMG_SIZE)),    # ViT 규격인 224x224로 리사이즈
    transforms.ToTensor(),                      # [0, 255] -> [0.0, 1.0] 텐서 변환 및 팩킹
    transforms.Normalize((0.5, 0.5, 0.5), (0.5, 0.5, 0.5)) # 학습 환경과 동일한 정규화
])

# 5. 라이브 카메라(웹캠) 스트리밍 시작
# 기본 내장 웹캠은 0번을 사용합니다. 외부 카메라는 1번 또는 2번일 수 있습니다.
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: 카메라를 열 수 없습니다.")
    exit()

print("실시간 추론을 시작합니다. 종료하려면 'q' 키를 누르세요.")

while True:
    # 카메라로부터 1프레임 읽어오기
    ret, frame = cap.read()
    if not ret:
        print("프레임을 가져올 수 없습니다.")
        break

    # [핵심 전처리 1] OpenCV는 기본이 BGR 채널이므로 PyTorch가 학습한 RGB 채널로 전환합니다.
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    
    # [핵심 전처리 2] 정의한 Transform 통과 -> 결과 차원: [3, 224, 224]
    input_tensor = transform(rgb_frame)
    
    # [핵심 전처리 3] 모델이 요구하는 Batch 차원 추가 ([3, 224, 224] -> [1, 3, 224, 224])
    input_batch = input_tensor.unsqueeze(0).to(device)

    # 6. AI 모델 추론
    with torch.no_grad():
        outputs = model(input_batch)
        
        # Softmax를 통과시켜 각 클래스별 확률값(%) 도출
        probabilities = torch.nn.functional.softmax(outputs[0], dim=0)
        
        # 가장 높은 확률을 가진 인덱스와 그 확률값 추출
        confidence, predicted_idx = torch.max(probabilities, 0)
        
        # 결과 텍스트 포맷팅 (예: "cat (84.5%)")
        class_name = classes[predicted_idx.item()]
        result_text = f"{class_name} ({confidence.item() * 100:.1f}%)"

    # 7. 화면에 결과 출력 (OpenCV 내장 기능 사용)
    # 추론 결과를 실시간 영상 프레임 왼쪽 상단에 자막으로 합성합니다.
    
    cv2.putText(
        frame, 
        result_text, 
        (20, 50),                  # 텍스트가 시작될 좌표 (x, y)
        cv2.FONT_HERSHEY_SIMPLEX,  # 폰트 종류
        1.2,                       # 폰트 크기 비율
        (0, 255, 0),               # 글자 색상 (BGR 순서 -> 초록색)
        3,                         # 선 두께
        cv2.LINE_AA
    )
    
    # 모니터에 자막이 합성된 실시간 윈도우 창 띄우기
    cv2.imshow('Pure PyTorch ViT Live Inference', frame)

    # 사용자가 'q' 키를 누르면 루프를 탈출하고 종료합니다.
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
# 8. 자원 해제
cap.release()
cv2.destroyAllWindows()