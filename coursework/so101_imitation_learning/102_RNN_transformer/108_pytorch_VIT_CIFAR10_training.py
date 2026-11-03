import torch
import torch.nn as nn
import torch.optim as optim
import torchvision
import torchvision.transforms as transforms
from torch.utils.data import DataLoader
import matplotlib.pyplot as plt
import numpy as np

# 1. 환경 설정 및 하이퍼파라미터
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
BATCH_SIZE = 64
LEARNING_RATE = 2e-4
EPOCHS = 5
IMG_SIZE = 224      # 입력 이미지 크기 (224x224)
PATCH_SIZE = 16     # 패치 한 개의 크기 (16x16)

print(f"Using device: {device}")

# 2. 데이터셋 준비 (CIFAR-10)
transform = transforms.Compose([
    transforms.Resize((IMG_SIZE, IMG_SIZE)),
    transforms.ToTensor(),
    transforms.Normalize((0.5, 0.5, 0.5), (0.5, 0.5, 0.5))
])

train_dataset = torchvision.datasets.CIFAR10(root='./data', train=True, download=True, transform=transform)
test_dataset = torchvision.datasets.CIFAR10(root='./data', train=False, download=True, transform=transform)

train_loader = DataLoader(train_dataset, batch_size=BATCH_SIZE, shuffle=True)
test_loader = DataLoader(test_dataset, batch_size=BATCH_SIZE, shuffle=False)

classes = ('plane', 'car', 'bird', 'cat', 'deer', 'dog', 'frog', 'horse', 'ship', 'truck')


# 3. 바닥부터 짜는 Pure PyTorch Vision Transformer (timm 프리!)
class VisionTransformer(nn.Module):
    def __init__(self, img_size=224, patch_size=16, in_channels=3, num_classes=10, d_model=192, nhead=3, num_layers=12):
        super().__init__()
        
        # (224 / 16) * (224 / 16) = 14 * 14 = 196개의 패치가 생성됩니다.
        self.num_patches = (img_size // patch_size) ** 2
        
        # [핵심 1] Patch Embedding: 이미지 격자를 칼로 자르듯 쪼개서 벡터로 만드는 단계
        # nn.Conv2d의 커널 크기와 스트라이드를 patch_size(16)로 주면 겹치지 않게 자르는 효과가 납니다.
        self.patch_embed = nn.Conv2d(in_channels, d_model, kernel_size=patch_size, stride=patch_size)
        
        # [핵심 2] CLS Token: 챗봇의 시작 토큰(<SOS>)처럼 문장 맨 앞에 붙여줄 '클래스 대표 토큰'
        self.cls_token = nn.Parameter(torch.zeros(1, 1, d_model))
        
        # [핵심 3] Positional Embedding: 패치들의 고유 위치 번호표 행렬 (총 196개 패치 + CLS 토큰 1개 = 197개)
        self.pos_embed = nn.Parameter(torch.zeros(1, self.num_patches + 1, d_model))
        
        # [핵심 4] Transformer Encoder: 챗봇 지하창고에서 보았던 파이토치 순정 인코더 블록 조립식 엔진
        encoder_layer = nn.TransformerEncoderLayer(d_model=d_model, nhead=nhead, dim_feedforward=d_model*4, batch_first=True, activation='gelu')
        self.transformer_encoder = nn.TransformerEncoder(encoder_layer, num_layers=num_layers)
        
        # 최종 분류 헤드 (192차원 문맥 벡터를 CIFAR10의 정답 클래스 개수인 10개 점수로 변환)
        self.mlp_head = nn.Linear(d_model, num_classes)
        
    def forward(self, x):
        # x 차원 변화: [Batch, 3, 224, 224] (이미지 덩어리 진입)
        print("forward: ", x)
        # 1. 패치 임베딩 거치기 -> [Batch, 192, 14, 14]
        x = self.patch_embed(x) 
        
        # 2. 2차원 평면을 트랜스포머가 읽을 수 있게 1차원으로 쫙 펴기(Flatten) -> [Batch, 192, 196]
        x = x.flatten(2) 
        
        # 3. 축 바꾸기 (순서대로 나열) -> [Batch, 196, 192] (196개 패치 단어가 각각 192차원 노트를 가진 상태)
        x = x.transpose(1, 2) 
        
        # 4. 맨 앞에 CLS 토큰 호위무사 결합하기
        # 배치 사이즈만큼 CLS 토큰 복사 -> [Batch, 1, 192]
        cls_tokens = self.cls_token.expand(x.size(0), -1, -1) 
        # 결합하여 총 197개 단어 벡터로 부풀림 -> [Batch, 197, 192]
        x = torch.cat((cls_tokens, x), dim=1) 
        
        # 5. 위치 정보(번호표) 더하기 -> [Batch, 197, 192]
        x = x + self.pos_embed 
        
        # 6. 순정 파이토치 트랜스포머 인코더 12층 통과하기 -> [Batch, 197, 192]
        x = self.transformer_encoder(x)
        
        # 7. 오직 맨 앞의 CLS 토큰(0번 인덱스) 자리의 최종 요약 벡터만 쏙 뽑아내기 -> [Batch, 192]
        cls_output = x[:, 0]
        
        # 8. 최종 10개 클래스 점수판(Logits) 계산 -> [Batch, 10]
        output = self.mlp_head(cls_output)
        return output

# 모델 선언 ('vit_tiny_patch16_224'와 완벽하게 동일한 순정 규격 규정)
model = VisionTransformer(img_size=224, patch_size=16, d_model=192, nhead=3, num_layers=12)
model = model.to(device)

# 4. 손실 함수 및 최적화 도구
criterion = nn.CrossEntropyLoss()
optimizer = optim.AdamW(model.parameters(), lr=LEARNING_RATE, weight_decay=0.05)

# 5. 학습(Training) 루프
print("Starting Training...")
model.train()
for epoch in range(EPOCHS):
    running_loss = 0.0
    for i, (images, labels) in enumerate(train_loader):
        images, labels = images.to(device), labels.to(device)

        optimizer.zero_grad()
        outputs = model(images)
        loss = criterion(outputs, labels)
        loss.backward()
        optimizer.step()

        running_loss += loss.item()
        if (i + 1) % 100 == 0:
            print(f"Epoch [{epoch+1}/{EPOCHS}], Step [{i+1}/{len(train_loader)}], Loss: {running_loss/100:.4f}")
            running_loss = 0.0

print("Finished Training")

# 6. 전체 정확도 측정 및 가중치 저장
correct = 0
total = 0
model.eval()
with torch.no_grad():
    for images, labels in test_loader:
        images, labels = images.to(device), labels.to(device)
        outputs = model(images)
        _, predicted = torch.max(outputs.data, 1)
        total += labels.size(0)
        correct += (predicted == labels).sum().item()

print(f'Accuracy on the 10000 test images: {100 * correct / total:.2f}%')
torch.save(model.state_dict(), "pure_vit_cifar10.pth")