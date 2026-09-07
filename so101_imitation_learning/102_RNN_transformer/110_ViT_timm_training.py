import torch
import torch.nn as nn
import torch.optim as optim
import torchvision
import torchvision.transforms as transforms
from torch.utils.data import DataLoader
import timm
import matplotlib.pyplot as plt
import numpy as np

# 1. 환경 설정 및 하이퍼파라미터
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
BATCH_SIZE = 64
LEARNING_RATE = 2e-4  # ViT는 보통 작은 학습률에서 안정적입니다.
EPOCHS = 5
IMG_SIZE = 224 # ViT 기본 입력 사이즈에 맞게 리사이즈

print(f"Using device: {device}")

# 2. 데이터셋 준비 (CIFAR-10)
# ViT 성능을 위해 이미지를 224x224로 리사이즈하고 정규화합니다.
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

# 3. 모델 정의 (timm 라이브러리 활용)
# 'vit_tiny_patch16_224'는 파라미터가 적어 학습 속도가 빠릅니다.
model = timm.create_model('vit_tiny_patch16_224', pretrained=True, num_classes=10)
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

# 6. 추론(Inference) 및 시각화
def imshow(img):
    img = img / 2 + 0.5 # unnormalize
    npimg = img.numpy()
    plt.imshow(np.transpose(npimg, (1, 2, 0)))
    plt.axis('off')

model.eval()
dataiter = iter(test_loader)
images, labels = next(dataiter)

# 일부 이미지만 추론
with torch.no_grad():
    outputs = model(images.to(device))
    _, predicted = torch.max(outputs, 1)

# 결과 출력 (상위 4개)
plt.figure(figsize=(10, 4))
for i in range(4):
    plt.subplot(1, 4, i + 1)
    imshow(images[i])
    plt.title(f"GT: {classes[labels[i]]}\nPred: {classes[predicted[i]]}")
plt.show()

# 7. 전체 정확도 측정
correct = 0
total = 0
with torch.no_grad():
    for images, labels in test_loader:
        images, labels = images.to(device), labels.to(device)
        outputs = model(images)
        _, predicted = torch.max(outputs.data, 1)
        total += labels.size(0)
        correct += (predicted == labels).sum().item()

print(f'Accuracy on the 10000 test images: {100 * correct / total:.2f}%')
torch.save(model.state_dict(), "vit_cifar10_timm.pth")