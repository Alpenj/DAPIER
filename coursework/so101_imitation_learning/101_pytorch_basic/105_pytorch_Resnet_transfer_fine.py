import torch
import torch.nn as nn
import torch.optim as optim
from torchvision import datasets, transforms, models
from torch.utils.data import DataLoader, random_split
import matplotlib.pyplot as plt
from torchsummary import summary

# 1. 환경 설정 및 데이터 준비
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
IMG_SIZE = 224
BATCH_SIZE = 16
DATA_PATH = './images'

# 전이학습용 전처리 (ImageNet 통계값 사용)
transform = transforms.Compose([
    transforms.Resize((IMG_SIZE, IMG_SIZE)),
    transforms.RandomHorizontalFlip(),
    transforms.ToTensor(),
    transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
])

dataset = datasets.ImageFolder(root=DATA_PATH, transform=transform)

# 데이터 분리 (학습 80%, 검증 20%)
train_size = int(0.8 * len(dataset))
val_size = len(dataset) - train_size
train_dataset, val_dataset = random_split(dataset, [train_size, val_size])

train_loader = DataLoader(train_dataset, batch_size=BATCH_SIZE, shuffle=True)
val_loader = DataLoader(val_dataset, batch_size=BATCH_SIZE, shuffle=False)

# 2. 모델 생성 (ResNet18)
model = models.resnet18(weights=models.ResNet18_Weights.IMAGENET1K_V1)
#summary(model, input_size=(3, 224, 224))

# 마지막 출력층 수정 (3개 클래스: can, cup, pet)
num_ftrs = model.fc.in_features
model.fc = nn.Linear(num_ftrs, len(dataset.classes))
model = model.to(device)

criterion = nn.CrossEntropyLoss()

# ---------------------------------------------------------
# STEP 1: 특징 추출 (Feature Extraction)
# ---------------------------------------------------------
print("\n[Step 1] Feature Extraction Starting...")

# 1단계에서는 마지막 층(fc)만 학습하도록 앞부분 고정
for param in model.parameters():
    param.requires_grad = False
for param in model.fc.parameters():
    param.requires_grad = True

# 분류기(fc)만 최적화
optimizer = optim.Adam(model.fc.parameters(), lr=0.001)

def train_model(epochs, phase_name):
    history = {'loss': [], 'acc': [], 'val_acc': []}
    for epoch in range(epochs):
        model.train()
        running_loss, correct, total = 0.0, 0, 0
        for imgs, labels in train_loader:
            imgs, labels = imgs.to(device), labels.to(device)
            optimizer.zero_grad()
            outputs = model(imgs)
            loss = criterion(outputs, labels)
            loss.backward()
            optimizer.step()
            running_loss += loss.item()
            _, pred = outputs.max(1)
            correct += pred.eq(labels).sum().item()
            total += labels.size(0)
        
        # 검증
        model.eval()
        v_correct, v_total = 0, 0
        with torch.no_grad():
            for v_imgs, v_labels in val_loader:
                v_imgs, v_labels = v_imgs.to(device), v_labels.to(device)
                v_outputs = model(v_imgs)
                _, v_pred = v_outputs.max(1)
                v_correct += v_pred.eq(v_labels).sum().item()
                v_total += v_labels.size(0)
        
        train_acc = 100. * correct / total
        val_acc = 100. * v_correct / v_total
        print(f"{phase_name} Epoch {epoch+1} Loss: {running_loss/len(train_loader):.4f} | Train Acc: {train_acc:.2f}% | Val Acc: {val_acc:.2f}%")
        history['loss'].append(running_loss/len(train_loader))
        history['acc'].append(train_acc)
        history['val_acc'].append(val_acc)
    return history

# 1단계 실행 (짧게 10회)
hist1 = train_model(10, "Feature-Ex")

# ---------------------------------------------------------
# STEP 2: 파인 튜닝 (Fine-Tuning)
# ---------------------------------------------------------
print("\n[Step 2] Fine-Tuning Starting...")

# 모든 레이어의 고정을 풀고 미세 조정 허용
for param in model.parameters():
    param.requires_grad = True

# 주의: 파인튜닝 시 학습률은 매우 낮게 설정 (0.0001 이하)
optimizer = optim.Adam(model.parameters(), lr=0.0001)

# 2단계 실행 (15회)
hist2 = train_model(15, "Fine-Tuning")

# 3. 결과 시각화
total_acc = hist1['acc'] + hist2['acc']
total_val_acc = hist1['val_acc'] + hist2['val_acc']

plt.figure(figsize=(10, 6))
plt.plot(total_acc, label='Train Accuracy')
plt.plot(total_val_acc, label='Validation Accuracy')
plt.axvline(x=9, color='r', linestyle='--', label='Fine-tuning Start')
plt.title('ResNet18 2-Step Transfer Learning')
plt.xlabel('Epochs')
plt.ylabel('Accuracy (%)')
plt.legend()
plt.show()

torch.save(model.state_dict(), "resnet18_final.pth")