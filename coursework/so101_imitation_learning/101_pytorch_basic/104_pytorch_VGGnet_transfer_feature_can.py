import torch
import torch.nn as nn
import torch.optim as optim
from torchvision import datasets, transforms, models
from torch.utils.data import DataLoader
import matplotlib.pyplot as plt
import os

# 1. 환경 설정
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
IMG_SIZE = 224  # VGG16의 권장 입력 사이즈는 224x224입니다.
BATCH_SIZE = 16 # 데이터가 적으므로 배치를 작게 가져갑니다.
EPOCHS = 20
DATA_PATH = './images'

# 2. 데이터 증강 및 전처리 (전이학습용)
# 모델이 이미 학습한 ImageNet의 평균과 표준편차로 정규화하는 것이 핵심입니다.
transform = transforms.Compose([
    transforms.Resize((IMG_SIZE, IMG_SIZE)),
    transforms.RandomHorizontalFlip(),
    transforms.RandomRotation(10),
    transforms.ToTensor(),
    transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
])

if not os.path.exists(DATA_PATH):
    print("Error: 'images' 폴더를 찾을 수 없습니다.")
else:
    dataset = datasets.ImageFolder(root=DATA_PATH, transform=transform)
    train_loader = DataLoader(dataset, batch_size=BATCH_SIZE, shuffle=True)
    num_classes = len(dataset.classes)

    # 3. VGG16 모델 로드 및 수정
    # 사전 학습된 가중치(Weights)를 함께 불러옵니다.
    model = models.vgg16(weights=models.VGG16_Weights.IMAGENET1K_V1)

    # [특징 추출기 고정] 이미 학습된 앞부분은 학습되지 않도록 고정합니다.
    for param in model.features.parameters():
        param.requires_grad = False

    # [분류기 수정] 마지막 출력층을 3개(can, cup, pet)로 바꿉니다.
    num_ftrs = model.classifier[6].in_features
    model.classifier[6] = nn.Linear(num_ftrs, num_classes)
    model = model.to(device)

    # 4. 손실함수 및 최적화 설정
    criterion = nn.CrossEntropyLoss()
    # 파라미터 중requires_grad=True인 것(분류기)만 최적화 대상에 넣습니다.
    optimizer = optim.Adam(model.classifier.parameters(), lr=0.0001)
    
    # Loss가 정체되면 학습률을 줄이는 스케줄러 추가
    scheduler = optim.lr_scheduler.ReduceLROnPlateau(optimizer, 'min', patience=3, factor=0.1)

    # 5. 학습 루프
    history = {'loss': [], 'acc': []}

    print(f"Using device: {device}")
    print(f"Classes: {dataset.classes}")
    
    for epoch in range(EPOCHS):
        model.train()
        running_loss, correct, total = 0.0, 0, 0

        for images, labels in train_loader:
            images, labels = images.to(device), labels.to(device)
            #print(len(images))
            
            optimizer.zero_grad()
            outputs = model(images)
            loss = criterion(outputs, labels)
            loss.backward()
            optimizer.step()

            running_loss += loss.item()
            _, predicted = outputs.max(1)
            total += labels.size(0)
            correct += predicted.eq(labels).sum().item()

        epoch_loss = running_loss / len(train_loader)
        epoch_acc = 100. * correct / total
        
        history['loss'].append(epoch_loss)
        history['acc'].append(epoch_acc)
        
        # 스케줄러 업데이트
        scheduler.step(epoch_loss)
        
        print(f"Epoch [{epoch+1}/{EPOCHS}] Loss: {epoch_loss:.4f} | Acc: {epoch_acc:.2f}%")

    # 6. 결과 시각화
    plt.figure(figsize=(12, 5))
    plt.subplot(1, 2, 1)
    plt.plot(history['loss'], label='Loss', color='red')
    plt.title('VGG16 Transfer Learning Loss')
    plt.subplot(1, 2, 2)
    plt.plot(history['acc'], label='Accuracy', color='blue')
    plt.title('VGG16 Transfer Learning Accuracy')
    plt.show()

    # 모델 저장
    torch.save(model.state_dict(), "vgg16_transfer_model.pth")
    print("VGG16 Transfer Learning 모델 저장 완료!")