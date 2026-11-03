import torch
import torch.nn as nn
import torch.optim as optim
from torchvision import datasets, transforms
from torch.utils.data import DataLoader
import matplotlib.pyplot as plt
import os
from torchsummary import summary


# 개선된 MiniVGGNet: Dropout 비중 강화
class MiniVGGNet(nn.Module):
    def __init__(self, num_classes=3):
        super(MiniVGGNet, self).__init__()
        
        # Block 1
        self.block1 = nn.Sequential(
            nn.Conv2d(3, 32, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.BatchNorm2d(32),
            nn.Conv2d(32, 32, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.BatchNorm2d(32),
            nn.MaxPool2d(kernel_size=2, stride=2),
            nn.Dropout(0.3) # Dropout 살짝 증가
        )
        
        # Block 2
        self.block2 = nn.Sequential(
            nn.Conv2d(32, 64, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.BatchNorm2d(64),
            nn.Conv2d(64, 64, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.BatchNorm2d(64),
            nn.MaxPool2d(kernel_size=2, stride=2),
            nn.Dropout(0.4) # 뒤쪽일수록 더 강하게 Dropout
        )
        
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

if __name__ == '__main__':
    IMG_SIZE, BATCH_SIZE, LEARNING_RATE, EPOCHS = 64, 32, 0.001, 30 # 에포크를 조금 더 늘림
    DATA_PATH = './images'
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    # --- [핵심] 데이터 증강(Augmentation) 추가 ---
    transform = transforms.Compose([
        transforms.Resize((IMG_SIZE, IMG_SIZE)),
        transforms.RandomHorizontalFlip(),    # 50% 확률로 좌우 반전
        transforms.RandomRotation(15),         # -15~15도 사이 랜덤 회전
        transforms.ColorJitter(brightness=0.2, contrast=0.2), # 밝기, 대비 조절
        transforms.ToTensor(),
        transforms.Normalize((0.5, 0.5, 0.5), (0.5, 0.5, 0.5))
    ])

    dataset = datasets.ImageFolder(root=DATA_PATH, transform=transform)
    train_loader = DataLoader(dataset, batch_size=BATCH_SIZE, shuffle=True)
    
    model = MiniVGGNet(num_classes=len(dataset.classes)).to(device)
    summary(model, input_size=(3, 64, 64))
    criterion = nn.CrossEntropyLoss()
    optimizer = optim.Adam(model.parameters(), lr=LEARNING_RATE)

    history = {'loss': [], 'acc': []}

    print(f"Using device: {device}")
    print("Starting Training with Augmentation...")
    
    for epoch in range(EPOCHS):
        model.train()
        running_loss, correct, total = 0.0, 0, 0

        for images, labels in train_loader:
            images, labels = images.to(device), labels.to(device)
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
        
        print(f"Epoch [{epoch+1}/{EPOCHS}] Loss: {epoch_loss:.4f} | Acc: {epoch_acc:.2f}%")

    # 그래프 출력
    plt.figure(figsize=(12, 5))
    plt.subplot(1, 2, 1)
    plt.plot(history['loss'], color='red', label='Loss')
    plt.title('Training Loss')
    plt.subplot(1, 2, 2)
    plt.plot(history['acc'], color='blue', label='Accuracy')
    plt.title('Training Accuracy')
    plt.show()

        # 모델의 가중치(state_dict)만 저장
    torch.save(model.state_dict(), "miniVGGnet.pth")
    print("모델 저장 완료: miniVGGnet.pth")