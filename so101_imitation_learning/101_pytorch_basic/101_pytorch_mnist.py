import torch
import torch.nn as nn
import torch.optim as optim
from torchvision import datasets, transforms
from torch.utils.data import DataLoader

# 1. 하이퍼파라미터 및 장치 설정
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
batch_size = 64
learning_rate = 0.001
epochs = 5

print(device)

# 2. 데이터셋 및 로더 (TensorFlow의 데이터 파이프라인 역할)
transform = transforms.Compose([
    transforms.ToTensor(),
    transforms.Normalize((0.1307,), (0.3081,))
])

train_dataset = datasets.MNIST(root='./data', train=True, transform=transform, download=True)
test_dataset = datasets.MNIST(root='./data', train=False, transform=transform)

train_loader = DataLoader(dataset=train_dataset, batch_size=batch_size, shuffle=True)
test_loader = DataLoader(dataset=test_dataset, batch_size=batch_size, shuffle=False)

# 3. 모델 정의 (nn.Module 상속)
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

model = MultiLayerPerceptron().to(device)

# 4. 손실 함수 및 최적화 도구
criterion = nn.CrossEntropyLoss() # Softmax가 내장되어 있음
optimizer = optim.Adam(model.parameters(), lr=learning_rate)

# 5. 학습 루프
for epoch in range(epochs):
    model.train() # 학습 모드 전환
    for batch_idx, (data, target) in enumerate(train_loader):
        data, target = data.to(device), target.to(device)

        # Gradient 초기화
        optimizer.zero_grad()
        # Forward pass
        output = model(data)
        # Loss 계산
        loss = criterion(output, target)
        # Backward pass (Backpropagation)
        loss.backward()
        # 가중치 업데이트
        optimizer.step()

        if batch_idx % 200 == 0:
            print(f"Epoch [{epoch+1}/{epochs}], Step [{batch_idx}/{len(train_loader)}], Loss: {loss.item():.4f}")

# 6. 추론 (Inference)
model.eval() # 평가 모드 전환 (Dropout, Batch Norm 등 비활성화)
correct = 0
with torch.no_grad(): # Gradient 계산 안 함 (메모리 절약)
    for data, target in test_loader:
        data, target = data.to(device), target.to(device)
        output = model(data)
        pred = output.argmax(dim=1, keepdim=True)
        correct += pred.eq(target.view_as(pred)).sum().item()

print(f'\nTest Accuracy: {correct}/{len(test_dataset)} ({100. * correct / len(test_dataset):.2f}%)')

# 모델의 가중치(state_dict)만 저장
torch.save(model.state_dict(), "mnist_mlp.pth")
print("모델 저장 완료: mnist_mlp.pth")