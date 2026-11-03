import torch
import torchvision
import torchvision.transforms as transforms
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim

def main():
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}")

    transform = transforms.Compose([
        transforms.ToTensor(),
        transforms.Normalize((0.5, 0.5, 0.5), (0.5, 0.5, 0.5))
    ])

    batch_size = 64
    trainset = torchvision.datasets.CIFAR10(root='./data', train=True, download=True, transform=transform)
    trainloader = torch.utils.data.DataLoader(trainset, batch_size=batch_size, shuffle=True, num_workers=2)

    testset = torchvision.datasets.CIFAR10(root='./data', train=False, download=True, transform=transform)
    testloader = torch.utils.data.DataLoader(testset, batch_size=batch_size, shuffle=False, num_workers=2)

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
            x = self.dropout_fc(x) # 이 부분이 수정되었습니다.
            x = self.fc2(x)
            return x

    net = ImprovedCNN().to(device)
    criterion = nn.CrossEntropyLoss()
    optimizer = optim.Adam(net.parameters(), lr=0.001)

    print(f"Starting Training for 10 epochs...")
    for epoch in range(10):
        running_loss = 0.0
        net.train()
        for i, data in enumerate(trainloader, 0):
            inputs, labels = data[0].to(device), data[1].to(device)

            optimizer.zero_grad()
            outputs = net(inputs)
            loss = criterion(outputs, labels)
            loss.backward()
            optimizer.step()

            running_loss += loss.item()
            if i % 200 == 199:
                print(f'[{epoch + 1}, {i + 1:5d}] loss: {running_loss / 200:.3f}')
                running_loss = 0.0

    print('Finished Training')

    net.eval()
    correct = 0
    total = 0
    with torch.no_grad():
        for data in testloader:
            images, labels = data[0].to(device), data[1].to(device)
            outputs = net(images)
            _, predicted = torch.max(outputs.data, 1)
            total += labels.size(0)
            correct += (predicted == labels).sum().item()

    print(f'Final Accuracy: {100 * correct / total:.2f}%')

    # 모델의 가중치(state_dict)만 저장
    torch.save(net.state_dict(), "cifar10_cnn.pth")
    print("모델 저장 완료: cifar10_cnn.pth")

if __name__ == '__main__':
    main()