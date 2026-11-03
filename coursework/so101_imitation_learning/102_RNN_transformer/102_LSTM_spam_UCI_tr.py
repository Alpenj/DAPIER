import pandas as pd
import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader
from sklearn.model_selection import train_test_split
from collections import Counter
import re

# 1. 데이터 로드 및 전처리
url = "https://raw.githubusercontent.com/mohitgupta-omg/Kaggle-SMS-Spam-Collection-Dataset-/master/spam.csv"
df = pd.read_csv(url, encoding='latin-1')[['v1', 'v2']]
df.columns = ['label', 'text']
df['label'] = df['label'].map({'ham': 0, 'spam': 1})

def tokenize(text):
    return re.sub(r"[^a-z0-9]", " ", text.lower()).split()

# 어휘 사전 구축 (상위 5000개 단어)
all_tokens = [tok for text in df['text'] for tok in tokenize(text)]
vocab_size = 5000
counts = Counter(all_tokens)
vocab = ["<pad>", "<unk>"] + [word for word, _ in counts.most_common(vocab_size-2)]
word2idx = {word: idx for idx, word in enumerate(vocab)}

def encode(text, max_len=50):
    tokens = tokenize(text)
    encoded = [word2idx.get(t, 1) for t in tokens[:max_len]]
    return encoded + [0] * (max_len - len(encoded))

# 2. Dataset 및 모델 정의
class SpamDataset(Dataset):
    def __init__(self, texts, labels):
        self.X = [torch.tensor(encode(t)) for t in texts]
        self.y = torch.tensor(labels.values)
    def __len__(self): return len(self.X)
    def __getitem__(self, idx): return self.X[idx], self.y[idx]

class LSTMClassifier(nn.Module):
    def __init__(self, vocab_size, embed_dim, hidden_dim):
        super().__init__()
        self.embedding = nn.Embedding(vocab_size, embed_dim, padding_idx=0)
        self.lstm = nn.LSTM(embed_dim, hidden_dim, batch_first=True)
        self.fc = nn.Linear(hidden_dim, 1)
        self.sigmoid = nn.Sigmoid()

    def forward(self, x):
        embedded = self.embedding(x)
        _, (hidden, _) = self.lstm(embedded)
        return self.sigmoid(self.fc(hidden[-1])).squeeze()

# 3. 학습 설정 및 실행
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
train_texts, test_texts, train_labels, test_labels = train_test_split(df['text'], df['label'], test_size=0.2)
train_loader = DataLoader(SpamDataset(train_texts, train_labels), batch_size=64, shuffle=True)

model = LSTMClassifier(vocab_size, 64, 128).to(device)
optimizer = torch.optim.Adam(model.parameters(), lr=0.001)
criterion = nn.BCELoss()

print("학습 시작...")
for epoch in range(50):
    model.train()
    total_loss = 0
    for inputs, labels in train_loader:
        inputs, labels = inputs.to(device), labels.float().to(device)
        optimizer.zero_grad()
        loss = criterion(model(inputs), labels)
        loss.backward()
        optimizer.step()
        total_loss += loss.item()
    print(f"Epoch {epoch+1}, Loss: {total_loss/len(train_loader):.4f}")

# 모델 및 사전 정보 저장
torch.save({
    'model_state': model.state_dict(),
    'word2idx': word2idx,
    'vocab_size': vocab_size
}, "spam_model.pth")
print("모델 저장 완료: spam_model.pth")