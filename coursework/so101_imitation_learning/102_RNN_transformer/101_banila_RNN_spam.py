import re
import random
from collections import Counter

import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader

# 1. 데이터 준비
random.seed(42)
torch.manual_seed(42)

samples = [
    ("free coupon claim now", 1), ("win cash prize now", 1),
    ("limited offer click now", 1), ("urgent your account winner", 1),
    ("free gift card available", 1), ("claim your free ticket", 1),
    ("meeting schedule for tomorrow", 0), ("please review the project report", 0),
    ("let us have lunch today", 0), ("your order has been shipped", 0),
    ("can we reschedule the meeting", 0), ("team update attached below", 0),
]
random.shuffle(samples)
train_samples, test_samples = samples[:10], samples[10:]

# 2. 토큰화 및 어휘 사전 구축
def tokenize(text):
    text = text.lower()
    text = re.sub(r"[^a-z0-9\s]", "", text)
    return text.split()

counter = Counter()
for text, _ in train_samples:
    counter.update(tokenize(text))

special_tokens = ["<pad>", "<unk>"]
vocab = special_tokens + sorted(counter.keys())
word2idx = {word: idx for idx, word in enumerate(vocab)}
PAD_IDX, UNK_IDX = word2idx["<pad>"], word2idx["<unk>"]

max_len = max(len(tokenize(text)) for text, _ in samples)

def encode(text):
    return [word2idx.get(tok, UNK_IDX) for tok in tokenize(text)]

def pad_sequence(seq, max_len):
    return seq + [PAD_IDX] * (max_len - len(seq))

# 3. Dataset 및 DataLoader 정의
class SpamDataset(Dataset):
    def __init__(self, samples, max_len):
        self.samples = samples
        self.max_len = max_len
    def __len__(self): return len(self.samples)
    def __getitem__(self, idx):
        text, label = self.samples[idx]
        padded = pad_sequence(encode(text), self.max_len)
        return {"input_ids": torch.tensor(padded, dtype=torch.long), "label": torch.tensor(label, dtype=torch.long)}

train_loader = DataLoader(SpamDataset(train_samples, max_len), batch_size=4, shuffle=True)
test_loader = DataLoader(SpamDataset(test_samples, max_len), batch_size=2)

# 4. RNN 모델 정의
class RNNClassifier(nn.Module):
    def __init__(self, vocab_size, embed_dim, hidden_dim, num_classes, pad_idx):
        super().__init__()
        self.embedding = nn.Embedding(vocab_size, embed_dim, padding_idx=pad_idx)
        self.rnn = nn.RNN(input_size=embed_dim, hidden_size=hidden_dim, batch_first=True)
        self.fc = nn.Linear(hidden_dim, num_classes)

    def forward(self, input_ids):
        embedded = self.embedding(input_ids)
        _, hidden = self.rnn(embedded)
        return self.fc(hidden[-1])

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
model = RNNClassifier(len(vocab), 16, 32, 2, PAD_IDX).to(device)
criterion = nn.CrossEntropyLoss()
optimizer = torch.optim.Adam(model.parameters(), lr=1e-3)

# 5. 학습 루프
for epoch in range(20):
    model.train()
    for batch in train_loader:
        optimizer.zero_grad()
        logits = model(batch["input_ids"].to(device))
        loss = criterion(logits, batch["label"].to(device))
        loss.backward()
        optimizer.step()
    
    # 간단한 평가 생략 (위의 텍스트 예시 참고)
    if (epoch + 1) % 5 == 0: print(f"Epoch {epoch+1} finished.")

# 6. 추론 예시
def predict_spam(text):
    model.eval()
    input_ids = torch.tensor([pad_sequence(encode(text), max_len)], dtype=torch.long).to(device)
    with torch.no_grad():
        pred = model(input_ids).argmax(dim=1).item()
    return "spam" if pred == 1 else "ham"

print(predict_spam("Are you student?"))