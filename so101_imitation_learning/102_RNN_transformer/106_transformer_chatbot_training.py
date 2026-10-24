import pandas as pd
import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader
from torch.nn.utils.rnn import pad_sequence
import re

# 1. 데이터 준비 및 전처리 [cite: 1178, 1185]
class ChatbotVocab:
    def __init__(self):
        self.word2idx = {"<PAD>": 0, "<SOS>": 1, "<EOS>": 2, "<UNK>": 3}
        self.idx2word = {0: "<PAD>", 1: "<SOS>", 2: "<EOS>", 3: "<UNK>"}
        self.vocab_size = 4

    def tokenize(self, text):
        return re.sub(r"[^가-힣a-zA-Z0-9]", " ", text).split()

    def build_vocab(self, sentences):
        for sentence in sentences:
            for word in self.tokenize(sentence):
                if word not in self.word2idx:
                    self.word2idx[word] = self.vocab_size
                    self.idx2word[self.vocab_size] = word
                    self.vocab_size += 1

    def encode(self, text, add_special=True):
        tokens = [self.word2idx.get(w, 3) for w in self.tokenize(text)]
        if add_special: return [1] + tokens + [2]
        return tokens

# 데이터 로드
df = pd.read_csv('ChatbotData.csv')
vocab = ChatbotVocab()
vocab.build_vocab(df['Q'].tolist() + df['A'].tolist())

class ChatDataset(Dataset):
    def __init__(self, df, vocab):
        self.questions = [vocab.encode(q) for q in df['Q']]
        self.answers = [vocab.encode(a) for a in df['A']]
    def __len__(self): return len(self.questions)
    def __getitem__(self, idx):
        return torch.tensor(self.questions[idx]), torch.tensor(self.answers[idx])

def collate_fn(batch):
    qs, ans = zip(*batch)
    return pad_sequence(qs, batch_first=True, padding_value=0), \
           pad_sequence(ans, batch_first=True, padding_value=0)

# 2. 트랜스포머 모델 정의 [cite: 1188, 1198]
class TransformerChatbot(nn.Module):
    def __init__(self, vocab_size, d_model=256, nhead=8, num_layers=4):
        super().__init__()
        self.embedding = nn.Embedding(vocab_size, d_model)
        self.transformer = nn.Transformer(d_model=d_model, nhead=nhead, 
                                          num_encoder_layers=num_layers, 
                                          num_decoder_layers=num_layers, batch_first=True)
        self.fc_out = nn.Linear(d_model, vocab_size)

    def forward(self, src, trg):
        trg_mask = self.transformer.generate_square_subsequent_mask(trg.size(1)).to(src.device)
        src_emb, trg_emb = self.embedding(src), self.embedding(trg)
        out = self.transformer(src_emb, trg_emb, tgt_mask=trg_mask)
        return self.fc_out(out)

# 3. 학습 시작
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(device)
model = TransformerChatbot(vocab.vocab_size).to(device)
optimizer = torch.optim.Adam(model.parameters(), lr=0.0001)
criterion = nn.CrossEntropyLoss(ignore_index=0)
loader = DataLoader(ChatDataset(df, vocab), batch_size=32, shuffle=True, collate_fn=collate_fn)

model.train()
for epoch in range(100): # 예시 10 에포크 [cite: 1552]
    for src, trg in loader:
        src, trg = src.to(device), trg.to(device)
        optimizer.zero_grad() # 기록지 지우기 [cite: 1557]
        output = model(src, trg[:, :-1]) # 문제 풀기 [cite: 1559]
        loss = criterion(output.reshape(-1, vocab.vocab_size), trg[:, 1:].reshape(-1)) # 채점 [cite: 1561]
        loss.backward() # 오답 분석 [cite: 1564]
        optimizer.step() # 수정 [cite: 1565]
    print(f"Epoch {epoch+1} Loss: {loss.item():.4f}")

# 학습된 모델 저장 [cite: 1812]
torch.save({'model_state': model.state_dict(), 'vocab': vocab}, 'chatbot_model.pth')