import torch
import torch.nn as nn
import torch.optim as optim

# 1. 데이터 준비 (간단한 번역 쌍)
data = [
    ("나 가고 싶어", "i want to go"),
    ("그거 좋아", "i like it"),
    ("나 행복해", "i am happy"),
    ("너는 학생이야", "you are a student"),
    ("이거 뭐야", "what is this")
]

# 단어 사전 구축
def build_vocab(sentences):
    vocab = {"<PAD>": 0, "<SOS>": 1, "<EOS>": 2, "<UNK>": 3}
    for sent in sentences:  # sent -> "claim now for free"
        for word in sent.split():  # word -> [ 'claim', 'now', 'for', 'free']
            if word not in vocab:  # 중복방지 
                vocab[word] = len(vocab)
    return vocab

ko_vocab = build_vocab([d[0] for d in data])
en_vocab = build_vocab([d[1] for d in data])
en_idx2word = {v: k for k, v in en_vocab.items()}

# 2. 모델 정의 (인코더 & 디코더)
class Encoder(nn.Module):
    def __init__(self, input_dim, emb_dim, hid_dim):
        super().__init__()
        self.embedding = nn.Embedding(input_dim, emb_dim)
        self.rnn = nn.LSTM(emb_dim, hid_dim, batch_first=True)

    def forward(self, src):
        _, (hidden, cell) = self.rnn(self.embedding(src))
        return hidden, cell  # 이것이 바로 Context Vector!

class Decoder(nn.Module):
    def __init__(self, output_dim, emb_dim, hid_dim):
        super().__init__()
        self.embedding = nn.Embedding(output_dim, emb_dim)
        self.rnn = nn.LSTM(emb_dim, hid_dim, batch_first=True)
        self.fc_out = nn.Linear(hid_dim, output_dim)

    def forward(self, input, hidden, cell):
        input = input.unsqueeze(1) # [batch, 1]
        output, (hidden, cell) = self.rnn(self.embedding(input), (hidden, cell))
        prediction = self.fc_out(output.squeeze(1))
        return prediction, hidden, cell

# 3. 학습 루프
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
HID_DIM = 128
enc = Encoder(len(ko_vocab), 64, HID_DIM).to(device)
dec = Decoder(len(en_vocab), 64, HID_DIM).to(device)
optimizer = optim.Adam(list(enc.parameters()) + list(dec.parameters()), lr=0.001)
criterion = nn.CrossEntropyLoss(ignore_index=0)

for epoch in range(100):
    total_loss = 0
    for ko, en in data:
        src = torch.tensor([[ko_vocab[w] for w in ko.split()]]).to(device)
        trg = torch.tensor([[en_vocab[w] for w in en.split()]]).to(device)
        print(src)
        print(trg)
        
        optimizer.zero_grad()
        hidden, cell = enc(src) # 인코더가 Context Vector 생성
        
        input_token = torch.tensor([1]).to(device) # <SOS> 시작
        loss = 0
        for t in range(trg.size(1)):
            output, hidden, cell = dec(input_token, hidden, cell)
            loss += criterion(output, trg[:, t])
            input_token = trg[:, t] # Teacher Forcing
            
        loss.backward()
        optimizer.step()
        total_loss += loss.item()

# 학습된 상태 저장
torch.save({'enc': enc.state_dict(), 'dec': dec.state_dict(), 'ko': ko_vocab, 'en': en_vocab}, "seq2seq.pth")