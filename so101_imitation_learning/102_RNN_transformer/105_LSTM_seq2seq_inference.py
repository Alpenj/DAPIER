import torch
import torch.nn as nn


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
    
# 모델 클래스는 위와 동일하게 선언되어 있어야 합니다.
def inference(text, model_path):
    checkpoint = torch.load(model_path)
    ko_vocab, en_vocab = checkpoint['ko'], checkpoint['en']
    en_idx2word = {v: k for k, v in en_vocab.items()}
    
    enc = Encoder(len(ko_vocab), 64, 128).to(device)
    dec = Decoder(len(en_vocab), 64, 128).to(device)
    enc.load_state_dict(checkpoint['enc'])
    dec.load_state_dict(checkpoint['dec'])
    
    enc.eval(); dec.eval()
    
    with torch.no_grad():
        # 1. 인코딩 (Context Vector 추출)
        src = torch.tensor([[ko_vocab.get(w, 3) for w in text.split()]]).to(device)
        hidden, cell = enc(src)
        
        # 2. 디코딩 (문장 생성)
        input_token = torch.tensor([1]).to(device) # <SOS>
        result = []
        for _ in range(10): # 최대 10단어
            output, hidden, cell = dec(input_token, hidden, cell)
            top1 = output.argmax(1).item()
            if top1 == 2: break # <EOS> 시 종료
            result.append(en_idx2word[top1])
            input_token = torch.tensor([top1]).to(device)
            
        return " ".join(result)

# 실행 예시
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
print(f"결과: {inference('이거 뭐니', 'seq2seq.pth')}")