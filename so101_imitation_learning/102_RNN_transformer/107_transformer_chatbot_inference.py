import torch
import torch.nn as nn  # 모델 클래스 정의를 위해 필요
import re              # 토큰화(정규표현식)를 위해 필요

# [필수] 1. 계산 장치 설정 (CPU 또는 GPU)
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# [필수] 2. 데이터 전처리를 위한 클래스 (학습 코드와 동일)
class ChatbotVocab:
    def __init__(self):
        self.word2idx = {"<PAD>": 0, "<SOS>": 1, "<EOS>": 2, "<UNK>": 3}
        self.idx2word = {0: "<PAD>", 1: "<SOS>", 2: "<EOS>", 3: "<UNK>"}
        self.vocab_size = 4

    def tokenize(self, text):
        return re.sub(r"[^가-힣a-zA-Z0-9]", " ", text).split()

    def encode(self, text, add_special=True):
        tokens = [self.word2idx.get(w, 3) for w in self.tokenize(text)]
        if add_special: return [1] + tokens + [2]
        return tokens

# [필수] 3. 모델 구조 정의 (학습 코드의 클래스와 100% 일치해야 함)
class TransformerChatbot(nn.Module):
    def __init__(self, vocab_size, d_model=256, nhead=8, num_layers=4):
        super().__init__()
        self.embedding = nn.Embedding(vocab_size, d_model)
        self.transformer = nn.Transformer(d_model=d_model, nhead=nhead, 
                                          num_encoder_layers=num_layers, 
                                          num_decoder_layers=num_layers, batch_first=True)
        self.fc_out = nn.Linear(d_model, vocab_size)

    def forward(self, src, trg):
        # 추론 시에는 학습과 달리 trg_mask가 생성 과정에 따라 달라짐
        trg_mask = self.transformer.generate_square_subsequent_mask(trg.size(1)).to(src.device)
        src_emb, trg_emb = self.embedding(src), self.embedding(trg)
        out = self.transformer(src_emb, trg_emb, tgt_mask=trg_mask)
        return self.fc_out(out)

# 4. 모델 로드 및 답변 생성 함수
def load_chatbot(path):
    # 저장된 파일에는 vocab 정보와 가중치(state_dict)가 들어있음
    checkpoint = torch.load(path, map_location=device, weights_only=False)
    vocab = checkpoint['vocab']
    
    # 모델 객체 생성 후 가중치 입히기
    model = TransformerChatbot(vocab.vocab_size).to(device)
    model.load_state_dict(checkpoint['model_state'])
    model.eval()
    return model, vocab

def get_response(question, model, vocab):
    model.eval()
    with torch.no_grad():
        # 입력을 텐서로 변환하고 배치 차원 추가 (.unsqueeze(0))
        src = torch.tensor([vocab.encode(question)]).to(device)
        trg_indices = [1] # <SOS> 토큰으로 시작
        
        for _ in range(20):
            trg_tensor = torch.tensor([trg_indices]).to(device)
            output = model(src, trg_tensor)
            next_token = output.argmax(dim=-1)[:, -1].item()
            
            trg_indices.append(next_token)
            if next_token == 2: break # <EOS> 토큰 시 중단
            
        return " ".join([vocab.idx2word.get(i, "<UNK>") for i in trg_indices[1:-1]])

# 5. 실행부
if __name__ == "__main__":
    try:
        model, vocab = load_chatbot('chatbot_model.pth')
        print("챗봇이 연결되었습니다. '종료'를 입력하면 멈춥니다.")
        while True:
            user_input = input("나: ")
            if user_input == "종료": break
            response = get_response(user_input, model, vocab)
            print(f"챗봇: {response}")
    except FileNotFoundError:
        print("모델 파일(chatbot_model.pth)이 없습니다. 먼저 학습을 진행해 주세요.")