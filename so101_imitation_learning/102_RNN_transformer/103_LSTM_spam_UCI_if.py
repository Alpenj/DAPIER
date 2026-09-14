import torch
import torch.nn as nn
import re

# 학습 시 사용한 모델 구조와 동일해야 함
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

def predict_spam(text, model, word2idx, device, max_len=50):
    model.eval()
    # 텍스트 전처리 및 인코딩
    tokens = re.sub(r"[^a-z0-9]", " ", text.lower()).split()
    encoded = [word2idx.get(t, 1) for t in tokens[:max_len]]
    padded = encoded + [0] * (max_len - len(encoded))
    
    input_tensor = torch.tensor([padded]).to(device)
    with torch.no_grad():
        prob = model(input_tensor).item()
    
    result = "SPAM" if prob > 0.5 else "HAM"
    return f"결과: {result} (확률: {prob*100:.2f}%)"

# 실행부
if __name__ == "__main__":
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    
    # 모델 로드
    checkpoint = torch.load("spam_model.pth", map_location=device, weights_only=False)
    word2idx = checkpoint['word2idx']
    vocab_size = checkpoint['vocab_size']
    
    model = LSTMClassifier(vocab_size, 64, 128).to(device)
    model.load_state_dict(checkpoint['model_state'])
    
    print("스팸 분류기가 준비되었습니다.")
    while True:
        user_input = input("메시지 입력 (종료: q): ")
        if user_input.lower() == 'q': break
        print(predict_spam(user_input, model, word2idx, device))