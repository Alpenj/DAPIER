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
        # x shape: [batch_size, text_length] 
        # 예: [64, 50] -> 64개 문장, 각 문장당 50개 단어 번호
        embedded = self.embedding(x)
        # embedded shape: [batch_size, text_length, embed_dim] (★순서 주의!)
        # 예: [64, 50, 128] -> 각 단어 번호가 128차원의 의미 벡터로 변신
        _, (hidden, _) = self.lstm(embedded)
        # hidden shape: [num_layers * num_directions, batch_size, hidden_dim]
        # 예: [1, 64, 256] -> 1개 층의 LSTM이 문장을 끝까지 읽고 남긴 64개 문장의 최종 요약본
        
        # hidden[-1]을 하면 맨 앞의 1차원이 사라지며 [64, 256]이 됩니다.
        # 이를 선형 레이어(fc)와 시그모이드에 통과시켜 각 문장당 0~1 사이의 스편 확률 1개씩을 얻습니다.
        return self.sigmoid(self.fc(hidden[-1])).squeeze()
    
import re
import torch

def predict_spam(text, model, word2idx, device, max_len=50):
    # [1단계: 추론 모드 전환] 
    # 모델에게 "이제 학습하는 거 아니고, 진짜 시험(평가) 보는 거니까 드롭아웃 같은 기능들 끄고 대기해!"라고 지시합니다.
    model.eval() 
    # [2단계: 텍스트 정제 및 토큰화]
    # 2-1. 대문자를 모두 소문자로 바꾸고, 알파벳과 숫자가 아닌 특수문자(공백, 구두점 등)는 전부 빈칸(" ")으로 청소합니다.
    clean_text = re.sub(r"[^a-z0-9]", " ", text.lower())
    # 2-2. 빈칸을 기준으로 단어들을 쪼개서 리스트로 만듭니다.
    tokens = clean_text.split()
    # [3단계: 정수 인코딩 및 최대 길이 제한]
    # 문장이 너무 길면 max_len(50단어)까지만 자르고, 각 단어를 사전 번호로 바꿉니다. 모르는 단어는 1번(<unk>).
    encoded = []
    for t in tokens[:max_len]:
        if t in word2idx:
            encoded.append(word2idx[t])
        else:
            encoded.append(1) # 사전에 없는 단어는 1번방 배치
            
    # [4단계: 패딩 작업]
    # 부족한 길이를 0번(<pad>)으로 채워서 무조건 50개짜리 숫자로 만듭니다.
    needed_zeros = max_len - len(encoded)
    padded = encoded + [0] * needed_zeros
    
    # [5단계: 미니배치 형태의 텐서로 변환] (★매우 중요!)
    # 우리 모델은 무조건 '묶음(배치)' 단위로 데이터를 받도록 설계되어 있습니다.
    # [padded]처럼 대괄호를 한 번 더 감싸서 "1개짜리 메일이 담긴 묶음이다!"라는 뜻의 2차원 [1, 50] 구조로 만듭니다.
    # 그 후 연산을 수행할 장치(CPU 또는 GPU)로 데이터를 보냅니다.
    input_tensor = torch.tensor([padded]).to(device)
    
    # [6단계: 인공지능 모델 예측]
    # torch.no_grad()는 "지금은 시험 보는 중이니까 굳이 오답 노트(그래디언트 고속도로) 계산하지 마, 메모리 아끼자!"라는 뜻입니다.
    with torch.no_grad():
        # 모델에 넣으면 0~1 사이의 확률 텐서 값이 나옵니다.
        prediction_tensor = model(input_tensor)
        # .item()을 쓰면 파이토치 텐서 껍데기를 벗겨내고 순수한 파이썬 숫자(실수)만 쏙 추출합니다.
        prob = prediction_tensor.item()
    
    # [7단계: 결과 해석 및 출력]
    # 확률이 0.5(50%)를 넘으면 스팸(SPAM), 안 넘으면 정상(HAM)으로 판정합니다.
    if prob > 0.5:
        result = "SPAM"
    else:
        result = "HAM"
        
    # 확률 값에 100을 곱해 소수점 둘째 자리까지 예쁘게 퍼센트(%)로 출력 양식을 만듭니다.
    return f"결과: {result} (확률: {prob * 100:.2f}%)"

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