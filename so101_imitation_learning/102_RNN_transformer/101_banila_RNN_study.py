import random 
from collections import Counter
import re
import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader

random.seed(42)

samples = [
    ("#Free coupon claim now", 1), ("#Win cash prize now", 1),
    ("#Limited offer click now", 1), ("#Urgent your account winner", 1),
    ("#Free gift card available", 1), ("#Claim your free ticket", 1),
    ("#Meeting schedule for tomorrow", 0), ("P#lease review the project report", 0),
    ("#Let us have lunch today", 0), ("Your# order has been shipped", 0),
    ("#Can we reschedule the meeting", 0), ("T#eam update attached below", 0),
    ("Are you student?", 0),
]

random.shuffle(samples)
#print(samples)

train_samples, test_samples = samples[:10], samples[10:]
#print(train_samples)
#print(test_samples)

def tokenize(text):
    text = text.lower()
    #print("1", text)
    text = re.sub(r"[^a-z0-9\s]", "", text)
    #print("2", text)
    #print(text.split())
    return text.split()

counter = Counter()
for text, _ in train_samples:
    text_tk = tokenize(text)
    #print(text_tk)
    counter.update(text_tk)
#print(counter)

special_tokens = ["<pad>", "<unk>"]
#print("1", sorted(counter.keys()))
vocab = special_tokens + sorted(counter.keys())
#print("2",  vocab)

word2idx = {word: idx for idx, word in enumerate(vocab)}
#print(word2idx)
PAD_IDX, UNK_IDX = word2idx["<pad>"], word2idx["<unk>"]
#print(PAD_IDX, UNK_IDX)

# 1. 모든 문장의 길이를 담을 빈 리스트를 만듭니다.
sentence_lengths = []

# 2. samples에 있는 모든 문장을 하나씩 꺼내어 반복합니다.
for text, _ in samples:
    # 2-1. 문장을 단어 단위로 쪼갭니다 (토큰화)
    tokens = tokenize(text)
    #print(tokens)
    # 2-2. 쪼개진 단어들의 개수(문장 길이)를 구합니다.
    current_length = len(tokens)
    # 2-3. 구한 길이를 리스트에 차곡차곡 모읍니다.
    sentence_lengths.append(current_length)
# 3. 모아둔 길이 중에서 가장 큰 값(가장 긴 문장의 길이)을 찾습니다.
max_len = max(sentence_lengths)
# 결과 확인용 출력
print("모든 문장의 길이 목록:", sentence_lengths)
print("그 중 가장 긴 문장의 길이 (max_len):", max_len)

def encode(text):
    # 1. 문장을 단어 단위로 쪼갭니다.
    # 예: "free coupon" -> ["free", "coupon"]
    tokens = tokenize(text)
    # 2. 숫자로 변환된 단어 번호들을 담을 빈 리스트를 만듭니다.
    encoded_indices = []
    # 3. 쪼개진 단어들을 하나씩 꺼내며 숫자로 바꿉니다.
    for tok in tokens:
        # 3-1. 만약 단어 사전(word2idx)에 이 단어가 있다면, 그 단어의 번호를 가져옵니다.
        if tok in word2idx:
            word_index = word2idx[tok]
        # 3-2. (매우 중요!) 만약 사전에 없는 신조어나 오타라면, 
        # 우리가 약속한 미상 토큰의 번호(UNK_IDX, 보통 1번)로 대체합니다.
        else:
            word_index = UNK_IDX
        # 4. 변환된 번호를 리스트에 차곡차곡 추가합니다.
        encoded_indices.append(word_index)
    # 5. 최종 완성된 숫자 배열을 반환합니다.
    return encoded_indices

def pad_sequence(seq, max_len):
    # 현재 문장의 길이를 잽니다.
    current_len = len(seq)
    # 만약 현재 문장이 목표로 하는 max_len보다 짧다면?
    if current_len < max_len:
        # 모자란 만큼 계산 (예: 목표는 7인데 현재 4단어라면, 3번 반복)
        needed_pads = max_len - current_len
        
        # 필요한 개수만큼 PAD_IDX(보통 0)를 뒤에 채워 넣습니다.
        for _ in range(needed_pads):
            seq.append(PAD_IDX)
            
    return seq

# 3. Dataset 및 DataLoader 정의
class SpamDataset(Dataset):
    def __init__(self, samples, max_len):
        self.samples = samples  # 원본 데이터(텍스트와 정답 레이블 세트)를 공장 창고에 저장합니다.
        self.max_len = max_len  # 문장 맞춤 길이 기준(예: 7단어)을 공장 벽에 붙여둡니다.
        print(self.samples, self.max_len)
       
    def __len__(self): 
        print(len(self.samples))
        return len(self.samples)  # 우리 창고에 전체 데이터가 몇 개 있는지 개수를 반환합니다.
    
    def __getitem__(self, idx):
        # 1. 창고에서 idx번째 데이터를 꺼내옵니다.
        text, label = self.samples[idx]  
        print("22", text)
        # 2. 앞서 만든 통역사(encode)와 재단사(pad_sequence)를 불러와 텍스트를 고정된 길이의 숫자 배열로 가공합니다.
        encode_data = encode(text)
        print("23", encode_data)
        padded = pad_sequence(encode_data, self.max_len)
        print("24", padded)
        # 3. 파이토치 모델이 연산할 수 있도록 가공된 리스트와 정답을 파이토치 텐서(Tensor)라는 특수 상자에 포장합니다.
        # 4. 딥러닝 모델에게 넘겨줄 최종 형태를 딕셔너리 형태로 예쁘게 반환합니다.
        tensor_1 = torch.tensor(padded, dtype=torch.long)
        tensor_2 = torch.tensor(label, dtype=torch.long)
        return {
            "input_ids": tensor_1, 
            "label": tensor_2
        }
train_loader = DataLoader(SpamDataset(train_samples, max_len), batch_size=4, shuffle=True)
test_loader = DataLoader(SpamDataset(test_samples, max_len), batch_size=2)

class RNNClassifier(nn.Module):
    def __init__(self, vocab_size, embed_dim, hidden_dim, num_classes, pad_idx):
        super().__init__()
        self.embedding = nn.Embedding(vocab_size, embed_dim, padding_idx=pad_idx)
        self.rnn = nn.RNN(input_size=embed_dim, hidden_size=hidden_dim, batch_first=True)
        self.fc = nn.Linear(hidden_dim, num_classes)

    def forward(self, input_ids):
        embedded = self.embedding(input_ids)
        _, hidden = self.rnn(embedded)
        context_vector = hidden[-1]
        
        #차원(Shape)과 실제 안에 들어있는 값(실수 배열)을 프린트합니다.
        print("\n=== [Forward] Context Vector (Hidden State) ===")
        print("Shape:", context_vector.shape)
        print("Vector Values:\n", context_vector)
        print("=============================================\n")
        return self.fc(hidden[-1])
    
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(device)
model = RNNClassifier(len(vocab), 16, 32, 2, PAD_IDX).to(device)
criterion = nn.CrossEntropyLoss()
optimizer = torch.optim.Adam(model.parameters(), lr=1e-3)

# 5. 학습 루프
for epoch in range(20):
    model.train()
    for batch in train_loader:
        #print(batch)
        optimizer.zero_grad()
        input_tensor = batch["input_ids"].to(device)
        #print("RNN 입력 텐서 Shape:", input_tensor.shape)
        logits = model(input_tensor.to(device))
        loss = criterion(logits, batch["label"].to(device))
        loss.backward()
        optimizer.step()
    
    # 간단한 평가 생략 (위의 텍스트 예시 참고)
    if (epoch + 1) % 5 == 0: print(f"Epoch {epoch+1} finished.")

def predict_spam(text):
    model.eval()
    text_encoded = encode(text)
    print("text encoding:", text_encoded)
    text_padding = pad_sequence(text_encoded, max_len)
    print("text padding: ", text_padding)
    input_ids = torch.tensor([text_padding], dtype=torch.long).to(device)
    with torch.no_grad():
        pred = model(input_ids).argmax(dim=1).item()
    return "spam" if pred == 1 else "ham"

print(predict_spam("Are you student?"))