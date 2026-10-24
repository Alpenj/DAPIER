import pandas as pd
import re
from collections import Counter
from torch.utils.data import Dataset, DataLoader
import torch
import torch.nn as nn
from sklearn.model_selection import train_test_split

# 1. 데이터 불러오기
url = "https://raw.githubusercontent.com/mohitgupta-omg/Kaggle-SMS-Spam-Collection-Dataset-/master/spam.csv"
df = pd.read_csv(url, encoding='latin-1')
# 2. 필요한 컬럼(v1, v2)만 남기고 이름 바꾸기
df = df[['v1', 'v2']]
df.columns = ['label', 'text']
# 3. 문자열 레이블(ham, spam)을 모델이 학습할 수 있게 숫자(0, 1)로 바꾸기
# ham은 정상(0), spam은 스팸(1)
df['label'] = df['label'].map({'ham': 0, 'spam': 1})
#print(df.head())

def tokenize(text):
    text_1 = re.sub(r"[^a-z0-9]", " ", text.lower())
    #print(text_1)
    text_2 = text_1.split()
    return text_2

# 1. 모든 단어(토큰)들을 차곡차곡 모아둘 빈 바구니를 만듭니다.
all_tokens = []
# 2. [바깥쪽 루프] 데이터프레임의 'text' 컬럼에서 문장(text)을 한 줄씩 꺼냅니다.
for text in df['text']:
    # 2-1. 꺼내온 문장을 단어 단위로 쪼갭니다. (예: "claim now" -> ["claim", "now"])
    #print(text)
    tokens = tokenize(text)
    #print(tokens)
    # 3. [안쪽 루프] 쪼개진 단어 리스트에서 단어(tok)를 하나씩 다시 꺼냅니다.
    for tok in tokens:
        # 4. 빈 바구니(all_tokens)에 단어를 하나씩 추가합니다.
        all_tokens.append(tok)
#print(all_tokens) # 단어들이 중복되어 있음

vocab_size = 5000
counts = Counter(all_tokens)
#print(counts)  # Counter({'i': 3008, 'to': 2242, 'you': 2241, 'a' ... 이런식으로 단어 당 출현빈도를 표시함  

# 1. 딥러닝 연산과 예외 처리에 꼭 필요한 특수 토큰 2개를 먼저 사전에 넣어둡니다.
# <pad>는 0번 방, <unk>는 1번 방으로 자동 배정됩니다.
vocab = ["<pad>", "<unk>"]
# 2. counts(Counter 객체)에서 가장 많이 등장한 단어를 위에서부터 지정한 개수만큼 가져옵니다.
# 전체 vocab_size 중 앞서 넣은 특수 토큰 2개를 뺀 나머지(vocab_size - 2)만큼의 상위 단어를 추출합니다.
top_words_with_counts = counts.most_common(vocab_size - 2)
# 3. 가져온 데이터는 [('단어1', 빈도수), ('단어2', 빈도수)] 형태의 묶음(튜플) 리스트입니다.
# 우리가 필요한 것은 '빈도수(숫자)'가 아니라 오직 '단어(글자)' 그 자체이므로, 반복문을 돌며 단어만 꺼냅니다.
for word, count in top_words_with_counts:
    # 4. 빈도수 정보는 무시하고, '단어'만 단어 사전(vocab) 리스트 뒤에 차곡차곡 이어 붙입니다.
    vocab.append(word)
# 최종 완성된 vocab 리스트의 인덱스(위치 번호)가 곧 단어의 고유 번호(정수 인코딩 값)가 됩니다!
#print("최종 단어 사전:", vocab)
word2idx = {}
for idx, word in enumerate(vocab):
    #print(idx, word)
    word2idx [word] = idx
print(word2idx)   # {'<pad>': 0, '<unk>': 1, 'i': 2, 'to': 3, 'you': 4, 'a': 5,

def encode(text, max_len=50):  # " I am student" -> [ 1, 3, 4, 0 ,0 ... .-,0 ]
    tokens = tokenize(text)  #"free coupon" -> ["free", "coupon"]
    tokens = tokens[:max_len]  # if len(tokens) < 50 -> It is not error.
    encoded = []
    for t in tokens:
        # word2idx 사전에 단어 't'가 있으면 그 번호를 가져오고,
        # 사전에 없는 모르는 단어라면 1번(<unk>)을 가져옵니다.
        if t in word2idx:
            num = word2idx[t]
        else:
            num = 1 # 1번은 사전에 없는 단어를 뜻하는 <unk> 방 번호입니다. 
        encoded.append(num)
    # 4단계: 문장의 빈칸을 0으로 채워 길이를 딱 50으로 맞춥니다. (패딩)
    # 현재 숫자로 바뀐 문장의 길이(len(encoded))를 확인합니다.
    current_len = len(encoded)
    # 목표 길이(50)에서 현재 길이를 빼면 채워야 할 0의 개수가 나옵니다.
    needed_zeros = max_len - current_len
    # 0이 가득 담긴 패딩 리스트를 만듭니다. (예: [0, 0, 0...])
    padding_list = [0] * needed_zeros
    # 원래 숫자로 바뀐 문장 뒤에 0번(<pad>)을 이어 붙입니다.
    final_output = encoded + padding_list
    return final_output

import torch
from torch.utils.data import Dataset

# 파이토치가 제공하는 기본 Dataset의 능력을 물려받아(상속) 클래스를 만듭니다.
class SpamDataset(Dataset): 
    # [1단계: 생성자] 데이터셋을 처음 만들 때 실행되는 초기화 함수입니다.
    # 텍스트 리스트(texts)와 정답 리스트(labels)를 재료로 받습니다.
    def __init__(self, texts, labels):
        
        # 1-1. 텍스트 데이터를 숫자로 바꾸고, 파이토치 텐서(Tensor)로 변환합니다.
        self.X = []
        for t in texts:
            # 우리가 앞서 만든 encode() 함수로 문장을 50개의 숫자로 바꿉니다.
            encoded_list = encode(t) 
            # 모델 연산이 가능하도록 파이토치 숫자형 데이터(Tensor)로 감싸줍니다.
            tensor_data = torch.tensor(encoded_list) 
            self.X.append(tensor_data)
            
        # 1-2. 정답 레이블(0 또는 1)도 판다스에서 꺼내와 통째로 하나의 거대한 텐서로 만듭니다.
        # labels.values는 판다스 시리즈를 넘파이(Numpy) 배열로 바꾼 뒤 텐서로 변환하는 표현입니다.
        self.y = torch.tensor(labels.values)

    # [2단계: 길이 반환] 이 데이터셋에 총 몇 개의 메일이 들어있는지 알려주는 함수입니다.
    # len(dataset)을 실행하면 이 함수가 호출됩니다.
    def __len__(self): 
        total_count = len(self.X)
        return total_count

    # [3단계: 데이터 꺼내기] "idx번째 데이터 세트(텍스트, 정답) 내놔!" 할 때 응답하는 함수입니다.
    # dataset[idx]를 실행하면 이 함수가 호출되어 'idx번째 메일 숫자 리스트'와 'idx번째 정답'을 쌍으로 묶어 줍니다.
    def __getitem__(self, idx): 
        sample_X = self.X[idx]  # idx번째 인코딩된 텍스트 텐서
        sample_y = self.y[idx]  # idx번째 정답(0 또는 1) 텐서
        
        return sample_X, sample_y
    
class LSTMClassifier(nn.Module):
    def __init__(self, vocab_size, embed_dim, hidden_dim):
        super().__init__()
        self.embedding = nn.Embedding(vocab_size, embed_dim, padding_idx=0)
        self.lstm = nn.LSTM(embed_dim, hidden_dim, batch_first=True)
        self.fc = nn.Linear(hidden_dim, 1)
        self.sigmoid = nn.Sigmoid()

    def forward(self, x):
        #print("10", x.shape)
        embedded = self.embedding(x)
        #print("11", embedded.shape)
        _, (hidden, _) = self.lstm(embedded)
        #print("13", hidden.shape)
        return self.sigmoid(self.fc(hidden[-1])).squeeze()
    
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print("연산장치: ", device)

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
