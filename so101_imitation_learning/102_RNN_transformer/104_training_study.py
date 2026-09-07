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
# vocab은 단어사전이다. 아래 함수는 단어사전을 만들기 위해서 
# 데이터셋의 입력인 문장들을 입력받는다. 
def build_vocab(sentences):
    # [1단계] 기본 특수 토큰 4개를 방 번호와 함께 미리 채워둡니다.
    # 번역기(Seq2Seq) 모델의 규칙을 위해 인위적으로 약속해둔 기호들입니다.
    vocab = {
        "<PAD>": 0,  # 문장 길이를 맞추기 위한 빈칸 채우기 기호
        "<SOS>": 1,  # Start Of Sequence: "이제 문장 시작한다!" 알림 기호
        "<EOS>": 2,  # End Of Sequence: "여기서 문장 끝났다!" 알림 기호
        "<UNK>": 3   # Unknown: 사전에 없는 낯선 단어 방어용 기호
    }
    # [2단계] 입력받은 모든 문장들을 하나씩 꺼내어 반복합니다.
    for sent in sentences:   
        # [3단계] 하나의 문장을 띄어쓰기 기준으로 단어별로 쪼갭니다.
        # 예: "claim now" -> ["claim", "now"]
        words = sent.split()
        # [4단계] 쪼개진 단어들을 하나씩 순서대로 검사합니다.
        for word in words:
            # [5단계] (핵심) 만약 이 단어가 아직 사전에 등록되지 않았다면?
            if word not in vocab:
                # [6단계] 현재 사전의 총 길이(len(vocab))를 새 단어의 방 번호로 지정합니다.
                # 예: 처음에 특수 토큰 4개가 있으므로 len(vocab)은 4입니다. 
                # 따라서 첫 번째 새 단어의 방 번호는 자동으로 '4'가 됩니다.
                vocab[word] = len(vocab)
    # [7단계] 모든 문장 순회가 끝나면 완성된 단어 사전을 반환합니다.
    return vocab

# [1단계] 한국어 문장들만 따로 모아둘 빈 바구니(리스트)를 만듭니다.
korean_sentences = []
# [2단계] 전체 data 리스트에서 (한국어, 영어) 쌍을 하나씩 꺼내며 반복합니다.
for d in data:
    # d는 ("나 가고 싶어", "i want to go") 같은 튜플 형태입니다.
    # d[0]은 앞쪽의 '한국어 문장'이고, d[1]은 뒤쪽의 '영어 문장'이 됩니다.
    ko_sent = d[0] 
    #print(ko_sent)
    # [3단계] 골라낸 한국어 문장을 바구니에 차곡차곡 담습니다.
    korean_sentences.append(ko_sent)

# [확인] 여기까지 실행되면 korean_sentences는 아래와 같이 변합니다.
# ['나 가고 싶어', '그거 좋아', '나 행복해', '너는 학생이야', '이거 뭐야']
#print(korean_sentences)
# [4단계] 한국어 문장 목록만 담긴 바구니를 build_vocab 함수에 통째로 넘겨줍니다.
ko_vocab = build_vocab(korean_sentences)
#print(ko_vocab)  # {'<PAD>': 0, '<SOS>': 1, '<EOS>': 2, '<UNK>': 3, '나': 4, '가고': 5, '싶어': 6, '그거': 7, '좋아': 8, '행복해': 9, '너는': 10, '학생이야': 11, '이거': 12, '뭐야': 13}

english_sentences = []
for d in data:
    en_sent = d[1]
    english_sentences.append(en_sent)

en_vocab = build_vocab(english_sentences)
#print(en_vocab)

en_idx2word = {}
for k, v in en_vocab.items():
    en_idx2word[v] = k
# 영어 꺼꾸로 vocab 만들기     
# en_idx2word {0: '<PAD>', 1: '<SOS>', 2: '<EOS>', 3: '<UNK>', 4: 'i', 5: 'want', 6: 'to', 7: 'go', 8: 'like', 9: 'it', 10: 'am', 11: 'happy', 12: 'you', 13: 'are', 14: 'a', 15: 'student', 16: 'what', 17: 'is', 18: 'this'}
#print('en_idx2word', en_idx2word)

# 2. 모델 정의 (인코더 & 디코더)
# 엔코더의 역할: 입력문장을 컨택스트 벡터로 만들기 
# input_dim: vocab의 길이 
# emb_dim: 토큰을 임베팅 벡터로 만들때 차원(길이)
# hid_dim: 컨텍스트 벡터의 차원(길이)
class Encoder(nn.Module):
    def __init__(self, input_dim, emb_dim, hid_dim):
        super().__init__()
        self.embedding = nn.Embedding(input_dim, emb_dim)
        self.rnn = nn.LSTM(emb_dim, hid_dim, batch_first=True)

    def forward(self, src):
        embed= self.embedding(src)
        #print('embed', embed)
        _, (hidden, cell) = self.rnn(embed)
        #print('hidden', hidden)
        #print('cell', cell)
        return hidden, cell  # hidden 이것이 바로 Context Vector! cell: LSTM의 장단기 기억 기록

# 디코더의 역할: 컨택스트 벡터를 문장으로 만들기  
# output_dim: 영어 vocab의 길이 
# emb_dim: 토큰을 임베팅 벡터로 만들때 차원(길이)
# hid_dim: 컨텍스트 벡터의 차원(길이)
class Decoder(nn.Module):
    def __init__(self, output_dim, emb_dim, hid_dim):
        super().__init__()
        self.embedding = nn.Embedding(output_dim, emb_dim)
        self.rnn = nn.LSTM(emb_dim, hid_dim, batch_first=True)
        self.fc_out = nn.Linear(hid_dim, output_dim)

    def forward(self, input, hidden, cell):
        input = input.unsqueeze(1) # [batch, 1]
        #print('input',  input)
        embed = self.embedding(input)
        #print('embed', embed)
        output, (hidden, cell) = self.rnn(embed, (hidden, cell))
        #print('output', output)
        #print('hidden', hidden)
        #print('cell', cell)
        prediction = self.fc_out(output.squeeze(1))
        #print('prediction', prediction)
        return prediction, hidden, cell
    
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
#print(device)
HID_DIM = 128 

#print(len(ko_vocab))  # 14 
#print(len(en_vocab))  # 19 
enc = Encoder(len(ko_vocab), 64, HID_DIM).to(device)
dec = Decoder(len(en_vocab), 64, HID_DIM).to(device)
optimizer = optim.Adam(list(enc.parameters()) + list(dec.parameters()), lr=0.001)
criterion = nn.CrossEntropyLoss(ignore_index=0)

for epoch in range(100):
    total_loss = 0
    for ko, en in data:  # 이 for문에서는 한번 루프에서는 하나의 한국어 영어 문장 세트를 받아서 처리함 
        src = None
        kor_token = []
        for w in ko.split(): # "이거 뭐야" -> tensor[12, 13]
            #print(w)
            #print(ko_vocab[w])
            kor_token.append(ko_vocab[w])
        src = torch.tensor([kor_token]).to(device)
        #print(src)
        '''
        이거
        12
        뭐야
        13
        tensor([[12, 13]])
        '''
        en_token = []
        for w in en.split(): # "what is this" -> tensor[16, 17, 18]
            #print(w)
            #print(en_vocab[w])
            en_token.append(en_vocab[w])
        trg = torch.tensor([en_token]).to(device)
        print(trg)
        '''
        what
        16
        is
        17
        this
        18
        tensor([[16, 17, 18]])
        '''
        optimizer.zero_grad()
        hidden, cell = enc(src) # 인코더가 tensor[12, 13]으로 hidden과 cell 생성  
        #print(hidden)

        input_token = torch.tensor([1]).to(device) # <SOS> 시작
        loss = 0
        #print('='*50)
        # 디코더는 단어를 하나 받아서 그 다음 단어를 생각해 내는 것이 목적이라, 단어 단위로 루프를 돌게됨 
        # 이에 비해서 엔코더는 이미 문장이 확정되어 있어서 pytorch 내부에서 자체적으로 단아별 루프를 돌린다. 
        for t in range(trg.size(1)):  # size() 결과값은 ( 배치사이즈, 문장길이) 즉 문장의 단어숫자만큼 루프를 돌게됨 
            # output: 예측된 단어 
            # input_token: 최초 시작할 때는 <SOS> 문자를 넣어줌 그 다음루프 부터는 그 다음 단어를 넣어줌 
            output, hidden, cell = dec(input_token, hidden, cell)
            #print('output', output)
            loss += criterion(output, trg[:, t])
            input_token = trg[:, t] # Teacher Forcing
            #print('input_tocken', input_token)
            #print(trg[:,t])

        loss.backward()
        optimizer.step()
        total_loss += loss.item()

# 학습된 상태 저장
torch.save({'enc': enc.state_dict(), 'dec': dec.state_dict(), 'ko': ko_vocab, 'en': en_vocab}, "seq2seq.pth")