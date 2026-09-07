import pandas as pd
import re 
from torch.utils.data import Dataset, DataLoader
import torch
import torch.nn as nn

class ChatbotVocab:
    def __init__(self):
        self.word2idx = {"<PAD>": 0, "<SOS>": 1, "<EOS>": 2, "<UNK>": 3}
        self.idx2word = {0: "<PAD>", 1: "<SOS>", 2: "<EOS>", 3: "<UNK>"}
        self.vocab_size = 4

    def tokenize(self, text):  #  "나는 배가 고프다 "
        text_re = re.sub(r"[^가-힣a-zA-Z0-9]", " ", text)
        #print(text_re)
        text_split = text_re.split()
        #print(text_split)
        return text_split  # ['나는', '배가', '고프다']
    
    # 단어사전 vocab을 만들기 
    def build_vocab(self, sentences):
        for sentence in sentences:
            #print(sentence)
            for word in self.tokenize(sentence):
                if word not in self.word2idx:
                    self.word2idx[word] = self.vocab_size # 딕셔너리 단어:인덱스번호
                    self.idx2word[self.vocab_size] = word # 딕셔너리 인덱스번호:단어
                    self.vocab_size += 1 

    def encode(self, text, add_special=True):
        # 1. 사람의 문장을 단어 단위로 쪼갭니다 (토큰화)
        # 예: "나 가고 싶어" -> ["나", "가고", "싶어"]
        tokens_list = self.tokenize(text)    
        
        # 2. 숫자로 변환된 단어 번호들을 담을 빈 바구니를 만듭니다.
        encoded_indices = []
        
        # 3. 쪼개진 단어들을 하나씩 꺼내며 숫자로 바꿉니다.
        for w in tokens_list: # ["나", "가고", "싶어"]
            # 단어 사전(word2idx)에 단어가 존재한다면 그 번호를 가져옵니다.
            if w in self.word2idx:
                word_num = self.word2idx[w]
            # (중요!) 사전에 없는 낯선 단어(신조어/오타 등)라면 무조건 3번 방으로 대체합니다.
            else:
                word_num = 3  # 3번은 미상 단어 토큰(<UNK>)의 번호입니다.
                
            # 변환된 번호를 바구니에 차곡차곡 추가합니다.
            encoded_indices.append(word_num)
            
        # 4. 특수 토큰(시작/끝 기호)을 붙여달라고 요청(add_special=True)했다면?
        if add_special:
            # 1번 토큰(<SOS>)을 맨 앞에, 2번 토큰(<EOS>)을 맨 뒤에 붙여서 반환합니다.
            return [1] + encoded_indices + [2]
        
        # 5. 특수 토큰이 필요 없다면 숫자로 변환된 배열만 그대로 반환합니다.
        return encoded_indices
            

df = pd.read_csv('ChatbotData.csv')
#print(df)

vocab = ChatbotVocab()
#vocab.tokenize("I am student. %$%^. 뭐라고 하는 거야@##@$#")

q_list = df['Q'].tolist()
#print(q_list)
a_list = df['A'].tolist()
#print(a_list)

# 단어사전 만들기 
# self.word2idx, self.idx2word에 저장됨 
vocab.build_vocab(q_list+a_list)

class ChatDataset(Dataset):
    def __init__(self, df, vocab):
        self.questions =[]
        for q in df['Q']:
            encoded_question = vocab.encode(q) # '저기' -> 62
            self.questions.append(encoded_question)
        #print('question ', self.questions)

        self.answers = []
        for a in df['A']:
            encoded_answer = vocab.encode(a) # '나는" -> 3 
            self.answers.append(encoded_answer)
        #print('answers ', self.answers)
    def __len__(self):
        return len(self.questions)
    def __getitem__(self, idx):
        return torch.tensor(self.questions[idx]), torch.tensor(self.answers[idx])
    

from torch.nn.utils.rnn import pad_sequence

def collate_fn(batch):
    # 1. 데이터로더가 낱개로 가져온 배치(batch) 상자에서 
    #    질문(qs) 바구니와 답변(ans) 바구니를 각각 따로 만듭니다.
    #print('collate ', batch)
    questions_list = []
    answers_list = []
    
    # 2. 묶여서 들어온 낱개 데이터들을 하나씩 꺼냅니다.
    # 예: batch = [(질문1, 답변1), (질문2, 답변2), ...]
    for q, a in batch:
        questions_list.append(q) # 질문은 질문 바구니에 쏙
        answers_list.append(a)   # 답변은 답변 바구니에 쏙
    #print('collate questions ', questions_list)
    # 3. (핵심!) 길이가 제각각인 질문 벡터들을 가장 긴 질문 길이에 맞춰 
    #    뒤에 0(패딩)을 채워 넣고 네모반듯한 하나의 덩어리(행렬 텐서)로 만듭니다.
    padded_qs = pad_sequence(questions_list, batch_first=True, padding_value=0)
    
    # 4. 답변 벡터들도 마찬가지로 가장 긴 답변 길이에 맞춰 0을 채우고 덩어리로 만듭니다.
    padded_ans = pad_sequence(answers_list, batch_first=True, padding_value=0)
    
    # 5. 최종적으로 예쁘게 규격이 맞춰진 질문 덩어리와 답변 덩어리를 반환합니다.
    #print('collate padding ', padded_qs, padded_ans)
    return padded_qs, padded_ans


import torch
import torch.nn as nn

class TransformerChatbot(nn.Module):
    def __init__(self, vocab_size, d_model=512, nhead=8, num_layers=6):
        super(TransformerChatbot, self).__init__()
        
        # 1. 단어 번호를 d_model(512) 크기의 vector로 변환하는 아파트
        self.embedding = nn.Embedding(vocab_size, d_model)
        
        # 2. 파이토치가 제공하는 기본 트랜스포머 모델 조립하기
        # 여기에 우리가 공부한 핵심 변수들이 고스란히 들어갑니다!
        self.transformer = nn.Transformer(
            d_model=d_model,              # 단어 표현 크기 (512)
            nhead=nhead,                  # 어텐션 전문가 수 (8)
            num_encoder_layers=num_layers, # 인코더 결재 라인 수 (6)
            num_decoder_layers=num_layers, # 디코더 결재 라인 수 (6)
            batch_first=True               # 데이터 차원을 (배치, 길이, d_model)로 고정
        )
        
        # 3. 최종 출력층: 예측된 d_model 벡터를 다시 단어 사전 크기(vocab_size)로 되돌림
        # d_model은 임베딩 벡터, vocal_size는 단어사전 softmax 같은 확률 크기로 정답을 표시 
        self.out = nn.Linear(d_model, vocab_size)

    def forward(self, src, tgt): # 토큰화된문장 X 배치사이즈가 입력 데이터
        '''
        tensor([[    1,  9060,  9061,  7008,  1021,  6790,  9062,  9063,     2],
        [    1,  6232,  4755,   848,     2,     0,     0,     0,     0],
        ...
        [    1,  5552,  5553,     2,     0,     0,     0,     0,     0]],
        device='cuda:0')
        '''
        #print("forward: ", src)
        # 사람이 준 질문(src)과 답변(tgt)을 d_model 크기의 숫자 세계로 변환
        src_emb = self.embedding(src)
        #  torch.Size([32, 9, 512]) 여기서 32 배치 9 문장길이 512 임베딩벡터 
        #print("forward: ", src_emb.shape)
        tgt_emb = self.embedding(tgt)
        #print("forward: ", tgt_emb.shape)
        
        # 트랜스포머 회사에 분석 요청하기
        # 인코더가 질문(src_emb)을 분석하고, 디코더가 답변(tgt_emb)을 참고해 연산합니다.
        output = self.transformer(src_emb, tgt_emb)  # output은 텐서 (batch size, 문장길이, 임베딩벡터차원)의 shape를 가짐  
        #print("forward: ",output.shape)  # torch.Size([32, 13, 512])
        # 최종 단어 번호표 형태로 출력
        # 32 x 13 x 512 텐서를 받아서 -> (아마 flattening이 있고) -> 32 x 13 x vocab_size 출력 
        # 이 vocab_size에는 모든 단어의 확률이 있고, 이 중에서 
        final_out = self.out(output)
        print("transformer forward:", final_out.shape)
        return final_out
    
# 학습 시작
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(device)
model = TransformerChatbot(vocab.vocab_size).to(device)
optimizer = torch.optim.Adam(model.parameters(), lr=0.0001)
criterion = nn.CrossEntropyLoss(ignore_index=0)
# ChatDataset 클래스는 객체가 만들어지면 다음 두 self 변수에 질문 및 답변 데이터를 보관함 
# self.questions self.answers 단어를 토큰 숫자로 변경한 어레이임 
# vocab은 단어사전엔데, sefl.word2idx self.idx2word 자체가 단어사전임 
loader = DataLoader(ChatDataset(df, vocab), batch_size=32, shuffle=True, collate_fn=collate_fn)
model.train()
print("Start training...")
for epoch in range(100):
    print("epoch start")
    for src, trg in loader:
        src, trg = src.to(device), trg.to(device)
        # src: 배치사이즈 x 토큰텐서어레이  e.g. 
        # src[0] -> tensor([   1, 9126,    2,    0,    0,    0,    0,    0,    0,    0,    0],device='cuda:0')
        #print("src: ", src.shape, src[0])  # src.shape -> torch.Size([32, 9])
        optimizer.zero_grad() # 기록지 지우기 [cite: 1557]
        # 트랜스포머에 입력하는 것은 다음 두가지 
        # 토큰화된 입력 문장: src
        # 토튼화된 출력 문장: trg
        output = model(src, trg[:, :-1]) # 문제 풀기 [cite: 1559]
        print("transformer output ", output.shape)
        # 트랜스포머는 이 두 입력을 어텐션 연산을 해서 출력을 내 놓는데 이것이 
        # 배치사이즈 x vocab_size의 텐서 ... 이 하나의 vocab_size 텐서에는 각 단어의 확률이 들어 있음 
        # 이 텐서를 flattening 한 후에 정래진 결과값 문장과 비교해서 accuracy 값을 얻는다 
        loss = criterion(output.reshape(-1, vocab.vocab_size), trg[:, 1:].reshape(-1)) # 채점 [cite: 1561]
        loss.backward() # 오답 분석 [cite: 1564]
        optimizer.step() # 수정 [cite: 1565]
    print(f"Epoch {epoch+1} Loss: {loss.item():.4f}")

    # 학습된 모델 저장 [cite: 1812]
torch.save({'model_state': model.state_dict(), 'vocab': vocab}, 'chatbot_model.pth')