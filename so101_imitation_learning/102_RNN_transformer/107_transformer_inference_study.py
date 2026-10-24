    import torch
    import re
    import torch.nn as nn

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    class ChatbotVocab:
        def __init__(self):
            self.word2idx = {"<PAD>": 0, "<SOS>": 1, "<EOS>": 2, "<UNK>": 3}
            self.idx2word = {0: "<PAD>", 1: "<SOS>", 2: "<EOS>", 3: "<UNK>"}
            self.vocab_size = 4

        def tokenize(self, text):
            # "나는 밥을 먹는다" -> ["나는". "밥을", "먹는다"]
            text_filtered = re.sub(r"[^가-힣a-zA-Z0-9]", " ", text)
            text_list = text_filtered.split()
            return text_list
        
        def encode(self, text, add_special=True):
            # 1단계: 문장을 단어(토큰) 조각으로 쪼갭니다.
            # 예: "나 너 좋아해" -> ["나", "너", "좋아해"]
            token_words = self.tokenize(text)
            #print("encoder: ", token_words)
            # 숫자 번호표를 담을 빈 상자(리스트)를 준비합니다.
            tokens = []
            # 2단계: 쪼개진 단어들을 하나씩 꺼내며 숫자로 바꿉니다.
            for w in token_words:
                # 단어 사전(word2idx)에서 단어 'w'의 번호표를 찾습니다.
                # 만약 사전에 없는 처음 보는 단어라면, 기본값으로 '3'번(보통 <UNK>: 모르는 단어)을 부여합니다.
                num_code = self.word2idx.get(w, 3)
                tokens.append(num_code)
            # 3단계: 문장의 앞뒤에 시작(SOS)과 끝(EOS)을 알리는 특수 번호표를 붙일지 결정합니다.
            if add_special: 
                # [1]은 문장의 시작(<SOS>), [2]는 문장의 끝(<EOS>)을 의미하는 약속된 숫자입니다.
                return [1] + tokens + [2]
            # 특수 토큰이 필요 없다면 순수 단어 번호들만 반환합니다.
            #print("tokens: ", tokens)  # 숫자화된 단어의 어레이 " 나 나 좋아야" -> [ 1, 12, 4, 5]
            return tokens
        
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
            #print("src.shape", src.shape)
            #print("trg.shape", trg.shape)
            trg_mask = self.transformer.generate_square_subsequent_mask(trg.size(1)).to(src.device)
            #print("trg_mask", trg_mask)
            
            src_emb, trg_emb = self.embedding(src), self.embedding(trg)
            #print("src_emb", src_emb.shape)
            #print("trg_emb", trg_emb.shape)
            out = self.transformer(src_emb, trg_emb, tgt_mask=trg_mask)
            #print("out", out.shape)
            final_out = self.fc_out(out)
            #print("final_out",final_out.shape)
            return final_out

    # chatbot_model.pth에는 기억을 저장하는 가중치와 단어사전이 들어 있음     
    checkpoint = torch.load('chatbot_model.pth', map_location=device, weights_only=False)
    # checkpoint는 추론모델, vocab 등을 가진 딕셔너리 
    #print(checkpoint['vocab'].word2idx)
    vocab = checkpoint['vocab']
    # vocab은 ChatbotVocab 클래스의 객체임.트레이닝때 만들어져서 chatbot_model.pth에 저장됨 
    #print(vocab)

    # 단어 사전을 포함해서 트랜스포머 빈 모델을 만든다. 
    model = TransformerChatbot(vocab.vocab_size).to(device)
    # 미리 트레이닝 된 가중치를 모델에 올린다.
    model.load_state_dict(checkpoint['model_state'])
    model.eval()

    # 추론 루프 시작 
    while True:
        # 사용자 입력받기
        user_input = input("니: ")  # 입력이 "나는 간다."  
        if user_input == "종료":
            break
        with torch.no_grad():
            encoded_input = vocab.encode(user_input)  # [1, 1006, 916, 2] 1 은 시작 2는 종료 토큰 
            print('encoded_input', encoded_input)  
            src = torch.tensor([encoded_input]).to(device)
            print('src', src)# tensor([[   1, 1006,  916,    2]], device='cuda:0')
            trg_indices  = [1] # <SOS> 토큰으로 시작 

            for i in range(20):
                print('*'*40)
                trg_tensor = torch.tensor([trg_indices]).to(device)
                print('trg_tensor', trg_tensor)
                # 1턴 trg_tensor tensor([[1]], device='cuda:0')
                # 2턴 trg_tensor tensor([[    1, 13913]], device='cuda:0')
                output = model(src, trg_tensor)
                # 1턴  tensor([[[-11.5656, -10.6763,   0.8001,  ...,  -5.2394,  -1.9585,  -2.4117]]], device='cuda:0')
                # 2턴  tensor([[[-11.5656, -10.6763,   0.8001,  ...,  -5.2394,  -1.9585,  -2.4117],
                # [ -9.7418,  -9.8453,   8.3498,  ...,  -3.0570,  -7.6702,  -5.1986],
                # [-10.0402,  -9.7278,  31.3199,  ...,  -4.4234,  -3.7692,  -0.9261]]],device='cuda:0')
                print('output', output)
                # 1턴 output shape torch.Size([1, 1, 20651])
                # 2턴 output shape torch.Size([1, 2, 20651])
                print('output shape', output.shape)
                next_token = output.argmax(dim=-1)[:, -1].item()     
                print('next_token', next_token) # 1턴 13913 모자라지  2턴 10901 않아요
                trg_indices.append(next_token)
                print('trg_indices', trg_indices)
                if next_token == 2:
                    break
            
            response = [] 
            for i in  trg_indices[1:-1]:
                word = vocab.idx2word.get(i, "<UNK")
                print('word', word, i) # 1턴 모자라지 13813, 2턴 않아요 10901 
                response.append(word)
            print('response', " ".join(response))
            
