import torch
import torch.nn as nn

# 1. 하이퍼파라미터 정의 (이해하기 쉽게 미니 규격으로 설정)
BATCH_SIZE = 2      # 이미지 2장 묶음
NUM_PATCHES = 196   # 이미지 조각 개수 (14x14)
d_model = 192       # 트랜스포머 내부 벡터 차원

print("=== [준비 단계: 부품 정의] ===")

# 부품 2: CLS 토큰 (모양: [1, 1, 192])
cls_token = nn.Parameter(torch.zeros(1, 1, d_model))
print(f"-> 생성된 CLS 토큰의 크기 : {cls_token.shape}")

# 부품 3: 위치 임베딩 (총 196개 패치 + CLS 토큰 1개 = 197개의 번호표)
# 모양: [1, 197, 192]
pos_embed = nn.Parameter(torch.randn(1, NUM_PATCHES + 1, d_model))
print(f"-> 생성된 위치 임베딩의 크기: {pos_embed.shape}")

# 부품 4: 순정 파이토치 트랜스포머 인코더 블록 (단 1개 층만 테스트)
encoder_layer = nn.TransformerEncoderLayer(d_model=d_model, nhead=3, batch_first=True)
transformer_encoder = nn.TransformerEncoder(encoder_layer, num_layers=1)
print("-> 트랜스포머 인코더 블록 준비 완료\n")


print("=== [forward 연산 시뮬레이션] ===")

# [가상 데이터] 패치 임베딩이 끝난 상태의 이미지 텐서 생성
# 모양: [Batch=2, 단어수=196, 차원=192]
x = torch.randn(BATCH_SIZE, NUM_PATCHES, d_model)
print(f"1. 인코더 입구에 도착한 순수 이미지 패치 크기: {x.shape}")

# 단계 A: 배치 사이즈(2)에 맞게 CLS 토큰 복사하기
# [1, 1, 192] -> [2, 1, 192]
cls_tokens = cls_token.expand(x.size(0), -1, -1)
print(f"2. 배치 크기만큼 복사된 CLS 토큰 크기      : {cls_tokens.shape}")

# 단계 B: torch.cat을 이용해 이미지 패치 '맨 앞(dim=1)'에 CLS 토큰 붙이기
# [2, 1, 192] 와 [2, 196, 192] 가 합쳐져서 단어가 197개가 됨!
x = torch.cat((cls_tokens, x), dim=1)
print(f"3. CLS 토큰이 결합된 텐서 크기 (단어수 197) : {x.shape}")

# 단계 C: 위치 정보(번호표) 행렬 그대로 더하기 (+)
# [2, 197, 192] + [1, 197, 192] -> 브로드캐스팅에 의해 배치 전체에 번호표가 똑같이 더해짐
x = x + pos_embed
print(f"4. 위치 임베딩(번호표)이 더해진 텐서 크기    : {x.shape}")

# 단계 D: 트랜스포머 인코더 방에 집어넣기
x = transformer_encoder(x)
print(f"5. 트랜스포머 인코더를 통과한 최종 텐서 크기: {x.shape}")

# 단계 E: 다른 조각들은 다 버리고 오직 맨 앞(0번 인덱스)의 CLS 토큰만 쏙 뽑아내기
# x[:, 0] 은 [Batch, 197, 192] 에서 모든 배치의 0번째 단어만 가져오라는 뜻
cls_output = x[:, 0]
print(f"6. 최종 분류를 위해 추출된 반장(CLS) 크기   : {cls_output.shape}")