import torch
import torch.nn as nn

# 1. 테스트용 하이퍼파라미터 정의
in_channels = 3    # 컬러 이미지 (RGB)
d_model = 192      # 트랜스포머 내부 벡터 차원
patch_size = 16    # 패치 크기 (16x16)

# 2. 검증하고 싶은 패치 임베딩 부품 하나만 단독으로 생성
patch_embed = nn.Conv2d(in_channels, d_model, kernel_size=patch_size, stride=patch_size)

# 3. [가상 데이터 생성] 가로세로 224인 컬러 이미지 1장이 배치로 들어왔다고 가정 (Batch=1)
# 모양: [Batch, Channel, Height, Width]
dummy_image = torch.randn(1, 3, 224, 224)
print(f"1. 처음 입력 이미지 텐서 크기: {dummy_image.shape}")

# 4. 부품에 통과시키기
output = patch_embed(dummy_image)
print(f"2. nn.Conv2d 통과 후 텐서 크기 : {output.shape}")

# 5. 이후 ViT forward() 함수에서 일어나는 과정 미리보기
# 2차원 격자판을 1차원으로 쫙 펴기 (Flatten)
flattened = output.flatten(2)
print(f"3. 2차원 평면 평탄화(Flatten) 후: {flattened.shape}")

# 축 바꾸기 (Transpose)
# 이것을 하는 이유가 Pytorch 트렌스포머가 텐서 시퀀스가 현재와 다름 
# Pytorch는 다음 시퀀스를 요구 [ batch_size, sequence_length, embedding_vector]
# 아래 코드를 처리하기 전에는 sequence_length와 embedding_vector의 순서가 바뀌어있음  
final_result = flattened.transpose(1, 2)
print(f"4. 최종 트랜스포머 입력 규격   : {final_result.shape}")