"""MiniVGG 학습과 추론이 함께 사용하는 모델 및 전처리 정의."""

from pathlib import Path

import torch
from torch import nn
from torchvision import transforms

IMAGE_SIZE = 64
NORMALIZE_MEAN = (0.5, 0.5, 0.5)
NORMALIZE_STD = (0.5, 0.5, 0.5)
MODEL_VERSION = 3


class MiniVGGNet(nn.Module):
    """JD 원본과 계층·채널·연산 순서가 같은 MiniVGG."""

    def __init__(self, num_classes: int = 3) -> None:
        super().__init__()
        self.block1 = nn.Sequential(
            nn.Conv2d(3, 32, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.BatchNorm2d(32),
            nn.Conv2d(32, 32, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.BatchNorm2d(32),
            nn.MaxPool2d(kernel_size=2, stride=2),
            nn.Dropout(0.3),
        )
        self.block2 = nn.Sequential(
            nn.Conv2d(32, 64, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.BatchNorm2d(64),
            nn.Conv2d(64, 64, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.BatchNorm2d(64),
            nn.MaxPool2d(kernel_size=2, stride=2),
            nn.Dropout(0.4),
        )
        self.fc_layers = nn.Sequential(
            nn.Flatten(),
            nn.Linear(64 * 16 * 16, 512),
            nn.ReLU(),
            nn.BatchNorm1d(512),
            nn.Dropout(0.5),
            nn.Linear(512, num_classes),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x = self.block1(x)
        x = self.block2(x)
        return self.fc_layers(x)


def build_train_transform(image_size: int = IMAGE_SIZE) -> transforms.Compose:
    """원본 수를 늘리지 않고 작은 시점·조명 변화만 학습한다."""

    return transforms.Compose(
        [
            transforms.Resize((image_size + 8, image_size + 8)),
            transforms.RandomResizedCrop(
                image_size,
                scale=(0.85, 1.0),
                ratio=(0.9, 1.1),
            ),
            transforms.RandomHorizontalFlip(p=0.5),
            transforms.RandomRotation(8),
            transforms.ColorJitter(brightness=0.12, contrast=0.12, saturation=0.08),
            transforms.ToTensor(),
            transforms.Normalize(NORMALIZE_MEAN, NORMALIZE_STD),
        ]
    )


def build_eval_transform(image_size: int = IMAGE_SIZE) -> transforms.Compose:
    """validation과 live cam에 동일하게 사용하는 결정적 전처리."""

    return transforms.Compose(
        [
            transforms.Resize((image_size, image_size)),
            transforms.ToTensor(),
            transforms.Normalize(NORMALIZE_MEAN, NORMALIZE_STD),
        ]
    )


def load_stable_checkpoint(
    checkpoint_path: Path,
    device: torch.device,
) -> tuple[MiniVGGNet, list[str], dict]:
    """메타데이터가 포함된 안정화 체크포인트를 검증하고 로드한다."""

    checkpoint = torch.load(checkpoint_path, map_location=device, weights_only=False)
    if not isinstance(checkpoint, dict) or "state_dict" not in checkpoint:
        raise ValueError(
            "이 파일은 이전 형식의 가중치입니다. 먼저 안정화 학습 코드를 실행해 "
            "miniVGGnet_stable.pth를 생성하세요."
        )
    if checkpoint.get("model_version") != MODEL_VERSION:
        raise ValueError(
            f"지원하지 않는 모델 버전입니다: {checkpoint.get('model_version')}"
        )

    class_names = checkpoint.get("class_names")
    if not isinstance(class_names, list) or not class_names:
        raise ValueError("체크포인트에 class_names가 없습니다.")

    model = MiniVGGNet(num_classes=len(class_names)).to(device)
    model.load_state_dict(checkpoint["state_dict"])
    model.eval()
    return model, class_names, checkpoint
