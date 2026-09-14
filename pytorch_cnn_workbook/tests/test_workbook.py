from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pandas as pd
import torch

PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

from cifar10_common import (
    CIFARNet,
    denormalize,
    distinct_confident_errors,
    find_overfit_onset,
)


def test_model_contract_for_all_ablation_modes() -> None:
    sample = torch.zeros(2, 3, 32, 32)
    for use_bn, use_dropout in ((True, True), (False, True), (True, False)):
        model = CIFARNet(use_bn=use_bn, use_dropout=use_dropout)
        assert model(sample).shape == (2, 10)


def test_denormalize_restores_unit_interval() -> None:
    normalized = torch.tensor([[[-1.0, 0.0, 1.0]]] * 3)
    restored = denormalize(normalized)
    assert torch.allclose(restored[0, 0], torch.tensor([0.0, 0.5, 1.0]))


def test_overfit_onset_requires_two_rises_and_two_falls() -> None:
    frame = pd.DataFrame(
        {
            "epoch": [1, 2, 3],
            "train_loss": [1.0, 0.8, 0.7],
            "test_loss": [0.9, 1.0, 1.1],
        }
    )
    assert find_overfit_onset(frame) == 2


def test_confident_errors_use_distinct_class_pairs() -> None:
    true = np.array([0, 0, 1, 2, 3, 4])
    predicted = np.array([1, 1, 2, 3, 4, 5])
    confidence = np.array([0.99, 0.98, 0.97, 0.96, 0.95, 0.94])
    selected = distinct_confident_errors(true, predicted, confidence, count=5)
    pairs = {(int(true[index]), int(predicted[index])) for index in selected}
    assert len(selected) == 5
    assert len(pairs) == 5
