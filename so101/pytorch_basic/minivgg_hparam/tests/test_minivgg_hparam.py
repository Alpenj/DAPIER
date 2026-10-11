from pathlib import Path

import torch

from minivgg_common import MiniVGGNet
from train_hparam import stratified_indices


def test_model_contract_matches_jd_original() -> None:
    model = MiniVGGNet(num_classes=3)
    assert sum(parameter.numel() for parameter in model.parameters()) == 8_457_635
    assert model.fc_layers[1].in_features == 64 * 16 * 16
    assert model.fc_layers[-1].out_features == 3
    assert model(torch.zeros(2, 3, 64, 64)).shape == (2, 3)


def test_stratified_split_is_disjoint_and_reproducible() -> None:
    targets = [0] * 119 + [1] * 64 + [2] * 107
    first_train, first_val = stratified_indices(targets, 0.20, 2026)
    second_train, second_val = stratified_indices(targets, 0.20, 2026)

    assert (first_train, first_val) == (second_train, second_val)
    assert len(first_train) == 232
    assert len(first_val) == 58
    assert set(first_train).isdisjoint(first_val)
    assert set(first_train) | set(first_val) == set(range(len(targets)))


def test_public_tree_does_not_include_training_images_or_checkpoint() -> None:
    project = Path(__file__).resolve().parents[1]
    assert not (project / "images").exists()
    assert not list(project.glob("*.pth"))
