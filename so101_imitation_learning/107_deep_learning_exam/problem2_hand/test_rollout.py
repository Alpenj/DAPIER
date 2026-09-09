"""CPU 수학 검사: python test_rollout.py. 이미지·GPU·장치·체크포인트 불필요."""
import torch
from torch import nn

import evaluate_rollout as app


class Commands(nn.Module):
    def __init__(self):
        super().__init__()
        self.observed = []

    def forward(self, images, joints, actions=None, sample=False):
        assert actions is None and not sample
        self.observed.append(joints.clone())
        commands = torch.full((4, 8, 10), .8)
        commands[:, 4:] = 1.4  # 뒤 4개를 잘못 실행하면 최종값이 달라진다.
        logits = torch.tensor([[0., 10., 0., 0.], [0., 0., 0., 10.], [0., 0., 0., 0.], [10., 0., 0., 0.]])
        return commands, None, None, logits


def main():
    torch.set_num_threads(2)
    model = Commands()
    result = app.rollout(model, torch.zeros(4, 3, 64, 64))
    trajectory = result["trajectory"]
    assert trajectory.shape == (4, 65, 10) and len(model.observed) == 16
    expected = torch.tensor([.8*(1-.65**t) for t in range(65)])
    torch.testing.assert_close(trajectory[0, :, 0], expected, atol=1e-6, rtol=1e-6)
    torch.testing.assert_close(trajectory[3], trajectory[0])  # 다른 분류여도 raw 명령은 대체하지 않는다.
    assert torch.count_nonzero(trajectory[1:3]) == 0  # none와 저신뢰도는 hold.
    assert torch.equal(result["response_fraction"], torch.tensor([1., 0., 0., 1.]))
    for chunk, observed in enumerate(model.observed):
        torch.testing.assert_close(observed, trajectory[:, chunk*4])
    row = {"correct": True, "response_fraction": 1., "initial_joint_rmse_rad": 1., "final_joint_rmse_rad": .1}
    metrics = app.summarize([row, {**row, "correct": False, "response_fraction": 0., "final_joint_rmse_rad": .3}])
    assert metrics["images"] == 2 and metrics["raw_vit_accuracy"] == .5 and metrics["response_fraction"] == .5
    assert metrics["within_criterion_fraction"] == .5 and metrics["initial_within_criterion_fraction"] == 0
    assert abs(metrics["mean_final_joint_rmse_rad"]-.2) < 1e-9
    print("PASS: 16 observations/64 ticks, first4 commands only, current-q feedback, prior-only, none/low-confidence hold, raw actions, aggregate criteria")


if __name__ == "__main__":
    main()
