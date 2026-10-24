from __future__ import annotations

import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np


PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

from compact_mobile_dual_so101 import (  # noqa: E402
    COMPACT_HOME_ACTION,
    build_compact_mobile_model,
    create_compact_mobile_data,
)
from compact_teleop_episode import record_compact_pregrasp_episode  # noqa: E402
from mobile_dual_so101 import resolve_so101_model  # noqa: E402
from vision_box_pregrasp import (  # noqa: E402
    calibration_from_mujoco,
    plan_right_pregrasp_from_depth,
    render_metric_depth,
)


try:
    ASSET = resolve_so101_model()
except FileNotFoundError:
    ASSET = None


@unittest.skipUnless(ASSET is not None, "SO-101 MuJoCo asset is unavailable")
class CompactTeleopEpisodeTest(unittest.TestCase):
    def test_stable_pregrasp_replay_is_accepted_for_existing_dataset(self) -> None:
        assert ASSET is not None
        model, _ = build_compact_mobile_model(model_path=ASSET)
        data = create_compact_mobile_data(model)
        calibration = calibration_from_mujoco(
            model, data, "workspace_depth_camera", width=640, height=460
        )
        plan = plan_right_pregrasp_from_depth(
            model,
            render_metric_depth(
                model, data, "workspace_depth_camera", width=640, height=460
            ),
            calibration,
        )
        actions = []
        start = np.asarray(COMPACT_HOME_ACTION)
        for raw_goal in plan.actions_rad:
            goal = np.asarray(raw_goal)
            steps = max(1, math.ceil(float(np.max(np.abs(goal - start))) / 0.30))
            actions.extend(
                tuple(start + fraction * (goal - start))
                for fraction in np.linspace(1 / steps, 1.0, steps)
            )
            start = goal
        actions.extend([tuple(start)] * 20)

        with tempfile.TemporaryDirectory() as temp_dir:
            manifest_path = record_compact_pregrasp_episode(
                Path(temp_dir) / "episode",
                episode_id="DAPIER-2026-09-04-compact-teleop-test",
                model=model,
                actions_rad=actions,
                target_base_m=plan.targets_base_m[-1],
                width=16,
                height=12,
            )
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            self.assertEqual(manifest["outcome"], {
                "status": "accepted",
                "success": True,
                "failure_reason": None,
            })
            self.assertEqual(
                manifest["task"]["skill"], "right_front_flap_pregrasp"
            )
            self.assertLess(
                manifest["simulation_validation"]["terminal_distance_m"], 0.015
            )
            self.assertFalse(
                manifest["simulation_validation"]["hardware_execution"]
            )


if __name__ == "__main__":
    unittest.main()
