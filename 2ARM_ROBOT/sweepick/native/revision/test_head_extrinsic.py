"""The measured head-camera pose is used when present, refused when malformed, and the scene camera remains the fallback."""
import json
import numpy as np
import pytest
import sweepick_setup as setup


def test_measured_file_is_rigid_and_used():
    T, doc = setup.measured_head_extrinsic()
    assert doc["status"].startswith("MEASURED")
    assert np.allclose(setup.mounted_robot_from_camera(), T)
    assert setup.mounted_camera_source()["source"] == "MEASURED"


def test_malformed_file_is_an_error(tmp_path, monkeypatch):
    bad = tmp_path / "x.json"; bad.write_text(json.dumps(dict(schema="sweepick.head-extrinsic.v1", T_robot_from_camera=(2 * np.eye(4)).tolist())))
    monkeypatch.setattr(setup, "MEASURED_HEAD_EXTRINSIC", bad)
    with pytest.raises(RuntimeError):
        setup.mounted_robot_from_camera()


def test_absent_file_falls_back_to_scene_camera(tmp_path, monkeypatch):
    monkeypatch.setattr(setup, "MEASURED_HEAD_EXTRINSIC", tmp_path / "none.json")
    assert setup.mounted_camera_source()["source"] == "SCENE_CAMERA"
    assert np.allclose(setup.mounted_robot_from_camera(), setup.scene_robot_from_camera())
