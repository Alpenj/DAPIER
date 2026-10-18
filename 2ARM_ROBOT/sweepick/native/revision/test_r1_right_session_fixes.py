"""2026-10-18 R1 (right arm): the session path used LEFT names (pads, tool) and gave up when the block crossed the image border."""
import numpy as np
import pytest
import sweepick_pick01 as pk

REF = dict(readable=True, inner_px=[53, 270], midline_px=161.5, gap_px=217)


def test_pad_geoms_follow_the_session_arm(monkeypatch):
    monkeypatch.setattr(pk, "SIDE", "right")
    assert pk.pad_geoms() == ("right_pgripper_pad_1", "right_pgripper_pad_2")
    assert pk.pad_geoms("left") == pk.PAD_GEOMS


def test_block_cut_by_the_left_border_gives_a_capped_lower_bound_towards_it():
    obj = dict(visible=True, stable=True, edges_px=[0, 195], centre_px=97.5, width_px=195, mm_per_px=40 / 195, touches_border=True, border_side="left")
    off = pk.wrist_offset(obj, REF, gripper_tick=3017)
    assert off["readable"] and off["border_bound"] and off["offset_mm"] < 0 and abs(off["offset_mm"]) <= 15.0


def test_block_cut_on_the_midline_side_gives_no_direction():
    obj = dict(visible=True, stable=True, edges_px=[0, 300], centre_px=150 + 30, width_px=300, mm_per_px=40 / 300, touches_border=True, border_side="left")
    obj["centre_px"] = 170.0
    assert not pk.wrist_offset(obj, REF)["readable"]


def test_both_borders_or_no_reference_stay_unreadable():
    obj = dict(visible=True, stable=True, edges_px=[0, 319], centre_px=159.5, width_px=319, mm_per_px=40 / 319, touches_border=True, border_side="both")
    assert not pk.wrist_offset(obj, REF)["readable"]
    obj2 = dict(obj, border_side="left", edges_px=[0, 100], centre_px=50.0)
    assert not pk.wrist_offset(obj2, dict(readable=False))["readable"]


def test_a_fully_visible_block_is_measured_as_before():
    obj = dict(visible=True, stable=True, edges_px=[100, 240], centre_px=170.0, width_px=140, mm_per_px=40 / 140, touches_border=False, border_side=None)
    off = pk.wrist_offset(obj, REF, gripper_tick=3017)
    assert off["readable"] and not off.get("border_bound") and np.isclose(off["offset_px"], 8.5)


def _scene(row=10, border=False, confirmed=True, axis_deg=94.0):
    import math
    return dict(wrist=dict(object=dict(visible=True, stable=True, touches_border=border, row=row, far_edge_slope_deg=0.0 if row == 0 else 3.0),
                           offset=dict(readable=True, reference_confirmed_in_this_frame=confirmed)),
                jaw_axis_xy=[math.cos(math.radians(axis_deg)), math.sin(math.radians(axis_deg))])


PLAN = dict(tilt_deg=10.0, jaw="y", object_observed_yaw_rad=0.07)       # target jaw axis 94.0 deg


def test_tilted_grasp_uses_the_model_frame_yaw():
    al = pk.current_wrist_alignment(PLAN, _scene(row=0))
    assert al["ok"] and al["check"] == "MODEL_FRAME_YAW" and "unknown" not in al


def test_model_frame_yaw_out_of_tolerance_fails():
    al = pk.current_wrist_alignment(PLAN, _scene(row=0, axis_deg=99.0))
    assert al["ok"] is False and al["check"] == "MODEL_FRAME_YAW"


def test_block_not_confirmed_stays_unknown():
    assert "unknown" in pk.current_wrist_alignment(PLAN, _scene(row=0, confirmed=False))
    assert "unknown" in pk.current_wrist_alignment(PLAN, _scene(row=0, border=True))


def test_far_edge_on_the_top_border_is_not_an_image_slope():
    al = pk.current_wrist_alignment(dict(PLAN, tilt_deg=0.0), _scene(row=0))
    assert al.get("check") == "MODEL_FRAME_YAW"


def _rotated_square(angle_deg, centre=(160, 120), side=120, shape=(240, 320)):
    import cv2
    m = np.zeros(shape, np.uint8); box = cv2.boxPoints(((centre[0], centre[1]), (side, side), angle_deg)).astype(np.int32)
    cv2.fillPoly(m, [box], 1); return m.astype(bool)


@pytest.mark.parametrize("deg", [0.0, 7.0, -12.0, 25.0])
def test_block_edge_angle_reads_the_rotation(deg):
    a, L = pk.block_edge_angle(_rotated_square(deg))
    assert L > 300 and abs(((a - deg) + 45) % 90 - 45) < 1.5


def test_block_edge_angle_ignores_the_image_border():
    a, L = pk.block_edge_angle(_rotated_square(10.0, centre=(40, 120), side=150))      # cut by the left border
    assert a is not None and abs(a - 10.0) < 2.0


def test_alignment_uses_the_wrist_edge_angle_first():
    sc = _scene(row=10); sc["wrist"]["object"].update(edge_angle_deg=8.0, edge_angle_len_px=400.0)
    al = pk.current_wrist_alignment(dict(PLAN, tilt_deg=0.0), sc)
    assert al["check"] == "WRIST_EDGE_ANGLE" and al["ok"] is False
    sc["wrist"]["object"].update(edge_angle_deg=1.5)
    assert pk.current_wrist_alignment(dict(PLAN, tilt_deg=0.0), sc)["ok"] is True


def test_short_outline_falls_back():
    sc = _scene(row=0); sc["wrist"]["object"].update(edge_angle_deg=8.0, edge_angle_len_px=50.0)
    assert pk.current_wrist_alignment(PLAN, sc)["check"] == "MODEL_FRAME_YAW"
