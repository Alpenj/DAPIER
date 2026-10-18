"""2026-10-18 182311_auto: a cut block's outline angle is used only when the seen outline is square (edge coherence)."""
import cv2, numpy as np
import sweepick_pick01 as P


def _rect(angle, centre, size=(160, 160)):
    m = np.zeros((240, 320), np.uint8); box = cv2.boxPoints((centre, size, angle)).astype(np.int32); cv2.fillPoly(m, [box], 1); return m.astype(bool)


def test_turned_block_cut_at_two_borders_is_coherent():
    m = _rect(14.0, (230, 70))
    assert m[0].any() and m[:, -1].any()                                        # cut at the top and the right, like look 01_pregrasp_0
    assert P.edge_coherence(m) >= P.EDGE_COHERENCE_MIN
    a, _ = P.block_edge_angle(m)
    assert abs(abs(a) - 14.0) < 3.0

