"""block_sized_patch: the block's top face is chosen among the block-height points; strips (wall foot) are refused (2026-10-18)."""
import numpy as np
import sweepick_pick01 as pk


def _square(cx, cy, side, n=900, z=0.04, seed=0):
    r = np.random.default_rng(seed); xy = r.uniform(-side / 2, side / 2, (n, 2)) + [cx, cy]
    return np.c_[xy, np.full(n, z)]


def test_wall_strip_with_more_points_does_not_win():
    block = _square(0.20, -0.07, 0.04, 700)
    r = np.random.default_rng(1); strip = np.c_[r.uniform(0.37, 0.40, 6000), r.uniform(-0.35, 0.35, 6000), np.full(6000, 0.04)]
    chosen, cands = pk.block_sized_patch(np.vstack([strip, block]), 0.02)
    assert chosen is not None and np.allclose(np.median(chosen[:, :2], 0), [0.20, -0.07], atol=0.003)
    assert any(c["verdict"].startswith("longer") for c in cands)


def test_only_a_strip_gives_no_block():
    r = np.random.default_rng(2); strip = np.c_[r.uniform(0.37, 0.40, 3000), r.uniform(-0.3, 0.3, 3000), np.full(3000, 0.04)]
    chosen, cands = pk.block_sized_patch(strip, 0.02)
    assert chosen is None and cands and not any(c["verdict"] == "BLOCK_SIZED" for c in cands)


def test_narrow_object_is_not_a_block():
    r = np.random.default_rng(3); thin = np.c_[r.uniform(0.14, 0.17, 400), r.uniform(-0.37, -0.36, 400), np.full(400, 0.04)]
    chosen, cands = pk.block_sized_patch(np.vstack([thin, _square(0.2, 0.1, 0.04, 600)]), 0.02)
    assert np.allclose(np.median(chosen[:, :2], 0), [0.2, 0.1], atol=0.003)
    assert any(c["verdict"].startswith("narrower") for c in cands)


def test_too_few_points():
    assert pk.block_sized_patch(_square(0.2, 0, 0.04, 20), 0.02) == (None, [])
