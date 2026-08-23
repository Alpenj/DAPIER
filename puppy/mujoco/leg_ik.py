#!/usr/bin/env python3
"""Puppy 다리의 닫힌 형태 역기구학.

Puppy의 한 다리는 joint1, joint2가 **둘 다 같은 축(1 0 0)** 을 돌린다. 게다가
사이에 낀 고정 변환도 전부 x축 회전이라, 다리 전체가 하나의 평면(base_dummy
프레임의 y-z 평면, 즉 시상면)에 갇힌다. 따라서 각도를 훑을 필요 없이
**평면 2링크 역기구학**으로 한 번에 풀린다.

기하 정리
---------
링크1 원점에서 발까지의 벡터는

    p(θ1, θ2) = R(φ1 + θ1) · [ d2 + R(θ2) · e ]                     ... (1)

    d2 = 링크2의 부착 오프셋            (링크1 프레임, y-z 성분)
    e  = d3 + R(φ3) · d4                (링크2 프레임에서 본 발까지의 벡터)
    φ1 = 링크1 고정 회전, φ3 = 링크3 고정 회전
    R(a) = x축 둘레 회전, (y, z) -> (y cos a - z sin a, y sin a + z cos a)

식 (1)에서 바깥 회전 R(φ1+θ1)은 길이를 바꾸지 않으므로

    |p| = |d2 + R(θ2)·e|
        = sqrt( L1² + Le² + 2·L1·Le·cos(θ2 + δ) )                   ... (2)

즉 **제2 코사인 법칙**이다. L1=|d2|, Le=|e|, δ는 두 벡터의 기준 방향 차이다.

역기구학
--------
목표 발 위치 p*가 주어지면 r = |p*| 이므로 식 (2)를 θ2에 대해 바로 푼다.

    cos(θ2 + δ) = (r² - L1² - Le²) / (2·L1·Le)
    θ2 = ±acos(...) - δ                                             ... (3)

θ2가 정해지면 w = d2 + R(θ2)·e 가 확정되고, 식 (1)에서

    θ1 = atan2(p*) - atan2(w) - φ1                                  ... (4)

해가 두 개(θ2의 부호) 나오는데, 이 로봇은 무릎이 뒤로 굽는 형상이라
몸통 높이가 더 높은 쪽을 고른다.

파라미터(L1, Le, δ, φ1, φ3, 힙 위치)는 하드코딩하지 않고 컴파일된 MuJoCo
모델에서 읽는다. URDF가 바뀌면 자동으로 따라간다.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import mujoco
import numpy as np

LEGS = ("lf", "rf", "lb", "rb")


def _x_rotation_angle(quat: np.ndarray) -> float:
    """순수 x축 회전 쿼터니언 (w, x, 0, 0) 에서 회전각을 뽑는다."""
    w, x, y, z = quat
    if abs(y) > 1e-9 or abs(z) > 1e-9:
        raise ValueError(f"x축 회전이 아니다: {quat}")
    return 2.0 * math.atan2(x, w)


def _rot(a: float, v: np.ndarray) -> np.ndarray:
    """x축 둘레 회전을 (y, z) 2벡터에 적용."""
    c, s = math.cos(a), math.sin(a)
    return np.array([v[0] * c - v[1] * s, v[0] * s + v[1] * c])


@dataclass
class LegGeometry:
    """다리 하나의 평면 2링크 파라미터."""

    name: str
    hip: np.ndarray      # base_dummy 프레임에서 링크1 원점의 (y, z)
    hip_x: float         # 평면 밖 오프셋 (좌/우)
    phi1: float          # 링크1 고정 회전
    d2: np.ndarray       # 링크2 부착 오프셋 (y, z)
    e: np.ndarray        # 링크2 프레임에서 본 발까지의 벡터 (y, z)
    foot_radius: float   # 발 실린더 반지름

    @property
    def l1(self) -> float:
        return float(np.linalg.norm(self.d2))

    @property
    def le(self) -> float:
        return float(np.linalg.norm(self.e))

    @property
    def delta(self) -> float:
        """식 (2)의 δ = atan2(e) - atan2(d2) 에 해당하는 기준 방향 차이."""
        return math.atan2(self.e[1], self.e[0]) - math.atan2(self.d2[1], self.d2[0])

    @property
    def reach(self) -> tuple[float, float]:
        """도달 가능한 |p| 범위."""
        return abs(self.l1 - self.le), self.l1 + self.le

    def fk(self, th1: float, th2: float) -> np.ndarray:
        """식 (1). 링크1 원점 기준 발 중심 (y, z)."""
        w = self.d2 + _rot(th2, self.e)
        return _rot(self.phi1 + th1, w)

    def ik(self, target: np.ndarray, elbow: int = +1) -> tuple[float, float]:
        """식 (3)(4). 목표 발 위치 -> (θ1, θ2)."""
        r = float(np.linalg.norm(target))
        lo, hi = self.reach
        if not (lo - 1e-9 <= r <= hi + 1e-9):
            raise ValueError(
                f"{self.name}: 도달 불가. |p|={r:.5f}, 가능 범위 {lo:.5f}~{hi:.5f}")
        cos_arg = (r * r - self.l1**2 - self.le**2) / (2.0 * self.l1 * self.le)
        th2 = elbow * math.acos(np.clip(cos_arg, -1.0, 1.0)) - self.delta
        w = self.d2 + _rot(th2, self.e)
        th1 = math.atan2(target[1], target[0]) - math.atan2(w[1], w[0]) - self.phi1
        return _wrap(th1), _wrap(th2)


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def extract(model: mujoco.MjModel) -> dict[str, LegGeometry]:
    """컴파일된 모델에서 다리별 평면 파라미터를 읽어낸다."""
    legs = {}
    for leg in LEGS:
        b1 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{leg}_link1")
        b2 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{leg}_link2")
        b3 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{leg}_link3")

        phi1 = _x_rotation_angle(model.body_quat[b1])
        _x_rotation_angle(model.body_quat[b2])          # 항등이어야 한다
        phi3 = _x_rotation_angle(model.body_quat[b3])

        d2 = model.body_pos[b2][1:].copy()
        d3 = model.body_pos[b3][1:].copy()

        # link3 안의 발 실린더 위치
        gid = next(g for g in range(model.ngeom)
                   if model.geom_bodyid[g] == b3
                   and model.geom_type[g] == mujoco.mjtGeom.mjGEOM_CYLINDER)
        d4 = model.geom_pos[gid][1:].copy()
        foot_radius = float(model.geom_size[gid][0])

        legs[leg] = LegGeometry(
            name=leg,
            hip=model.body_pos[b1][1:].copy(),
            hip_x=float(model.body_pos[b1][0]),
            phi1=phi1,
            d2=d2,
            e=d3 + _rot(phi3, d4),
            foot_radius=foot_radius,
        )
    return legs


def stance_height_limits(legs: dict[str, LegGeometry],
                         foot_forward: float = 0.0) -> tuple[float, float]:
    """네 발을 같은 높이에 놓을 수 있는 몸통 높이의 해석적 범위.

    다리 하나가 낼 수 있는 |p|는 [|L1-Le|, L1+Le] 구간이다. 발을 힙 기준
    (dy, dz)에 놓으려면 dy는 힙의 전후 위치로 고정이므로, 세로 성분은

        |dz| = sqrt(r² - dy²),   r ∈ [|L1-Le|, L1+Le]

    가 된다. 여기에 힙의 z 오프셋을 더하면 그 다리가 가능한 몸통 높이 구간이
    나온다. 네 다리 구간의 교집합이 전체 가능 범위다.

    이 로봇은 뒷다리 힙이 5.337 mm 높아서 위쪽 한계를 뒷다리가 결정한다.
    """
    lo_all, hi_all = 0.0, math.inf
    for leg in legs.values():
        dy = foot_forward
        r_lo, r_hi = leg.reach
        if r_hi <= abs(dy):
            raise ValueError(f"{leg.name}: 전후 오프셋 {foot_forward}가 도달 범위 밖")
        hi = math.sqrt(r_hi**2 - dy**2) - leg.hip[1]
        lo = (math.sqrt(r_lo**2 - dy**2) if r_lo > abs(dy) else 0.0) - leg.hip[1]
        lo_all = max(lo_all, lo)
        hi_all = min(hi_all, hi)
    return lo_all, hi_all


def solve_stance(legs: dict[str, LegGeometry], height: float,
                 foot_forward: float = 0.0,
                 joint_range: tuple[float, float] | None = None
                 ) -> dict[str, tuple[float, float]]:
    """네 발을 같은 높이에 놓는 관절각을 푼다.

    height : base_dummy 원점에서 발 중심까지의 수직 거리
    foot_forward : **각 다리의 힙 바로 아래**를 원점으로 한 발의 전후 위치(+가 앞).
                   0이면 발이 자기 힙 바로 밑에 놓인다.
    joint_range : (lo, hi). 주어지면 이 범위를 벗어나는 해를 버린다.

    힙의 부착 높이가 앞뒤로 다르므로(이 로봇은 5.337 mm), 같은 관절각을 주면
    발 높이가 어긋난다. 목표를 '발 위치'로 주면 그 차이가 자동으로 흡수된다.

    2R 역기구학은 무릎이 앞으로 굽는 해와 뒤로 굽는 해 두 개를 준다.
    네 다리가 서로 다른 쪽으로 굽으면 안 되므로 **부호를 전체에서 하나로
    통일**한다.
    """
    def in_range(value: float) -> bool:
        if joint_range is None:
            return True
        return joint_range[0] <= value <= joint_range[1]

    for elbow in (+1, -1):
        attempt = {}
        for name, leg in legs.items():
            target = np.array([foot_forward, -height - leg.hip[1]])
            try:
                th1, th2 = leg.ik(target, elbow)
            except ValueError:
                break
            if np.linalg.norm(leg.fk(th1, th2) - target) > 1e-9:
                break
            if not (in_range(th1) and in_range(th2)):
                break
            attempt[name] = (th1, th2)
        else:
            return attempt

    raise ValueError(
        f"해가 없다 (height={height}, forward={foot_forward}, range={joint_range}). "
        "stance_height_limits로 도달 범위를 먼저 확인할 것.")
