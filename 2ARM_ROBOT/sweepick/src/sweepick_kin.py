"""호환 진입점: sweepick.control.sweepick_joint_kinematics. 제어 구현은 새 경로 한 곳에 있다."""
import sys
from sweepick.control import sweepick_joint_kinematics as _implementation
sys.modules[__name__] = _implementation
