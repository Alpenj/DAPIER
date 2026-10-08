"""호환 진입점: sweepick.manipulation.sweepick_right_receive_planner. 제어 구현은 새 경로 한 곳에 있다."""
import sys
from sweepick.manipulation import sweepick_right_receive_planner as _implementation
sys.modules[__name__] = _implementation
