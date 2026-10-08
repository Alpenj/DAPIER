"""호환 진입점: sweepick.control.sweepick_servo_access. 제어 구현은 새 경로 한 곳에 있다."""
import sys
from sweepick.control import sweepick_servo_access as _implementation
sys.modules[__name__] = _implementation
