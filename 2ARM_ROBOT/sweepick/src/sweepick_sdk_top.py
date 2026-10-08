"""호환 진입점: sweepick.perception.sweepick_os30a_observation. 제어 구현은 새 경로 한 곳에 있다."""
import sys
from sweepick.perception import sweepick_os30a_observation as _implementation
sys.modules[__name__] = _implementation
