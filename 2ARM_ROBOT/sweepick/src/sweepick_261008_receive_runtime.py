"""호환 진입점: sweepick.integration.sweepick_receive_runtime. 제어 구현은 새 경로 한 곳에 있다."""
import sys
from sweepick.integration import sweepick_receive_runtime as _implementation
sys.modules[__name__] = _implementation
