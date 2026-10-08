"""호환 진입점: sweepick.integration.sweepick_manipulation_handoff. 제어 구현은 새 경로 한 곳에 있다."""
import sys
from sweepick.integration import sweepick_manipulation_handoff as _implementation
sys.modules[__name__] = _implementation
