"""호환 진입점: sweepick.perception.sweepick_feedback_observation. 제어 구현은 새 경로 한 곳에 있다."""
import sys
from sweepick.perception import sweepick_feedback_observation as _implementation
if __name__ == "__main__":
    raise SystemExit(_implementation.main())
else:
    sys.modules[__name__] = _implementation
