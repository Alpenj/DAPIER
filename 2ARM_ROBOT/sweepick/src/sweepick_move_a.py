"""호환 진입점: sweepick.control.sweepick_trajectory_executor. 제어 구현은 새 경로 한 곳에 있다."""
import sys
from sweepick.control import sweepick_trajectory_executor as _implementation
if __name__ == "__main__":
    raise SystemExit(_implementation.main())
else:
    sys.modules[__name__] = _implementation
