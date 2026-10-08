"""호환 진입점: sweepick.perception.sweepick_wrist_capture. 제어 구현은 새 경로 한 곳에 있다."""
import sys
from sweepick.perception import sweepick_wrist_capture as _implementation
if __name__ == "__main__":
    raise SystemExit(_implementation.main())
else:
    sys.modules[__name__] = _implementation
