"""호환 진입점: sweepick.integration.sweepick_manipulation_session. 제어 구현은 새 경로 한 곳에 있다."""
import sys
from sweepick.integration import sweepick_manipulation_session as _implementation
if __name__ == "__main__":
    raise SystemExit(_implementation.main())
else:
    sys.modules[__name__] = _implementation
