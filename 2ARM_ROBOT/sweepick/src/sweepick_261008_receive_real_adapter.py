"""호환 진입점: sweepick.integration.sweepick_handoff_snapshot_adapter. 제어 구현은 새 경로 한 곳에 있다."""
import sys
from sweepick.integration import sweepick_handoff_snapshot_adapter as _implementation
if __name__ == "__main__":
    raise SystemExit(_implementation.main())
else:
    sys.modules[__name__] = _implementation
