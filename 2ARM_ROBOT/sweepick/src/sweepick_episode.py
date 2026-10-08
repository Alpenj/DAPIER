"""호환 진입점: sweepick.recording.sweepick_episode_recorder. 제어 구현은 새 경로 한 곳에 있다."""
import sys
from sweepick.recording import sweepick_episode_recorder as _implementation
sys.modules[__name__] = _implementation
