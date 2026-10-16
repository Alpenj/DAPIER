"""2026-10-16: a recorder arm-read failure keeps the exception chain (the OS error under lerobot's generic ConnectionError).
Synthetic readers and buses only; recording only - the drop, the next open and the gates are unchanged."""
import errno
import time

import test_sweepick_episode as TE


class FailingRight(TE.Bus):
    attempts = 0

    def __init__(self, side):
        if side == "right":
            FailingRight.attempts += 1
            try:
                raise OSError(errno.EBUSY, "Device or resource busy")
            except OSError as e:
                raise ConnectionError("\nCould not connect on port '/dev/dapier/right_arm'. Make sure you are using the correct port."
                                      "\nTry running `lerobot-find-port`\n") from e
        super().__init__(side)


def test_a_failed_recorder_open_keeps_the_os_error_under_the_connection_error(tmp_path):
    FailingRight.attempts = 0
    ep = TE.episode(tmp_path, open_arm=FailingRight)
    time.sleep(0.08)
    doc = ep.close()
    right = [e for e in doc["errors"] if e["what"].startswith("right arm read")]
    assert right, doc["errors"]
    first = right[0]
    assert first["what"].startswith("right arm read: ConnectionError:")                 # the existing text is kept as it was
    chain = first["exception"]["chain"]
    assert [c["type"] for c in chain[:2]] == ["ConnectionError", "OSError"]
    assert chain[1]["errno"] == errno.EBUSY and first["exception"]["errno"] == errno.EBUSY
    assert first["exception"]["port"] is None                                           # never assumed; it is in the message
    assert "recorder right arm read" == first["exception"]["stage"]
    assert any(r["side"] == "left" for r in TE.rows(tmp_path, "joints"))               # the other arm is still recorded


def test_an_error_without_a_cause_is_recorded_as_a_one_link_chain(tmp_path):
    class Lost(TE.Bus):
        def sync_read(self, name, normalize=False):
            if self.side == "right":
                raise RuntimeError("servo response error: ID=6")
            return super().sync_read(name, normalize)
    ep = TE.episode(tmp_path, open_arm=Lost)
    time.sleep(0.05)
    doc = ep.close()
    right = [e for e in doc["errors"] if e["what"].startswith("right arm read")]
    assert right and [c["type"] for c in right[0]["exception"]["chain"]] == ["RuntimeError"]
    assert right[0]["exception"]["errno"] is None
