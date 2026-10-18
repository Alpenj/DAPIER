#!/usr/bin/env bash
# Desktop shortcut (2026-10-18): start the sweepick Task console if it is not listening on 8765, then open it in the browser.
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
if ! ss -ltn 2>/dev/null | grep -q '127.0.0.1:8765 '; then
    mkdir -p review
    setsid nohup bash -c './run.sh >> review/server.log 2>&1' >/dev/null 2>&1 &
    for i in $(seq 1 40); do ss -ltn 2>/dev/null | grep -q '127.0.0.1:8765 ' && break; sleep 0.25; done
fi
xdg-open http://127.0.0.1:8765/ >/dev/null 2>&1 &
