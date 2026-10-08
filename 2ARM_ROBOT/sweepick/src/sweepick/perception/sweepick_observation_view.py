"""Read-only viewer of a running PICK session: a local web page that shows the newest images the session itself has
saved (left wrist frames recorded while a stage moves, and the top / wrist / overlay images of the last look).
It opens NO camera and NO motor port, so it cannot disturb the session. The top camera is only read at each look, so its
picture changes between stages, not during a move.   usage: python -m sweepick.perception.sweepick_observation_view SESSION_DIR [PORT]   then open http://127.0.0.1:PORT"""
import http.server, json, sys, time
from pathlib import Path

ROOT = Path(sys.argv[1]).expanduser(); PORT = int(sys.argv[2]) if len(sys.argv) > 2 else 8765
PAGE = """<!doctype html><meta charset=utf-8><title>PICK viewer</title><style>body{background:#111;color:#ddd;font:14px sans-serif;margin:12px}img{max-width:100%;background:#000}
.g{display:grid;grid-template-columns:1fr 1fr;gap:12px}h3{margin:6px 0;font-weight:500}small{color:#999}pre{white-space:pre-wrap;font:12px monospace;color:#bbb}</style>
<div class=g><div><h3>왼 손목 (가장 최근 저장 프레임) <small id=wn></small></h3><img id=w></div><div><h3>상단 카메라 (마지막 관측, 모델 겹침) <small id=tn></small></h3><img id=t></div></div>
<h3>세션 진행</h3><pre id=s></pre>
<script>async function tick(){try{const r=await (await fetch('/state?'+Date.now())).json();
if(r.wrist){w.src='/img?p='+encodeURIComponent(r.wrist)+'&'+r.wrist_m;wn.textContent=r.wrist_label}
if(r.top){t.src='/img?p='+encodeURIComponent(r.top)+'&'+r.top_m;tn.textContent=r.top_label}
s.textContent=r.trail}catch(e){}setTimeout(tick,300)}tick()</script>"""


def newest(pattern):
    best = None
    for p in ROOT.glob(pattern):
        try:
            m = p.stat().st_mtime
        except OSError:
            continue
        if best is None or m > best[1]:
            best = (p, m)
    return best


def state():
    out = dict(trail="")
    cands = [x for x in (newest("*/frames/*.jpg"), newest("looks/*/left_wrist.jpg")) if x]
    if cands:
        p, m = max(cands, key=lambda x: x[1]); out.update(wrist=str(p.relative_to(ROOT)), wrist_m=m, wrist_label=f"{p.parent.parent.name if p.parent.name == 'frames' else p.parent.name} · {time.strftime('%H:%M:%S', time.localtime(m))}")
    t = newest("looks/*/overlay.png") or newest("looks/*/top.png")
    if t:
        out.update(top=str(t[0].relative_to(ROOT)), top_m=t[1], top_label=f"{t[0].parent.name} · {time.strftime('%H:%M:%S', time.localtime(t[1]))}")
    try:
        tr = json.loads((ROOT / "SESSION.json").read_text())["trail"]
        out["trail"] = "\n".join(" ".join(f"{k}={v}" for k, v in r.items() if k in ("step", "gate", "ok", "result", "controller_result", "height", "k", "offset_mm", "allowed_mm", "why", "end", "placement", "label") and v is not None)[:230] for r in tr[-14:])
    except Exception:
        pass
    return out


class H(http.server.BaseHTTPRequestHandler):
    def log_message(self, *a): pass
    def do_GET(self):
        try:
            if self.path.startswith("/state"):
                b = json.dumps(state()).encode(); ct = "application/json"
            elif self.path.startswith("/img"):
                rel = self.path.split("p=", 1)[1].split("&", 1)[0]
                from urllib.parse import unquote
                p = (ROOT / unquote(rel)).resolve()
                if ROOT.resolve() not in p.parents:
                    raise FileNotFoundError
                b = p.read_bytes(); ct = "image/png" if p.suffix == ".png" else "image/jpeg"
            else:
                b = PAGE.encode(); ct = "text/html; charset=utf-8"
            self.send_response(200); self.send_header("Content-Type", ct); self.send_header("Cache-Control", "no-store"); self.end_headers(); self.wfile.write(b)
        except Exception:
            self.send_response(404); self.end_headers()


if __name__ == "__main__":
    http.server.ThreadingHTTPServer(("127.0.0.1", PORT), H).serve_forever()
