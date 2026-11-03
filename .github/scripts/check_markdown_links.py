"""Fail when a relative Markdown link or #anchor in this repository does not resolve.

Usage: python3 .github/scripts/check_markdown_links.py [repo_root]

Paths are matched with exact letter case, the way GitHub serves them.
Links inside fenced code blocks and absolute URLs are not checked.
"""
from __future__ import annotations

import os
import re
import sys
import urllib.parse
from pathlib import Path

FENCE = re.compile(r"^(\s*)(```|~~~)")
INLINE = re.compile(
    r"(!?)\[((?:[^\[\]]|\[[^\[\]]*\])*)\]\(\s*(<[^>]*>|[^)\s]+)((?:\s+(?:\"[^\"]*\"|'[^']*'))?)\s*\)"
)
REF_DEF = re.compile(r"^ {0,3}\[[^\]]+\]:[ \t]*(<[^>]*>|\S+)")
HTML_ATTR = re.compile(r"\b(?:src|href)\s*=\s*([\"'])([^\"']*)\1", re.I)
SKIP = ("http://", "https://", "mailto:", "tel:", "data:", "ftp:", "/")


def prose_chunks(text: str):
    in_code, marker, buf = False, "", []
    for line in text.splitlines():
        m = FENCE.match(line)
        if not in_code and m:
            in_code, marker = True, m.group(2)
            continue
        if in_code:
            if line.strip().startswith(marker):
                in_code = False
            continue
        buf.append(line)
    return "\n".join(buf)


def slug(heading: str) -> str:
    text = re.sub(r"!?\[([^\]]*)\]\([^)]*\)", r"\1", heading)
    text = re.sub(r"<[^>]+>", "", text).replace("`", "").replace("*", "")
    return re.sub(r"[^\w\- ]", "", text.strip().lower()).replace(" ", "-")


def anchors(md: Path, cache: dict) -> set[str]:
    if md not in cache:
        found, seen = set(), {}
        prose = prose_chunks(md.read_text(encoding="utf-8", errors="ignore"))
        for line in prose.splitlines():
            m = re.match(r"^\s{0,3}#{1,6}\s+(.*?)\s*#*\s*$", line)
            if m:
                s = slug(m.group(1))
                n = seen.get(s, 0)
                found.add(s if n == 0 else f"{s}-{n}")
                seen[s] = n + 1
        found |= set(re.findall(r"<a\s+(?:id|name)=[\"']([^\"']+)[\"']", prose))
        cache[md] = {a.lower() for a in found}
    return cache[md]


def exists_exact(root: Path, rel: Path) -> bool:
    current = root
    for part in rel.parts:
        if part in ("", "."):
            continue
        if part == "..":
            current = current.parent
            continue
        try:
            names = os.listdir(current)
        except (FileNotFoundError, NotADirectoryError):
            return False
        if part not in names:
            return False
        current = current / part
    return True


def main() -> int:
    root = Path(sys.argv[1] if len(sys.argv) > 1 else ".").resolve()
    broken, cache = [], {}
    for md in sorted(root.rglob("*.md")):
        if ".git" in md.parts:
            continue
        prose = prose_chunks(md.read_text(encoding="utf-8", errors="ignore"))
        targets = [m.group(3) for m in INLINE.finditer(prose)]
        targets += [m.group(1) for line in prose.splitlines() if (m := REF_DEF.match(line))]
        targets += [m.group(2) for m in HTML_ATTR.finditer(prose)]
        here = md.relative_to(root).as_posix()
        for t in targets:
            t = t[1:-1] if t.startswith("<") and t.endswith(">") else t
            if not t or t.lower().startswith(SKIP) or "://" in t:
                continue
            path, _, anchor = t.partition("#")
            path = path.split("?")[0]
            if not path:
                if anchor and urllib.parse.unquote(anchor).lower() not in anchors(md, cache):
                    broken.append(f"{here}: {t} (anchor)")
                continue
            rel_from_root = Path(os.path.normpath(md.parent.relative_to(root) / urllib.parse.unquote(path)))
            if rel_from_root.parts and rel_from_root.parts[0] == "..":
                broken.append(f"{here}: {t} (outside repository)")
                continue
            if not exists_exact(root, rel_from_root):
                broken.append(f"{here}: {t}")
                continue
            target = root / rel_from_root
            if anchor and target.suffix.lower() == ".md":
                if urllib.parse.unquote(anchor).lower() not in anchors(target, cache):
                    broken.append(f"{here}: {t} (anchor)")
    for b in broken:
        print(f"broken link: {b}")
    print(f"Markdown links checked; broken: {len(broken)}")
    return 1 if broken else 0


if __name__ == "__main__":
    sys.exit(main())
