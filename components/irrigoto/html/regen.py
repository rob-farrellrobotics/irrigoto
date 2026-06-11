#!/usr/bin/env python3
"""Regenerate components/irrigoto/html/*_html.h from their .html sources.

The .html files are the source of truth; the *_html.h files are what the
firmware embeds (C++ raw-string literals). Edit the .html, run this script,
commit both files. `regen.py --check` exits nonzero if any pair is out of
sync without rewriting anything.

History: these pairs drifted when edits went straight into the .h --
zone_setup.html was ~220 lines behind its header by b435, and cal/fs/landing
had no .html source at all until they were re-extracted from the headers.
"""
import re
import sys
from pathlib import Path

HERE = Path(__file__).parent
WRAP = re.compile(r'R"([A-Z]+)\(\r?\n(.*)\)\1"', re.S)


def main() -> int:
    check = "--check" in sys.argv
    fail = False
    for h in sorted(HERE.glob("*_html.h")):
        html = HERE / h.name.replace("_html.h", ".html")
        raw = h.read_bytes().decode("utf-8")
        m = WRAP.search(raw)
        if not m:
            print(f"{h.name}: cannot parse raw-string wrapper")
            fail = True
            continue
        if not html.exists():
            print(f"{h.name}: MISSING source {html.name} -- extract it from the header body")
            fail = True
            continue
        body = html.read_bytes().decode("utf-8")
        if m.group(2).replace("\r\n", "\n") == body.replace("\r\n", "\n"):
            print(f"{h.name}: in sync with {html.name}")
            continue
        if check:
            print(f"{h.name}: OUT OF SYNC with {html.name}")
            fail = True
            continue
        delim = m.group(1)
        new = (
            f"/* Auto-generated from {html.name} -- edit the .html, then run regen.py */\n"
            f'R"{delim}(\n{body}'
            + ("" if body.endswith("\n") else "\n")
            + f'){delim}"\n'
        )
        h.write_bytes(new.encode("utf-8"))
        print(f"{h.name}: regenerated from {html.name}")
    return 1 if fail else 0


if __name__ == "__main__":
    sys.exit(main())
