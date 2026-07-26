#!/usr/bin/env python3
"""Regenerate components/irrigoto/*_html.h from html/*.html sources.

The .html files (in this directory) are the source of truth; the *_html.h
files ONE LEVEL UP (component root) are what the firmware embeds (C++
raw-string literals). Edit the .html, run this script, commit both files.
`regen.py --check` exits nonzero if any pair is out of sync without
rewriting anything.

The generated headers live at the component ROOT (not here) so ESPHome's
standard source copy ships them into the build tree next to irrigoto.c —
see the include-path note in ../__init__.py (GitHub issue #4: ESPHome
2026.7's native-IDF builder drops add_build_flag -I flags, which broke
the old keep-them-in-html/ approach).

History: these pairs drifted when edits went straight into the .h --
zone_setup.html was ~220 lines behind its header by b435, and cal/fs/landing
had no .html source at all until they were re-extracted from the headers.
"""
import re
import sys
from pathlib import Path

HERE = Path(__file__).parent          # html/ — .html sources
OUT = HERE.parent                     # component root — *_html.h payloads
WRAP = re.compile(r'R"([A-Z]+)\(\r?\n(.*)\)\1"', re.S)


def main() -> int:
    check = "--check" in sys.argv
    fail = False
    for h in sorted(OUT.glob("*_html.h")):
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
        # The include guard keeps the fragment inert when included outside
        # irrigoto.c's payload sites (ESPHome 2026.7+ esphome.h glob-includes
        # every copied component header — GitHub issue #4). Headers lacking
        # it are stale even if the body matches.
        guarded = "#ifdef IRRIGOTO_HTML_PAYLOAD" in raw
        if guarded and m.group(2).replace("\r\n", "\n") == body.replace("\r\n", "\n"):
            print(f"{h.name}: in sync with {html.name}")
            continue
        if not guarded and check:
            print(f"{h.name}: missing IRRIGOTO_HTML_PAYLOAD guard")
            fail = True
            continue
        if check:
            print(f"{h.name}: OUT OF SYNC with {html.name}")
            fail = True
            continue
        delim = m.group(1)
        new = (
            f"/* Auto-generated from {html.name} -- edit the .html, then run regen.py */\n"
            "#ifdef IRRIGOTO_HTML_PAYLOAD\n"
            f'R"{delim}(\n{body}'
            + ("" if body.endswith("\n") else "\n")
            + f'){delim}"\n'
            "#endif /* IRRIGOTO_HTML_PAYLOAD */\n"
        )
        h.write_bytes(new.encode("utf-8"))
        print(f"{h.name}: regenerated from {html.name}")
    return 1 if fail else 0


if __name__ == "__main__":
    sys.exit(main())
