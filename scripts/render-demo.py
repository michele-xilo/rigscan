#!/usr/bin/env python3
"""Render the real output of `rigscan scan` on examples/broken_arm.urdf to assets/demo.png.

Needs `cargo build --release -p rigscan-cli` and Chrome (set CHROME to its binary if it is not the macOS default).
"""
import html
import os
import re
import subprocess
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
CHROME = os.environ.get("CHROME", "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome")
COMMAND = ["scan", "--color", "always", "examples/broken_arm.urdf"]
COLOURS = {"31": "#F26B63", "33": "#F0B33C", "36": "#3CC4B3"}

out = subprocess.run([ROOT / "target/release/rigscan", *COMMAND], cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True).stdout
body = html.escape(out.rstrip())
body = re.sub(r"\x1b\[(\d+)m", lambda m: "</span>" if m[1] == "0" else f'<span style="color:{COLOURS[m[1]]}">', body)
lines = out.count("\n") + 3
page = f"""<!doctype html><meta charset="utf-8"><style>
html,body{{margin:0;background:#F3EFE6}}
.t{{margin:28px;background:#14202E;color:#F3EFE6;border-radius:14px;overflow:hidden;width:1180px;
box-shadow:0 8px 24px #14202E33;font:500 17px/1.45 ui-monospace,"SF Mono",Menlo,monospace}}
.b{{height:34px;background:#1d2d3f;display:flex;gap:8px;align-items:center;padding-left:16px}}
.b i{{width:12px;height:12px;border-radius:50%;background:#3d4f63;display:block}}
pre{{margin:0;padding:18px 24px 22px;white-space:pre}}.p{{color:#8a97a6}}
</style><div class="t"><div class="b"><i></i><i></i><i></i></div>
<pre><span class="p">$</span> rigscan scan examples/broken_arm.urdf

{body}</pre></div>"""
with tempfile.TemporaryDirectory() as tmp:
    src = Path(tmp) / "demo.html"
    src.write_text(page)
    height = int(lines * 17 * 1.45 + 34 + 40 + 56)
    subprocess.run(
        [CHROME, "--headless=new", "--disable-gpu", "--hide-scrollbars", f"--screenshot={ROOT / 'assets/demo.png'}",
         f"--window-size=1236,{height}", src.as_uri()],
        check=True, capture_output=True,
    )
