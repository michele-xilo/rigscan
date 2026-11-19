"""The Space's logic: run the Rigscan CLI on an uploaded file and shape the result for the page.

The CLI is the one built from the same tag as the Space, shipped next to this file as `rigscan`
(or named by RIGSCAN_BIN). Each call works in a temporary directory that is removed afterwards:
nothing an upload holds is kept.
"""
import json
import os
import shutil
import subprocess
import tempfile
import time
from pathlib import Path

# A robot description is text; a few megabytes is far above any hand-made one.
MAX_BYTES = 5 * 1024 * 1024
TIMEOUT = 60
HERE = Path(__file__).resolve().parent
TARGETS = {"URDF": "urdf", "MJCF": "mjcf", "USD": "usd"}
EXTENSION = {"urdf": "urdf", "mjcf": "xml", "usd": "usda"}


def binary():
    for candidate in (os.environ.get("RIGSCAN_BIN"), HERE / "rigscan", shutil.which("rigscan")):
        if candidate and Path(candidate).is_file():
            return str(candidate)
    raise FileNotFoundError("the rigscan binary was not found")


def version():
    out = subprocess.run([binary(), "--version"], capture_output=True, text=True, timeout=TIMEOUT)
    return out.stdout.strip().split()[-1] if out.returncode == 0 else "unknown"


def _sweep(older_than=600):
    """Remove downloads left by earlier calls, so nothing an upload produced stays around."""
    cutoff = time.time() - older_than
    for d in Path(tempfile.gettempdir()).glob("rigscan-out-*"):
        if d.stat().st_mtime < cutoff:
            shutil.rmtree(d, ignore_errors=True)


def _check(path):
    if not path:
        return "Choose a file first."
    p = Path(path)
    if p.stat().st_size > MAX_BYTES:
        return f"{p.name} is too large: the limit is {MAX_BYTES // (1024 * 1024)} MB."
    return None


def _run(args, cwd):
    try:
        return subprocess.run(args, capture_output=True, text=True, timeout=TIMEOUT, cwd=cwd)
    except subprocess.TimeoutExpired:
        return None


def _staged(path, work):
    # A copy under its own name: the CLI picks the format from the extension and a Gradio
    # upload is named by a temporary path.
    dest = Path(work) / Path(path).name
    shutil.copyfile(path, dest)
    return dest


def scan(path):
    """Findings and the verdict for one file: {error, verdict, findings, elements}."""
    problem = _check(path)
    if problem:
        return {"error": problem, "verdict": "", "findings": [], "elements": []}
    with tempfile.TemporaryDirectory() as work:
        run = _run([binary(), "scan", str(_staged(path, work)), "--output-format", "json"], work)
    if run is None:
        return {"error": "The scan took too long.", "verdict": "", "findings": [], "elements": []}
    try:
        file = json.loads(run.stdout)["files"][0]
    except (ValueError, KeyError, IndexError):
        return {"error": run.stderr.strip() or "Rigscan could not read the file.",
                "verdict": "", "findings": [], "elements": []}
    if "error" in file:
        message = file["error"].split(": ", 1)[-1]
        return {"error": message, "verdict": "", "findings": [], "elements": []}
    v = file["verdict"]
    healthy = v["critical"] == 0 and v["warnings"] == 0
    verdict = (f"{'Healthy' if healthy else 'Not healthy'}: {v['healthy']} healthy, "
               f"{v['warnings']} warning, {v['critical']} critical")
    return {"error": None, "verdict": verdict, "findings": file["findings"], "elements": file["elements"]}


def convert(path, to):
    """The converted text and what was lost: {error, text, output, losses, report, summary}."""
    empty = {"text": None, "output": None, "losses": [], "report": None, "summary": ""}
    problem = _check(path)
    if problem:
        return {**empty, "error": problem}
    with tempfile.TemporaryDirectory() as work:
        source = _staged(path, work)
        _sweep()
        # Kept for the download after the temporary directory is gone.
        out = Path(tempfile.mkdtemp(prefix="rigscan-out-")) / f"{source.stem}.{EXTENSION[to]}"
        report = out.with_suffix(".loss.json")
        run = _run([binary(), "convert", str(source), "--to", to, "-o", str(out),
                    "--loss-report", str(report)], work)
        if run is None:
            return {**empty, "error": "The conversion took too long."}
        if run.returncode > 1:
            shutil.rmtree(out.parent, ignore_errors=True)
            return {**empty, "error": run.stderr.strip() or "The conversion failed."}
        doc = json.loads(report.read_text())
        losses = [{"id": e["id"], "kind": e["kind"], "element": e["name"] or "", "meaning": e["summary"]}
                  for e in doc["lost"]]
        n = len(losses)
        summary = (f"Converted to {to.upper()}; {n} thing{'s' if n != 1 else ''} could not be carried over."
                   if n else f"Converted to {to.upper()}; nothing was lost.")
        return {"error": None, "text": out.read_text(), "output": str(out), "losses": losses,
                "report": doc, "report_path": str(report), "summary": summary}
