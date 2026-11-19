#!/usr/bin/env python3
"""Checks the Space's logic against the built CLI, without Gradio.

    cargo build -p rigscan-cli && python3 space/test_rigscan_space.py
"""
import os
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
os.environ.setdefault("RIGSCAN_BIN", str(ROOT / "target/debug/rigscan"))

import rigscan_space as space  # noqa: E402

EXAMPLES = ROOT / "examples"
FORMATS = {"urdf": "arm.urdf", "mjcf": "arm.xml", "usd": "arm.usda"}


def test_a_healthy_example_scans_clean_in_every_format():
    for name in FORMATS.values():
        r = space.scan(str(EXAMPLES / name))
        assert r["error"] is None, (name, r)
        assert r["findings"] == [], (name, r["findings"])
        assert r["verdict"].startswith("Healthy"), r["verdict"]


def test_a_broken_example_lists_its_findings():
    r = space.scan(str(EXAMPLES / "broken_arm.urdf"))
    rules = {f["rule"] for f in r["findings"]}
    assert {"RS001", "RS012"} <= rules, rules
    assert r["verdict"].startswith("Not healthy"), r["verdict"]


def test_every_example_converts_to_every_other_format():
    for src, name in FORMATS.items():
        for to in FORMATS:
            if to == src:
                continue
            r = space.convert(str(EXAMPLES / name), to)
            assert r["error"] is None, (src, to, r["error"])
            assert r["text"].strip(), (src, to)
            assert Path(r["output"]).read_text() == r["text"]
            assert r["report"]["to"] == to, r["report"]
            for row in r["losses"]:
                assert row["meaning"], row


def test_the_losses_name_what_was_left_out():
    r = space.convert(str(EXAMPLES / "arm.urdf"), "mjcf")
    ids = {row["id"] for row in r["losses"]}
    assert {"limit/effort", "limit/velocity"} <= ids, ids
    assert r["summary"].startswith("Converted"), r["summary"]


def test_a_robot_with_critical_findings_is_refused_with_a_reason():
    r = space.convert(str(EXAMPLES / "broken_arm.urdf"), "mjcf")
    assert r["text"] is None
    assert "not converted" in r["error"], r["error"]


def test_converting_to_the_same_format_is_an_error():
    r = space.convert(str(EXAMPLES / "arm.urdf"), "urdf")
    assert "already URDF" in r["error"], r["error"]


def test_a_file_that_is_not_a_robot_is_an_error_not_a_crash():
    with tempfile.TemporaryDirectory() as d:
        p = Path(d) / "x.urdf"
        p.write_text("not xml at all")
        assert space.scan(str(p))["error"]
        assert space.convert(str(p), "mjcf")["error"]


def test_a_file_over_the_cap_is_refused_before_it_is_read():
    with tempfile.TemporaryDirectory() as d:
        p = Path(d) / "big.urdf"
        p.write_bytes(b"<robot/>" + b" " * space.MAX_BYTES)
        assert "too large" in space.scan(str(p))["error"]
        assert "too large" in space.convert(str(p), "mjcf")["error"]


def test_an_unsupported_extension_and_an_unknown_target_are_refused():
    with tempfile.TemporaryDirectory() as d:
        p = Path(d) / "x.exe"
        p.write_text("<robot/>")
        assert "Unsupported" in space.scan(str(p))["error"]
        assert "Unsupported" in space.convert(str(p), "mjcf")["error"]
    assert "Unknown target" in space.convert(str(EXAMPLES / "arm.urdf"), "../x")["error"]


def test_an_odd_file_name_never_reaches_the_output_path():
    with tempfile.TemporaryDirectory() as d:
        p = Path(d) / "my robot; rm -rf $HOME.urdf"
        p.write_text((EXAMPLES / "arm.urdf").read_text())
        r = space.convert(str(p), "mjcf")
        assert r["error"] is None, r["error"]
        assert Path(r["output"]).name == "my_robot__rm_-rf__HOME.xml", r["output"]


def test_a_path_outside_the_upload_and_example_folders_is_refused():
    assert "Choose a file" in space.scan("/etc/hosts")["error"]
    assert "Choose a file" in space.convert("/etc/hosts", "mjcf")["error"]


def test_no_file_is_an_error():
    assert space.scan(None)["error"]
    assert space.convert(None, "mjcf")["error"]


if __name__ == "__main__":
    failed = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_"):
            try:
                fn()
                print(f"ok   {name}")
            except Exception as e:  # noqa: BLE001
                failed += 1
                print(f"FAIL {name}: {e!r}")
    sys.exit(1 if failed else 0)
