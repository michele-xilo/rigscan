#!/usr/bin/env bash
# Assemble the Hugging Face Space in $1 (default: space-build): the app, the examples and the Linux
# CLI the app calls. Needs `cargo build --release -p rigscan-cli` first.
set -euo pipefail
cd "$(dirname "$0")/.."
out="${1:-space-build}"
rm -rf "$out"
mkdir -p "$out/examples"
cp space/app.py space/rigscan_space.py space/requirements.txt space/README.md "$out/"
cp examples/arm.urdf examples/arm.xml examples/arm.usda examples/broken_arm.urdf "$out/examples/"
cp target/release/rigscan "$out/rigscan"
echo "staged $out ($(du -sh "$out" | cut -f1))"
