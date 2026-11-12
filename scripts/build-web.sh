#!/usr/bin/env bash
# Build the browser viewer into web/: the WASM module, its JS glue, the logo and the sample robot.
# Needs `rustup target add wasm32-unknown-unknown` and wasm-bindgen-cli at the version pinned in
# crates/rigscan-wasm/Cargo.toml (`cargo install wasm-bindgen-cli --version 0.2.130 --locked`).
set -euo pipefail
cd "$(dirname "$0")/.."

cargo build -p rigscan-wasm --target wasm32-unknown-unknown --profile wasm
wasm-bindgen --target web --no-typescript --remove-name-section --remove-producers-section --out-dir web/pkg target/wasm32-unknown-unknown/wasm/rigscan_wasm.wasm

mkdir -p web/assets web/sample
cp assets/logo.svg assets/logo-dark.svg assets/logo-transparent.svg assets/favicon.svg web/assets/
cp examples/broken_arm.urdf web/sample/
echo "built web/ ($(wc -c < web/pkg/rigscan_wasm_bg.wasm) bytes of wasm); serve it with: python3 -m http.server -d web 8000"
