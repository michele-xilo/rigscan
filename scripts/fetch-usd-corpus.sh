#!/usr/bin/env bash
# Downloads the USD files of newton-assets (Apache-2.0, each robot has its own LICENSE) into the
# gitignored corpus/ directory. Only .usd, .usda and .usdc files: the tests compose layers but do
# not read meshes or textures. Pinned to one commit so results are reproducible.
set -euo pipefail

REPO=newton-physics/newton-assets
COMMIT=ac1f172b6f53915f9b81aa9ccd765ae792cc264c
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DEST="$ROOT/corpus/newton_assets"

# A blobless clone with sparse checkout fetches only the matching files; the tree listing alone is
# too large for the GitHub tree API (truncated).
if [ ! -d "$DEST/.git" ]; then
    git clone -q --filter=blob:none --no-checkout "https://github.com/$REPO.git" "$DEST"
fi
git -C "$DEST" sparse-checkout set --no-cone '*.usd' '*.usda' '*.usdc' '/*/LICENSE*'
git -C "$DEST" checkout -q "$COMMIT"
printf '%s\n' "$REPO" "$COMMIT" > "$DEST/SOURCE"
echo "corpus: $(find "$DEST" -not -path '*/.git/*' \( -name '*.usd' -o -name '*.usda' -o -name '*.usdc' \) | wc -l | tr -d ' ') files in $DEST ($REPO@${COMMIT:0:7})"
