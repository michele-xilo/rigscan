#!/usr/bin/env bash
# Downloads the real-robot URDF corpus used by the end-to-end tests into the
# gitignored corpus/ directory. Pinned to one commit so results are reproducible.
set -euo pipefail

REPO=Gepetto/example-robot-data
COMMIT=78b6101a6387de721316b0db717351d899f97b4f
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DEST="$ROOT/corpus/example-robot-data"

mkdir -p "$DEST"
printf '%s\n' "$REPO" "$COMMIT" > "$DEST/SOURCE"

fetch() {
    local path=$1 out="$DEST/$1"
    [ -s "$out" ] && return 0
    mkdir -p "$(dirname "$out")"
    curl -fsS --retry 3 -o "$out.tmp" "https://raw.githubusercontent.com/$REPO/$COMMIT/$path"
    mv "$out.tmp" "$out"
}
export -f fetch
export REPO COMMIT DEST

xargs -P 8 -I{} bash -c 'fetch {}' < "$ROOT/scripts/corpus-urdf.txt"
echo "corpus: $(wc -l < "$ROOT/scripts/corpus-urdf.txt") files in $DEST ($REPO@${COMMIT:0:7})"
