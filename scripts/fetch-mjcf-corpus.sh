#!/usr/bin/env bash
# Downloads the MJCF files of MuJoCo Menagerie into the gitignored corpus/ directory. Only the
# .xml files: the tests resolve includes but not mesh files. Pinned to one commit so results are
# reproducible.
set -euo pipefail

REPO=google-deepmind/mujoco_menagerie
COMMIT=98a70c569c4c1b6da467a20012f766ee1f8dc521
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DEST="$ROOT/corpus/mujoco_menagerie"

mkdir -p "$DEST"
printf '%s\n' "$REPO" "$COMMIT" > "$DEST/SOURCE"

LIST="$DEST/files.txt"
curl -fsS --retry 3 "https://api.github.com/repos/$REPO/git/trees/$COMMIT?recursive=1" \
    | python3 -c 'import json,sys; [print(e["path"]) for e in json.load(sys.stdin)["tree"] if e["type"]=="blob" and e["path"].endswith(".xml")]' \
    > "$LIST"

fetch() {
    local path=$1 out="$DEST/$1"
    [ -s "$out" ] && return 0
    mkdir -p "$(dirname "$out")"
    curl -fsS --retry 3 -o "$out.tmp" "https://raw.githubusercontent.com/$REPO/$COMMIT/$path"
    mv "$out.tmp" "$out"
}
export -f fetch
export REPO COMMIT DEST

xargs -P 8 -I{} bash -c 'fetch {}' < "$LIST"
echo "corpus: $(wc -l < "$LIST") files in $DEST ($REPO@${COMMIT:0:7})"
