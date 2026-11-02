#!/usr/bin/env bash
# Re-record every committed trace fixture. Usage: mlmp/tools/recordTraces.sh [image]
#
# Each cell records a raw mlmp.trace/1 stream into a scratch directory, packs it into the
# compact mlmp.traceviz/1 fixture the GIF and the player read (mlmp/tools/packTrace.py), and
# drops the raw stream: raw recordings are 0.3-0.9 GB apiece because every frame re-dumps the
# whole planner graph, so only the packed form is committed.
#
# Exit 1 cells (QRRT everywhere, BiQRRT on the harder scenes) are expected outcomes, not script
# failures, so the loop deliberately runs without `set -e`.
set -uo pipefail
IMAGE="${1:-ghcr.io/tomsabala/mlmp:latest}"
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
OUT="$ROOT/docs/traces"
SCENES="$ROOT/docs/scenes"
RAW="$(mktemp -d)"
trap 'rm -rf "$RAW"' EXIT
mkdir -p "$OUT" "$SCENES"
for s in 1 2 3 4 5 6; do
  cp "$ROOT/mlmp/src/inputs/experiment/scene$s.json" "$SCENES/scene$s.json"
  for a in BiQRRT QRRT; do
    lower=$(echo "$a" | tr '[:upper:]' '[:lower:]')
    name="scene$s-$lower"
    docker run --rm --user "$(id -u):$(id -g)" -v "$RAW:/out" "$IMAGE" \
      --algo "$a" --input-file "scene$s.json" --time-limit 10 --seed 42 \
      --trace-interval-ms 50 --trace-max-frames 400 \
      --trace-file "/out/$name.jsonl" > "$OUT/$name.run.json" && code=0 || code=$?
    python3 "$ROOT/mlmp/tools/packTrace.py" --trace "$RAW/$name.jsonl" \
      --run "$OUT/$name.run.json" --out "$OUT/$name.json" >/dev/null || echo "pack failed: $name"
    rm -f "$RAW/$name.jsonl"
    frames=$(python3 -c "import json;print(json.load(open('$OUT/$name.run.json'))['trace']['frames'])" 2>/dev/null || echo '-')
    bytes=$(stat -c%s "$OUT/$name.json" 2>/dev/null || echo 0)
    printf '%-18s exit=%s frames=%s packed_bytes=%s\n' "$name" "$code" "$frames" "$bytes"
  done
done
