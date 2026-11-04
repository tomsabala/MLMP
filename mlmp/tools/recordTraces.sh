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
#
# A traced run is NOT byte-reproducible, even at a fixed seed: frame capture spends part of the
# wall-clock budget and getPlannerData draws from the global RNG, so path fields differ between
# two identical traced invocations and from the untraced --seed 42 numbers. Measured: the same
# seed and image traced twice gave path_length_raw 5.8111136351137445 and 5.2440537673478556,
# while the committed scene1-biqrrt.run.json records 5.859011316773332. Re-recording therefore
# changes the .run.json numbers; that is expected, not a regression.
#
# The image must be new enough to tag tree membership (component 1/2/3 on trace vertices).
# ghcr.io/tomsabala/mlmp:latest predates tagging, so it is deliberately not the default.
set -uo pipefail
IMAGE="${1:-mlmp:dev}"
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
    tagged=$(python3 - "$OUT/$name.json" <<'PY' 2>/dev/null || echo '-'
import json, sys
d = json.load(open(sys.argv[1]))
print(sum(1 for v in d["vertices"] if v[1] != 0))
PY
)
    if [ "$tagged" = "0" ]; then
      echo "FATAL: $name packed with every vertex untagged (component 0)." >&2
      echo "       The image '$IMAGE' most likely predates tree tagging; use mlmp:dev or a" >&2
      echo "       release built after it. Refusing to commit single-colour fixtures." >&2
      exit 3
    fi
    rm -f "$RAW/$name.jsonl"
    frames=$(python3 -c "import json;print(json.load(open('$OUT/$name.run.json'))['trace']['frames'])" 2>/dev/null || echo '-')
    bytes=$(stat -c%s "$OUT/$name.json" 2>/dev/null || echo 0)
    printf '%-18s exit=%s frames=%s packed_bytes=%s\n' "$name" "$code" "$frames" "$bytes"
  done
done
