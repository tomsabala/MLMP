#!/usr/bin/env bash
# Re-record every committed trace fixture. Usage: mlmp/tools/recordTraces.sh [image] [cell ...]
#
# With no cell arguments every cell in CELLS below is recorded: 280 s of planner budget plus
# the packing of several GB of raw stream, which measured ~15 minutes of wall clock here.
# Naming one or more cells ("recordTraces.sh mlmp:dev scene3-qrrt") re-records just those,
# which is what re-measuring a single budget after a planner change actually needs.
#
# Each cell records a raw mlmp.trace/1 stream into a scratch directory, packs it into the
# compact mlmp.traceviz/1 fixture the GIF and the player read (mlmp/tools/packTrace.py), and
# drops the raw stream: raw recordings are 0.3-0.9 GB apiece at a 10 s budget because every
# frame re-dumps the whole planner graph, so only the packed form is committed.
#
# Exit 1 cells (every QRRT cell, and BiQRRT on scenes 4 and 6) are expected outcomes, not
# script failures, so the loop deliberately runs without `set -e`.
#
# A traced run is NOT byte-reproducible, even at a fixed seed: frame capture spends part of the
# wall-clock budget and getPlannerData draws from the global RNG, so path fields differ between
# two identical traced invocations and from the untraced --seed 42 numbers. Measured: the same
# seed and image traced twice gave path_length_raw 5.8111136351137445 and 5.2440537673478556;
# scene1-biqrrt at the unchanged 10 s budget moved from 6.556839336283462 to 10.393613458786107
# across two re-recordings. Re-recording therefore changes the .run.json numbers; that is
# expected, not a regression. The CI planar regression pin is an *untraced* run and is unaffected.
#
# The image must be new enough to label tree membership on trace vertices: BiQRRT tags
# components 1/2/3 itself, and since derivation landed the binary labels every other planner's
# frames by connectivity. ghcr.io/tomsabala/mlmp:latest predates both, so it is deliberately
# not the default.

# Per-cell budget: "scene algo budget_s interval_ms".
#
# The budget is the smallest of {10, 20, 30, 60} s at which that planner solves this scene with
# tracing on, seed 42, on mlmp:dev. The explorer's default view is solution playback, so a cell
# that cannot produce a path is worth far less than one that can — 11 of the 12 fixtures this
# table replaced had path_states 0.
#
# Untraced probes only narrow the search; every budget below was confirmed with tracing on,
# because every frame re-dumps the whole planner graph, so capture gets more expensive as the
# graph grows and the planner keeps a shrinking share of the wall clock. Measured on this
# image: QRRT solves scene1 untraced at 20 s and scene5 untraced at 30 s, but with tracing on
# it solves neither, even at 60 s — where scene1/QRRT reached only 29 373 graph vertices in
# 60 s against the 11 339 a traced 10 s run reaches.
#
# The interval keeps the whole run inside --trace-max-frames 400: budget_s * 1000 / interval
# must stay <= 400, or the frames stop a third of the way in and the replay freezes early.
# Capture overhead stretches the real cadence, so the counts land under the cap (a 60 s/150 ms
# cell records ~313-325 frames) while the last frame still sits within 0.5 % of the budget.
#
# A cell that solves at no budget in the set is recorded at its partner's budget, so a scene's
# two panels are the same planner time and the side-by-side is a fair comparison rather than
# one planner being given longer. Scenes 4 and 6 solve for neither planner, so they keep 10 s.
CELLS=(
  # scene1: BiQRRT's first solution lands at 0.47 s traced. 10 s is the budget the rest of the
  # repo pins, and this is the one cell that already solved before this table existed.
  "1 BiQRRT 10 50"
  # scene1: QRRT solves at no budget traced - 20 s and 60 s both failed with tracing on,
  # though an untraced 20 s run does solve. Recorded at BiQRRT's 10 s to match the pair.
  "1 QRRT 10 50"
  # scene2: BiQRRT needs 60 s with tracing on; 10 s, 20 s and 30 s traced all failed.
  "2 BiQRRT 60 150"
  # scene2: QRRT solved at none of 10/20/30/60 s untraced, so traced is hopeless too.
  "2 QRRT 60 150"
  # scene3: BiQRRT needs 20 s with tracing on; 10 s traced failed although 10 s untraced solves.
  "3 BiQRRT 20 50"
  # scene3: QRRT solves untraced only at 60 s, and traced not even there (2.5 MB for an empty
  # path), so it is recorded at BiQRRT's 20 s.
  "3 QRRT 20 50"
  # scene4: solved by neither planner - untraced probes failed at 10, 20, 30 and 60 s for both.
  # Kept at 10 s: a longer budget buys a denser cloud and no path.
  "4 BiQRRT 10 50"
  "4 QRRT 10 50"
  # scene5: BiQRRT needs 30 s with tracing on; 10 s and 20 s traced failed.
  "5 BiQRRT 30 100"
  # scene5: QRRT solves untraced at 30 s but traced at neither 30 s nor 60 s.
  "5 QRRT 30 100"
  # scene6: solved by neither planner - untraced probes failed at 10, 20, 30 and 60 s for both.
  "6 BiQRRT 10 50"
  "6 QRRT 10 50"
)

set -uo pipefail
IMAGE="${1:-mlmp:dev}"
shift || true
WANT=("$@")
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
OUT="$ROOT/docs/traces"
SCENES="$ROOT/docs/scenes"
RAW="$(mktemp -d)"
trap 'rm -rf "$RAW"' EXIT
mkdir -p "$OUT" "$SCENES"
for s in 1 2 3 4 5 6; do
  cp "$ROOT/mlmp/src/inputs/experiment/scene$s.json" "$SCENES/scene$s.json"
done
for cell in "${CELLS[@]}"; do
  read -r s a budget interval <<<"$cell"
  lower=$(echo "$a" | tr '[:upper:]' '[:lower:]')
  name="scene$s-$lower"
  if [ ${#WANT[@]} -gt 0 ]; then
    match=0
    for w in "${WANT[@]}"; do [ "$w" = "$name" ] && match=1; done
    [ "$match" = 1 ] || continue
  fi
  docker run --rm --user "$(id -u):$(id -g)" -v "$RAW:/out" "$IMAGE" \
    --algo "$a" --input-file "scene$s.json" --time-limit "$budget" --seed 42 \
    --trace-interval-ms "$interval" --trace-max-frames 400 \
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
    echo "       Expected either the planner's own tags or labels derived from connectivity." >&2
    echo "       Either the image '$IMAGE' predates tree labelling (use mlmp:dev or a release" >&2
    echo "       built after it), or no vertex in the whole run was connected to the start or" >&2
    echo "       goal state — a run with nothing to show. Refusing to commit single-colour" >&2
    echo "       fixtures either way; check '$OUT/$name.run.json' to tell the two apart." >&2
    exit 3
  fi
  rm -f "$RAW/$name.jsonl"
  # path_states is reported next to solved because the explorer animates run.path: a solved run
  # with fewer than two states has nothing to play and means the budget is still too small.
  read -r frames solved states <<<"$(python3 - "$OUT/$name.run.json" <<'PY' 2>/dev/null || echo '- - -'
import json, sys
d = json.load(open(sys.argv[1]))
print(d["trace"]["frames"], d["solved"], d["path_states"])
PY
)"
  bytes=$(stat -c%s "$OUT/$name.json" 2>/dev/null || echo 0)
  printf '%-18s budget=%-3s interval=%-4s exit=%s solved=%-5s path_states=%-3s frames=%-4s packed_bytes=%s\n' \
    "$name" "$budget" "$interval" "$code" "$solved" "$states" "$frames" "$bytes"
done
