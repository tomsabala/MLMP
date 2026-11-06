#!/usr/bin/env bash
# Re-record every committed trace fixture. Usage: mlmp/tools/recordTraces.sh [image] [cell ...]
#
# With no cell arguments every cell in CELLS below is recorded: 320 s of planner budget plus
# the packing of several GB of raw stream, which measured ~20 minutes of wall clock here.
# Naming one or more cells ("recordTraces.sh mlmp:dev scene3-qrrt") re-records just those,
# which is what re-measuring a single budget after a planner change actually needs.
#
# Each cell records a raw mlmp.trace/1 stream into a scratch directory, packs it into the
# compact mlmp.traceviz/1 fixture the GIF and the player read (mlmp/tools/packTrace.py), and
# drops the raw stream: raw recordings are 0.3-0.9 GB apiece at a 10 s budget because every
# frame re-dumps the whole planner graph, so only the packed form is committed.
#
# Exit 1 cells (every QRRT cell, and BiQRRT on scenes 3, 4 and 6) are expected outcomes, not
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
# The budget is the smallest of {10, 20, 30, 60, 120} s at which that planner solves this scene
# with tracing on, seed 42, on mlmp:dev. The explorer's default view is solution playback, so a
# cell that cannot produce a path is worth far less than one that can.
#
# Every budget below was re-measured after the collision-resolution fix (KinematicChainSpace::
# validSegmentCount now sizes the motion validator from the swept distance of the arm, and
# Solver::planarCheckStep pins that step to a fifth of the scene's shortest obstacle edge).
# A motion now costs one to two orders of magnitude more state-validity checks than it did, so
# the planner covers far less ground per second and three cells that used to solve no longer do:
# scene3/BiQRRT (was 20 s traced) now fails untraced at 10, 30, 60 and 120 s, and QRRT, which
# used to solve scene1 untraced at 20 s and scene5 untraced at 30 s, now solves scene1 at no
# untraced budget up to 60 s and scene5 only at 60 s untraced (first solution 28.2 s).
#
# Untraced probes only narrow the search; every budget below was confirmed with tracing on,
# because every frame re-dumps the whole planner graph, so capture gets more expensive as the
# graph grows and the planner keeps a shrinking share of the wall clock. Measured after the fix:
# BiQRRT solves scene2 and scene5 untraced at 10 s, but traced it needs 60 s for either.
#
# The interval keeps the whole run inside --trace-max-frames 400: budget_s * 1000 / interval
# must stay <= 400, or the frames stop a third of the way in and the replay freezes early.
# Capture overhead stretches the real cadence, so the counts land under the cap (a 60 s/150 ms
# cell records ~340-380 frames) while the last frame still sits within 0.5 % of the budget.
#
# A cell that solves at no budget in the set is recorded at its partner's budget, so a scene's
# two panels are the same planner time and the side-by-side is a fair comparison rather than
# one planner being given longer. Scenes 3, 4 and 6 solve for neither planner, so they keep 10 s.
CELLS=(
  # scene1: the only cell that solves at the 10 s budget the rest of the repo pins — BiQRRT's
  # first solution lands at 3.56 s traced (1.80 s untraced). This is the explorer's default.
  "1 BiQRRT 10 50"
  # scene1: QRRT solves at no budget — untraced probes failed at 10 s and 60 s after the fix
  # (before it, untraced 20 s solved). Recorded at BiQRRT's 10 s to match the pair.
  "1 QRRT 10 50"
  # scene2: BiQRRT needs 60 s with tracing on; 10 s, 20 s and 30 s traced all failed, 10 s and
  # 20 s twice. Untraced it still solves at 10 s, so the whole gap here is capture overhead.
  "2 BiQRRT 60 150"
  # scene2: QRRT solved at neither 10 s nor 60 s untraced, so traced is hopeless too.
  "2 QRRT 60 150"
  # scene3: BiQRRT no longer solves this scene at all — untraced probes failed at 10, 30, 60
  # and 120 s. Before the fix it solved traced at 20 s and was the explorer's default; the
  # finer collision resolution is the cause. Kept at 10 s, like the other unsolved scenes.
  "3 BiQRRT 10 50"
  # scene3: QRRT failed untraced at 10 s and has never solved this scene traced.
  "3 QRRT 10 50"
  # scene4: solved by neither planner — BiQRRT failed untraced at 10, 30, 60 and 120 s, QRRT
  # at 10 s. Kept at 10 s: a longer budget buys a denser cloud and no path.
  "4 BiQRRT 10 50"
  "4 QRRT 10 50"
  # scene5: 60 s is the only traced budget at which BiQRRT has solved this scene after the fix,
  # and only 1 run in 4 did (10 s, 20 s and 30 s traced all failed; untraced it still solves at
  # 10 s, first solution 5.08 s). The committed fixture is from a 60 s run that did not solve;
  # re-recording may well produce a solved one. Kept at 60 s rather than dropped to 10 s because
  # 10 s cannot solve it at all, and the 60 s graph is much the richer picture either way.
  "5 BiQRRT 60 150"
  # scene5: QRRT solves untraced at 60 s (first solution 28.2 s) but not traced at 60 s, which
  # is the only budget it could be recorded at without outspending its partner.
  "5 QRRT 60 150"
  # scene6: solved by neither planner — BiQRRT failed untraced at 10, 30, 60 and 120 s, QRRT
  # at 10 s.
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
