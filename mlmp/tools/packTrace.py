#!/usr/bin/env python3
"""Pack one raw mlmp.trace/1 stream into the compact mlmp.traceviz/1 fixture the players read.

    python3 mlmp/tools/packTrace.py --trace t.jsonl --run t.run.json --out docs/traces/x.json

Why a second format: every mlmp.trace/1 frame re-dumps the entire planner graph, so a 10 s
scene1 recording is 0.3-0.9 GB — unservable from GitHub Pages and unfit for git. Two measured
properties of the stream make it compressible without losing anything a renderer draws:

  * A vertex at bundle level L owns only its first L+1 links per robot. The tail above L is
    identity-fibre padding re-derived from the planner's current section every frame, which is
    why the raw states churn: every one of the 193 frames in the committed scene1/QRRT
    recording re-dumps a full q vector for each graph vertex it holds at that instant.
    Truncating to the owned prefix collapses that whole stream to 15 236 distinct vertices —
    the same order as the planner's own graph, which docs/traces/scene1-qrrt.run.json reports
    as 15 247 vertices. (Not equal: distinct prefixes merge states that differ only in the
    padding, and the last frame is captured before the run's final vertices are added.) The
    padding was all of the churn. The figures above come from the committed fixtures; the raw
    streams are too large to commit, so a distinct-full-q count cannot be recomputed here.
  * Truncated that way, each frame's vertex and edge sets are supersets of the previous frame's
    (the graph only grows). Assigning ids in order of first appearance therefore makes frame k
    exactly `vertices[:nv]` and `edges[:ne]`.

So the fixture stores every vertex and edge once, and each frame is a timestamp plus two
counts. packTrace asserts both properties and fails loudly rather than silently dropping data.

A vertex's identity here is (level, truncated q) — the component is NOT part of it. On a
tree_source == "derived" stream the label is a property of the graph at that instant and moves
as edges arrive, so one configuration can be labelled 0 then 1 then 2; the stored row keeps the
last label it carried, and a replay therefore colours a vertex by the side it ends up on.
Planner-tagged streams never relabel.
"""
import argparse
import json
import sys
from pathlib import Path

TREE_CODES = {0: "untagged", 1: "start", 2: "goal", 3: "section"}
RUN_KEYS = (
    "planner",
    "scene",
    "solved",
    "exact",
    "status",
    "time_solve_s",
    "time_first_solution_s",
    "path_length_raw",
    "path_length_simplified",
    "path_states",
    "graph_vertices",
    "graph_edges",
)


def packTrace(tracePath, runPath, digits):
    header = None
    vertexIds = {}
    vertices = []
    edgeIds = {}
    edges = []
    frames = []
    # "planner" (BiQRRT tagged its own trees) or "derived" (captureFrame labelled the graph by
    # connectivity because the planner tagged nothing). Constant within a run; the players say
    # which one the colours mean, so a disagreement mid-stream is a bug, not a merge.
    treeSource = None
    # The stream is far too large to buffer, so a decode failure is held open for one line
    # instead: if anything later parses, the held line was mid-stream corruption, not the
    # truncated final line a killed recorder leaves behind.
    pendingBadLine = None

    with open(tracePath, "r") as f:
        for lineNo, line in enumerate(f, start=1):
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
            except json.JSONDecodeError:
                # Only a truncated final line is tolerable; anything earlier is corruption.
                if pendingBadLine is not None:
                    raise SystemExit(
                        f"{tracePath}: line {pendingBadLine} is not valid JSON and line "
                        f"{lineNo} fails to decode as well; the stream is corrupt"
                    )
                pendingBadLine = lineNo
                continue
            if pendingBadLine is not None:
                raise SystemExit(
                    f"{tracePath}: line {pendingBadLine} is not valid JSON but line {lineNo} "
                    "parses; that is mid-stream corruption, not a truncated final line"
                )
            if header is None:
                header = obj
                if header.get("schema") != "mlmp.trace/1":
                    raise SystemExit(f"{tracePath}: not an mlmp.trace/1 stream")
                continue

            # A frame with no vertices omits the key entirely: it labels nothing, so it makes no
            # claim to agree or disagree with. Every frame that does carry labels must agree.
            frameSource = obj.get("tree_source")
            if frameSource is not None:
                if treeSource is None:
                    treeSource = frameSource
                elif frameSource != treeSource:
                    raise SystemExit(
                        f"{tracePath}: frame {obj['frame']} reports tree_source "
                        f"{frameSource!r} after {treeSource!r}; the stream mixes label sources"
                    )

            robots = header["robots"]
            local = []
            for v in obj["vertices"]:
                level = int(v["level"])
                component = int(v.get("component", 0))
                q = v["q"]
                owned = tuple(
                    round(q[j * robots + r], digits)
                    for j in range(level + 1)
                    for r in range(robots)
                )
                # Keyed by geometry alone. A derived component is a property of the graph at
                # that instant, not of the configuration: as edges arrive, the endpoint nearest
                # a vertex can change, so the same configuration may be labelled 0 then 1 then
                # 2 across frames (measured on scene5/QRRT: 2 790 relabels in 20 frames). The
                # packed format stores one row per vertex, so the row keeps the LAST label the
                # vertex carried — a replay therefore colours a vertex by the side it ends up
                # on. Planner-tagged streams never relabel, so this only affects
                # tree_source == "derived" fixtures.
                key = (level, owned)
                vid = vertexIds.get(key)
                if vid is None:
                    vid = len(vertices)
                    vertexIds[key] = vid
                    vertices.append([level, component, *owned])
                else:
                    vertices[vid][1] = component
                local.append(vid)

            seen = set(local)
            if seen and max(seen) + 1 != len(seen):
                raise SystemExit(
                    f"{tracePath}: frame {obj['frame']} dropped vertices seen in an earlier "
                    "frame; the append-only assumption no longer holds"
                )
            nv = len(seen)

            frameEdges = set()
            for a, b in obj["edges"]:
                if a >= len(local) or b >= len(local):
                    continue
                ea, eb = local[a], local[b]
                if ea == eb:
                    continue
                key = (ea, eb) if ea < eb else (eb, ea)
                eid = edgeIds.get(key)
                if eid is None:
                    eid = len(edges)
                    edgeIds[key] = eid
                    edges.append([key[0], key[1]])
                frameEdges.add(eid)
            if frameEdges and max(frameEdges) + 1 != len(frameEdges):
                raise SystemExit(
                    f"{tracePath}: frame {obj['frame']} dropped edges seen in an earlier frame"
                )

            frames.append([round(obj["t_s"], 3), nv, len(frameEdges)])

    if header is None:
        raise SystemExit(f"{tracePath}: no header line")

    # Every id is created by the frame that first shows it, and each frame's ids are a prefix,
    # so the last frame must hold all of them. A fixture whose frames all count 0 while the
    # vertex table is full renders as an empty player, which is exactly the silent failure this
    # packer exists to prevent.
    if frames and (frames[-1][1] != len(vertices) or frames[-1][2] != len(edges)):
        raise SystemExit(
            f"{tracePath}: last frame counts {frames[-1][1]} vertices and {frames[-1][2]} edges "
            f"but the fixture holds {len(vertices)} and {len(edges)}; the per-frame prefix is "
            "not the whole table"
        )

    run = {}
    if runPath and Path(runPath).exists():
        try:
            full = json.loads(Path(runPath).read_text())
        except json.JSONDecodeError:
            full = {}
        run = {k: full[k] for k in RUN_KEYS if k in full}

    return {
        "schema": "mlmp.traceviz/1",
        "source": "mlmp.trace/1",
        "planner": header["planner"],
        "scene": header["scene"],
        "seed": header["seed"],
        "robots": header["robots"],
        "links": header["links"],
        "time_limit_s": header["time_limit_s"],
        "interval_ms": header["interval_ms"],
        "tree_codes": {str(k): v for k, v in TREE_CODES.items()},
        "tree_source": treeSource or "planner",
        "run": run,
        "vertices": vertices,
        "edges": edges,
        "frames": frames,
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--trace", required=True)
    ap.add_argument("--run")
    ap.add_argument("--out", required=True)
    ap.add_argument("--round", type=int, default=4, help="angle decimals kept (default 4)")
    args = ap.parse_args()

    packed = packTrace(args.trace, args.run, args.round)
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(packed, separators=(",", ":")))
    print(
        "%s: %d frames, %d vertices, %d edges, %d bytes"
        % (out, len(packed["frames"]), len(packed["vertices"]), len(packed["edges"]),
           out.stat().st_size)
    )


if __name__ == "__main__":
    try:
        main()
    except SystemExit:
        raise
    except Exception as exc:  # pragma: no cover - CLI convenience
        print("Error: %s" % exc, file=sys.stderr)
        sys.exit(1)
