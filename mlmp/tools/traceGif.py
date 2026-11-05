#!/usr/bin/env python3
"""Render the project's animated figures from the committed fixtures and run records.

    # the hero picture: both arms playing back a solution
    python3 mlmp/tools/traceGif.py --mode motion --out docs/solution-motion.gif
    # the algorithm picture: BiQRRT's two trees beside QRRT's one
    python3 mlmp/tools/traceGif.py --mode trees --out docs/biqrrt-vs-qrrt.gif

`motion` reads a scene plus one mlmp.run/1 result and animates every robot along that run's
solution: arms as real segments, a fading ghost trail, each tip's curve accumulating, and the
start and goal configurations outlined throughout. The `path` in a run result is the
*simplified* path — scene1/BiQRRT/seed 42 has four states for an 8-D problem — so consecutive
states are densified with KinematicChainSpace::interpolate
(mlmp/src/state/kinematic_chain.h:112-139), which goes the short way round the circle and
re-wraps into [-PI, PI]; web/src/renderer.js mirrors the same branches as interpolateConfig /
densifyPath.

Drawing straight chords between those four states instead would be wrong in shape, not in
length: measured on that solution, robot 0's tip travels 0.875 scene units along the true
curve against 0.813 along the chords (1.08x), but the curve leaves the chord by up to 0.083 —
7.6% of the 1.09 x 0.90 scene, ~46 px on a 600 px canvas — and does it where the tip bends
around an obstacle. Interpolating naively (`from + (to - from) * t`, no wrap) is worse than
wrong-looking: on this solution joint 0 of robot 0 runs from +PI to -1.27, the short way is
+1.87 rad through the branch cut, and a naive lerp drags it -4.41 rad the other way through
poses the planner never validated — i.e. straight through an obstacle.

`trees` is the original side-by-side: BiQRRT left, QRRT right, both as projected chain-tip
clouds. Fixtures come from mlmp/tools/recordTraces.sh (record + pack). Rendering rules there
are mirrored verbatim in docs/app.js so the GIF and the trace player cannot disagree. Forward
kinematics follow KinematicChainValidityChecker (mlmp/src/state/kinematic_chain.h:199-222):
cumulative angles, index q[link*robots+robot], one constant jointLength per robot. A fixture
vertex at bundle level L stores only its first L+1 links per robot — the tail above L is
identity-fibre padding, not planner output, and is never drawn.

Colour in `trees` comes from the vertex's tree NAME, never from the planner name. BiQRRT tags
its own trees; for every other planner the binary derives start/goal from graph connectivity,
and the fixture records which of the two in its top-level "tree_source". Fixtures packed
before derivation existed carry no "tree_source" and leave every non-BiQRRT vertex at
component 0, so their panel renders in the single untagged grey — re-record with a current
image to get a blue start tree and a red goal tree on that side.
"""
import argparse
import json
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.collections import LineCollection  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from matplotlib.patches import Polygon  # noqa: E402
from PIL import Image  # noqa: E402

TREE_COLORS = {
    "start": "#1f77b4",
    "goal": "#d62728",
    "section": "#9467bd",
    "untagged": "#7f7f7f",
}
PADDING = 0.05

# Motion mode. Deliberately outside TREE_COLORS: those four names are the tree vocabulary
# web/src/renderer.js asserts against (web/test/renderer.test.mjs group 6), and a robot is
# not a tree.
ROBOT_COLORS = ("#1c7ed6", "#f76707")
GHOST_FRAMES = 6
HOLD_FRAMES = 12
TWO_PI = 2.0 * math.pi
# A GIF has no scrubber, so the loop has to read as one gesture: aim for this and let the
# frame delay fall out of however many frames the densified path needs. 33 ms is the floor
# because browsers clamp delays below ~20 ms back up to 100 ms.
TARGET_LOOP_MS = 5000


def loadPacked(path):
    packed = json.loads(Path(path).read_text())
    if packed.get("schema") != "mlmp.traceviz/1":
        raise SystemExit(f"{path}: not an mlmp.traceviz/1 fixture")
    return packed


def hexToRgb(h):
    return tuple(int(h[i : i + 2], 16) / 255.0 for i in (1, 3, 5))


def chainTips(q, level, robots, sceneRobots):
    """Tip of each robot's chain truncated to its first `level + 1` links."""
    tips = []
    for r, robot in enumerate(sceneRobots):
        x = robot["pinnedPosition"]["x"]
        y = robot["pinnedPosition"]["y"]
        length = robot["jointLength"]
        theta = 0.0
        for j in range(level + 1):
            idx = j * robots + r
            if idx >= len(q):
                break
            theta += q[idx]
            x += length * math.cos(theta)
            y += length * math.sin(theta)
        tips.append((x, y))
    return tips


def chainJointsRad(robot, anglesRad):
    """Every joint position of one robot, base first, for cumulative angles in radians."""
    x = robot["pinnedPosition"]["x"]
    y = robot["pinnedPosition"]["y"]
    length = robot["jointLength"]
    theta = 0.0
    pts = [(x, y)]
    for a in anglesRad:
        theta += a
        x += length * math.cos(theta)
        y += length * math.sin(theta)
        pts.append((x, y))
    return np.array(pts)


def chainJoints(robot, anglesDeg):
    """Scene JSON stores integer degrees (mlmp/src/models/scene.h:340-344)."""
    return chainJointsRad(robot, [math.radians(a) for a in anglesDeg])


def stateJoints(q, r, robots, links, robot):
    """Joints of robot `r` for a full state. Layout is interleaved by link,
    q[link * numRobots + robot] (mlmp/src/state/kinematic_chain.h:210)."""
    return chainJointsRad(robot, [q[j * robots + r] for j in range(links)])


def interpolateState(a, b, t):
    """KinematicChainSpace::interpolate, mlmp/src/state/kinematic_chain.h:112-139. Per
    coordinate: linear when the gap is at most PI, otherwise the short way round, re-wrapped
    into [-PI, PI]. Mirrored branch for branch — see the module docstring for what the naive
    version costs."""
    out = []
    for f, g in zip(a, b):
        diff = g - f
        if abs(diff) <= math.pi:
            out.append(f + diff * t)
        else:
            diff = TWO_PI - diff if diff > 0.0 else -TWO_PI - diff
            v = f - diff * t
            if v > math.pi:
                v -= TWO_PI
            elif v < -math.pi:
                v += TWO_PI
            out.append(v)
    return out


def densifyPath(states, substeps):
    """Every segment of a path result split into `substeps` configurations, plus the goal."""
    if len(states) < 2:
        raise SystemExit("the run has %d path states; nothing to animate" % len(states))
    dense = []
    for a, b in zip(states, states[1:]):
        for s in range(substeps):
            dense.append(interpolateState(a, b, s / float(substeps)))
    dense.append(list(states[-1]))
    return dense


def sceneBounds(scene, links):
    xs, ys = [], []
    for obstacle in scene["obstacles"]:
        for p in obstacle["points"]:
            xs.append(p["x"])
            ys.append(p["y"])
    for robot in scene["robots"]:
        reach = links * robot["jointLength"]
        xs += [robot["pinnedPosition"]["x"] - reach, robot["pinnedPosition"]["x"] + reach]
        ys += [robot["pinnedPosition"]["y"] - reach, robot["pinnedPosition"]["y"] + reach]
    return (min(xs) - PADDING, max(xs) + PADDING, min(ys) - PADDING, max(ys) + PADDING)


def drawStatic(ax, scene, bounds):
    for obstacle in scene["obstacles"]:
        pts = np.array([[p["x"], p["y"]] for p in obstacle["points"]])
        ax.add_patch(Polygon(pts, closed=True, facecolor="gray", edgecolor="black", alpha=0.5))
    for robot in scene["robots"]:
        for angles in (robot["startAngles"], robot["goalAngles"]):
            pts = chainJoints(robot, angles)
            ax.plot(pts[:, 0], pts[:, 1], color="black", alpha=0.25, linewidth=1.0)
            ax.plot(pts[:, 0], pts[:, 1], "o", color="black", alpha=0.25, markersize=2.0)
    ax.set_xlim(bounds[0], bounds[1])
    ax.set_ylim(bounds[2], bounds[3])
    ax.set_aspect("equal", adjustable="box")
    ax.set_xticks([])
    ax.set_yticks([])


class Panel:
    """Everything frame-independent is computed once; a frame is then a prefix slice."""

    def __init__(self, packed, scene, maxLevel):
        self.packed = packed
        self.frames = packed["frames"]
        self.run = packed.get("run", {})
        robots = packed["robots"]
        codes = {int(k): v for k, v in packed["tree_codes"].items()}
        sceneRobots = scene["robots"]

        tips, colors, sizes, isSection = [], [], [], []
        for level, component, *q in packed["vertices"]:
            tree = codes.get(component, "untagged")
            rgb = hexToRgb(TREE_COLORS.get(tree, TREE_COLORS["untagged"]))
            alpha = 0.35 + 0.65 * level / max(1, maxLevel)
            vertexTips = chainTips(q, level, robots, sceneRobots)
            tips.append(vertexTips)
            for _ in vertexTips:
                colors.append(rgb + (alpha,))
                sizes.append(6.0 if tree == "section" else 3.0)
                isSection.append(tree == "section")
        self.robotsPerVertex = len(sceneRobots)
        self.tips = np.array(tips).reshape(-1, 2) if tips else np.zeros((0, 2))
        self.colors = np.array(colors) if colors else np.zeros((0, 4))
        self.sizes = np.array(sizes)
        self.isSection = np.array(isSection, dtype=bool)

        segments, segColors = [], []
        tipsByVertex = tips
        for a, b in packed["edges"]:
            rgb = hexToRgb(TREE_COLORS.get(codes.get(packed["vertices"][a][1], "untagged")))
            for ta, tb in zip(tipsByVertex[a], tipsByVertex[b]):
                segments.append((ta, tb))
                segColors.append(rgb + (0.25,))
        self.segments = np.array(segments) if segments else np.zeros((0, 2, 2))
        self.segColors = np.array(segColors) if segColors else np.zeros((0, 4))

    def draw(self, ax, index):
        if not self.frames:
            return 0.0
        t, nv, ne = self.frames[min(index, len(self.frames) - 1)]
        pts = nv * self.robotsPerVertex
        segs = ne * self.robotsPerVertex
        if segs:
            ax.add_collection(
                LineCollection(
                    self.segments[:segs], colors=self.segColors[:segs], linewidths=0.6
                )
            )
        if pts:
            mask = self.isSection[:pts]
            # `section` last so the findSection() configurations stay legible on both trees.
            for layer in (~mask, mask):
                if layer.any():
                    ax.scatter(
                        self.tips[:pts][layer, 0],
                        self.tips[:pts][layer, 1],
                        c=self.colors[:pts][layer],
                        s=self.sizes[:pts][layer],
                        linewidths=0,
                    )
        return t

    def caption(self):
        run = self.run
        if not run:
            return ""
        status = run.get("status", "?")
        if run.get("solved"):
            return "%s · len %.2f · first %.2f s" % (
                status,
                run.get("path_length_raw") or 0.0,
                run.get("time_first_solution_s") or 0.0,
            )
        return status


def curveLength(points):
    if len(points) < 2:
        return 0.0
    return float(np.sum(np.linalg.norm(np.diff(points, axis=0), axis=1)))


def maxChordDeviation(tips, substeps, segments):
    """How far the densified tip curve leaves the straight chord the raw path states would
    have drawn — the number that says whether densifying was worth it."""
    worst = 0.0
    for k in range(segments):
        a = tips[k * substeps]
        b = tips[(k + 1) * substeps]
        d = b - a
        n = float(np.dot(d, d))
        rel = tips[k * substeps : (k + 1) * substeps + 1] - a
        t = np.clip(rel @ d / n, 0.0, 1.0) if n else np.zeros(len(rel))
        worst = max(worst, float(np.max(np.linalg.norm(rel - np.outer(t, d), axis=1))))
    return worst


def writeGif(rendered, out, msPerFrame):
    """One shared palette, taken from the last (richest) frame: Pillow only delta-encodes GIF
    frames that agree on a palette, and most of either figure is static between frames, so
    sharing it is worth several megabytes."""
    palette = rendered[-1].convert("P", palette=Image.ADAPTIVE, colors=64)
    # dither=0 is Image.Dither.NONE; the int spelling works on Pillow before and after 9.1.
    images = [img.quantize(palette=palette, dither=0) for img in rendered]

    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    images[0].save(
        out,
        save_all=True,
        append_images=images[1:],
        duration=msPerFrame,
        loop=0,
        optimize=True,
    )
    # Report what landed on disk, not what was handed to Pillow: `optimize` collapses runs of
    # identical frames into one with a longer delay (which is how the motion GIF's hold at the
    # goal pose costs nothing), and a GIF stores delays in centiseconds, so the per-frame value
    # is whatever Pillow could round to.
    with Image.open(out) as gif:
        frames = getattr(gif, "n_frames", len(images))
        size = gif.size
        total = 0
        for i in range(frames):
            gif.seek(i)
            total += gif.info.get("duration", msPerFrame)
    print(
        "%s: %d frames, %dx%d, %d ms/frame, %.1f s loop, %d bytes"
        % (out, frames, size[0], size[1], msPerFrame, total / 1000.0, out.stat().st_size)
    )


def renderMotion(args, scene):
    """The solution played back: real arms, a ghost trail, and each tip's own curve."""
    runPath = (
        Path(args.run)
        if args.run
        else Path(args.traces) / f"{args.scene}-{args.planner}.run.json"
    )
    run = json.loads(runPath.read_text())
    if run.get("schema") != "mlmp.run/1":
        raise SystemExit(f"{runPath}: not an mlmp.run/1 result")
    if run.get("model") == "urdf":
        raise SystemExit(f"{runPath}: motion mode draws planar chains; this run is model=urdf")
    if not run.get("solved"):
        raise SystemExit(f"{runPath}: the run did not solve, so there is no path to animate")
    if args.substeps < 1:
        raise SystemExit("--substeps must be at least 1")

    width, height = (int(v) for v in (args.size or "640x600").lower().split("x"))
    robots = run["robots"]
    links = run["links"]
    states = run["path"]
    dense = densifyPath(states, args.substeps)
    sceneRobots = scene["robots"]

    # Precomputed once: a frame is then a lookup, and the tip curves are prefix slices.
    joints = [
        [stateJoints(q, r, robots, links, sceneRobots[r]) for r in range(robots)] for q in dense
    ]
    tips = [np.array([frame[r][-1] for frame in joints]) for r in range(robots)]
    bounds = sceneBounds(scene, links)
    for r in range(robots):
        arc = curveLength(tips[r])
        chordTips = np.array([stateJoints(q, r, robots, links, sceneRobots[r])[-1] for q in states])
        chord = curveLength(chordTips)
        # The length barely moves; the shape does. This line is the evidence that the
        # interpolation above is the wrap-aware one and not a straight chord.
        print(
            "robot %d tip: %.3f along the interpolated curve vs %.3f along straight chords "
            "between the %d path states (%.2fx), max deviation %.3f on a %.2f x %.2f scene"
            % (
                r,
                arc,
                chord,
                len(states),
                arc / chord if chord else 0.0,
                maxChordDeviation(tips[r], args.substeps, len(states) - 1),
                bounds[1] - bounds[0],
                bounds[3] - bounds[2],
            )
        )

    fig, ax = plt.subplots(figsize=(width / 100.0, height / 100.0), dpi=100)
    fig.subplots_adjust(left=0.02, right=0.98, top=0.90, bottom=0.07)
    legend = [
        Line2D(
            [],
            [],
            color=ROBOT_COLORS[r % len(ROBOT_COLORS)],
            label="robot %s" % sceneRobots[r].get("id", r + 1),
        )
        for r in range(robots)
    ]
    legend.append(Line2D([], [], color="black", alpha=0.25, label="start / goal"))
    fig.legend(handles=legend, loc="lower center", ncol=len(legend), frameon=False, fontsize=6)
    title = (
        "%s · %s · %s · len %.2f\n"
        "%d path states densified to %d configurations, %d-D joint space"
    ) % (
        run.get("scene", args.scene),
        run.get("planner", "?"),
        run.get("status", "?"),
        run.get("path_length_simplified") or 0.0,
        len(states),
        len(dense),
        robots * links,
    )

    # The last pose is held for a beat: a GIF restarts without warning, and the reader needs
    # to see the arms arrive before the loop throws them back to the start.
    order = list(range(len(dense))) + [len(dense) - 1] * HOLD_FRAMES
    rendered = []
    for i in order:
        ax.clear()
        drawStatic(ax, scene, bounds)
        for r in range(robots):
            color = ROBOT_COLORS[r % len(ROBOT_COLORS)]
            trace = tips[r][: i + 1]
            if len(trace) > 1:
                ax.plot(trace[:, 0], trace[:, 1], color=color, linewidth=0.9, alpha=0.55)
            for k, g in enumerate(range(max(0, i - GHOST_FRAMES), i)):
                pts = joints[g][r]
                ax.plot(
                    pts[:, 0],
                    pts[:, 1],
                    color=color,
                    linewidth=1.4,
                    alpha=0.05 + 0.20 * (k + 1) / GHOST_FRAMES,
                    solid_capstyle="round",
                )
            pts = joints[i][r]
            ax.plot(
                pts[:, 0], pts[:, 1], color=color, linewidth=2.4, solid_capstyle="round", zorder=4
            )
            ax.plot(pts[:, 0], pts[:, 1], "o", color=color, markersize=3.6, zorder=5)
            ax.plot(pts[0, 0], pts[0, 1], "s", color=color, markersize=4.6, zorder=5)
        ax.set_title(title, fontsize=7)
        fig.canvas.draw()
        buf = np.asarray(fig.canvas.buffer_rgba())
        rendered.append(Image.fromarray(buf, "RGBA").convert("RGB"))

    writeGif(
        rendered,
        args.out or "docs/solution-motion.gif",
        args.ms_per_frame or frameDelay(len(order)),
    )


def frameDelay(frames):
    """Rounded to a whole centisecond because that is all a GIF can store: asking for 59 ms
    writes 50 and the loop silently runs 15% fast. 40 ms floor — browsers clamp delays below
    ~20 ms back up to 100 ms."""
    return max(40, min(90, int(round(TARGET_LOOP_MS / float(frames) / 10.0)) * 10))


def renderTrees(args, scene):
    """The original algorithm picture: two fixtures side by side as chain-tip clouds."""
    width, height = (int(v) for v in (args.size or "640x320").lower().split("x"))
    packedPanels = [
        loadPacked(Path(args.traces) / f"{args.scene}-{planner}.json")
        for planner in ("biqrrt", "qrrt")
    ]

    links = packedPanels[0]["links"]
    bounds = sceneBounds(scene, links)
    panels = [Panel(p, scene, links - 1) for p in packedPanels]
    total = max(len(p.frames) for p in panels)
    if total == 0:
        raise SystemExit("both fixtures are empty")

    fig, axes = plt.subplots(1, 2, figsize=(width / 100.0, height / 100.0), dpi=100)
    fig.subplots_adjust(left=0.01, right=0.99, top=0.86, bottom=0.12, wspace=0.02)
    legend = [
        Line2D([], [], marker="o", linestyle="", color=c, label=name)
        for name, c in TREE_COLORS.items()
    ]
    fig.legend(handles=legend, loc="lower center", ncol=4, frameon=False, fontsize=6)

    rendered = []
    for i in range(total):
        for ax, panel in zip(axes, panels):
            ax.clear()
            drawStatic(ax, scene, bounds)
            t = panel.draw(ax, i)
            ax.set_title(
                "%s · t = %.2f s\n%s" % (panel.packed["planner"], t, panel.caption()),
                fontsize=7,
            )
        fig.canvas.draw()
        buf = np.asarray(fig.canvas.buffer_rgba())
        rendered.append(Image.fromarray(buf, "RGBA").convert("RGB"))

    writeGif(rendered, args.out or "docs/biqrrt-vs-qrrt.gif", args.ms_per_frame or 50)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument(
        "--mode",
        choices=("motion", "trees"),
        default="motion",
        help="motion: animate a run's solution (default). trees: the BiQRRT/QRRT tree clouds.",
    )
    ap.add_argument("--scene", default="scene1")
    ap.add_argument("--planner", default="biqrrt", help="motion: which run record to animate")
    ap.add_argument(
        "--run",
        default=None,
        help="motion: mlmp.run/1 JSON; default {traces}/{scene}-{planner}.run.json",
    )
    ap.add_argument(
        "--out",
        default=None,
        help="default docs/solution-motion.gif (motion), docs/biqrrt-vs-qrrt.gif (trees)",
    )
    ap.add_argument("--traces", default="docs/traces")
    ap.add_argument("--scenes", default="docs/scenes")
    ap.add_argument(
        "--substeps",
        type=int,
        default=24,
        help="motion: interpolated configurations per path segment (default 24)",
    )
    ap.add_argument(
        "--ms-per-frame",
        type=int,
        default=0,
        help="0 = 50 for trees, or whatever fits motion into a ~%d ms loop" % TARGET_LOOP_MS,
    )
    ap.add_argument("--size", default=None, help="default 640x600 (motion), 640x320 (trees)")
    args = ap.parse_args()

    scene = json.loads((Path(args.scenes) / f"{args.scene}.json").read_text())
    if args.mode == "motion":
        renderMotion(args, scene)
    else:
        renderTrees(args, scene)


if __name__ == "__main__":
    try:
        main()
    except SystemExit:
        raise
    except Exception as exc:  # pragma: no cover - CLI convenience
        print("Error: %s" % exc, file=sys.stderr)
        sys.exit(1)
