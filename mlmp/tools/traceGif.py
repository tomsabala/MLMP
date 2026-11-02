#!/usr/bin/env python3
"""Render the README GIF from committed mlmp.traceviz/1 fixtures: BiQRRT left, QRRT right.

    python3 mlmp/tools/traceGif.py --scene scene1 --out docs/biqrrt-vs-qrrt.gif

Fixtures come from mlmp/tools/recordTraces.sh (record + pack). Rendering rules here are
mirrored verbatim in docs/app.js so the GIF and the trace player cannot disagree. Forward
kinematics follow KinematicChainValidityChecker (mlmp/src/state/kinematic_chain.h:199-222):
cumulative angles, index q[link*robots+robot], one constant jointLength per robot. A fixture
vertex at bundle level L stores only its first L+1 links per robot — the tail above L is
identity-fibre padding, not planner output, and is never drawn.
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
    "path": "#2ca02c",
    "untagged": "#7f7f7f",
}
PADDING = 0.05


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


def chainJoints(robot, anglesDeg):
    x = robot["pinnedPosition"]["x"]
    y = robot["pinnedPosition"]["y"]
    length = robot["jointLength"]
    theta = 0.0
    pts = [(x, y)]
    for a in anglesDeg:
        theta += math.radians(a)
        x += length * math.cos(theta)
        y += length * math.sin(theta)
        pts.append((x, y))
    return np.array(pts)


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

        tips, colors, sizes, isPath = [], [], [], []
        for level, component, *q in packed["vertices"]:
            tree = codes.get(component, "untagged")
            rgb = hexToRgb(TREE_COLORS.get(tree, TREE_COLORS["untagged"]))
            alpha = 0.35 + 0.65 * level / max(1, maxLevel)
            vertexTips = chainTips(q, level, robots, sceneRobots)
            tips.append(vertexTips)
            for _ in vertexTips:
                colors.append(rgb + (alpha,))
                sizes.append(6.0 if tree == "path" else 3.0)
                isPath.append(tree == "path")
        self.robotsPerVertex = len(sceneRobots)
        self.tips = np.array(tips).reshape(-1, 2) if tips else np.zeros((0, 2))
        self.colors = np.array(colors) if colors else np.zeros((0, 4))
        self.sizes = np.array(sizes)
        self.isPath = np.array(isPath, dtype=bool)

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
            mask = self.isPath[:pts]
            # `path` last so the lifted solution stays legible on top of both trees.
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


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--scene", default="scene1")
    ap.add_argument("--out", default="docs/biqrrt-vs-qrrt.gif")
    ap.add_argument("--traces", default="docs/traces")
    ap.add_argument("--scenes", default="docs/scenes")
    ap.add_argument("--ms-per-frame", type=int, default=50)
    ap.add_argument("--size", default="640x320")
    args = ap.parse_args()

    width, height = (int(v) for v in args.size.lower().split("x"))
    scene = json.loads((Path(args.scenes) / f"{args.scene}.json").read_text())
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

    # One shared palette, taken from the last (richest) frame: Pillow only delta-encodes GIF
    # frames that agree on a palette, and the clouds are static apart from the new vertices,
    # so sharing it is worth several megabytes.
    palette = rendered[-1].convert("P", palette=Image.ADAPTIVE, colors=64)
    # dither=0 is Image.Dither.NONE; the int spelling works on Pillow before and after 9.1.
    images = [img.quantize(palette=palette, dither=0) for img in rendered]

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    images[0].save(
        out,
        save_all=True,
        append_images=images[1:],
        duration=args.ms_per_frame,
        loop=0,
        optimize=True,
    )
    print(
        "%s: %d frames, %dx%d, %d bytes"
        % (out, len(images), images[0].size[0], images[0].size[1], out.stat().st_size)
    )


if __name__ == "__main__":
    try:
        main()
    except SystemExit:
        raise
    except Exception as exc:  # pragma: no cover - CLI convenience
        print("Error: %s" % exc, file=sys.stderr)
        sys.exit(1)
