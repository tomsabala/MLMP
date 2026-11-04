#!/usr/bin/env python3
"""Render an mlmp.run/1 URDF solution as an MP4 and a GIF.

The 3D counterpart of plotScene.py. It consumes the frozen mlmp.run/1 object, so it
needs no C++ change and cannot affect a timed run.

    mlmp -a BiQRRT -i ur5_scene1.json -t 2 --seed 42 > run.json
    python3 renderUrdf.py --run run.json --out-prefix /tmp/ur5
    mlmp ... | python3 renderUrdf.py --out-prefix /tmp/ur5     # run object on stdin

A run produced by `docker run mlmp` records the paths it saw inside the container, which
do not exist on the host; point the tool at the checkout instead:

    docker run --rm mlmp:dev -a BiQRRT -i ur5_scene1.json -t 2 --seed 42 > run.json
    python3 mlmp/tools/renderUrdf.py --run run.json --out-prefix /tmp/ur5 \
        --urdf mlmp/src/inputs/robots/ur5.urdf \
        --scene mlmp/src/inputs/experiment/ur5_scene1.json

Requirements (not baked into the Docker image; like plotScene.py this runs from a
checkout):
    pip install 'mujoco>=3.2' imageio imageio-ffmpeg
    apt-get install libosmesa6 libgl1

MuJoCo is only a camera here. FCL, inside mlmp, is the authoritative collision model:
a rendered frame that looks like a graze is not a bug unless FCL also rejects it.
"""

import argparse
import json
import math
import os
import sys

import numpy as np

# Must be set before mujoco is imported, or the offscreen context picks EGL/GLFW and
# fails on a headless host.
os.environ.setdefault("MUJOCO_GL", "osmesa")

try:
    import mujoco
    import imageio.v2 as imageio
except ImportError as exc:  # pragma: no cover - environment problem, not logic
    sys.exit(
        f"{exc}\n"
        "pip install 'mujoco>=3.2' imageio imageio-ffmpeg; needs libosmesa6 and libgl1"
    )

MAX_FRAMES = 900
JOINT_STEP_RAD = 0.02


def quat_from_rpy(rpy):
    """URDF fixed-axis roll-pitch-yaw -> MuJoCo (w, x, y, z).

    Same composition order as mlmp::poseFromXyzRpy (Z * Y * X); using a different one
    here would silently desynchronise the picture from the collision model.
    """
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ]


def add_obstacles(spec, obstacles):
    """Inject the scene's obstacles3d into the model's worldbody.

    MuJoCo takes half-extents for a box and [radius, half-length] for a cylinder; the
    scene file uses URDF conventions (full size, full length).
    """
    for obstacle in obstacles:
        kind = obstacle["type"]
        pose = obstacle.get("pose", {})
        geom = spec.worldbody.add_geom()

        if kind == "box":
            geom.type = mujoco.mjtGeom.mjGEOM_BOX
            geom.size = [s / 2.0 for s in obstacle["size"]]
        elif kind == "sphere":
            geom.type = mujoco.mjtGeom.mjGEOM_SPHERE
            geom.size = [obstacle["radius"], 0.0, 0.0]
        elif kind == "cylinder":
            geom.type = mujoco.mjtGeom.mjGEOM_CYLINDER
            geom.size = [obstacle["radius"], obstacle["length"] / 2.0, 0.0]
        else:
            sys.exit(f"unknown obstacle3d type: {kind}")

        geom.pos = pose.get("xyz", [0.0, 0.0, 0.0])
        geom.quat = quat_from_rpy(pose.get("rpy", [0.0, 0.0, 0.0]))
        geom.rgba = [0.8, 0.3, 0.3, 1.0]


CAMERA_POS = [1.4, -1.4, 1.1]
CAMERA_TARGET = [0.0, 0.0, 0.4]


def look_at_quat(eye, target, up=(0.0, 0.0, 1.0)):
    """Camera orientation (w, x, y, z) that puts `target` in the middle of the frame."""
    eye = np.asarray(eye, dtype=float)
    forward = np.asarray(target, dtype=float) - eye
    forward /= np.linalg.norm(forward)
    z_axis = -forward
    x_axis = np.cross(np.asarray(up, dtype=float), z_axis)
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)

    rotation = np.column_stack((x_axis, y_axis, z_axis))
    quat = np.empty(4)
    mujoco.mju_mat2Quat(quat, rotation.flatten())
    return quat.tolist()


def build_model(run, scene, width, height):
    spec = mujoco.MjSpec.from_file(run["robot_urdf"])

    # MuJoCo's offscreen framebuffer defaults to 640x480 and the Renderer refuses any
    # larger image, so the model has to be told the render size before it is compiled.
    spec.visual.global_.offwidth = width
    spec.visual.global_.offheight = height

    robot = scene["robots"][0]
    base = robot.get("basePose")
    if base and spec.worldbody.bodies:
        root = spec.worldbody.bodies[0]
        root.pos = base.get("xyz", [0.0, 0.0, 0.0])
        root.quat = quat_from_rpy(base.get("rpy", [0.0, 0.0, 0.0]))

    add_obstacles(spec, scene.get("obstacles3d", []))

    # Without an explicit light the render comes out nearly black: MuJoCo's headlight
    # alone leaves a mean pixel value under 4/255 for this model.
    light = spec.worldbody.add_light()
    light.type = mujoco.mjtLightType.mjLIGHT_DIRECTIONAL
    light.pos = [0.0, 0.0, 2.5]
    light.dir = [0.0, 0.0, -1.0]

    # MjSpec cameras take a quaternion, not MJCF's xyaxes, so the look-at basis is built
    # here: MuJoCo points a camera down its own -z with +y up.
    camera = spec.worldbody.add_camera()
    camera.name = "iso"
    camera.pos = CAMERA_POS
    camera.quat = look_at_quat(CAMERA_POS, CAMERA_TARGET)

    return spec.compile()


def waypoint_frames(path):
    """Linear interpolation in joint space, uniformly subsampled past the frame cap."""
    frames = []
    for a, b in zip(path, path[1:]):
        a = np.asarray(a, dtype=float)
        b = np.asarray(b, dtype=float)
        steps = max(2, math.ceil(float(np.max(np.abs(b - a))) / JOINT_STEP_RAD))
        for i in range(steps):
            frames.append(a + (b - a) * (i / steps))
    frames.append(np.asarray(path[-1], dtype=float))

    if len(frames) > MAX_FRAMES:
        idx = np.linspace(0, len(frames) - 1, MAX_FRAMES).round().astype(int)
        frames = [frames[i] for i in idx]
    return frames


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", help="mlmp.run/1 JSON file; stdin when omitted")
    parser.add_argument("--out-prefix", required=True,
                        help="writes <prefix>.mp4 and <prefix>.gif")
    # A run produced by `docker run mlmp` records container paths
    # (/opt/mlmp/scenes/...), which do not exist on the host; these override them.
    parser.add_argument("--urdf", help="override robot_urdf from the run object")
    parser.add_argument("--scene", help="override scene_path from the run object")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    args = parser.parse_args()

    with open(args.run) if args.run else sys.stdin as handle:
        run = json.load(handle)

    if "error" in run:
        sys.exit(f"run failed: {run['error']}")
    if run.get("model") != "urdf":
        sys.exit(f"not a URDF run: model={run.get('model')!r}")
    if not run.get("solved") or not run.get("path"):
        sys.exit(f"nothing to render: solved={run.get('solved')} status={run.get('status')}")

    if args.urdf:
        run["robot_urdf"] = args.urdf
    scene_path = args.scene or run["scene_path"]
    for label, path in (("robot_urdf", run["robot_urdf"]), ("scene_path", scene_path)):
        if not os.path.exists(path):
            sys.exit(f"{label} does not exist here: {path}\n"
                     "A run produced inside the Docker image records container paths; "
                     "pass --urdf and --scene to point at the checkout.")

    with open(scene_path) as handle:
        scene = json.load(handle)

    model = build_model(run, scene, args.width, args.height)
    data = mujoco.MjData(model)
    frames = waypoint_frames(run["path"])

    images = []
    with mujoco.Renderer(model, args.height, args.width) as renderer:
        for q in frames:
            data.qpos[: model.nq] = q[: model.nq]
            mujoco.mj_forward(model, data)
            renderer.update_scene(data, camera="iso")
            images.append(renderer.render())

    mp4 = f"{args.out_prefix}.mp4"
    gif = f"{args.out_prefix}.gif"
    imageio.mimsave(mp4, images, fps=30)
    imageio.mimsave(gif, images, fps=20, loop=0)

    mean = float(np.mean(images[len(images) // 2]))
    print(f"{mp4} {gif} frames={len(images)} mean_pixel={mean:.1f}")


if __name__ == "__main__":
    main()
