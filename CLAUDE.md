# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A single C++17 CLI (`mlmp`) that runs 2D multi-robot kinematic-chain motion planning on OMPL 1.7.0,
comparing OMPL's `ompl::multilevel` bundle-space planners and single-level baselines against a custom
bidirectional bundle-space planner, `BiQRRT`. `ROADMAP.md` is the plan of record; read the "Track gate"
section before changing anything the CLI prints.

## Build and run

```sh
# Docker (builds OMPL 1.7.0 from source, then mlmp) — what CI and releases use
docker build -t mlmp:dev .
docker run --rm mlmp:dev --algo BiQRRT --input-file scene1.json --time-limit 10 --seed 42

# Local: needs OMPL 1.7.0 installed from source (distro libompl-dev is too old; EITStar needs 1.7.0)
cmake -S mlmp -B mlmp/build && cmake --build mlmp/build -j
mlmp/build/mlmp -a BiQRRT -i mlmp/src/inputs/experiment/scene1.json -t 10 --seed 42 | python3 -m json.tool
```

The default build type is Release. Only `main.cpp` and `planner/src/BiQRRTImpl.cpp` are compiled;
everything else is header-only. A bare scene name (`scene1.json`) resolves against `MLMP_SCENE_DIR`
(the image sets `/opt/mlmp/scenes`; the `mlmp/src/inputs/*.json` dev scenes go to its `dev/` subfolder).

`mlmp/tools/plotScene.py` renders scenes with matplotlib. It reads `../src/inputs/experiment/`, so run
it from `mlmp/tools/`. `mlmp/tools/renderUrdf.py` is its 3D counterpart: it pipes a `mlmp.run/1`
object from a URDF scene into MuJoCo and writes an MP4 and a GIF. It needs
`pip install 'mujoco>=3.2' imageio imageio-ffmpeg` plus apt `libosmesa6 libgl1`, and is not in
the image.

`docs/` is a dependency-free trace player (`docs/app.js`) over 12 committed `mlmp.traceviz/1`
fixtures in `docs/traces/`, plus the README GIF. `mlmp/tools/recordTraces.sh` re-records them
through the Docker image and `mlmp/tools/packTrace.py`; `mlmp/tools/traceGif.py` renders the GIF
from the committed fixtures alone. A traced run is not byte-reproducible, so the `.run.json`
files beside the fixtures are records of one recording, not pinned targets. All 12 were recorded
with a derivation-capable image, so the BiQRRT fixtures carry `"tree_source": "planner"` and the
QRRT ones `"derived"`.

The same tree is published as an app image (`docker/app/Dockerfile`, `docker/app/Caddyfile`)
by `.github/workflows/apps-image.yml` to `ghcr.io/tomsabala/mlmp:apps-mount-<sha>` and served at
`https://apps.tom-sabala.dev/a/mlmp/`: `site/` at the root, `docs/` under `/explorer/`,
`web/dist` under `/live/`. The container's Caddy strips the `/a/mlmp` prefix, so the slug
appears in `docker/app/Caddyfile` and nowhere else; `docker/app/Dockerfile.dockerignore` is
what lets `COPY web/dist/` see the gitignored build output. Deploying a new build means
bumping the `apps-mount-<sha>` pin in `tom-sabala.dev`'s `frontend/src/apps/apps.json`.
GitHub Pages is not used.

## Tests

There is no unit-test suite. `.github/workflows/ci.yml` is the test, and it runs against the Docker image:
1. **Smoke**: every planner on `scene4.json` for 2 s must exit 0 or 1, never anything else.
2. **Solve**: BiQRRT must solve `scene1.json` within 10 s.
3. **Contract/determinism**: two `--seed 42` runs must produce identical `path_length_raw` and
   `path_length_simplified`, along with schema assertions.
4. **Trace contract**: a 3 s `--trace-file` BiQRRT run must exit 0 or 1 (3 s is below the solve
   budget), and its JSONL must carry an `mlmp.trace/1` header, `len(q) == robots * links` on
   every vertex, `trace.errors == 0`, `trace.frames` matching the line count, both `start` and
   `goal` among the `tree` labels, no `untagged` vertex, and `tree_source == "planner"` on
   every frame. The same step then traces QRRT on the same scene, seed and budget and requires
   `tree_source == "derived"` plus `start`-labelled vertices — `captureFrame` must label the
   graph of a planner that tags nothing.
5. **Bad input**: a missing scene file must exit 2 with `error.kind == "io"`.
6. **URDF solve**: BiQRRT must solve `ur5_scene1.json` within 2 s with `model == "urdf"`, `dof == 6`.
7. **Planar regression pin**: `scene1.json --seed 42` must still report `path_length_raw ==
   6.1990845726570605`. This is the Track D constraint — the planar numbers must not move.

To check a change locally, rebuild the image and run those same `docker run` commands. Only BiQRRT gates
the solve check, because QRRT doesn't solve any scene reliably. Expect `scene4`/`scene6` to exit 1.

## Architecture

**`main.cpp` owns all I/O; `Solver` owns none.** `main.cpp` parses the CLI (boost::program_options),
resolves and loads the scene, opens the trace file, and prints exactly one JSON object to stdout. It
also installs a stderr OMPL log handler, because OMPL's default handler writes to stdout and would
corrupt the output. `mlmp::Solver::solve(const RunOptions&) -> RunResult` (`models/solver.h`) does no
printing or file I/O, so a future Emscripten entry point can call it directly. Keep it that way.

**The output contract is frozen.** `models/run_result.h` defines `RunOptions`, `RunResult`, and the
`mlmp.run/1` serializer. It uses `nlohmann::ordered_json` so key order is kept. The JSONL format is
`mlmp.trace/1`. Renaming or removing a field breaks the parallel ROADMAP tracks, so add fields instead.
On failure the binary prints an error object `{"schema":"mlmp.run/1","error":{kind,message}}` and exits 2.
Exit codes: 0 solved (approximate solutions count; `exact` tells them apart), 1 no solution, 2 usage/io/runtime error.

**Planner set.** `kMultilevelAlgorithms` / `kSingleLevelAlgorithms` in `solver.h` are the single source
of truth: the CLI validates against them and `buildPlanner` / `makeSingleLevelPlanner` dispatch on them.
To add a planner, put it in a set, add a dispatch branch, and add it to the CI smoke loop.

**Bundle hierarchy.** For multilevel planners, `buildPlanner` builds one `KinematicChainSpace` +
`SpaceInformation` + validity checker per abstraction level, using chains with 1..N-1 links, then the
full N-link chain on top. They are passed as `si_vec` to an `ompl::multilevel::BundleSpaceSequence<...>`.
`BiQRRT` is `BundleSpaceSequence<BiQRRTImpl>` (`planner/`), a two-tree subclass of `BundleSpaceGraph`.
Level spaces are kept in `levelSpaceInfos_` so motion-check counts can be summed over every level.
State layout is interleaved by link, so a lower level is a prefix of the higher one:
`values[link * numRobots + robot]`.

**URDF scenes take a parallel path, chosen at load time.** `metadata.model` is `"planar"`
(default) or `"urdf"`; `Scene::loadUrdfScene` handles the latter and the planar branch is left
byte-identical, because the planar scenes are the regression set. A URDF scene supports exactly
one robot. `models/urdf_robot.h` flattens the URDF into a serial chain of actuated joints
(fixed joints folded into neighbouring transforms, branches/meshes/prismatic rejected with
`UnsupportedUrdf` -> `error.kind == "usage"`); `state/urdf_chain.h` is a plain `RealVectorStateSpace`
in joint space, deliberately overriding nothing so `Projection_RN_RM` still makes level k a prefix
of level k+1; `state/urdf_validity_checker.h` does FCL narrowphase, keeping only the shapes of the
first k joints so a level's geometry matches its DOF. `Scene::getN()` returns the actuated joint
count and `getR()` returns 1, so `solve()`'s level sequence needs no model-specific arithmetic.
The whole thing is behind `MLMP_ENABLE_URDF` (default ON, needs `liburdfdom-dev` + `libfcl-dev`);
with it off a URDF scene exits 2 with `error.kind == "usage"`.

**URDF collision resolution is set explicitly, planar is not.** OMPL's default
`longestValidSegmentFraction` of 0.01 over a 6-DOF arm's ~29 rad extent sweeps ~0.14 m between
checks, and a measured BiQRRT solution contained a state its own checker rejects.
`Solver::setUrdfCheckResolution` pins the step to `kUrdfCheckStepRad` (0.02 rad, ~1 cm swept).
The planar path keeps OMPL's default on purpose: changing it moves every pinned planar number.
`BiQRRTImpl::setup()` likewise only forces `maxDistance_ = 0.1` when nobody called `setRange`,
and `Solver` calls `BiQRRT::setRange(maxExtent * 0.05)` for URDF runs only.

**Geometry** (`state/kinematic_chain.h`). `KinematicChainSpace` is a `RealVectorStateSpace` with
custom angle-wrapping `distance`/`interpolate`/`enforceBounds`. The custom distance isn't additive along
interpolation, so `path_length_simplified` can be *greater* than `path_length_raw`. Never assert that
raw >= simplified. `KinematicChainValidityChecker` checks each chain for self-intersection, then robot
against robot, then against obstacle segments. `Scene` closes the obstacles into polygons and adds a
padded bounding box. Scene JSON angles are integer degrees, converted to radians on load.

**Invariants that are easy to break:**
- **Termination must stay non-threaded.** Always call `ss->solve(PlannerTerminationCondition)` with a
  `steady_clock` lambda, never `ss->solve(double)`: OMPL runs budgets of 1 s or more on a `std::thread`,
  and that breaks Emscripten. First-solution timing and trace-frame capture also happen inside this
  lambda, on the planner thread.
- **Seed before any OMPL object exists.** `ompl::RNG::setSeed` runs in `main` before `Scene`/`Solver`,
  because `KinematicChainProjector`'s constructor draws from the global RNG.
- **Seed reproducibility covers path fields only, and only for an untraced run.** Multilevel planners
  use the whole time budget, so vertex, sample, and check counters follow host speed.
  `CountingStateSampler` forwards to the default sampler so that counting doesn't change the RNG
  sequence. Tracing does change it: `captureFrame` calls `BundleSpaceSequence::getPlannerData`, whose
  path-restriction code draws from the global RNG, and captures land on wall-clock boundaries — so a
  traced multilevel run consumes a host-speed-dependent number of draws and its `path_length_raw` is
  not reproducible. Both determinism gates (`ci.yml`, `web/test/smoke.mjs`) therefore compare untraced
  runs.
- **Counters are plain `uint64_t` in `RunCounters`**, since everything is single-threaded. Every level's
  space and checker must be wired to the same `counters_` (use `makeChainSpace`/`makeUrdfSpace`).
  `solve()` resets counters and level handles so it can be called more than once.
- **`captureFrame` needs a fresh `PlannerData` every time**, because `BundleSpaceSequence::getPlannerData`
  throws if the one it gets is already populated. A failed frame counts toward `trace.errors`; it isn't
  a run failure.
- **Tree labels are derived when the planner tags none.** Only `BiQRRTImpl::getPlannerData` sets
  `component`; for every other planner `captureFrame` runs `deriveComponents`, a multi-source BFS
  from `PlannerData`'s start (1) and goal (2) vertices over the frame's adjacency, nearest seed
  winning and start winning ties, leaving 0 where neither endpoint reaches. Component 3
  (`section`) is BiQRRT's `findSection()` tag and is never derived. Each frame reports which case
  it was in `tree_source` (`"planner"` / `"derived"`), a key the viewers surface, so do not drop
  it or let a run emit both values.

`state/oriented_real_vector_state_space.h` isn't included anywhere (dead code). `BiQRRTImpl::setup()`
still defaults `maxDistance_` to `0.1` whenever no caller set a range, and that planar default is a
known open item (ROADMAP Track F / §2.5). If you change it, re-measure the CI solve gate and the
planar regression pin.

## Releases

Pushing a `v*` tag publishes `ghcr.io/tomsabala/mlmp:<tag>` and `:latest` (`.github/workflows/release.yml`).
