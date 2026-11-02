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
it from `mlmp/tools/`.

## Tests

There is no unit-test suite. `.github/workflows/ci.yml` is the test, and it runs against the Docker image:
1. **Smoke**: every planner on `scene4.json` for 2 s must exit 0 or 1, never anything else.
2. **Solve**: BiQRRT must solve `scene1.json` within 10 s.
3. **Contract/determinism**: two `--seed 42` runs must produce identical `path_length_raw` and
   `path_length_simplified`, along with schema assertions.
4. **Bad input**: a missing scene file must exit 2 with `error.kind == "io"`.

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
  space and checker must be wired to the same `counters_` (use `makeChainSpace`). `solve()` resets
  counters and level handles so it can be called more than once.
- **`captureFrame` needs a fresh `PlannerData` every time**, because `BundleSpaceSequence::getPlannerData`
  throws if the one it gets is already populated. A failed frame counts toward `trace.errors`; it isn't
  a run failure.

`state/oriented_real_vector_state_space.h` isn't included anywhere (dead code). `BiQRRTImpl::setup()`
hardcodes `maxDistance_ = 0.1`, and that is a known open item (ROADMAP Track F / §2.5). If you change it,
re-measure the CI solve gate.

## Releases

Pushing a `v*` tag publishes `ghcr.io/tomsabala/mlmp:<tag>` and `:latest` (`.github/workflows/release.yml`).
