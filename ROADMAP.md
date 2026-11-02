# MLMP Roadmap

Target: turn a coursework planner with unreproducible numbers into a living experiment whose
central result is visible in ten seconds and reproducible from a seed.

The three outcomes a stranger can verify in under a minute, in priority order:

1. A 10-second animation shows two trees growing through a bundle-space hierarchy.
2. A published benchmark states **when BiQRRT wins and when it loses**, with confidence
   intervals and a mechanism.
3. `docker run ghcr.io/tomsabala/mlmp --algo BiQRRT --input-file scene1.json` reproduces a run
   on any machine with Docker.

---

## Phase 0 — Restore a working baseline

`Dockerfile` was pinned to `ubuntu:23.04`, EOL since January 2024, so `apt update` and every step
after it failed: the only documented entry point had been dead for two years. It is now a
two-stage build on `ubuntu:24.04` whose runtime image carries a compiled binary and the benchmark
scenes, so nothing is compiled to try it:

```sh
docker run --rm ghcr.io/tomsabala/mlmp:latest --algo BiQRRT --input-file scene1.json --time-limit 10
```

- **OMPL pinned to 1.7.0** (was 1.6.0). `src/ompl/multilevel` is identical between the two but
  for one fix in `BundleSpaceSequenceImpl.h` — `solve` no longer ends the last bundle level early
  on `foundKLevelSolution_` — which helps BiQRRT and QRRT equally; 1.7.0 is also the only release
  with `EITstar`. No Ubuntu release ships it, hence the source build in the builder stage.
- **Dead dependencies removed**: CGAL (referenced by no source file) and its `-frounding-math`;
  `src/common/segment.h`, a stale pre-refactor copy of `geometry.h`; `nlohmann_json` now comes
  from the distribution rather than a `FetchContent` network fetch on every configure.
- **12 planners behind one CLI contract** (see Phase 2), with `mlmp/src/models/solver.h` as the
  single source of truth for the list. Exit codes `0` solved / `1` no solution in budget / `2`
  usage or IO error make a broken binary distinguishable from a hard scene.
- **A segfault on `scene1` is gone.** `BiQRRTImpl::init()` overrides the base `init()` without
  assigning `BundleSpaceGraph::vStart_`, which has no initialiser; `getSolution()` indexes the
  graph with it. It worked only when the uninitialised value happened to be 0, so the crash was
  deterministic per scene. `init()` now calls `setStartIndex`.
- **CI** builds the image, smoke-runs every planner, and checks the solve and bad-input cases on
  every push; a `v*` tag publishes to GHCR. The solve gate is BiQRRT on `scene1`, measured 3/3 at
  a 10-second budget.

---

## Track gate — the `Solver` contract

Phase 0 left one file in the way of everything else: four of the six tracks below want
incompatible edits to the same 45 lines of `mlmp/src/models/solver.h`. The gate lands
first, alone, and fixes the contract the tracks code against. Renaming a field after this
point is a cross-track break.

- `Solver::solve(const RunOptions&) -> RunResult`. No printing, no `boost::program_options`,
  no file IO inside the solver: `main.cpp` owns the CLI and the output stream, so an
  Emscripten entry point can call the same function.
- **Non-threaded termination.** The budget is a `PlannerTerminationCondition` built from a
  `std::function` polling `steady_clock`, never `ss.solve(double)`, which routes any budget
  ≥ 1 s to a `std::thread` (`Planner.cpp:142-144`) that Emscripten cannot have without
  COOP/COEP headers GitHub Pages will not set.
- **One JSON object on stdout**, logs on stderr (OMPL's default handler writes INFO/DEBUG to
  stdout and would corrupt it). Exit codes unchanged: 0 solved, 1 no solution in budget,
  2 usage or IO error.
- **`--seed`**, applied via `ompl::RNG::setSeed` before any RNG exists, and echoed back in
  the result whether it was given or generated.
- **Raw and simplified path length as separate fields**; `--no-simplify` skips the
  simplifier. Reporting only the simplified length measured the simplifier as much as the
  planner — and measurably misreported it: under `KinematicChainSpace`'s custom non-Euclidean
  `distance()`, `simplifyMax` can return a *longer* summed length than the raw path
  (`scene1`, seed 7: raw 8.81, simplified 10.84), because B-spline smoothing inserts states
  and this metric is not additive along interpolation. `path_length_raw >= path_length_simplified`
  is therefore not an invariant and must not be asserted.
- **`time_first_solution_s` for every planner**, by polling `hasExactSolution()` in the
  termination condition. The intermediate-solution callback only fires for optimizing
  planners, and since 1.7.0 the multilevel planners report the whole budget as `Runtime`.
- **A trace hook.** `RunOptions::traceSink` gets one JSON frame per interval, captured
  inside the termination-condition poll — the planner's own thread, between iterations —
  so no locking and no half-mutated graph.
- **Counters that survive the hierarchy**: validity checks and samples are counted on
  *every* bundle level, not only the top space.
- Two latent defects fixed on the way: the file-scope `std::vector<Environment> envs` grew
  by `n` copies per `solve()` and handed out `&envs.at(...)` pointers that dangle after the
  next push_back, so a second solve in one process was unsafe — which Tracks B and E both
  need; and all levels shared identical obstacle copies anyway, so one `Environment` member
  replaces the vector.

### `mlmp.run/1` — stdout, one object

`schema`, `planner`, `scene`, `scene_path`, `robots`, `links`, `seed`, `time_limit_s`,
`ompl_version`, `solved`, `exact`, `status`, `time_solve_s`, `time_first_solution_s`,
`time_simplify_s`, `solution_events[{t_s,cost}]`, `path_length_raw`,
`path_length_simplified`, `path_states`, `path`, `graph_vertices`, `graph_edges`,
`state_validity_checks`, `states_sampled`, `motion_checks`, `motion_checks_valid`,
`trace{file,frames,truncated,errors}`.

`status` is one of `unknown`, `invalid_start`, `invalid_goal`, `unrecognized_goal_type`,
`timeout`, `approximate_solution`, `exact_solution`, `crash`, `abort`, `infeasible`.
`solved` is true for approximate solutions too; `exact` is the field that separates them.
Failures emit `{"schema":"mlmp.run/1","error":{"kind":"usage"|"io"|"runtime","message":…}}`
and exit 2. No stdout at all means the process crashed — the fourth failure mode §2.2 asks
for.

**What is reproducible under a fixed seed, and what is not.** The multilevel planners consume
the entire `--time-limit` (`time_solve_s` is 10.001 for `--time-limit 10`), so iteration-count
fields — `graph_vertices`, `graph_edges`, `state_validity_checks`, `states_sampled`,
`motion_checks` — track host speed, not the seed, and vary by a few percent between identical
invocations. The *solution* is seed-determined: `path_length_raw` and `path_length_simplified`
reproduce exactly across runs and across budgets (`scene1`, seed 42: raw 6.199084573 at both
`--time-limit 3` and `10`; seeds 7 and 99 give different lengths). A harness comparing runs
must compare the path fields, not the counters.

### `mlmp.trace/1` — `--trace-file`, JSONL

Line 1 is a header (`schema`, `planner`, `scene`, `seed`, `robots`, `links`,
`time_limit_s`, `interval_ms`); every later line is a frame:

```json
{"frame":7,"t_s":0.35,"vertices":[{"level":0,"component":0,"start":true,"goal":false,"q":[0.1,-0.4]}],"edges":[[0,1]]}
```

Every vertex carries a full-dimension state: `BundleSpaceSequence::getPlannerData` lifts
each level's states into the total space and tags them with `setLevel`. `--trace-interval-ms`
(default 50) and `--trace-max-frames` (default 400) bound the output; hitting the cap sets
`trace.truncated`. Measured on `scene1`/BiQRRT at a 10 s budget: 197 frames spanning levels
0–3, every `q` of full dimension 8, `trace.errors` 0.

---

## Parallel tracks

Six tracks, concurrent after the gate. Each entry states what it owns, what it may not
touch, and what makes it done.

### Track A — Trace player and GIF (Phase 1)

- **Scope.** Consume `mlmp.trace/1`: 10-second looping GIF for the README, and a static
  scrubber page that steps frames, toggles bundle levels, and runs BiQRRT beside QRRT on the
  same seed. Extend `BiQRRTImpl::getPlannerData` to tag start-tree versus goal-tree
  membership — today it only chains to the base and logs the two tree sizes, and the generic
  `component` field does not separate them.
- **Requires.** The gate. Nothing else.
- **Constraints.** No solver in the browser — that is Track E, and conflating them costs
  megabytes. Static JSON on GitHub Pages, no toolchain, no hosting. A traced run is never a
  timing run: frame capture copies the whole graph. The QRRT side-by-side stays unconvincing
  until Track F lands, since QRRT solved no scene 3/3 at a 10 s budget.
- **Done when.** The GIF is in the README and the player replays both planners from committed
  trace files.

### Track B — Experiment harness (§2.1–2.4)

- **Scope.** New `mlmp/tools/experiment.py`: one planner per process, serialized, pinned with
  `taskset`, host specs recorded, reading `mlmp.run/1` from stdout instead of regex. n ≥ 50
  per cell, bootstrap 95% CIs, the four failure modes kept distinct (exit 1 = no solution,
  exit 2 + `error.kind`, other exit = crash, unparseable stdout = parse error). Both time
  metrics from §2.4, which the gate already emits.
- **Requires.** The gate. Track F before the full campaign.
- **Constraints.** Never two timed planners at once — that is the defect that invalidated the
  old numbers. Never report a run that used `--trace-file`. Never resurrect `results.json`.
  Running the campaign before Track F's `maxDistance_` parameter confounds the BiQRRT/QRRT
  gap with a hardcoded step size and buys a rerun. Reproducibility is asserted on the path
  fields, not the iteration counters — see the gate's note on what a fixed seed does and does
  not pin down.
- **Done when.** `python3 mlmp/tools/experiment.py --seed 42` twice produces identical tables
  and CI regenerates a small subset on every push.

### Track C — Procedural scenes (§2.6)

- **Scope.** Scene generator with a computed difficulty metric (narrow-passage width, C-space
  obstacle ratio, chain DOF, inter-robot coupling, start/goal homotopy distance); 500+ scenes
  from a seed; the six originals kept as named regression scenes.
- **Requires.** Nothing. The scene JSON schema is untouched by the gate.
- **Constraints.** Shares the scene schema with Track D: any schema change is agreed with
  Track D first and both land together. Scenes are generated from a seed, never committed as
  blobs.
- **Done when.** The generator produces a scene set spanning the difficulty range from a seed,
  and `Scene::loadScene` reads it unmodified.

### Track D — URDF and 3D (Phase 5)

- **Scope.** URDF loading, Panda or UR5, 3D chains, heterogeneous teams, more than two agents,
  MuJoCo or Drake rendering.
- **Requires.** Nothing. The gate's `Solver::makeChainSpace`/`buildSetup` split is the seam:
  one place constructs the state space.
- **Constraints.** Scene-schema coordination with Track C. The existing planar scenes must
  keep producing identical results — they are the regression set. The collision checker stays
  as it is unless profiling names it (see Non-goals).
- **Done when.** A URDF-described arm plans and renders through the same CLI contract.
- **Status.** Delivered for one 6-DOF UR5 (`R=1`): `mlmp -a BiQRRT -i ur5_scene1.json` loads
  the vendored `inputs/robots/ur5.urdf`, plans in 6-D joint space over the same bundle
  hierarchy with FCL collision checking, and prints `mlmp.run/1` with
  `model`/`dof`/`robot_model`/`robot_urdf` added; `mlmp/tools/renderUrdf.py` turns that
  stdout into an MP4 and a GIF. Still open: heterogeneous teams and multi-agent URDF scenes
  (the interleaved `values[link * numRobots + robot]` layout and the `Projection_RN_RM`
  prefix invariant make them a separate design — the loader rejects `robots.size() != 1`),
  and branched URDFs such as a Panda gripper (the single-chain loader throws
  `UnsupportedUrdf`).

### Track E — Run the planner in the browser (Phase 6)

- **Scope.** Emscripten build, Web Worker, in-page scene editor; drag an obstacle and watch
  the hierarchy re-solve.
- **Requires.** The gate (non-threaded termination, no `program_options` inside `Solver`).
- **Constraints.** Single-threaded; no COOP/COEP headers available. Boost.Filesystem and
  Boost.Serialization must be cross-compiled or trimmed out of the OMPL build. Measure the
  compressed payload, do not estimate. Replay is Track A's and must not be rebuilt here.
- **Done when.** A visitor edits a scene, picks a planner, and sees the trees grow, on a
  static host that sets no special headers.

### Track F — Planner hygiene (§2.5)

- **Scope.** Make `maxDistance_` a declared `Planner::declareParam` with a `SelfConfig`-derived
  default instead of the hardcoded `0.1` at `BiQRRTImpl.cpp:29`, and answer whether the
  BiQRRT/QRRT gap is real or a step-size artefact (QRRT's `SelfConfig` default logged as
  1.78–3.55 across `scene1`'s levels).
- **Requires.** The gate. The raw-versus-simplified half of §2.5 is already delivered by it.
- **Constraints.** Must land before Track B's campaign. The new default changes BiQRRT
  behaviour, so re-measure the CI solve gate (`scene1`, 3/3 at 10 s) before merging.
- **Done when.** The step size is a parameter with a scale-derived default and the gap has a
  measured explanation.

### What cannot be parallelised

The gate precedes A, B, E and F. Track F precedes Track B's full campaign. The campaign
precedes Phase 3, and Phase 3 precedes Phase 4 — learned sampling without rigorous baselines
is an unfalsifiable claim, which is the whole reason Phase 2 exists.

---

## Phase 1 — Visualize the bundle hierarchy

The data already exists in `BiQRRTImpl::getPlannerData`.

- Dump planner data per iteration (vertices, edges, tree membership, bundle level).
- Animate: both trees growing from start and goal, bundle levels stacked so the coarse
  level visibly solves first and then **restricts** the fine level's sampling, and the
  frame where the two trees connect and the path collapses out.
- Side-by-side with QRRT on the same scene and seed — the bidirectional advantage becomes
  something you can see rather than something a table asserts.
- 10-second looping GIF at the top of the README.
- **A trace player**, which is the GIF with a scrubber. The per-iteration dump above is the
  whole input: a static page replays it, steps frame by frame, toggles bundle levels, and runs
  BiQRRT and QRRT side by side on the same seed. No solver in the browser, so it is a few
  hundred KB of JSON on GitHub Pages with no toolchain and no hosting requirements. Running the
  planner live in the browser is Phase 6 and costs vastly more; do not conflate them.

Multilevel planning is genuinely beautiful and almost nobody renders the bundle hierarchy.

---

## Phase 2 — Rebuild the experiment harness

**The current results are not merely old. They are invalid.** This must be fixed before
any new claim is made, because every downstream phase depends on trustworthy baselines.

### 2.1 What was wrong with `mlmp/tools/experiment.py`

The gate deleted this script, `plotResults.py` and `results.json`; the table is the
acceptance list for the replacement, not a description of live code.

| Defect | Evidence |
|---|---|
| Results are not reproducible by the committed code | `scenes = ['scene2.json']`, but `results.json` contains `scene1.json`–`scene5.json` |
| `plot()` cannot run | `analyzeResults` builds `{time_limit: …}` with **int** keys; `plot()` reads `analysis[...][f'{time_limit}']`, a **str** key → `KeyError`. It was written against a JSON round-trip, never against its own output |
| Timing measurements are meaningless | All 125 runs (5 algos × 5 budgets × 5 reps) are submitted to one `ThreadPoolExecutor` at once, then wall-clock `Runtime` is parsed from stdout. Every planner is timed while competing with every other planner for CPU |
| No statistical power | `repetitions = 5`, bare `np.mean`, no variance, no CIs. `success_rate: 0.6` is 3-of-5 |
| Failures are invisible | `main()` catches every exception and prints it; `runAlgorithm` returns `(None, None)` for parse failure, timeout, and crash alike — indistinguishable |
| Results parsed by regex from stdout | `re.search(r'Runtime\n([\d.]+)', output)` |

### 2.2 Replacement

- **One planner per process, serialized.** No concurrency during a timed run. Pin to a CPU
  (`taskset`), disable turbo/boost drift where possible, record host specs in the output.
- **Structured output.** Delivered by the gate: the binary emits `mlmp.run/1` on stdout, no
  regex.
- **Seeded and reproducible.** Delivered by the gate: `--seed` reaches `ompl::RNG::setSeed`
  before any RNG exists, and the seed is echoed back whether given or generated. Note what a
  seed pins down — the solution path, not the iteration counters; see the Track gate.
- **n ≥ 50 per cell**, bootstrap 95% confidence intervals on every reported statistic.
- **Distinguish failure modes.** Timeout, no-solution-found, crash, and parse error are
  four different outcomes, not one `None`.
- Consider `ompl::tools::Benchmark` + the Planner Arena log format instead of hand-rolling
  — it gives the statistical machinery and a known output format for free.
- Delete `tools/results.json` and regenerate it. Do not carry invalid numbers forward.
  Delivered by the gate: the file is gone.

**Done when:** `python3 mlmp/tools/experiment.py --seed 42` twice produces identical tables,
and CI regenerates a small subset on every push.

### 2.3 Planner set

The comparison spans 12 planners: multilevel `BiQRRT, QRRT, QRRTStar, QMP, QMPStar`;
single-level `RRT, RRTConnect, PRMStar, BITStar, AITStar, EITStar, LBKPIECE`. All 12 are already
reachable from the CLI (Phase 0); what is missing is the harness that uses them.

A multilevel-only table cannot attribute a BiQRRT result to bidirectionality versus the bundle
hierarchy; `RRT`/`RRTConnect`/`QRRT`/`BiQRRT` isolates both factors, and the optimal planners are
the only honest comparison for `QRRTStar`/`QMPStar` path lengths.

### 2.4 Anytime planners need two time metrics

`BITStar`, `AITStar`, `EITStar`, `PRMStar` and `QRRTStar` consume the whole budget by design, so a
single wall-clock number is meaningless against them: record time-to-first-solution and final
cost separately (`ProblemDefinition::setIntermediateSolutionCallback`, or
`ompl::tools::Benchmark`'s progress properties).

Two measurements from Phase 0 make this concrete, both from 3 repetitions per cell on one host:

- **Every multilevel planner now reports the full budget as its runtime.** With the 1.7.0 fix the
  last bundle level keeps growing until the termination condition, so a solved 10-second run
  prints `Runtime 10.03`. The reported number is the budget, not the solve time — exactly the
  metric this section replaces.
- **No scene gives QRRT a reliable solve.** QRRT was 0/3 on all six scenes at a 10-second budget
  and its best result at 30 s was 2/3 on `scene1`; BiQRRT was 3/3 on `scene1` and `scene5` at
  10 s. The CI solve gate is therefore BiQRRT-only. Whether that gap is real or an artefact of
  the step sizes — BiQRRT hardcodes `maxDistance_ = 0.1` while QRRT keeps the `SelfConfig`
  default, logged as 1.78–3.55 across the levels of `scene1` — is the first question this
  harness has to answer, and it is the reason 2.5 is not optional.

### 2.5 Planner hygiene

- `BiQRRTImpl::setup()` hardcodes `maxDistance_ = 0.1`; make it a declared parameter
  (`Planner::declareParam`) with a `SelfConfig`-derived default, since the magic number is tied
  to the current scene scale.
- `Solver::solve` ran `simplifyMax` on every solution before reporting length, which measured
  the simplifier as much as the planner. **Delivered by the gate:** `path_length_raw` and
  `path_length_simplified` are separate fields and `--no-simplify` skips the simplifier.

### 2.6 Scale the scene set

Six hand-authored JSON scenes on the unit square with 4-joint arms and 2 robots was a
scope forced by hand-authoring. It no longer is.

- Procedural scene generator with a **computed difficulty metric**: narrow-passage width,
  C-space obstacle ratio, chain DOF, inter-robot coupling, start/goal homotopy distance.
- 500+ scenes spanning the difficulty range, generated from a seed, not committed as blobs.
- Keep the six originals as named regression scenes.

---

## Phase 3 — The finding

A benchmark that only produces plots is a plot dump. State a rule and a mechanism:

> BiQRRT dominates QRRT* below narrow-passage ratio X and loses above it, because
> bidirectional growth wastes samples when the goal tree cannot escape the restriction
> imposed by the coarse level.

Requirements: the claim is falsifiable, the failure region is stated explicitly, and the
numbers behind it regenerate from a seed. Publish as a short technical write-up — OMPL
discourse, arXiv preprint, or a post. Unread work convinces nobody; distribution is most
of the difference between a project with zero stars and one people cite.

---

## Phase 4 — Learned sampling

Only after Phase 2 and 3. Rigorous baselines are what make this credible; doing it first
produces an unfalsifiable ML claim.

- Train a sampler on solved instances to predict promising bundle-space restrictions.
- Report the delta against the Phase 2 baselines on held-out scenes, same metrics, same CIs.
- Negative results are publishable here and more credible than a tuned win.

---

## Phase 5 — Make it look like a robot

4-joint planar chains in a unit square read as a toy regardless of the algorithm's quality.

- URDF loading; plan for a Panda or UR5.
- 3D chains, heterogeneous robot teams, more than two agents.
- Render in MuJoCo or Drake.

Same algorithms, different perceived seriousness.

---

## Phase 6 — Run the planner in the browser

The Phase 1 trace player replays a run that was computed elsewhere. This phase lets a visitor
drag an obstacle and watch the hierarchy re-solve — the one capability a replay cannot fake, and
the reason to pay for it.

The dependency surface is already small enough to try. `ldd` on the shipped binary lists exactly
three non-system libraries — `libompl`, `libboost_filesystem`, `libboost_serialization` — plus
`libboost_program_options`; there is no CGAL, no X11, no OpenGL and no GPU. Four obstacles, in
the order they bite:

- **`solve(double)` spawns a thread.** `Planner.cpp:142-144` routes any budget `>= 1.0` s to the
  `std::thread`-backed `PlannerThreadedTerminationCondition`. Emscripten pthreads need
  `SharedArrayBuffer`, which needs COOP/COEP response headers, which **GitHub Pages cannot
  set**. Fix on our side instead: call `solve(const PlannerTerminationCondition &)` with a
  non-threaded condition, which is a handful of lines in `Solver::solve` and keeps the build
  single-threaded.
- **Boost has no Emscripten port.** `libompl` links Boost.Filesystem and Boost.Serialization
  itself; only `PlannerDataStorage` and `Benchmark` use them and we call neither, so either
  cross-compile both for wasm or trim them out of the OMPL build.
- **Payload.** `libompl.so.1.7.0` is 11.3 MB native. Dead-code elimination against the handful
  of planners actually registered should cut that hard, but budget for megabytes, and measure
  the compressed transfer rather than guessing.
- **A blocking solve freezes the tab.** The planner occupies its whole budget by design, so it
  belongs in a Web Worker. That is orthogonal to pthreads and needs no special headers.

`Scene::loadScene` takes a filename and an `ifstream`, which the Emscripten virtual filesystem
satisfies unchanged, so the scene editor can write a file and reuse the existing loader rather
than growing a second parse path. `boost::program_options` is dead weight in this build; the
entry point becomes an embind function, not `main`.

**Done when:** a visitor edits a scene in the browser, picks a planner, and sees the trees grow
without a server, on a static host that sets no special headers.

---

## Non-goals

- LLM features. This is a planning problem.
- Reimplementing OMPL. Extend it.
- Upstreaming BiQRRT into `ompl::multilevel`. The repo is the artifact; a maintainer review
  cycle is not a goal of this project.
- Integrating CGEO, or writing a geometry kernel for this project. The collision checker in
  `mlmp/src/state/kinematic_chain.h:243` is a Stack Overflow–cited segment intersection on raw
  doubles sitting in `isValidImpl`, the hottest loop in the planner, and
  `mlmp/src/common/geometry.h` is two structs of naked doubles. If that ever shows up as a
  measured bottleneck, the answer is an established computational-geometry library, not a
  personal one — there are better maintained kernels than anything written here.
- Carrying `tools/results.json` forward. Those numbers were produced under thread
  contention by a script that cannot regenerate them.
