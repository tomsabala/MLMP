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

`path` is the state sequence of the *simplified* path: `simplifyMax` runs in place on the
solution before the states are copied out, so `path` corresponds to `path_length_simplified`
unless `--no-simplify` was given.

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
reproduce exactly across runs and across budgets (`scene1`, seed 42: raw 20.627278999 at both
`--time-limit 3` and `10`; seeds 7 and 99 give different lengths). A harness comparing runs
must compare the path fields, not the counters.

### `mlmp.trace/1` — `--trace-file`, JSONL

Line 1 is a header (`schema`, `planner`, `scene`, `seed`, `robots`, `links`,
`time_limit_s`, `interval_ms`); every later line is a frame:

```json
{"frame":7,"t_s":0.35,"tree_source":"planner","vertices":[
  {"level":0,"component":1,"tree":"start","start":true,"goal":false,"q":[0.1,-0.4,0.0,0.0]},
  {"level":0,"component":2,"tree":"goal","start":false,"goal":true,"q":[0.9,1.2,0.0,0.0]}],
 "edges":[[0,1]]}
```

(shown wrapped; each frame is one line. Key order is `frame`, `t_s`, `tree_source`,
`vertices`, `edges`. `edges` are `[i, j]` index pairs into that frame's own `vertices`
array.)

`--trace-interval-ms` (default 50) and `--trace-max-frames` (default 400) bound the output;
hitting the cap sets `trace.truncated`. Measured on the committed `scene1`/BiQRRT recording at
a 10 s budget: 199 frames spanning levels 0–3, every `q` of full dimension 8, `trace.errors` 0.

**Tree tagging, and where the labels come from.** `component` is written by
`BiQRRTImpl::getPlannerData`, which records `firstNew = data.numVertices()` before chaining to
the base call and tags only the vertex range the base call appends, via
`PlannerDataVertexAnnotated::setComponent` — lower levels of the sequence are already in `data`
and belong to other instances with their own trees. Every other planner tags nothing, so
`captureFrame` derives the same two labels instead (`Solver::deriveComponents`,
`mlmp/src/models/solver.h:500-526`): when no vertex in the frame carries a non-zero component,
a multi-source BFS runs from `PlannerData`'s start vertices (seeded 1) and goal vertices
(seeded 2) over that frame's own adjacency — nearest seed wins, start wins a tie — and
vertices neither endpoint reaches keep 0. Component 3 is BiQRRT's `findSection()` tag and is
never derived.

Each frame says which of the two it is in **`tree_source`**: `"planner"` when the planner
tagged its own trees, `"derived"` when the labels were computed from connectivity. The two are
different claims and a consumer that colours by tree must disclose which one it is showing.
Derivation is O(V + E) per frame and runs only when the planner tagged nothing, so a traced
BiQRRT run is untouched.

`tree` is the string form of `component`, produced by `treeLabel`
(`mlmp/src/models/run_result.h:97-104`), which is the single definition of the vocabulary:

|`component`|`tree`|meaning|
|---|---|---|
|0|`untagged`|no tag and no derivation reached it — on a derived frame, a vertex in no component containing a start or goal vertex|
|1|`start`|start tree (tagged by BiQRRT, otherwise derived)|
|2|`goal`|goal tree (tagged by BiQRRT, otherwise derived)|
|3|`section`|in the graph, in neither tree — mostly `findSection()` configurations, present whether or not the run solved; BiQRRT only, never derived|

No consumer may test a planner name to decide how to colour a vertex. That is no longer only a
convention: `captureFrame` labels every planner's frames, so a planner name carries no colour
information a consumer could use. Unknown or missing tags read as `untagged`. `start` and
`goal` are `PlannerData::isStartVertex` / `isGoalVertex` — which endpoint a vertex *is*, not
which tree it belongs to.

**Every `q` is full dimension, and most of it is padding.** `BundleSpaceSequence::getPlannerData`
lifts each level's states into the total space, so a frame's `q` is always `robots * links`
long, interleaved as `q[link * robots + robot]`. But a vertex at bundle level `L` only owns its
first `L + 1` links per robot; everything above is identity-fibre padding re-derived from the
planner's current section on every frame. The measured consequence (`mlmp/tools/packTrace.py`
docstring): the committed `scene1`/QRRT recording holds 193 frames that each re-dump a full `q`
per graph vertex, and truncating to the owned prefix collapses the whole stream to 15 236
distinct vertices — the same order as the run's own 15 247 `graph_vertices`, not equal to it
because distinct prefixes merge states differing only in padding. A renderer that draws the
untruncated tail renders churn that belongs to no vertex.

**Frames are cumulative snapshots, not deltas.** Each one re-dumps the entire planner graph, so
a consumer renders the latest frame and drops it. Retaining frames is why a raw 10 s recording
is 0.3–0.9 GB, which is what `mlmp.traceviz/1` below exists to fix.

CI's "Trace contract" step asserts frame shape, `len(q) == robots * links`, `trace.errors == 0`,
that both `start` and `goal` appear, that `untagged` never does for BiQRRT, and that BiQRRT's
frames report `tree_source == "planner"`. A second short QRRT run on the same scene, seed and
budget asserts the other half: `tree_source == "derived"` on every frame and `start`-labelled
vertices present — the regression that made eleven of the twelve planners draw as one
undifferentiated grey graph.

Traces recorded before `6974fd6` carry `component == 0` everywhere: `ompl::multilevel` never
calls `setComponent`, so the field was unused before Track A, and derivation did not exist yet.

### `mlmp.traceviz/1` — committed fixtures

A raw `mlmp.trace/1` stream (0.3–0.9 GB per 10 s run, because every frame re-dumps the whole
graph) can neither be committed nor served. `mlmp/tools/packTrace.py` converts one stream plus
its `mlmp.run/1` object into the form the players read.

Every vertex and edge is stored once, in order of first appearance, each vertex truncated to
the prefix it owns: `[level, component, …q]`. Each frame is `[t_s, vertexCount, edgeCount]`,
so frame `k` is exactly `vertices[:nv]` / `edges[:ne]`. Top-level keys: `schema`, `source`,
`planner`, `scene`, `seed`, `robots`, `links`, `time_limit_s`, `interval_ms`, `tree_codes`,
`tree_source` (the stream's one `mlmp.trace/1` `tree_source` value, propagated by
`packTrace.py`, which refuses a stream that mixes the two), `run` (an `mlmp.run/1` subset, for
captions), `vertices`, `edges`, `frames`.

Both properties the format relies on — truncation losing nothing a renderer draws, and the
truncated vertex and edge sets growing append-only — are asserted by `packTrace.py`, which
exits non-zero rather than silently dropping data.

Measured: 12 fixtures (6 scenes × BiQRRT/QRRT) packed at 0.36–0.82 MB each, plus their
`.run.json` siblings, under `docs/traces/`. All 12 were re-recorded with a derivation-capable
image: the BiQRRT six carry `"tree_source": "planner"`, the QRRT six `"derived"`, with
`start`/`goal` on every vertex an endpoint reaches. A consumer reading a fixture with no
`tree_source` key must treat it as `"planner"`, which is the default `packTrace.py` itself
applies to a frame that carries no labels at all — the vertex-less first frame of every run.

A live stream does not need this format. A renderer driven by snapshots can be fed either one,
which is what keeps a single drawing path for the static page and for Track E.

---

## Parallel tracks

Six tracks, concurrent after the gate. Each entry states what it owns, what it may not
touch, and what makes it done.

### Track A — Trace player and GIF (Phase 1)

- **Scope.** Consume `mlmp.trace/1`: 10-second looping GIF for the README, and a static
  scrubber page that steps frames, toggles bundle levels, and runs BiQRRT beside QRRT on the
  same seed.
- **Requires.** The gate. Nothing else.
- **Constraints.** No solver in the browser — that is Track E, and conflating them costs
  megabytes. Static JSON on GitHub Pages, no toolchain, no hosting. A traced run is never a
  timing run: frame capture copies the whole graph. The QRRT side-by-side stays unconvincing
  until Track F lands, since QRRT solved no scene 3/3 at a 10 s budget. The view extents come
  from the scene and are never renormalised per frame — a renderer that rescales as the cloud
  grows can be neither scrubbed nor compared.
- **Shipped** (`track-a/trace-player`, `6974fd6`, `62c0119`): tree tagging in `mlmp.trace/1`
  and the CI trace contract; `mlmp/tools/recordTraces.sh` (`--seed 42 --time-limit 10
  --trace-interval-ms 50 --trace-max-frames 400`, raw streams discarded by its `trap`) and
  `mlmp/tools/packTrace.py`; `mlmp/tools/traceGif.py` and the 5.7 MB README GIF; `docs/` — a
  dependency-free scrubber page with per-scene BiQRRT-vs-QRRT panels, frame stepping, level
  toggles and run captions; 12 packed fixtures and 6 scene copies.
- **Not shipped: the module shape** (state at the end of `track-a/trace-player`, both items
  since closed by Track E). `docs/app.js` exported nothing, resolved twelve elements by id
  (eight in `el`, four in `panels`) and did random access over a whole packed fixture, so
  Track E could not import it. And nothing committed drew `mlmp.run/1`'s `path` array — the
  purple on the page is `component == 3`, the `section` vertices, which is a different thing.
- **Done when.** `web/src/renderer.js` exists (Track E owns it — step 7 of
  `WASM_PLANNER_PORT_PLAN.md`), `docs/app.js` draws through it, and `showPath` renders the
  run's solution path. **Done** on `track-e/wasm-browser-planner`: the module ships at
  `web/src/renderer.js` with a byte-identical committed copy at `docs/renderer.js` (CI runs
  `cmp` on the pair), `docs/app.js` is a module that imports it, and the two solved runs
  (`scene1`/`scene5` BiQRRT) draw their `mlmp.run/1` `path` in green. That array is the
  *simplified* path — `Solver` runs `simplifyMax` in place before exporting it
  (`mlmp/src/models/solver.h:269-282`), unless `--no-simplify` was passed — and it is drawn
  as an ordered chain-tip trajectory with full arms only at its first and last state, so the
  waypoint order is readable instead of a pile of overlapping arms.

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
  compressed payload, do not estimate.
- **Constraint: Track E owns `web/src/renderer.js`.** Track A landed page-shaped, not module-
  shaped, so the module is built here to the contract in `WASM_PLANNER_PORT_PLAN.md` and
  `docs/app.js` is retrofitted onto it. There is exactly one JS drawing path and one JS
  forward-kinematics implementation. `mlmp/tools/traceGif.py` keeps a Python copy because it
  cannot import JS; the two are kept honest by a constant/parity check, not by convention.
- **Constraint: two forward-kinematics rules, not one.** The renderer truncates to the
  vertex's level and plots tips only, one dot per robot, because everything above the level is
  padding. `showPath` uses the untruncated FK, because `mlmp.run/1`'s `path` entries are real
  full-dimension configurations; it draws the ordered chain-tip trajectory through them and
  full arms only at the first and last state, since one arm per waypoint buries the path.
- **Shipped instead: one container image, not Pages.** All three surfaces — the overview page
  (`site/`), the trace player (`docs/`) and the playground (`web/dist`) — are served by a
  single static image (`docker/app/`) on `apps.tom-sabala.dev`, published by
  `.github/workflows/apps-image.yml` as `ghcr.io/tomsabala/mlmp:apps-mount-<sha>` and pinned
  in that gateway's manifest. GitHub Pages is not used: its workflow never deployed (it failed
  at `actions/configure-pages` on every run because no Pages site existed), and one artifact
  per repository was the constraint that made publishing both surfaces awkward in the first
  place.
- **Done when.** A visitor edits a scene, picks a planner, and sees the trees grow, on a
  static host that sets no special headers — and the live frames are drawn by the same module
  the committed fixtures are.
- **Shipped** (`track-e/wasm-browser-planner`): `Dockerfile.wasm` (static, thread-free,
  Boost-component-free OMPL 1.7.0 built with `emscripten/emsdk:6.0.13`), `mlmp/src/wasm/entry.cpp`
  behind an `if(EMSCRIPTEN)` branch in `mlmp/CMakeLists.txt`, the `web/` playground (module
  worker + obstacle editor + newest-edit solve queue), `web/src/renderer.js` with its verified
  `docs/renderer.js` copy, a `wasm` CI job and `apps-image.yml` publishing one static image
  serving the overview at `/a/mlmp/`, the trace player at `/a/mlmp/explorer/` and the
  playground at `/a/mlmp/live/`.
- **Measured, not estimated.** Payload `gzip -9`: `mlmp.wasm` 454 511 B + `mlmp.mjs` 22 278 B =
  476 789 B, 9.1% of the 5 MiB gate. Throughput on one host, traced BiQRRT/`scene1`/seed 42/10 s:
  native 8 104 vertices and 23 221 samples versus wasm 5 379 and 14 982 — wasm runs at ≈0.65×
  native. First exact solution 0.10 s native versus 0.9–1.8 s in the browser, which is why the
  budget slider defaults to 10 s: at 5 s the browser run times out on `scene1`.
- **Found while building it: a traced multilevel run is not seed-reproducible.** `captureFrame`
  calls `BundleSpaceSequence::getPlannerData`, whose path-restriction code draws from the global
  RNG, and captures land on wall-clock boundaries — so the number of draws tracks host speed and
  `path_length_raw` varies run to run. Untraced runs repeat exactly (verified in both targets).
  `solveScene` therefore installs no trace sink when given no callback, and both determinism
  gates compare untraced runs. Track B must not enable `--trace-file` on measurement runs.

### Track F — Planner hygiene (§2.5)

- **Scope.** Make `maxDistance_` a declared `Planner::declareParam` with a `SelfConfig`-derived
  default instead of the hardcoded `0.1` at `BiQRRTImpl.cpp:32`, and answer whether the
  BiQRRT/QRRT gap is real or a step-size artefact (QRRT's `SelfConfig` default logged as
  1.78–3.55 across `scene1`'s levels).
- **Requires.** The gate. The raw-versus-simplified half of §2.5 is already delivered by it.
- **Constraints.** Must land before Track B's campaign. The new default changes BiQRRT
  behaviour, so re-measure the CI solve gate (`scene1`, 3/3 at 10 s) before merging. It also
  invalidates the committed fixtures and the README GIF, which are recordings of planner
  behaviour and not derived artefacts: re-recording is `mlmp/tools/recordTraces.sh` followed by
  `mlmp/tools/traceGif.py`, and nothing else. Today's QRRT panels are honest failures — a
  timeout and one grey untagged cloud — and stay that way until this track lands.
- **Done when.** The step size is a parameter with a scale-derived default and the gap has a
  measured explanation.

### What cannot be parallelised

The gate precedes A, B, E and F. Track F precedes Track B's full campaign. The campaign
precedes Phase 3, and Phase 3 precedes Phase 4 — learned sampling without rigorous baselines
is an unfalsifiable claim, which is the whole reason Phase 2 exists.

Two edges were added by Track A's handoff. Track E's `web/src/renderer.js` precedes Track A's
retrofit of `docs/app.js`, which is the reverse of the original dependency. And a
publishing workflow in Track E precedes nothing, but must not land without also publishing
`docs/` — all three surfaces ship as one image (`docker/app/`, `apps-image.yml`), so dropping
one of them from that image takes it offline.

---

## Phase 1 — Visualize the bundle hierarchy

The data already exists in `BiQRRTImpl::getPlannerData`.

**Delivered** (`track-a/trace-player`): the per-iteration dump, by the gate; tree membership
and bundle level in every frame (`6974fd6`); the side-by-side page with BiQRRT and QRRT on the
same scene and seed; the 10-second looping GIF at the top of the README; the scrubber that
steps frame by frame and toggles bundle levels.

**Outstanding**: the animation's stacked-levels reading of the hierarchy — the coarse level
solving first and then *restricting* the fine level's sampling — which is visible in the
scrubber but is not what the GIF narrates. The side-by-side also stays unconvincing until
Track F lands, since QRRT solves no scene at a 10 s budget; it is now at least legible, since
`captureFrame` derives `start`/`goal` labels for planners that tag nothing.

**Closed since**: the solution path from `mlmp.run/1` is drawn by `showPath`, and the renderer
is a module (`web/src/renderer.js`) shared with Track E rather than a page script.

No solver in the browser, so it is static JSON on GitHub Pages with no toolchain and no
hosting requirements. Running the planner live in the browser is Phase 6 and costs vastly
more; do not conflate them.

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
