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

## Phase 1 — Visualize the bundle hierarchy

The data already exists in `BiQRRTImpl::getPlannerData`.

- Dump planner data per iteration (vertices, edges, tree membership, bundle level).
- Animate: both trees growing from start and goal, bundle levels stacked so the coarse
  level visibly solves first and then **restricts** the fine level's sampling, and the
  frame where the two trees connect and the path collapses out.
- Side-by-side with QRRT on the same scene and seed — the bidirectional advantage becomes
  something you can see rather than something a table asserts.
- 10-second looping GIF at the top of the README. Interactive version (web) optional.

Multilevel planning is genuinely beautiful and almost nobody renders the bundle hierarchy.

---

## Phase 2 — Rebuild the experiment harness

**The current results are not merely old. They are invalid.** This must be fixed before
any new claim is made, because every downstream phase depends on trustworthy baselines.

### 2.1 What is wrong with `tools/experiment.py`

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
- **Structured output.** The C++ binary emits JSON (`{planner, scene, seed, solved,
  time_s, path_length, states_sampled, collision_checks}`) — no stdout regex.
- **Seeded and reproducible.** `--seed` through to `ompl::RNG::setSeed`. The same seed set
  must reproduce the same table.
- **n ≥ 50 per cell**, bootstrap 95% confidence intervals on every reported statistic.
- **Distinguish failure modes.** Timeout, no-solution-found, crash, and parse error are
  four different outcomes, not one `None`.
- Consider `ompl::tools::Benchmark` + the Planner Arena log format instead of hand-rolling
  — it gives the statistical machinery and a known output format for free.
- Delete `tools/results.json` and regenerate it. Do not carry invalid numbers forward.

**Done when:** `python3 tools/experiment.py --seed 42` twice produces identical tables, and
CI regenerates a small subset on every push.

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
- `Solver::solve` runs `simplifyMax` on every solution before reporting length, which measures
  the simplifier as much as the planner — report raw and simplified length separately.

### 2.6 Scale the scene set

Six hand-authored JSON scenes on the unit square with 4-joint arms and 2 robots was a
scope forced by hand-authoring. It no longer is.

- Procedural scene generator with a **computed difficulty metric**: narrow-passage width,
  C-space obstacle ratio, chain DOF, inter-robot coupling, start/goal homotopy distance.
- 500+ scenes spanning the difficulty range, generated from a seed, not committed as blobs.
- Keep the six originals as named regression scenes.

---

## Phase 3 — Use CGEO as the geometry backend

`mlmp/src/state/kinematic_chain.h:243` is a Stack Overflow–cited segment intersection on raw
doubles, sitting in `isValidImpl` — the hottest loop in the planner.
`mlmp/src/common/geometry.h` is two structs of naked doubles with a `print()` method.

CGEO is a C++ segment-intersection kernel with a sweep line and a balanced status tree.
Link them:

- CGEO exposes a C++ consumer API (headers + static lib, not only the Python extension).
- MLMP's collision checker calls it; exact predicates replace epsilon comparisons in the
  validity check, which is where degeneracy actually bites a planner.
- Broad-phase via CGEO's BVH instead of all-pairs segment tests.
- Benchmark before/after — this is a measurable claim, not a refactor.

The payoff is mutual: the planner is the demanding real-time consumer that makes the
kernel's API good, and planner benchmark regressions become kernel test cases.

See `CGEO/ROADMAP.md` Phase 5.

---

## Phase 4 — The finding

A benchmark that only produces plots is a plot dump. State a rule and a mechanism:

> BiQRRT dominates QRRT* below narrow-passage ratio X and loses above it, because
> bidirectional growth wastes samples when the goal tree cannot escape the restriction
> imposed by the coarse level.

Requirements: the claim is falsifiable, the failure region is stated explicitly, and the
numbers behind it regenerate from a seed. Publish as a short technical write-up — OMPL
discourse, arXiv preprint, or a post. Unread work convinces nobody; distribution is most
of the difference between a project with zero stars and one people cite.

---

## Phase 5 — Learned sampling

Only after Phase 2 and 4. Rigorous baselines are what make this credible; doing it first
produces an unfalsifiable ML claim.

- Train a sampler on solved instances to predict promising bundle-space restrictions.
- Report the delta against the Phase 2 baselines on held-out scenes, same metrics, same CIs.
- Negative results are publishable here and more credible than a tuned win.

---

## Phase 6 — Make it look like a robot

4-joint planar chains in a unit square read as a toy regardless of the algorithm's quality.

- URDF loading; plan for a Panda or UR5.
- 3D chains, heterogeneous robot teams, more than two agents.
- Render in MuJoCo or Drake.

Same algorithms, different perceived seriousness.

---

## Non-goals

- LLM features. This is a planning and geometry problem.
- Reimplementing OMPL. Extend it.
- Upstreaming BiQRRT into `ompl::multilevel`. The repo is the artifact; a maintainer review
  cycle is not a goal of this project.
- Carrying `tools/results.json` forward. Those numbers were produced under thread
  contention by a script that cannot regenerate them.
