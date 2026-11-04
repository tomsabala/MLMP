# Track E — run the planner in the browser (wasm)

## Status: implemented on `track-e/wasm-browser-planner`

This document is the design; the branch is the build. Where they differ, the branch is right —
the five deltas below were forced by things only a build reveals.

1. **`web/src/renderer.js` shipped, with a committed byte-identical copy at `docs/renderer.js`**
   (CI runs `cmp` on the pair), because `docs/` must stay servable straight from the branch
   while the Pages source is still "deploy from branch". `showPath` shipped with it, and
   `docs/app.js` is now a module that imports the pair of them. Constants reproduce
   `docs/app.js` exactly, not `mlmp/tools/traceGif.py`: the two shipped renderers disagreed on
   four values (dot radii, edge width, obstacle stroke alpha, joint-dot size), and re-rendering
   the committed 5.7 MB GIF for a cosmetic match was not worth it. The parity test asserts only
   what is genuinely shared: `TREE_COLORS`, `PADDING` and the alpha formula.
2. **`emscripten/emsdk:6.0.13`, not a 3.1.x pin.** A 3.1.64 `-sEXPORT_ES6` module opens with a
   static `import { createRequire } from 'module'` (unresolvable in a browser) and detects a
   worker by `typeof importScripts`, which is `undefined` inside a module worker — so it cannot
   be loaded by `new Worker(url, { type: "module" })` at all. 6.x uses `globalThis.WorkerGlobalScope`
   and a guarded dynamic `import("node:module")`.
3. **Six OMPL patches, not three, plus two toolchain fixes.** Besides the Boost-components,
   STATIC and link-list patches: `omplConfig.cmake.in` hard-codes `find_dependency(Boost
   REQUIRED COMPONENTS serialization filesystem system)` and breaks every consumer; the XXL
   planner and `AdjacencyList` include `boost/thread`, which refuses to compile without
   `-pthread`; and Boost.Serialization's archive templates reach the link from
   `PlannerData.cpp` via `StateStorage.h` (one unused function, `extractStateStorage`), taking
   the `*Storage` units and the Thunder/Lightning databases with them. Boost's headers are
   copied into the Emscripten sysroot rather than found at `/usr/include`, because putting
   `/usr/include` on the include path shadows the sysroot's libc headers; and jammy-era Boost
   1.74 needs `-D_LIBCPP_ENABLE_CXX17_REMOVED_UNARY_BINARY_FUNCTION`.
4. **A traced multilevel run is not seed-reproducible.** Frame capture draws from the global
   RNG and captures land on wall-clock boundaries, so `solveScene` installs no trace sink when
   handed no callback, and the determinism check compares untraced runs. See the Track E card
   in `ROADMAP.md`.
5. **The budget slider defaults to 10 s, not 5 s**, and a blocked endpoint arrives as
   `error.kind == "runtime"` with `"Invalid initial states."` from `BundleSpaceGraph::setup`
   rather than as `status == "invalid_start"`; the page maps both to the same plain sentence.

## Context

Build an Emscripten/wasm port of `mlmp` so a visitor on a static host (GitHub Pages, no
COOP/COEP headers) can edit a 2D scene, pick a planner, solve, and watch the bundle-space
trees grow live. Goal defined by `ROADMAP.md:313-338` (Track E card) and
`ROADMAP.md:538-570` (Phase 6 body).

The `Solver` contract gate is already landed and is what makes this possible:
`mlmp::Solver::solve(const RunOptions&) -> RunResult` (`mlmp/src/models/solver.h:136`)
does no printing, no option parsing and no file IO, and termination is a non-threaded
`PlannerTerminationCondition` built from a `std::function` (`solver.h:197-222`). Neither
regresses here.

End state: `Dockerfile.wasm` produces `web/dist/`; a committed static page under `web/`
hosts a Worker that runs the planner; CI builds the module, smoke-tests it under Node,
enforces a payload budget, and deploys to GitHub Pages on push to `main`. The native CLI,
the Docker image, `mlmp.run/1` and `mlmp.trace/1` are unchanged.

## Verified facts — do not re-derive

- **`PRMStar` cannot run in this build.** `ompl::geometric::PRM::solve` spawns a thread
  unconditionally: `std::thread slnThread([this, &ptc, &sol] { checkForSolution(ptc, sol); });`
  at `src/ompl/geometric/planners/prm/src/PRM.cpp:501`, joined at `:509`. Without
  `-pthread` this aborts the Worker. Phase 6's obstacle list omits this.
- **OMPL hardcodes a shared library.** `src/ompl/CMakeLists.txt:38-41` is
  `add_library(ompl SHARED ${OMPL_SOURCE_CODE})` for non-MSVC; `BUILD_SHARED_LIBS=OFF`
  does not change it. A static archive is required for dead-code elimination, which is
  what trims both the payload and the Boost libraries.
- **OMPL requires compiled Boost components at configure time.**
  `CMakeLists.txt:50` is
  `find_package(Boost 1.68 REQUIRED COMPONENTS serialization filesystem system program_options)`.
  Under the Emscripten toolchain these cannot be found, so configure fails unless patched.
- **Boost.Serialization *headers* are unavoidable.** `base/PlannerData.h:48` includes
  `boost/serialization/access.hpp`, `multilevel/datastructures/PlannerDataVertexAnnotated.h:42`
  includes `boost/serialization/export.hpp`, and `Solver` uses both types
  (`solver.h:285,536,561`). Headers must be on the include path; the compiled libraries
  must not be linked. Only `base/src/PlannerDataStorage.cpp`, `base/src/StateStorage.cpp`,
  `tools/lightning/src/LightningDB.cpp` and `tools/thunder/src/ThunderDB.cpp` reference
  them, none reachable from `Solver`.
- **`tools/benchmark/src/MachineSpecs.cpp:147` reads `/proc/self/stat`**, absent under
  MEMFS. Harmless only because its sole caller is `Benchmark.cpp` (`:70,152,530`),
  unreachable from `Solver` — a second reason dead-object elimination must actually work.
- **Threads must not leak in via CMake.** `CMakeLists.txt:76` runs `find_package(Threads QUIET)`
  and `src/ompl/CMakeLists.txt:49` links `$<$<BOOL:${Threads_FOUND}>:Threads::Threads>`.
  If that resolves, `-pthread` enters the build and the module needs SharedArrayBuffer.
- **Exceptions are mandatory**: `solver.h:237-244` catches `ompl::Exception` per trace
  frame, and `BundleSpaceSequence::getPlannerData` throws if handed a populated
  `PlannerData` (hence the fresh instance per frame at `solver.h:536`).
- **C-space bounds are scene-independent**: `KinematicChainSpace` fixes bounds to ±π
  (`kinematic_chain.h:55-58`). Moving an obstacle changes only `Environment` segments and
  the padded bounding box (`scene.h:150-161`), never the state space or its dimension.
- **Scene angles are integer degrees**: `scene.h:116,119` use `angle.get<int>()`. The
  editor must never emit fractional angles — it does not touch angles at all.
- **`Scene::loadScene` takes a filename only** (`scene.h:33`) and opens the `ifstream`
  itself. `ROADMAP.md:564` describes the signature wrongly; the MEMFS approach still works.
- **Eigen3 is a compile-time dependency** (`src/ompl/CMakeLists.txt:46`, header-only, so
  it never appears in `ldd`). `FLANN`, `Triangle`, `Spot` must stay off.
- **`cmake --build --target ompl`** builds only the library, skipping the unconditional
  `add_subdirectory(tests|demos|py-bindings)` at `CMakeLists.txt:138-141`. The existing
  `Dockerfile` already uses this.

Measured by Track A on `track-a/trace-player` and carried over here; not re-derived:

- **Frame shape.** A frame is `{frame, t_s, tree_source, vertices[], edges[]}`; `edges` are
  `[i, j]` index pairs into that frame's own `vertices`; a vertex is
  `{level, component, tree, start, goal, q}` (`solver.h:539-570`).
- **Tag vocabulary** is `0/1/2/3 → untagged/start/goal/section`, defined once in `treeLabel`
  (`run_result.h:97-104`). `component` is written by `BiQRRTImpl::getPlannerData`
  (`mlmp/src/planner/src/BiQRRTImpl.cpp:176-219`), which tags only the vertex range the base
  call appends. `3` means in the graph but in neither tree — mostly the configurations
  `findSection()` adds — and is present whether or not the run solved, so it is not the
  solution path.
- **Labels exist for every planner, derived when necessary.** If no vertex in a frame carries a
  non-zero component, `Solver::deriveComponents` (`solver.h:500-526`) labels the graph instead:
  multi-source BFS from `PlannerData`'s start vertices (1) and goal vertices (2) over that
  frame's adjacency, nearest seed winning, start winning ties, 0 left where neither endpoint
  reaches. `3` is never derived. The frame's `tree_source` is `"planner"` or `"derived"`
  accordingly, and `packTrace.py` lifts the stream's single value to a top-level `tree_source`
  in `mlmp.traceviz/1`, immediately after `tree_codes`. A UI that colours by tree must say
  which of the two it is showing.
- **A vertex at level `L` owns only its first `L + 1` links per robot**; the tail is
  identity-fibre padding re-derived each frame. The committed `scene1`/QRRT recording holds
  193 frames that each re-dump a full `q` per graph vertex; truncating to the owned prefix
  collapses the whole stream to 15 236 distinct vertices, the same order as the run's own
  15 247 `graph_vertices` (not equal — distinct prefixes merge states differing only in the
  padding).
- **Frames are cumulative snapshots**, not deltas; retaining them is what makes a raw 10 s
  recording 0.3–0.9 GB, and why `mlmp.traceviz/1` exists for the committed fixtures.
- **Level filtering is load-bearing, not cosmetic**: unchecking L3 on `scene1` drops painted
  pixels from 192 280 to 77 542 (Track A's measurement, not re-verified here).
- **`mlmp/tools/plotScene.py:65` uses `padding = 1`**; both shipped renderers use `0.05`. Do
  not copy the plotting script's framing.
- **Tracing costs throughput**: `scene5`/BiQRRT timed out in one recording batch and solved at
  1.05 s in the next, same seed and budget. Captions must report what the run object reported
  and never infer solved-ness from frame contents.

## Fixed decisions

| Decision | Choice |
|---|---|
| OMPL build shape | Patch `SHARED` → `STATIC` in the wasm build stage |
| Planner set | All 12 listed; `PRMStar` greyed out in the UI and rejected by the entry point |
| Exception mode | `-fwasm-exceptions` |
| Optimization | `-O3` |
| Payload budget | ≤ 5 MB gzip for `mlmp.wasm` + `mlmp.mjs`, enforced in CI |
| Re-solve trigger | On pointerup; queue only the newest scene while a solve is in flight |
| Scene handoff | `FS.writeFile` to MEMFS, then `Scene::loadScene(path)` unchanged |
| Live frames | Reuse `RunOptions::traceSink` → one `postMessage` per frame |
| Editor scope | Drag, add and delete obstacles; nothing else |
| Build plumbing | New `Dockerfile.wasm` |
| Artifacts | Not committed; CI builds and deploys |
| CI | Build + headless Node smoke + gzip size gate |
| Deploy | `actions/deploy-pages` on push to `main`; one artifact, `docs/` at the root with `web/dist` under `live/` |
| `PRMStar` guard | In the wasm entry point only; `solver.h` untouched |
| Time budget | Slider 1–10 s, default 5 s |
| Seed | Editable field, default 42, re-seeded per solve |
| Scenes | `scene1` default, picker over all six experiment scenes |
| Result panel | Full fields, labelled as a traced run, not a benchmark |
| Invalid scenes | Solver is authoritative; render `invalid_start` / `invalid_goal` |

## Renderer — owned by Track E

Track A shipped page-shaped, not module-shaped: `docs/app.js` exports nothing, resolves twelve
elements by id (eight in `el`, four in `panels`) and does random access over a whole packed
fixture. So Track E builds `web/src/renderer.js` to the specification below (step 7), and
Track A retrofits `docs/app.js` onto it. There is exactly one JS drawing path and one JS
forward-kinematics implementation.

```js
export function createTracePlayer({ canvas, scene }) -> {
  pushFrame(frame),      // one mlmp.trace/1 frame; rendered as the current state, not accumulated
  setScene(scene),       // new scene JSON mid-session; recomputes extents + static layer, redraws
  setLevels(levels),     // Set<number> of visible bundle levels; default all
  showPath(path),        // mlmp.run/1 `path` (the simplified path): full-dimension states,
                         // untruncated FK, drawn as an ordered tip trajectory with full arms
                         // only at its endpoints
  reset(),               // clear vertices/edges/cached extents, keep canvas and scene
  destroy()
}
```

1. **Module, not page.** No globals, no DOM lookups beyond the passed `canvas`. The page owns
   its controls; the module exposes `setLevels` rather than reading checkboxes.
2. **Snapshot semantics.** Each `mlmp.trace/1` frame is a complete cumulative graph, not a
   delta. `pushFrame` renders it and retains nothing; per-vertex tips, colour and alpha are
   computed on push. A scrubber is therefore `reset()` + `pushFrame(snapshot_k)`, so one API
   serves both the live Worker stream and a committed fixture. Nothing may require the full
   frame array up front — Track E has no file and no final frame count.
3. **Level-truncated FK, one copy.** A vertex at level `L` owns only its first `L + 1` links
   per robot; the tail is identity-fibre padding. For robot `r`, starting at `pinnedPosition`
   with `theta = 0`:
   `for j in 0..L: theta += q[j*robots + r]; x += jointLength*cos(theta); y += jointLength*sin(theta)`.
   Plot the tip, one dot per robot. Tolerate a short `q` — packed `mlmp.traceviz/1` vertices
   arrive pre-truncated — by breaking when the index passes the end of the array.
4. **`showPath` is the one place the untruncated FK is correct.** `mlmp.run/1` `path` entries
   are real full-dimension configurations — and the *simplified* path, since `Solver` runs
   `simplifyMax` in place before exporting them (`solver.h:269-282`). Draw it as an ordered
   trajectory of chain tips with full arms at the first and last state only: an arm per
   waypoint hides the ordering. It must stay visually distinct from `component == 3`
   (`section`) vertices, which are graph vertices in neither tree — mostly `findSection()`
   configurations — and are not the solution path.
5. **Scene is a separate, replaceable input.** `mlmp.trace/1` carries no geometry. The
   renderer takes scene JSON (obstacles, `pinnedPosition`, `jointLength`, `metadata.n`)
   separately and accepts a new one at runtime via `setScene`, because Track E changes
   obstacles between solves. Link count comes from `scene.metadata.n`, never from the trace
   header, so the renderer does not depend on a header the live stream may not have delivered.
6. **Tags are optional, but a planner name is never a substitute.** `tree` / `component` may
   be absent or unknown → treat as `untagged`. Never test a planner name to decide how to
   colour a vertex: `captureFrame` labels the frames of planners that tag nothing, so the name
   carries no colour information. The page, not the renderer, discloses whether a frame's
   labels came from the planner or were derived (`tree_source`).
7. **Bundle level is a visual channel while all levels are shown**, not only a filter. The
   live demo's whole point is seeing the coarse level solve first and restrict the fine level.
8. **Extents are fixed from the scene**, never renormalised per frame. A view that rescales as
   the cloud grows can be neither scrubbed nor compared across planners.
9. **`reset()` between runs** — Track E re-solves many times in one page session.

Division of labour: Track E writes the module. Track A replaces `docs/app.js`'s drawing with
`reset()` + `pushFrame(snapshotFromPackedPrefix(k))`, keeping scene selection, play/pause, the
slider, the level checkboxes and the captions — which are the page, not the renderer.

## Approach

### Step 1 — `Dockerfile.wasm` builder stage: patched static OMPL for wasm

New file `Dockerfile.wasm` at the repo root. Base `FROM emscripten/emsdk:3.1.64 AS builder`
(Debian bookworm, ships cmake and python3; `libboost-dev` there is 1.74, satisfying OMPL's
`1.68` minimum).

Install header-only dependencies with apt: `libboost-dev libeigen3-dev nlohmann-json3-dev git ca-certificates`.

Clone OMPL exactly as the native `Dockerfile` does:
`git clone --depth 1 -b "${OMPL_VERSION}" https://github.com/ompl/ompl.git /src/ompl` with
`ARG OMPL_VERSION=1.7.0`.

Apply three patches before configure. Each is a `sed -i` with a fixed-string match, and
each must be followed by a `grep` that fails the build if the expected result is absent —
a silently-missed patch produces a 11 MB module or a pthread build, both of which pass
compilation:

1. `/src/ompl/CMakeLists.txt`: replace
   `find_package(Boost 1.68 REQUIRED COMPONENTS serialization filesystem system program_options)`
   with `find_package(Boost 1.68 REQUIRED)` (headers only).
2. `/src/ompl/src/ompl/CMakeLists.txt`: replace `add_library(ompl SHARED ${OMPL_SOURCE_CODE})`
   with `add_library(ompl STATIC ${OMPL_SOURCE_CODE})`.
3. `/src/ompl/src/ompl/CMakeLists.txt`: delete the lines `Boost::filesystem`,
   `Boost::serialization`, `Boost::system` and
   `"$<$<BOOL:${Threads_FOUND}>:Threads::Threads>"` from the `target_link_libraries(ompl …)`
   block, leaving `Eigen3::Eigen` and the FLANN/Triangle generator expressions.

Configure with `emcmake cmake -S /src/ompl -B /src/ompl/build` and these options:

```
-DCMAKE_BUILD_TYPE=Release
-DCMAKE_INSTALL_PREFIX=/opt/wasm
-DOMPL_BUILD_TESTS=OFF
-DOMPL_BUILD_DEMOS=OFF
-DOMPL_REGISTRATION=OFF
-DCMAKE_FIND_ROOT_PATH_MODE_INCLUDE=BOTH
-DCMAKE_FIND_ROOT_PATH_MODE_PACKAGE=BOTH
-DCMAKE_CXX_FLAGS="-O3 -fwasm-exceptions"
```

The two `FIND_ROOT_PATH_MODE` overrides are what let the host-installed, architecture-
independent Boost/Eigen/nlohmann headers and CMake config files be found through the
Emscripten toolchain, which otherwise restricts all `find_package` searches to its own
sysroot.

Build and install with `cmake --build /src/ompl/build --target ompl -j"$(nproc)"` and
`cmake --install /src/ompl/build`. `--target ompl` skips `tests`, `demos` and
`py-bindings`, which `CMakeLists.txt:138-141` adds unconditionally.

### Step 2 — Emscripten branch in `mlmp/CMakeLists.txt`

Edit `mlmp/CMakeLists.txt`. The native build must be byte-identical; guard everything new
with `if(EMSCRIPTEN)`.

- Wrap `find_package(Boost REQUIRED COMPONENTS program_options)` (line 11) in
  `if(NOT EMSCRIPTEN)`. `boost::program_options` is dead weight in this build
  (`ROADMAP.md:566`) and is not available for wasm.
- Keep `find_package(ompl 1.7 REQUIRED)` and `find_package(nlohmann_json 3.11 REQUIRED)`
  shared. Header-only Boost used by `solver.h:14` (`boost/range`) and
  `kinematic_chain.h:8` (`boost/math/constants`) needs no component.
- Under `if(EMSCRIPTEN)`, define target `mlmp_wasm` from `src/wasm/entry.cpp` and
  `src/planner/src/BiQRRTImpl.cpp` (not `src/main.cpp`), linking `ompl::ompl` and
  `nlohmann_json::nlohmann_json`, with the same `target_include_directories(… PRIVATE
  ${CMAKE_SOURCE_DIR}/src)`. Under `else()`, define the existing `mlmp` executable
  unchanged.
- Set the link flags on `mlmp_wasm` via `target_link_options`:

```
-O3 -fwasm-exceptions --bind
-sMODULARIZE=1 -sEXPORT_ES6=1 -sEXPORT_NAME=createMlmpModule
-sENVIRONMENT=worker,node
-sALLOW_MEMORY_GROWTH=1 -sINITIAL_MEMORY=64MB -sSTACK_SIZE=8MB
-sFORCE_FILESYSTEM=1 -sEXPORTED_RUNTIME_METHODS=FS
```

  `-sSTACK_SIZE=8MB` is deliberate: Emscripten's default is 64 KB since 3.1.27, and OMPL's
  graph and A* code recurses deeply enough to blow it. `-sFORCE_FILESYSTEM=1` plus the
  exported `FS` is what makes the MEMFS scene handoff possible from an embind-only module.
  `-sENVIRONMENT=worker,node` covers the page's Worker and the CI smoke test, and nothing
  else. Set `OUTPUT_NAME mlmp` and `SUFFIX .mjs` so Node treats the glue as an ES module.

### Step 3 — wasm entry point

New file `mlmp/src/wasm/entry.cpp`. It is the wasm analogue of `main.cpp`: it owns all I/O
and option handling, and `Solver` stays clean. Exactly one exported function plus a trivial
`main`.

```cpp
std::string solveScene(std::string scenePath,
                       std::string algo,
                       double timeLimitS,
                       double seed,              // double: JS numbers; cast to uint_fast32_t
                       double traceIntervalS,
                       unsigned int traceMaxFrames,
                       emscripten::val onFrame); // called with one JSON string per frame
```

Returns a **JSON string**, not an embind object: `mlmp::toJson(result).dump()` on success,
`mlmp::errorJson(kind, message).dump()` on failure. The caller does `JSON.parse`. This
guarantees the browser and the CLI emit the identical `mlmp.run/1` bytes and keeps embind
out of the schema. Frames are passed to `onFrame` the same way, as `frame.dump()` strings,
so they are identical to `mlmp.trace/1` lines.

Body, in order:

1. `ompl::msg::setLogLevel(ompl::msg::LOG_NONE)` — OMPL's default handler writes to stdout,
   which in wasm is `console.log`; there is no stdout contract to protect but the spam and
   cost are real. No custom output handler is needed (the `StderrOutputHandler` in
   `main.cpp:23-30` exists only to protect the CLI's stdout).
2. If `algo == "PRMStar"`, return
   `errorJson("usage", "PRMStar is unavailable in the browser build: ompl::geometric::PRM::solve requires threads").dump()`.
   This is the only place `PRMStar` is blocked in C++; `solver.h` is not touched.
3. If `!mlmp::isSupportedAlgorithm(algo)`, return
   `errorJson("usage", "unknown algorithm: " + algo).dump()`.
4. `ompl::RNG::setSeed(static_cast<std::uint_fast32_t>(seed))` **before** constructing
   anything else, mirroring `main.cpp:155-157`: `KinematicChainProjector`'s constructor
   draws from the global generator. Because the RNG is process-global and the Worker is
   long-lived, this re-seed on every call is what makes solve N reproducible.
5. `mlmp::Scene scene; scene.loadScene(scenePath);` inside `try`/`catch (const std::exception&)`
   → `errorJson("io", e.what())`.
6. Fill `mlmp::RunOptions`: `algo`, `sceneName = scenePath`, `scenePath`, `timeLimit`,
   `simplify = true`, `traceIntervalS`, `traceMaxFrames`, and
   `traceSink = [&onFrame](const mlmp::Json& f) { onFrame(f.dump()); }`.
   Leave `traceFile` empty — it is informational only (`run_result.h:29`), and
   `result.traced` is set from `traceSink` being non-empty (`solver.h:151`), so the result's
   `trace` object still reports `frames`, `truncated` and `errors`.
7. `mlmp::Solver solver(scene); auto result = solver.solve(options);` inside
   `try`/`catch (const std::exception&)` → `errorJson("runtime", e.what())`. Return
   `mlmp::toJson(result).dump()`.

A fresh `Scene` and a fresh `Solver` per call is required, not an optimisation: `Solver`
copies the scene and builds `Environment env_` in its constructor (`solver.h:133-134,
287-291`), so a moved obstacle cannot reach an existing instance.

Bind with:

```cpp
EMSCRIPTEN_BINDINGS(mlmp) { emscripten::function("solveScene", &solveScene); }
int main() { return 0; }
```

The trivial `main` avoids an undefined-symbol link error in an embind-only module.

### Step 4 — `Dockerfile.wasm` app stage and `web/dist` assembly

Second stage in the same file, still on the builder image. Copy `mlmp/` into the image,
configure with `emcmake cmake -S /src/mlmp -B /src/mlmp/build -DCMAKE_BUILD_TYPE=Release
-DCMAKE_PREFIX_PATH=/opt/wasm -DCMAKE_FIND_ROOT_PATH_MODE_INCLUDE=BOTH
-DCMAKE_FIND_ROOT_PATH_MODE_PACKAGE=BOTH`, then `cmake --build … -j"$(nproc)"`.

Assemble the publishable tree into `/out`:

- `mlmp.mjs`, `mlmp.wasm` from the build directory.
- `scenes/scene1.json` … `scene6.json` copied from `mlmp/src/inputs/experiment/`.
- `scenes/index.json`: a generated array of the six scene filenames, so the page's picker
  has no hardcoded list.
- The committed page sources from `web/` (`index.html`, `src/*.js`, `src/*.css`).

Final stage `FROM scratch AS dist` with `COPY --from=builder /out /` so CI can extract the
tree with `docker build --target dist --output type=local,dest=web/dist .`.

`web/dist/` is build output: add it to `.gitignore` and `.dockerignore`.

### Step 5 — Web Worker

New file `web/src/worker.js`, a module worker (`new Worker(url, { type: "module" })`).

Protocol, main → worker:

```js
{ type: "solve", id, sceneJson, algo, timeLimitS, seed, traceIntervalMs, traceMaxFrames }
```

Worker → main:

```js
{ type: "ready" }
{ type: "frame", id, frame }    // frame: parsed mlmp.trace/1 frame object
{ type: "result", id, result }  // result: parsed mlmp.run/1 object (success or error shape)
```

On load the worker imports `./mlmp.mjs`, awaits `createMlmpModule()`, caches the instance
and posts `{type:"ready"}`. The module is instantiated **once** for the worker's lifetime;
nothing is terminated or respawned.

On `solve`: `Module.FS.writeFile("/scene.json", sceneJson)` then
`Module.solveScene("/scene.json", algo, timeLimitS, seed, traceIntervalMs / 1000,
traceMaxFrames, (line) => postMessage({type:"frame", id, frame: JSON.parse(line)}))`,
then post the parsed return value as `result`. The whole call is synchronous and blocking
— that is the entire reason it lives in a Worker (`ROADMAP.md:561-562`).

Trace parameters from the page: `traceIntervalMs = 50`, `traceMaxFrames = 400`, matching
the CLI defaults (`main.cpp` `--trace-interval-ms` / `--trace-max-frames`). At a 10 s
maximum budget the cap cannot be hit; `result.trace.truncated` is surfaced anyway.

### Step 6 — Page, editor and solve queue

New files `web/index.html`, `web/src/main.js`, `web/src/editor.js`, `web/src/app.css`.

**Layout.** A single canvas with a control strip: planner `<select>` (all 12 names from
`kMultilevelAlgorithms` + `kSingleLevelAlgorithms`, in that order, with `PRMStar` rendered
`disabled` and titled "unavailable in the browser build: OMPL's PRM requires threads"), a
scene `<select>` populated from `scenes/index.json` and defaulting to `scene1.json`, a
time-budget `<input type="range" min="1" max="10" step="1" value="5">` with its value shown,
a seed `<input type="number" value="42">`, a Solve button, and an "Add obstacle" toggle.

**View transform — owned by the renderer, not this step.** `web/src/renderer.js` computes
extents from the scene (step 7: `PADDING = 0.05` in scene units around the union of obstacle
points and, per robot, `pinnedPosition ± links * jointLength`). This step imports the
projection it exposes for the editor's hit-testing rather than deriving a second one.
Deliberately excluded from the extents is `Scene`'s padded boundary box (`scene.h:150-161`,
padding `1.0`), which on a unit-square scene is several times the interesting area and would
render everything tiny. Extents are recomputed on scene switch via `setScene`, never on
obstacle drag, so the view does not jump while dragging — and never per frame.

**Editor** (`editor.js`) owns a mutable copy of the loaded scene JSON and emits
`change` events carrying the new scene object. Only `obstacles` is ever modified; `robots`,
`metadata` and all angles are passed through verbatim from the source file, which is what
keeps the integer-degree constraint at `scene.h:116,119` satisfied without any rounding
logic.

- *Drag*: `pointerdown` inside a polygon (even-odd point-in-polygon test) selects it;
  `pointermove` translates every vertex by the pointer delta; `pointerup` commits.
- *Add*: with the toggle on, each click appends a vertex to a provisional polygon rendered
  as a dashed outline; `Enter` or a double-click closes it. Fewer than 3 vertices discards
  it silently. No self-intersection check — `Scene::loadObstacles` turns any point list
  into closed segments (`scene.h:129-147`) and the checker handles arbitrary segments.
- *Delete*: `Delete`/`Backspace` removes the selected polygon. Removing them all is legal;
  `obstacles: []` still loads, and `addBoundarySegments` still produces walls from the
  robot extents.
- Serialise coordinates rounded to 4 decimal places, and set each obstacle's `info` to its
  stringified index, matching the shape of the committed scenes.

**Solve queue** (`main.js`). State: `inFlight` (the id of the running solve, or null) and
`pending` (the newest scene awaiting a run, or null).

- A `change` from the editor redraws the obstacle immediately, dims the previous trees, and
  calls `requestSolve(scene)`.
- `requestSolve` assigns a monotonically increasing `id`. If `inFlight` is null it posts to
  the worker and sets `inFlight = id`. Otherwise it overwrites `pending` with the scene and
  returns — only the newest edit survives.
- On `result`, if `msg.id !== inFlight` the message is ignored; otherwise render it, clear
  `inFlight`, and if `pending` is non-null immediately start it.
- On `frame`, ignore unless `msg.id === inFlight`, else `player.pushFrame(msg.frame)`.
- Disable the Solve button and show "solving…" while `inFlight` is set. Dragging stays
  enabled throughout — that is the point of queueing rather than blocking.

**Result panel.** Render `status`, `exact`, `path_length_raw`, `path_length_simplified`,
`time_first_solution_s`, `graph_vertices`, `states_sampled`,
`trace.frames`/`trace.truncated`/`trace.errors`, and a `trees` row reading
`planner-reported` or `derived from connectivity` from the frames' `tree_source`, under a
fixed caption: *"Traced in-browser run — frame capture copies the planner graph, so these
timings and counters are not benchmark numbers."* This is required, not cosmetic:
`ROADMAP.md:234-235` states a traced run is never a timing run, and every browser run is
traced by construction.

**View modes.** A `View` selector offers `graph + edges` (default), `vertices only` and
`path only`, so a dense graph can be cleared away to read the returned path. The same three
modes exist on the committed-fixture player, and both pages carry a disclosure next to the
legend saying whether the tree colours were reported by the planner or derived from graph
connectivity.

**Error and invalid-scene rendering.** The returned object is either a result or
`{schema, error:{kind,message}}` (`run_result.h:168-171`). Branch on the presence of
`error` and show the message inline. For results, map `status` (`run_result.h:81-95`) to a
message: `invalid_start` → "this obstacle blocks a robot's start configuration",
`invalid_goal` → "…goal configuration", `timeout` → "no solution found in N s", and draw
the path via `player.showPath(result.path)` when `solved` is true. There is no JS-side
collision check anywhere: the C++ checker is the only authority.

### Step 7 — `web/src/renderer.js`

Track E's deliverable, not a fallback, and it does not block on Track A. Implement
`createTracePlayer` to the contract above, canvas 2D. Every fixed value below is lifted
verbatim from the two shipped copies (`docs/app.js`, `mlmp/tools/traceGif.py`), which already
agree; `main.js` then imports the module and uses only its six methods.

- **Colour by tree** (`TREE_COLORS`): `start #1f77b4`, `goal #d62728`, `section #9467bd`,
  `untagged #7f7f7f`. These are the labels every planner now carries, tagged or derived.
  Green `#2ca02c` is not a tree colour: it is `PATH_COLOR`, reserved for `showPath`, the only
  thing that draws the genuine returned solution path.
- **Opacity by level**: `alpha = 0.35 + 0.65 * level / max(1, links - 1)`.
- **Dots**: radius `2.4` for `section`, `1.5` otherwise; the `section` layer drawn last so
  those configurations stay legible on top of both trees.
- **Edges**: one segment per robot between the two endpoints' tips, each endpoint using its
  own level for FK, `lineWidth 0.8`, stroke alpha `0.25`, coloured by the source vertex;
  skipped entirely when either endpoint's level is hidden.
- **Static layer**: obstacle polygons filled `rgba(128,128,128,0.5)` and stroked
  `rgba(0,0,0,0.7)`; start and goal arms as polylines in `rgba(0,0,0,0.25)` with joint dots of
  radius `1.6`. Scene angles are integer degrees and are converted to radians here.
- **Extents**: `PADDING = 0.05` in scene units around the union of obstacle points and, per
  robot, `pinnedPosition ± links * jointLength`. Equal-aspect scene→pixel projection with `y`
  flipped, sized by device pixel ratio.

Port, do not rewrite, these frame-independent functions from `docs/app.js`: `chainTips`
(`:64`), `chainJoints` (`:84`), `sceneBounds` (`:99`), `projector` (`:150`), `drawStatic`
(`:160`), `drawFrame` (`:197`), `hexToRgba` (`:51`), `TREE_COLORS` (`:14`), `PADDING` (`:20`).
Leave behind `el` (`:22`), `panels` (`:33`), `state` (`:38`), `preparePanel` (`:123`),
`render` (`:251`), `setFrame` (`:273`), `buildLevelToggles` (`:278`), `loadScene` (`:311`),
`tick` (`:337`) — those are the page, and Track A keeps them.

**Reconcile with step 6.** Step 6's view-transform paragraph ("the union of all obstacle
points and, per robot, the disc `pinnedPosition ± jointLength * metadata.n`, plus a 10%
margin") is superseded here: the renderer owns extents, the margin is `PADDING = 0.05` in
scene units and not a percentage, and step 6 keeps only the editor's hit-testing. One framing
rule, not two.

### Step 8 — CI job

Add a `wasm` job to `.github/workflows/ci.yml`, parallel to `build-and-smoke`, using the
same `docker/setup-buildx-action@v3` and GHA cache pattern already present at
`ci.yml:20-28`.

1. Build and extract:
   `docker build -f Dockerfile.wasm --target dist --output type=local,dest=web/dist .`
2. **Size gate**: fail if
   `gzip -9 -c web/dist/mlmp.wasm | wc -c` plus `gzip -9 -c web/dist/mlmp.mjs | wc -c`
   exceeds 5 242 880 bytes. Print both numbers unconditionally so the measurement is in the
   log whether it passes or not (`ROADMAP.md:319-320`: measure, do not estimate).
3. **Node smoke** via a new committed `web/test/smoke.mjs`, run with `node web/test/smoke.mjs`:
   - For each of the 11 supported planners (all 12 minus `PRMStar`), solve `scene1.json`
     at `timeLimitS = 2`, seed 42: assert the parsed object has `schema === "mlmp.run/1"`
     and **no** `error` key. This is the wasm equivalent of the native per-planner smoke
     at `ci.yml:30-44` and is what catches a `std::thread` or `/proc` abort.
   - `PRMStar` at the same settings must return `error.kind === "usage"` and must not throw
     or abort.
   - `BiQRRT` on `scene1.json`, seed 42, `timeLimitS = 10`: assert `solved === true`,
     `exact === true`, `seed === 42`, `path_length_raw > 0`, and that at least one trace
     frame was delivered to the callback.
   - Run that last case twice and assert `path_length_raw` and `path_length_simplified`
     match between the two. Counters are deliberately **not** compared — the multilevel
     planners consume the whole budget, so vertex and sample counts track runner speed, the
     same reasoning recorded at `ci.yml:69-71`.
   - Do **not** compare any value against the native build's numbers; differing libm
     implementations make cross-target floating-point equality an invalid assertion.

### Step 9 — Pages deployment

**A repository has one Pages source, and `docs/` already is it.** Track A's player is served
from `docs/` and the README links `https://tomsabala.github.io/MLMP/`. A workflow that
publishes `web/dist` alone takes that page offline, so this one publishes a **single**
artifact whose root is `docs/` — the player, the 12 packed fixtures, the GIF and the scene
copies — with the freshly built `web/dist` copied into `live/` inside it. The player keeps
resolving at `/MLMP/`, the wasm playground lands at `/MLMP/live/`, and nothing Track A
committed moves.

New `.github/workflows/pages.yml`: on push to `main`, build and extract `web/dist` exactly as
the CI job does, `cp -r docs _site && cp -r web/dist _site/live`, then
`actions/upload-pages-artifact@v3` with `path: _site` and `actions/deploy-pages@v4`, with
`permissions: { pages: write, id-token: write }` and a `github-pages` environment. No
`.nojekyll` concerns since nothing is underscore-prefixed.

**One-time manual step, not discoverable from the repo**: the repository's Pages source must
be switched from "Deploy from a branch → /docs" to "GitHub Actions". Until it is, the workflow
runs green and the site does not change. Record it in the PR description so it is not found as
a 404.

Both surfaces are cross-linked: `docs/index.html` gains a link to `live/`, and `web/index.html`
a link back to `../`. A visitor who lands on either must be able to reach the other.

## Critical files & anchors

| File | Anchor | Why |
|---|---|---|
| `mlmp/src/models/solver.h` | `solve()` at `:136`, PTC lambda `:197-222`, `captureFrame` `:379-420` | The contract the wasm entry point calls. Read before writing `entry.cpp`; do not modify. |
| `mlmp/src/main.cpp` | `main()` at `:118-190` | The template for `entry.cpp`: seed ordering, option fill, trace sink wiring, error shapes. |
| `mlmp/src/models/run_result.h` | `RunOptions` `:22-32`, `toJson` `:108-151`, `errorJson` `:155-158` | Exact field names and the two wire shapes the page branches on. |
| `mlmp/CMakeLists.txt` | `find_package(Boost …)` `:11`, `add_executable` `:14-17` | The one file where the native and wasm builds diverge. |
| `mlmp/src/state/kinematic_chain.h` | `computeRobotSegments` `:199-222`, bounds `:55-58` | The FK the renderer must reproduce, and the reason obstacle edits never change the state space. |

## Verification

Prerequisites: Docker with buildx; Node ≥ 18 for the smoke test. All commands from the
repo root.

1. **Build the module.**
   `docker build -f Dockerfile.wasm --target dist --output type=local,dest=web/dist .`
   Expect `web/dist/mlmp.wasm`, `web/dist/mlmp.mjs`, `web/dist/scenes/scene1.json` … `scene6.json`,
   `web/dist/scenes/index.json`, `web/dist/index.html`.

2. **Payload measurement (the roadmap's explicit requirement).**
   `gzip -9 -c web/dist/mlmp.wasm | wc -c; gzip -9 -c web/dist/mlmp.mjs | wc -c`
   Sum must be ≤ 5 242 880. Record both numbers. If the sum exceeds the budget, the first
   lever is confirming the static patch took effect:
   `emnm web/dist/mlmp.wasm | grep -c Benchmark` (or `wasm-objdump -x`) should find no
   `ompl::tools::Benchmark`, `PlannerDataStorage` or `CForest` symbols. Their presence means
   patch 2 or the `--gc-sections` behaviour failed, not that the planner set is too large.

3. **Headless planner proof.** `node web/test/smoke.mjs`
   Expect: 11 planners returning `mlmp.run/1` with no `error`; `PRMStar` returning
   `error.kind === "usage"`; `BiQRRT`/`scene1`/seed 42/10 s returning `solved: true`,
   `exact: true`, `path_length_raw > 0`, trace frames > 0; and identical
   `path_length_raw` across two such runs. This is the end-to-end proof that the planner
   actually plans in wasm — a build that links but aborts on `PRM`'s thread or on
   `/proc/self/stat` passes step 1 and fails here.
   Also assert the renderer's FK in the same `node` run: a synthetic level-0 vertex whose `q`
   carries nonzero entries above the owned prefix must produce the same tips as the identical
   vertex with that tail zeroed. If it does not, the renderer is drawing identity-fibre
   padding, which is the one bug that looks like a working renderer.

4. **Browser proof** (manual, and the actual Done-when of `ROADMAP.md:569-570`).
   `python3 -m http.server 8000 --directory web/dist`, open `http://127.0.0.1:8000/`.
   - The page loads with `scene1` drawn, planner `BiQRRT` selected, `PRMStar` greyed out.
   - Press Solve: trees appear progressively during the 5 s budget, not all at once at the
     end. Progressive appearance is the thing being verified — it is what distinguishes
     this from Track A's replay.
   - The result panel shows `exact: true` and a non-null `time_first_solution_s`, under the
     traced-run caption.
   - Drag an obstacle onto a robot's start pose and release: within well under a second the
     panel reports `invalid_start` with the inline message, and no frames were drawn.
   - Drag it back off: a normal solve runs and trees grow again.
   - Drag three times in quick succession while a solve is running: exactly one further
     solve runs afterwards, for the final obstacle position, and no stale frames appear.
   - Add an obstacle across the corridor between start and goal, then solve: expect either
     a longer path or `timeout`; both are correct outcomes to display.
   - Delete all obstacles and solve: a solution is still found (the padded boundary walls
     from `scene.h:150-161` remain).
   - Uncheck the top bundle level: the cloud visibly thins and the view does **not** move.
     Both halves matter — a renderer that renormalises extents fails the second.
   - Confirm the browser devtools console shows no `pthread`, `SharedArrayBuffer` or
     `Atomics` error, and the Network tab shows no COOP/COEP requirement.

5. **Retrofit proof** (after Track A rewires `docs/app.js` onto the module).
   `python3 -m http.server 8000 --directory docs`, open `http://127.0.0.1:8000/`, and scrub
   each scene's BiQRRT and QRRT panels end to end. The frames must be indistinguishable from
   the pre-retrofit page — same colours, same dot sizes, same extents. The fixtures did not
   change, so any visible difference is a regression in the shared module.

6. **Native regression.** `docker build -t mlmp:dev .` and
   `docker run --rm mlmp:dev --algo BiQRRT --input-file scene1.json --time-limit 10 --seed 42`
   must still exit 0 and print the same `path_length_raw` as before this work. Step 2 of the
   plan edits `mlmp/CMakeLists.txt`, which the native build shares.

## Assumptions & contingencies

- **`mlmp/tools/traceGif.py` keeps a Python FK**, because it cannot import a JS module. Two
  implementations of the same drawing therefore exist and will drift unless something cheap
  checks them: either assert the shared constants match (`TREE_COLORS`, `PADDING`, the
  `0.35 + 0.65 * level / max(1, links - 1)` alpha, the `2.4` / `1.5` radii), or render one
  `scene1` frame both ways and compare tip coordinates. They agree today; the point is to
  notice when they stop.
- **`emscripten/emsdk:3.1.64` supports `-fwasm-exceptions` and module workers.** If the
  pinned tag fails to build, move forward to the latest `emscripten/emsdk` release tag
  rather than falling back to `-fexceptions`; record the tag that worked. Only if wasm
  exceptions themselves prove unavailable should `-fexceptions` be substituted, and then
  the payload measurement in step 2 must be redone.
- **`-DCMAKE_FIND_ROOT_PATH_MODE_INCLUDE=BOTH` / `…_PACKAGE=BOTH` make the host's
  header-only Boost, Eigen3 and nlohmann_json visible to the Emscripten toolchain.** If
  configure still reports any of them missing, copy the headers and CMake config files into
  the Emscripten sysroot instead — `cp -r /usr/include/boost /usr/include/eigen3
  /usr/include/nlohmann "$(em-config CACHE)/sysroot/include/"` and the matching
  `/usr/share/eigen3` and `/usr/lib/*/cmake/nlohmann_json` directories — and drop the two
  flags.
- **BiQRRT solves `scene1` within the 5 s default in wasm.** If the step 3 smoke shows it
  needs more than 5 s, change the slider default to 10 s rather than shrinking or
  substituting the scene; the scene set is the committed regression set
  (`ROADMAP.md:464`). Record the measured wasm-vs-native throughput ratio either way — it
  is the number Track B will want when it reads these results.
