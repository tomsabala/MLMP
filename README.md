# Motion Planning Experiment

```sh
docker run --rm ghcr.io/tomsabala/mlmp:latest --algo BiQRRT --input-file scene1.json --time-limit 10
```

The GHCR package is currently **private**, so that pull needs
`docker login ghcr.io` with a token carrying `read:packages`. Until it is made public, build
locally instead — `docker build -t mlmp:dev .`, then substitute `mlmp:dev` for the image name in
every command below.

A 2D multi-robot kinematic-chain motion-planning experiment built on OMPL's
[`ompl::multilevel`](https://ompl.kavrakilab.org/multiLevelPlanning.html) bundle-space planners,
plus a custom bidirectional bundle-space planner, `BiQRRT`.

The work follows [Multilevel motion planning: A fiber bundle formulation](https://journals.sagepub.com/doi/full/10.1177/02783649231209337)
by [Andreas Orthey](https://scholar.google.com/citations?user=bQKreEMAAAAJ), Sohaib Akbar, and
[Prof. Marc Toussaint](https://scholar.google.com/citations?user=t2X4Mg8AAAAJ&hl=iw).

See [ROADMAP.md](ROADMAP.md) for where this is going.

## Usage

```
--input-file, -i   scene to plan on (required)
--time-limit, -t   planning time budget in seconds (default: 1.0)
--verbose,    -v    OMPL log output (default: false)
--algo,       -a   planner to run (default: BiQRRT)
--help             print the option list
```

### Algorithms

**Multilevel (bundle-space) planners** — plan on a sequence of kinematic chains of increasing
length, lifting solutions up the hierarchy:

`BiQRRT`, `QRRT`, `QRRTStar`, `QMP`, `QMPStar`

**Single-level baselines** — plan directly on the full chain:

`RRT`, `RRTConnect`, `PRMStar`, `BITStar`, `AITStar`, `EITStar`, `LBKPIECE`

The baselines are here to make `BiQRRT` results attributable: `RRT` vs. `RRTConnect` isolates
bidirectionality and `QRRT` vs. `BiQRRT` isolates it again inside the hierarchy, so a win can be
credited to the right mechanism rather than to "multilevel" in general. The asymptotically
optimal planners (`PRMStar`, `BITStar`, `AITStar`, `EITStar`) are the only honest path-length
comparison for `QRRTStar` and `QMPStar`.

### Exit codes

| Code | Meaning |
|---|---|
| 0 | A solution was found |
| 1 | No solution within the time budget |
| 2 | Usage error, unknown algorithm, or scene could not be read |

### Scene resolution

`--input-file` is used as-is when it names an existing file. Otherwise it is treated as a bare
scene name and resolved against `MLMP_SCENE_DIR`, which the image sets to `/opt/mlmp/scenes`.
Both of these work:

```sh
docker run --rm ghcr.io/tomsabala/mlmp:latest -a BiQRRT -i scene5.json -t 10
docker run --rm -v "$PWD/mlmp/src/inputs/experiment:/scenes" ghcr.io/tomsabala/mlmp:latest -i /scenes/scene1.json -t 10
```

The baked scenes are the six benchmark scenes `scene1.json`…`scene6.json`, plus the smaller
development scenes under `dev/`. Of those, BiQRRT solved `scene1` and `scene5` 3/3 at
`--time-limit 10`, while `scene4` and `scene6` were solved 0/3 by both BiQRRT and QRRT
at that budget, so expect exit 1 there. Those are stopwatch observations on one host, not a
benchmark — the real numbers are Phase 2 of the roadmap.

## Building

### Image

```sh
docker build -t mlmp:dev .
```

The builder stage compiles OMPL from source and then `mlmp`; the runtime stage carries only the
OMPL shared libraries, the `mlmp` binary and the scenes.

### Local, without Docker

```sh
cmake -S mlmp -B mlmp/build && cmake --build mlmp/build -j
```

Requires, from the distribution:

```
build-essential cmake
libboost-program-options-dev libboost-serialization-dev libboost-filesystem-dev libboost-system-dev
libeigen3-dev nlohmann-json3-dev
```

and **OMPL 1.7.0 installed from source** — no Ubuntu release ships it (noble's `libompl-dev` is
1.5.2, and `EITStar` only exists from 1.7.0 onwards):

```sh
git clone --depth 1 -b 1.7.0 https://github.com/ompl/ompl.git
cmake -S ompl -B ompl/build -DCMAKE_BUILD_TYPE=Release -DOMPL_BUILD_TESTS=OFF -DOMPL_REGISTRATION=OFF
cmake --build ompl/build --target ompl -j && sudo cmake --install ompl/build
```

The binary lands at `mlmp/build/mlmp`.

## Experiment harness

`mlmp/tools/experiment.py` sweeps algorithms × scenes × time limits and plots the results; it
expects the binary at `mlmp/build/mlmp` and is run from `mlmp/tools`. It is slated for a rewrite
— see Phase 2 of the roadmap.

[git link](https://github.com/tomsabala/MLMP)
