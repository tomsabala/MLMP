/* The wasm analogue of main.cpp: it owns all I/O and option handling, Solver owns none.
 *
 * solveScene returns the mlmp.run/1 object as a JSON *string* and hands every trace frame to the
 * callback as a mlmp.trace/1 JSON string. Serialising in C++ guarantees the browser and the CLI
 * emit identical bytes, and keeps embind out of the frozen schema — the caller JSON.parses.
 *
 * The call is synchronous and blocking for the whole time budget, which is why the page runs it
 * inside a Worker. */
#include <cstdint>
#include <exception>
#include <string>

#include <emscripten/bind.h>
#include <emscripten/val.h>

#include <ompl/util/Console.h>
#include <ompl/util/RandomNumbers.h>

#include <models/run_result.h>
#include <models/scene.h>
#include <models/solver.h>

namespace {

/* `scenePath` is an absolute MEMFS path written by the caller, so main.cpp's resolveScenePath /
 * MLMP_SCENE_DIR lookup is deliberately not reused. `seed` is a double because every JS number
 * is one. */
std::string solveScene(std::string scenePath, std::string algo, double timeLimitS, double seed,
                       double traceIntervalS, unsigned int traceMaxFrames,
                       emscripten::val onFrame) {
    /* No custom output handler: main.cpp's StderrOutputHandler exists only to keep OMPL's
     * logging off the CLI's stdout contract, and there is no stdout contract here. */
    ompl::msg::setLogLevel(ompl::msg::LOG_NONE);

    if (algo == "PRMStar") {
        return mlmp::errorJson("usage",
                               "PRMStar is unavailable in the browser build: "
                               "ompl::geometric::PRM::solve requires threads")
            .dump();
    }
    if (!mlmp::isSupportedAlgorithm(algo)) {
        return mlmp::errorJson("usage", "unknown algorithm: " + algo).dump();
    }

    /* Before any OMPL object exists, exactly as main.cpp does: KinematicChainProjector's
     * constructor draws from the global generator. The Worker is long-lived and the RNG is
     * process-global, so re-seeding per call is what makes solve N reproducible. */
    ompl::RNG::setSeed(static_cast<std::uint_fast32_t>(seed));

    mlmp::Scene scene;
    try {
        scene.loadScene(scenePath);
    } catch (const std::exception& e) {
        return mlmp::errorJson("io", e.what()).dump();
    }

    mlmp::RunOptions options;
    options.algo = algo;
    options.sceneName = scenePath;
    options.scenePath = scenePath;
    options.timeLimit = timeLimitS;
    options.simplify = true;
    options.traceIntervalS = traceIntervalS;
    options.traceMaxFrames = traceMaxFrames;
    /* traceFile stays empty: it is informational. result.traced follows traceSink being set,
     * so a traced run still reports frames, truncation and errors.
     *
     * No callback means no sink at all, mirroring an absent --trace-file on the CLI. That is
     * not just an optimisation: capturing a frame calls BundleSpaceSequence::getPlannerData,
     * whose path-restriction machinery draws from the global RNG, and captures land on
     * wall-clock boundaries. A traced multilevel run therefore consumes a host-speed-dependent
     * number of random draws and is not seed-reproducible; an untraced one is (measured:
     * BiQRRT/scene1/seed 42 repeats path_length_raw exactly while its vertex count varies).
     */
    if (!onFrame.isUndefined() && !onFrame.isNull()) {
        options.traceSink = [&onFrame](const mlmp::Json& frame) { onFrame(frame.dump()); };
    }

    try {
        /* A fresh Scene and Solver per call is required, not an optimisation: Solver holds its
         * Scene by value and builds Environment in its constructor, so a moved obstacle cannot
         * reach an existing instance. */
        mlmp::Solver solver(scene);
        const mlmp::RunResult result = solver.solve(options);
        return mlmp::toJson(result).dump();
    } catch (const std::exception& e) {
        return mlmp::errorJson("runtime", e.what()).dump();
    }
}

}

EMSCRIPTEN_BINDINGS(mlmp) {
    emscripten::function("solveScene", &solveScene);
}

/* An embind-only module still links a main(); without it the link fails on an undefined entry
 * symbol. */
int main() {
    return 0;
}
