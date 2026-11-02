#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include <boost/program_options.hpp>

#include <ompl/util/Console.h>
#include <ompl/util/RandomNumbers.h>

#include <models/run_result.h>
#include <models/scene.h>
#include <models/solver.h>

namespace po = boost::program_options;

namespace {

/* OMPL's default OutputHandlerSTD writes INFO/DEBUG to std::cout, which would
 * interleave with the result object and corrupt it under --verbose. */
class StderrOutputHandler : public ompl::msg::OutputHandler {
public:
    void log(const std::string& text, ompl::msg::LogLevel level, const char* filename,
             int line) override {
        std::cerr << "[" << static_cast<int>(level) << "] " << text
                  << " (" << filename << ":" << line << ")\n";
    }
};

struct CliOptions {
    std::string inputFile;
    bool verbose = false;
    double timeLimit = 1.0;
    std::string algo;
    bool hasSeed = false;
    unsigned long seed = 0;
    bool noSimplify = false;
    std::string traceFile;
    unsigned int traceIntervalMs = 50;
    unsigned int traceMaxFrames = 400;
};

/* Every failure prints a human message on stderr and exactly one mlmp.run/1 error
 * object on stdout, so a harness never has to parse prose. */
int emitError(const std::string& kind, const std::string& message) {
    std::cerr << "Error: " << message << "\n";
    std::cout << mlmp::errorJson(kind, message).dump() << "\n";
    return 2;
}

void validateAlgorithm(const std::string& value) {
    if (!mlmp::isSupportedAlgorithm(value)) {
        throw po::validation_error(po::validation_error::invalid_option_value, value);
    }
}

/* A scene argument that names an existing file is used as-is; a bare name is
 * resolved against MLMP_SCENE_DIR (the published image bakes the scenes in). */
std::string resolveScenePath(const std::string& inputFile) {
    namespace fs = std::filesystem;
    if (fs::exists(inputFile)) {
        return inputFile;
    }
    const char* dir = std::getenv("MLMP_SCENE_DIR");
    const fs::path candidate = fs::path(dir ? dir : "/opt/mlmp/scenes") / inputFile;
    if (fs::exists(candidate)) {
        return candidate.string();
    }
    throw std::runtime_error("scene not found: " + inputFile +
                             " (also tried " + candidate.string() + ")");
}

void parseArguments(int argc, char* argv[], CliOptions& opts) {
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "")
        ("input-file,i", po::value<std::string>(&opts.inputFile)->required(), "input file")
        ("time-limit,t", po::value<double>(&opts.timeLimit)->default_value(1.0), "run time limit")
        ("verbose,v", po::value<bool>(&opts.verbose)->default_value(false), "verbose")
        ("algo,a", po::value<std::string>(&opts.algo)
            ->default_value("BiQRRT")
            ->notifier(validateAlgorithm),
            ("set the algorithm (allowed values: " + mlmp::supportedAlgorithmList() + ")").c_str())
        ("seed", po::value<unsigned long>(&opts.seed),
            "RNG seed; omitted means a generated seed, reported either way")
        ("no-simplify", po::bool_switch(&opts.noSimplify), "skip path simplification")
        ("trace-file", po::value<std::string>(&opts.traceFile),
            "write a mlmp.trace/1 JSONL stream to this path")
        ("trace-interval-ms", po::value<unsigned int>(&opts.traceIntervalMs)->default_value(50),
            "trace sampling interval")
        ("trace-max-frames", po::value<unsigned int>(&opts.traceMaxFrames)->default_value(400),
            "stop tracing after this many frames");

    po::variables_map vm;
    try {
        po::store(po::parse_command_line(argc, argv, desc), vm);

        if (vm.count("help")) {
            std::cout << desc << "\n";
            std::exit(0);
        }

        po::notify(vm);
    } catch (const po::error& e) {
        std::cerr << desc << "\n";
        std::exit(emitError("usage", e.what()));
    }

    opts.hasSeed = vm.count("seed") > 0;
}

}

/* Exit codes: 0 solved, 1 no solution within the time budget, 2 usage/IO error.
 * No stdout at all means the process crashed. */
int main(int argc, char* argv[]) {
    static StderrOutputHandler handler;
    ompl::msg::useOutputHandler(&handler);

    CliOptions cli;
    parseArguments(argc, argv, cli);

    if (cli.inputFile.empty()) {
        return emitError("usage", "Required arguments are missing.");
    }

    if (!cli.verbose) {
        ompl::msg::setLogLevel(ompl::msg::LOG_NONE);
    }

    /* Before any OMPL object exists: KinematicChainProjector's constructor draws from
     * the global generator, so seeding after Scene/Solver construction would not be
     * reproducible. */
    if (cli.hasSeed) {
        ompl::RNG::setSeed(static_cast<std::uint_fast32_t>(cli.seed));
    }
    const std::uint_fast32_t seedInUse = ompl::RNG::getSeed();

    mlmp::Scene scene;
    std::string scenePath;
    try {
        scenePath = resolveScenePath(cli.inputFile);
        scene.loadScene(scenePath);
    } catch (const std::exception& e) {
        return emitError("io", e.what());
    }

    mlmp::RunOptions options;
    options.algo = cli.algo;
    options.sceneName = cli.inputFile;
    options.scenePath = scenePath;
    options.timeLimit = cli.timeLimit;
    options.simplify = !cli.noSimplify;
    options.traceIntervalS = cli.traceIntervalMs / 1000.0;
    options.traceMaxFrames = cli.traceMaxFrames;

    std::ofstream traceOut;
    if (!cli.traceFile.empty()) {
        traceOut.open(cli.traceFile);
        if (!traceOut) {
            return emitError("io", "unable to open trace file: " + cli.traceFile);
        }
        options.traceFile = cli.traceFile;

        const mlmp::Json header = {{"schema", "mlmp.trace/1"},
                                   {"planner", cli.algo},
                                   {"scene", cli.inputFile},
                                   {"seed", static_cast<std::uint64_t>(seedInUse)},
                                   {"robots", scene.getR()},
                                   {"links", scene.getN()},
                                   {"time_limit_s", cli.timeLimit},
                                   {"interval_ms", cli.traceIntervalMs}};
        traceOut << header.dump() << "\n";

        options.traceSink = [&traceOut](const mlmp::Json& frame) {
            traceOut << frame.dump() << "\n";
        };
    }

    try {
        mlmp::Solver solver(scene);
        const mlmp::RunResult result = solver.solve(options);
        std::cout << mlmp::toJson(result).dump() << "\n";
        return result.solved ? 0 : 1;
    } catch (const std::exception& e) {
        return emitError("runtime", e.what());
    }
}
