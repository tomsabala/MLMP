#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>

#include <boost/program_options.hpp>

#include <ompl/util/Console.h>

#include <models/scene.h>
#include <models/solver.h>

namespace po = boost::program_options;

namespace {

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

void parseArguments(int argc, char* argv[],
                    std::string& inputFile,
                    bool& verbose,
                    double& timeLimit,
                    std::string& algo) {
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "")
        ("input-file,i", po::value<std::string>(&inputFile)->required(), "input file")
        ("time-limit,t", po::value<double>(&timeLimit)->default_value(1.0), "run time limit")
        ("verbose,v", po::value<bool>(&verbose)->default_value(false), "verbose")
        ("algo,a", po::value<std::string>(&algo)
            ->default_value("BiQRRT")
            ->notifier(validateAlgorithm),
            ("set the algorithm (allowed values: " + mlmp::supportedAlgorithmList() + ")").c_str());

    po::variables_map vm;
    try {
        po::store(po::parse_command_line(argc, argv, desc), vm);

        if (vm.count("help")) {
            std::cout << desc << "\n";
            std::exit(0);
        }

        po::notify(vm);
    } catch (const po::error& e) {
        std::cerr << "Error: " << e.what() << "\n";
        std::cerr << desc << "\n";
        std::exit(2);
    }
}

}

/* Exit codes: 0 solved, 1 no solution within the time budget, 2 usage/IO error. */
int main(int argc, char* argv[]) {
    std::string inputFile;
    bool verbose = false;
    double timeLimit = 1.0;
    std::string algoName;

    parseArguments(argc, argv, inputFile, verbose, timeLimit, algoName);

    if (inputFile.empty()) {
        std::cerr << "Error: Required arguments are missing.\n";
        std::exit(2);
    }

    if (!verbose) {
        ompl::msg::setLogLevel(ompl::msg::LOG_NONE);
    }

    try {
        mlmp::Scene scene;
        scene.loadScene(resolveScenePath(inputFile));

        mlmp::Solver solver(scene);
        return solver.solve(algoName, timeLimit) ? 0 : 1;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 2;
    }
}
