#include <cstdlib>
#include <iostream>
#include <set>
#include <string>

#include <boost/program_options.hpp>

#include <ompl/util/Console.h>

#include <models/scene.h>
#include <models/solver.h>
#include <planner/BiQRRT.h>

namespace po = boost::program_options;

namespace {

const std::set<std::string> ALLOWED_ALGORITHMS = {
    "QRRTStar", "QRRT", "BiQRRT", "QMP", "QMPStar"
};

void validateAlgorithm(const std::string& value) {
    if (ALLOWED_ALGORITHMS.find(value) == ALLOWED_ALGORITHMS.end()) {
        throw po::validation_error(po::validation_error::invalid_option_value, value);
    }
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
            "set the algorithm (allowed values: QMP, QMPStar, QRRT, QRRTStar, BiQRRT)");

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
        std::exit(1);
    }
}

}

int main(int argc, char* argv[]) {
    std::string inputFile;
    bool verbose = false;
    double timeLimit = 1.0;
    std::string algoName;

    parseArguments(argc, argv, inputFile, verbose, timeLimit, algoName);

    if (inputFile.empty()) {
        std::cerr << "Error: Required arguments are missing.\n";
        std::exit(1);
    }

    if (!verbose) {
        ompl::msg::setLogLevel(ompl::msg::LOG_NONE);
    }

    mlmp::Scene scene;
    scene.loadScene(inputFile);

    mlmp::Solver solver(scene);
    solver.solve(algoName, timeLimit);

    return 0;
}
