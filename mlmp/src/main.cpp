
#include <boost/program_options.hpp>
#include <iostream>
#include <string>
#include <boost/math/constants/constants.hpp>

#include <planner/BiQRRT.h> 
#include <models/scene.h>
#include <models/solver.h>

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateSpace.h>
#include <ompl/multilevel/planners/qrrt/QRRT.h>

using namespace ompl::base;
using namespace ompl::multilevel;

using SE3State = ScopedState<SE3StateSpace>;
using SO3State = ScopedState<SO3StateSpace>;
using R3State = ScopedState<RealVectorStateSpace>;
const double pi = boost::math::constants::pi<double>();

namespace po = boost::program_options;


void validate_algorithm(const std::string& value, const std::set<std::string>& allowed_values) {
    if (allowed_values.find(value) == allowed_values.end()) {
        throw po::validation_error(po::validation_error::invalid_option_value, value);
    }
}

void parse_arguments(int ac, char* av[], std::string& inputFile,
                                         bool& verbose,
                                         double& timeLimit,
                                         std::string& algo) {
    std::set<std::string> allowed_algorithms = {"QRRTStar", "QRRT", "BiQRRT", "QMP", "QMPStar"};

    // Define the options
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "")
        ("input-file,i", po::value<std::string>(&inputFile)->required(), "input file")
        ("time-limit,t", po::value<double>(&timeLimit)->default_value(1.0), "run time limit")
        ("verbose,v", po::value<bool>(&verbose)->default_value(false), "verbose")
        ("algo,a", po::value<std::string>(&algo)->default_value("BiQRRT")->notifier([&allowed_algorithms](const std::string& value) {
                validate_algorithm(value, allowed_algorithms);
            }), "set the algorithm (allowed values: QMP, QMPStar, QRRT, QRRTStar BiQRRT)");

    po::variables_map vm;
    try {
        po::store(po::parse_command_line(ac, av, desc), vm);
        
        if (vm.count("help")) {
            std::cout << desc << "\n";
            std::exit(1);
        }

        po::notify(vm);
    } catch (const po::error& e) {
        std::cerr << "Error: " << e.what() << "\n";
        std::cerr << desc << "\n";
        std::exit(1);
    }
}


int main(int argc, char* argv[])
{
    std::string inputFile = "";
    bool verbose;
    double timeLimit;
    std::string algoName;

    parse_arguments(argc, argv, inputFile, verbose, timeLimit, algoName);
    
    if (inputFile.empty()) {
        std::cerr << "Error: Required arguments are missing.\n";
        std::exit(1);
    }
    
    if(!verbose) {
        ompl::msg::setLogLevel(ompl::msg::LOG_NONE);
    }

    mlmp::Scene scene;
    scene.loadScene(inputFile);
     
    mlmp::Solver solver(scene);
    solver.solve(algoName, timeLimit);
}