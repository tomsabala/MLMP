
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

using SE3State = ScopedState<SE3StateSpace>;
using SO3State = ScopedState<SO3StateSpace>;
using R3State = ScopedState<RealVectorStateSpace>;
const double pi = boost::math::constants::pi<double>();

namespace po = boost::program_options;

void parse_arguments(int ac, char* av[], std::string& inputFile,
                                         bool& verbose,
                                         double& timeLimit,
                                         bool& internalV) {
    // Define the options
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "")
        ("input-file", po::value<std::string>(&inputFile)->required(), "input file")
        ("time-limit", po::value<double>(&timeLimit)->default_value(1.0), "run time limit")
        ("v", po::value<bool>(&verbose)->default_value(false), "verbose")
        ("iv", po::value<bool>(&internalV)->default_value(false), "internal verbose");

    po::variables_map vm;
    try {
        po::store(po::parse_command_line(ac, av, desc), vm);
        
        // Handle the help option
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
    //############################################################################
    // Step 1: Setup planning problem using several quotient-spaces
    //############################################################################
    // Setup scene
    std::string inputFile = "";
    bool verbose;
    bool internalV;
    double timeLimit;

    parse_arguments(argc, argv, inputFile, verbose, timeLimit, internalV);
    
    if (inputFile.empty()) {
        std::cerr << "Error: Required arguments are missing.\n";
        std::exit(1);
    }
    
    if(!verbose) {
        ompl::msg::setLogLevel(ompl::msg::LOG_NONE);
    }

    mlmp::Scene scene;
    scene.loadScene(inputFile, internalV);
     
    mlmp::Solver solver(scene, verbose);
    solver.solve(timeLimit);
}