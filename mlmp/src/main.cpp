
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

void parse_arguments(int ac, char* av[], std::string& abstractionLevel,
                                         std::string& inputFile) {
    // Define the options
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "")
        ("abstraction", po::value<std::string>(&abstractionLevel)->required(), "set abstraction level")
        ("input-file", po::value<std::string>(&inputFile), "input file");

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
    std::string abstractionLevel = "";
    std::string inputFile = "";

    parse_arguments(argc, argv, abstractionLevel, inputFile);
    
    if (abstractionLevel.empty() || inputFile.empty()) {
        std::cerr << "Error: Required arguments are missing.\n";
        std::exit(1);
    }

    mlmp::Scene scene;
    scene.loadScene(inputFile, true);
    const auto abstraction = mlmp::abstraction::fromValue(abstractionLevel); 
     
    // mlmp::Solver solver(scene, abstraction);
    // solver.setup();
    // const auto& result = solver.solve();
    // if (!result) {
    //     std::cout<<"FAILED"<<std::endl;
    // }


    std::cout << "-----DEBUG-----" << std::endl;
    auto rvSpace(std::make_shared<RealVectorStateSpace>(3));
    rvSpace->setBounds(0, 360);
    rvSpace->setLongestValidSegmentFraction(0.01);
    SpaceInformationPtr siPtr(std::make_shared<SpaceInformation>(rvSpace));
    siPtr->setStateValidityChecker([&](const State *state) -> bool {
        return scene.isStateValid(state, 1, 3);  
    });
    ompl::base::ScopedState<> state(rvSpace);
    state[0] = 30;
    state[1] = 0;
    state[2] = 0;

    scene.isStateValid(state.get(), 1, 3);
}