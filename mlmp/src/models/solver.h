#include <math.h>

#include <abstraction_type.h>
#include <scene.h>

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateSpace.h>
#include <ompl/multilevel/planners/qrrt/QRRT.h>

typedef mlmp::abstraction::AbstractionType AbstractionType;
using namespace ompl::base;

namespace mlmp {
    class Solver {
        Scene scene;
        AbstractionType abstractionLevel;
        ProblemDefinitionPtr pdef;
        std::vector<SpaceInformationPtr> siVec;

        public:

        Solver(Scene scene, const AbstractionType& abstractionType) : scene(scene), abstractionLevel(abstractionType) {}

        void setup() {
            auto originSpace(std::make_shared<RealVectorStateSpace>(scene.getN() * scene.getR()));
            originSpace->setBounds(0, 360);
            originSpace->setLongestValidSegmentFraction(0.01);
            SpaceInformationPtr originSiPtr(std::make_shared<SpaceInformation>(originSpace));
            originSiPtr->setStateValidityChecker([&](const State *state) -> bool {
             return scene.isStateValid(state, scene.getR(), scene.getN());  
            });

            switch (abstractionLevel)
            {
            case AbstractionType::Projection:
                for (auto i=1; i<scene.getN(); ++i) {
                    const auto& numberOfRobots = scene.getR();
                    auto rvSpace(std::make_shared<RealVectorStateSpace>(numberOfRobots * i));
                    rvSpace->setBounds(0, 360);
                    rvSpace->setLongestValidSegmentFraction(0.01);
                    SpaceInformationPtr siPtr(std::make_shared<SpaceInformation>(rvSpace));
                    siPtr->setStateValidityChecker([&](const State *state) -> bool {
                     return scene.isStateValid(state, numberOfRobots, i);  
                    });
                    siVec.emplace_back(siPtr);
                }
                break;
            
            case AbstractionType::PriorityOrder:
                for (auto i=1; i<scene.getR(); ++i) {
                    const auto& numberOfJoints = scene.getN();
                    auto rvSpace(std::make_shared<RealVectorStateSpace>(numberOfJoints * i));
                    rvSpace->setBounds(0, 360);
                    rvSpace->setLongestValidSegmentFraction(0.01);
                    SpaceInformationPtr siPtr(std::make_shared<SpaceInformation>(rvSpace));
                    siPtr->setStateValidityChecker([&](const State *state) -> bool {
                     return scene.isStateValid(state, i, numberOfJoints);  
                    });
                    siVec.emplace_back(siPtr);
                }
                break;
            }

            siVec.emplace_back(originSiPtr);

            // Define Planning Problem
            ompl::base::ScopedState<> start(originSpace), goal(originSpace);
            for (auto r=0; r<scene.getR(); ++r) {
                for (auto j=0; j<scene.getN(); ++j) {
                    start[r*scene.getN() + j] = scene.getStartAngleAt(r, j);
                    goal[r*scene.getN() + j] = scene.getGoalAngleAt(r, j);
                }
            }
        
            pdef = std::make_shared<ProblemDefinition>(originSiPtr);
            pdef->setStartAndGoalStates(start, goal);
        }
    
        bool solve() {
            auto planner = std::make_shared<ompl::multilevel::BiQRRT>(siVec);
  
            // Planner can be used as any other OMPL algorithm
            planner->setProblemDefinition(pdef);
            planner->setup();
        
            PlannerStatus solved = planner->Planner::solve(60);
        
            if (solved)
            {
                std::cout << std::string(80, '-') << std::endl;
                pdef->getSolutionPath()->print(std::cout);
                std::cout << std::string(80, '-') << std::endl;
                return 1;
            }
            return 0;
        }
    };
}