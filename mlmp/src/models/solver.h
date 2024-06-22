#include <math.h>
#include <boost/range/algorithm_ext/push_back.hpp>

#include <abstraction_type.h>
#include <scene.h>
#include <state/kinematic_chain.h>
#include "common/geometry.h"


#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateSpace.h>
#include <ompl/multilevel/planners/qrrt/QRRT.h>

typedef mlmp::abstraction::AbstractionType AbstractionType;
using namespace ompl::base;
std::vector<Environment> envs;

namespace mlmp {
    PlannerPtr GetPlanner(std::vector<int> sequenceLinks, std::vector<mlmp::common::Point> pinnedPositions, double linkLength, unsigned int numRobots, unsigned int numLinks, SpaceInformationPtr si)
    {
        std::vector<SpaceInformationPtr> si_vec;
    
        for (unsigned int k = 0; k < sequenceLinks.size(); k++)
        {
            auto links = sequenceLinks.at(k) - 1;
            assert(links < numLinks);
            OMPL_INFORM("Create MultiLevel Chain with %d links, and %d robots", links, numRobots);
            auto spaceK(std::make_shared<KinematicChainSpace>(numRobots, links, linkLength, &envs.at(links)));

            auto siK = std::make_shared<SpaceInformation>(spaceK);
            siK->setStateValidityChecker(std::make_shared<KinematicChainValidityChecker>(siK, pinnedPositions, numRobots, links));
            spaceK->setup();
            si_vec.push_back(siK);
        }
    
        OMPL_INFORM("Add Original Chain with %d links, and %d robots", numLinks, numRobots);
        si_vec.push_back(si);
    
        auto planner = std::make_shared<ompl::multilevel::BiQRRT>(si_vec);
    
        std::string qName = "BiQRRT[";
        for (unsigned int k = 0; k < sequenceLinks.size(); k++)
        {
            int links = sequenceLinks.at(k);
            qName += std::to_string(links) + ",";
        }
        qName += std::to_string(numLinks);
        qName += "]";
        planner->setName(qName);
        return planner;
    }
    
    
    class Solver {
        Scene scene;
        ProblemDefinitionPtr pdef;
        std::vector<SpaceInformationPtr> siVec;
        bool verbose;

        public:

        Solver(Scene scene, bool debug) : scene(scene), verbose(debug) {}

        void solve(double& timeLimit) {
            if (verbose) {
                std::cout<<"------On solver::solve------"<<std::endl;
                std::cout<<"robots "<<scene.getR()<<" | joints "<<scene.getN()<<std::endl;
            }

            std::vector<mlmp::common::Segment> commonSegments = scene.getObstaclesSegments();
            Environment env = commonSegments;
    
            for (unsigned int k = 0; k < scene.getN(); k++)
            {
                envs.push_back(scene.getObstaclesSegments());
            }

            auto chain(std::make_shared<KinematicChainSpace>(scene.getR(), scene.getN(), scene.getJointLength(), &env));
            ompl::geometric::SimpleSetup ss(chain);
        
            ss.setStateValidityChecker(std::make_shared<KinematicChainValidityChecker>(ss.getSpaceInformation(), scene.getPinnedPositions(), scene.getR(), scene.getN()));
        
            ompl::base::ScopedState<> start(chain), goal(chain);
            std::vector<double> startVec;
            startVec.reserve(scene.getN()*scene.getR());
            std::vector<std::vector<double>> startAnglesByRobots = scene.getStartAngles();
            for (unsigned int i = 0; i < scene.getN(); ++i) {
                for (unsigned int j = 0; j < scene.getR(); ++j) {
                    startVec.emplace_back(startAnglesByRobots[j][i]);
                }
            }
            std::vector<double> goalVec;
            goalVec.reserve(scene.getN()*scene.getR());
            std::vector<std::vector<double>> goalAnglesByRobots = scene.getGoalAngles();
            for (unsigned int i = 0; i < scene.getN(); ++i) {
                for (unsigned int j = 0; j < scene.getR(); ++j) {
                    goalVec.emplace_back(goalAnglesByRobots[j][i]);
                }
            }

            chain->setup();
            chain->copyFromReals(start.get(), startVec);
            chain->copyFromReals(goal.get(), goalVec);
            ss.setStartAndGoalStates(start, goal);
            
            std::vector<int> discrete;
            boost::push_back(discrete, boost::irange(2, scene.getN() + 1));
            ompl::base::PlannerPtr planner = GetPlanner(discrete, scene.getPinnedPositions(), scene.getJointLength(), scene.getR(), scene.getN(), ss.getSpaceInformation());

            ss.setPlanner(planner);
            
            PlannerStatus status = ss.solve(timeLimit);
        
            double timeToCompute = ss.getLastPlanComputationTime();
        
            if (status)
            {
                const ompl::base::ProblemDefinitionPtr pdef = planner->getProblemDefinition();
                std::cout << std::string(80, '-') << std::endl;
                pdef->getSolutionPath()->print(std::cout);
                std::cout << std::string(80, '-') << std::endl;           
            }
            else
            {
                OMPL_ERROR("Failed finding solution after %f seconds.", timeToCompute);
            }
        }
    };
}