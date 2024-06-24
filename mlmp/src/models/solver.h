#include <math.h>
#include <boost/range/algorithm_ext/push_back.hpp>

#include <scene.h>
#include <state/kinematic_chain.h>
#include "common/geometry.h"


#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateSpace.h>
#include <ompl/multilevel/planners/qrrt/QRRT.h>
#include <ompl/multilevel/planners/qmp/QMP.h>
#include <ompl/multilevel/planners/qrrt/QRRTStar.h>
#include <ompl/multilevel/planners/qmp/QMPStar.h>

using namespace ompl::base;
std::vector<Environment> envs;

namespace mlmp {
    PlannerPtr GetQMPStar(std::vector<SpaceInformationPtr> si_vec, std::vector<int> sequenceLinks, unsigned int numLinks) {
        auto planner = std::make_shared< ompl::multilevel::QMPStar>(si_vec);
            std::string qName = "QMPStar[";

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

    PlannerPtr GetQRRTStar(std::vector<SpaceInformationPtr> si_vec, std::vector<int> sequenceLinks, unsigned int numLinks) {
        auto planner = std::make_shared< ompl::multilevel::QRRTStar>(si_vec);
        std::string qName = "QRRTStar[";

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

    PlannerPtr GetQMP(std::vector<SpaceInformationPtr> si_vec, std::vector<int> sequenceLinks, unsigned int numLinks) {
        auto planner = std::make_shared< ompl::multilevel::QMP>(si_vec);
        std::string qName = "QMP[";

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

    PlannerPtr GetQRRT(std::vector<SpaceInformationPtr> si_vec, std::vector<int> sequenceLinks, unsigned int numLinks) {
        auto planner = std::make_shared< ompl::multilevel::QRRT>(si_vec);
        std::string qName = "QRRT[";

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

    PlannerPtr GetBiQRRT(std::vector<SpaceInformationPtr> si_vec, std::vector<int> sequenceLinks, unsigned int numLinks) {
        auto planner = std::make_shared< ompl::multilevel::BiQRRT>(si_vec);
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


    PlannerPtr GetPlanner(std::string algo, std::vector<int> sequenceLinks, std::vector<mlmp::common::Point> pinnedPositions, double linkLength, unsigned int numRobots, unsigned int numLinks, SpaceInformationPtr si)
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
        
        if (algo == "BiQRRT") {
            return GetBiQRRT(si_vec, sequenceLinks, numLinks);
        }
        if (algo == "QRRT") {
            return GetQRRT(si_vec, sequenceLinks, numLinks);
        }
        if (algo == "QMP") {
            return GetQMP(si_vec, sequenceLinks, numLinks);
        }
        if (algo == "QRRTStar") {
            return GetQRRTStar(si_vec, sequenceLinks, numLinks);
        }
        if (algo == "QMPStar") {
            return GetQMPStar(si_vec, sequenceLinks, numLinks);
        }
        throw std::runtime_error("Incompatible algorithm: " + algo);
        
    }
    
    
    class Solver {
        Scene scene;
        ProblemDefinitionPtr pdef;
        std::vector<SpaceInformationPtr> siVec;

        public:

        Solver(Scene scene) : scene(scene) {}

        void solve(std::string algo, double& timeLimit) {
            OMPL_DEBUG("------On solver::solve------");
            OMPL_DEBUG("robots %d joints %d", scene.getR(), scene.getN());

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
            ompl::base::PlannerPtr planner = GetPlanner(algo, discrete, scene.getPinnedPositions(), scene.getJointLength(), scene.getR(), scene.getN(), ss.getSpaceInformation());
            if (planner == nullptr) {
                OMPL_DEBUG("Incompatible algorithm");
                return;
            }
            ss.setPlanner(planner);
            
            PlannerStatus status = ss.solve(timeLimit);

            double timeToCompute = ss.getLastPlanComputationTime();
            if (status)
            {
                const ompl::base::ProblemDefinitionPtr pdef = planner->getProblemDefinition();
                ob::PathPtr path = pdef->getSolutionPath();
                og::PathSimplifierPtr simplifier = ss.getPathSimplifier();
                simplifier->simplifyMax(*(path->as<og::PathGeometric>()));
                pdef->getSolutionPath()->print(std::cout);  
                std::cout<<std::string(80, '*')<<std::endl;
                std::cout<<"Length\n"<<pdef->getSolutionPath()->length()<<std::endl; 
                std::cout<<std::string(80, '*')<<std::endl;        
                std::cout<<"Runtime\n"<<timeToCompute<<std::endl;
            }
            else
            {
                std::cout<<"Not Found"<<std::endl;
            }   
        }
    };
}