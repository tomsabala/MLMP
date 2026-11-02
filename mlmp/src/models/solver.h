#ifndef SOLVER_H
#define SOLVER_H

#include <cmath>
#include <set>
#include <string>
#include <boost/range/algorithm_ext/push_back.hpp>

#include <models/scene.h>
#include <state/kinematic_chain.h>
#include "common/geometry.h"

#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/informedtrees/AITstar.h>
#include <ompl/geometric/planners/informedtrees/EITstar.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>

#include <ompl/multilevel/planners/qrrt/QRRT.h>
#include <ompl/multilevel/planners/qmp/QMP.h>
#include <ompl/multilevel/planners/qrrt/QRRTStar.h>
#include <ompl/multilevel/planners/qmp/QMPStar.h>
#include <planner/BiQRRT.h>

namespace og = ompl::geometric;

using namespace ompl::base;

namespace mlmp {

/* The supported planner set. These two sets are the single source of truth:
 * the CLI validates against them and GetPlanner dispatches on them. */
inline const std::set<std::string> kMultilevelAlgorithms = {
    "BiQRRT", "QRRT", "QRRTStar", "QMP", "QMPStar"};
inline const std::set<std::string> kSingleLevelAlgorithms = {
    "RRT", "RRTConnect", "PRMStar", "BITStar", "AITStar", "EITStar", "LBKPIECE"};

inline bool isSupportedAlgorithm(const std::string& algo) {
    return kMultilevelAlgorithms.count(algo) > 0 || kSingleLevelAlgorithms.count(algo) > 0;
}

inline std::string supportedAlgorithmList() {
    std::string list;
    for (const auto* group : {&kMultilevelAlgorithms, &kSingleLevelAlgorithms}) {
        for (const auto& name : *group) {
            if (!list.empty()) {
                list += ", ";
            }
            list += name;
        }
    }
    return list;
}

namespace {
    std::vector<Environment> envs;

    std::string buildPlannerName(const std::string& baseName,
                                  const std::vector<int>& sequenceLinks,
                                  unsigned int numLinks) {
        std::string name = baseName + "[";
        for (int links : sequenceLinks) {
            name += std::to_string(links) + ",";
        }
        name += std::to_string(numLinks) + "]";
        return name;
    }

    template<typename PlannerType>
    PlannerPtr createPlanner(const std::string& baseName,
                             /* BundleSpaceSequence takes siVec by non-const reference. */
                             std::vector<SpaceInformationPtr>& si_vec,
                             const std::vector<int>& sequenceLinks,
                             unsigned int numLinks) {
        auto planner = std::make_shared<PlannerType>(si_vec);
        planner->setName(buildPlannerName(baseName, sequenceLinks, numLinks));
        return planner;
    }

    PlannerPtr makeSingleLevelPlanner(const std::string& algo, const SpaceInformationPtr& si) {
        if (algo == "RRT") {
            return std::make_shared<og::RRT>(si);
        }
        if (algo == "RRTConnect") {
            return std::make_shared<og::RRTConnect>(si);
        }
        if (algo == "PRMStar") {
            return std::make_shared<og::PRMstar>(si);
        }
        if (algo == "BITStar") {
            return std::make_shared<og::BITstar>(si);
        }
        if (algo == "AITStar") {
            return std::make_shared<og::AITstar>(si);
        }
        if (algo == "EITStar") {
            return std::make_shared<og::EITstar>(si);
        }
        if (algo == "LBKPIECE") {
            /* Uses KinematicChainSpace's default projection, registered in
             * KinematicChainSpace::registerProjections. */
            return std::make_shared<og::LBKPIECE1>(si);
        }
        throw std::runtime_error("Incompatible algorithm: " + algo);
    }
}

    PlannerPtr GetPlanner(const std::string& algo,
                          const std::vector<int>& sequenceLinks,
                          const std::vector<mlmp::common::Point>& pinnedPositions,
                          double linkLength,
                          unsigned int numRobots,
                          unsigned int numLinks,
                          SpaceInformationPtr si) {
        /* Single-level planners plan directly on the full chain; building the
         * abstraction spaces for them would be pointless work. */
        if (kSingleLevelAlgorithms.count(algo) > 0) {
            return makeSingleLevelPlanner(algo, si);
        }

        std::vector<SpaceInformationPtr> si_vec;

        for (unsigned int k = 0; k < sequenceLinks.size(); k++) {
            auto links = sequenceLinks.at(k) - 1;
            assert(links < numLinks);
            OMPL_INFORM("Create MultiLevel Chain with %d links, and %d robots", links, numRobots);
            auto spaceK = std::make_shared<KinematicChainSpace>(numRobots, links, linkLength, &envs.at(links));

            auto siK = std::make_shared<SpaceInformation>(spaceK);
            siK->setStateValidityChecker(std::make_shared<KinematicChainValidityChecker>(siK, pinnedPositions, numRobots, links));
            spaceK->setup();
            si_vec.push_back(siK);
        }

        OMPL_INFORM("Add Original Chain with %d links, and %d robots", numLinks, numRobots);
        si_vec.push_back(si);

        if (algo == "BiQRRT") {
            return createPlanner<ompl::multilevel::BiQRRT>("BiQRRT", si_vec, sequenceLinks, numLinks);
        }
        if (algo == "QRRT") {
            return createPlanner<ompl::multilevel::QRRT>("QRRT", si_vec, sequenceLinks, numLinks);
        }
        if (algo == "QMP") {
            return createPlanner<ompl::multilevel::QMP>("QMP", si_vec, sequenceLinks, numLinks);
        }
        if (algo == "QRRTStar") {
            return createPlanner<ompl::multilevel::QRRTStar>("QRRTStar", si_vec, sequenceLinks, numLinks);
        }
        if (algo == "QMPStar") {
            return createPlanner<ompl::multilevel::QMPStar>("QMPStar", si_vec, sequenceLinks, numLinks);
        }
        throw std::runtime_error("Incompatible algorithm: " + algo);
    }
    
    
    class Solver {
        Scene scene_;

    public:
        explicit Solver(const Scene& scene) : scene_(scene) {}

        bool solve(const std::string& algo, double timeLimit) {
            OMPL_DEBUG("------On solver::solve------");
            OMPL_DEBUG("robots %d joints %d", scene_.getR(), scene_.getN());

            Environment env = scene_.getObstaclesSegments();

            for (unsigned int k = 0; k < scene_.getN(); k++) {
                envs.push_back(scene_.getObstaclesSegments());
            }

            auto chain = std::make_shared<KinematicChainSpace>(
                scene_.getR(), scene_.getN(), scene_.getJointLength(), &env);
            ompl::geometric::SimpleSetup ss(chain);

            ss.setStateValidityChecker(std::make_shared<KinematicChainValidityChecker>(
                ss.getSpaceInformation(), scene_.getPinnedPositions(), scene_.getR(), scene_.getN()));

            ompl::base::ScopedState<> start(chain), goal(chain);
            std::vector<double> startVec = flattenAngles(scene_.getStartAngles());
            std::vector<double> goalVec = flattenAngles(scene_.getGoalAngles());

            chain->setup();
            chain->copyFromReals(start.get(), startVec);
            chain->copyFromReals(goal.get(), goalVec);
            ss.setStartAndGoalStates(start, goal);

            std::vector<int> discrete;
            boost::push_back(discrete, boost::irange(2, scene_.getN() + 1));

            ompl::base::PlannerPtr planner = GetPlanner(
                algo, discrete, scene_.getPinnedPositions(),
                scene_.getJointLength(), scene_.getR(), scene_.getN(),
                ss.getSpaceInformation());

            ss.setPlanner(planner);
            PlannerStatus status = ss.solve(timeLimit);
            double timeToCompute = ss.getLastPlanComputationTime();

            if (status) {
                const auto pdef = planner->getProblemDefinition();
                auto path = pdef->getSolutionPath();
                ss.getPathSimplifier()->simplifyMax(*(path->as<og::PathGeometric>()));

                pdef->getSolutionPath()->print(std::cout);
                std::cout << std::string(80, '*') << std::endl;
                std::cout << "Length\n" << pdef->getSolutionPath()->length() << std::endl;
                std::cout << std::string(80, '*') << std::endl;
                std::cout << "Runtime\n" << timeToCompute << std::endl;
            } else {
                std::cout << "Not Found" << std::endl;
            }

            return static_cast<bool>(status);
        }

    private:
        std::vector<double> flattenAngles(const std::vector<std::vector<double>>& anglesByRobots) const {
            std::vector<double> result;
            result.reserve(scene_.getN() * scene_.getR());
            for (unsigned int i = 0; i < scene_.getN(); ++i) {
                for (unsigned int j = 0; j < scene_.getR(); ++j) {
                    result.emplace_back(anglesByRobots[j][i]);
                }
            }
            return result;
        }
    };

}

#endif
