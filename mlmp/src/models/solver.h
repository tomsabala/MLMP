#ifndef SOLVER_H
#define SOLVER_H

#include <cmath>
#include <boost/range/algorithm_ext/push_back.hpp>

#include <scene.h>
#include <state/kinematic_chain.h>
#include "common/geometry.h"

#include <ompl/base/SpaceInformation.h>
#include <ompl/multilevel/planners/qrrt/QRRT.h>
#include <ompl/multilevel/planners/qmp/QMP.h>
#include <ompl/multilevel/planners/qrrt/QRRTStar.h>
#include <ompl/multilevel/planners/qmp/QMPStar.h>

using namespace ompl::base;

namespace mlmp {

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
                             const std::vector<SpaceInformationPtr>& si_vec,
                             const std::vector<int>& sequenceLinks,
                             unsigned int numLinks) {
        auto planner = std::make_shared<PlannerType>(si_vec);
        planner->setName(buildPlannerName(baseName, sequenceLinks, numLinks));
        return planner;
    }
}

    PlannerPtr GetPlanner(const std::string& algo,
                          const std::vector<int>& sequenceLinks,
                          const std::vector<mlmp::common::Point>& pinnedPositions,
                          double linkLength,
                          unsigned int numRobots,
                          unsigned int numLinks,
                          SpaceInformationPtr si) {
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

        void solve(const std::string& algo, double timeLimit) {
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

            if (!planner) {
                OMPL_DEBUG("Incompatible algorithm");
                return;
            }

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