#ifndef SOLVER_H
#define SOLVER_H

#include <cassert>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <boost/range/algorithm_ext/push_back.hpp>

#include <common/counters.h>
#include <models/run_result.h>
#include <models/scene.h>
#include <state/counting_state_sampler.h>
#include <state/kinematic_chain.h>
#include "common/geometry.h"

#include <ompl/base/PlannerData.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/util/Exception.h>
#include <ompl/util/RandomNumbers.h>

#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/informedtrees/AITstar.h>
#include <ompl/geometric/planners/informedtrees/EITstar.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>

#include <ompl/multilevel/datastructures/PlannerDataVertexAnnotated.h>
#include <ompl/multilevel/planners/qrrt/QRRT.h>
#include <ompl/multilevel/planners/qmp/QMP.h>
#include <ompl/multilevel/planners/qrrt/QRRTStar.h>
#include <ompl/multilevel/planners/qmp/QMPStar.h>
#include <planner/BiQRRT.h>

namespace og = ompl::geometric;

using namespace ompl::base;

namespace mlmp {

/* The supported planner set. These two sets are the single source of truth:
 * the CLI validates against them and Solver::buildPlanner dispatches on them. */
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

/* Owns one scene and runs planners against it. No printing, no option parsing and no
 * file IO: main.cpp owns the CLI and the output stream, so an Emscripten entry point
 * can call solve() directly. Safe to call solve() more than once on one instance. */
class Solver {
public:
    explicit Solver(const Scene& scene)
        : scene_(scene), env_(scene.getObstaclesSegments()) {}

    RunResult solve(const RunOptions& options) {
        OMPL_DEBUG("------On solver::solve------");
        OMPL_DEBUG("robots %d joints %d", scene_.getR(), scene_.getN());

        /* Repeat calls must not accumulate tallies or level handles. */
        counters_ = {};
        levelSpaceInfos_.clear();

        RunResult result;
        result.planner = options.algo;
        result.sceneName = options.sceneName;
        result.scenePath = options.scenePath;
        result.robots = scene_.getR();
        result.links = scene_.getN();
        result.timeLimitS = options.timeLimit;
        result.traced = static_cast<bool>(options.traceSink);
        result.traceFile = options.traceFile;

        auto ss = buildSetup();

        std::vector<int> discrete;
        boost::push_back(discrete, boost::irange(2, scene_.getN() + 1));

        const SpaceInformationPtr topSi = ss->getSpaceInformation();
        PlannerPtr planner = buildPlanner(options.algo, discrete, topSi);
        ss->setPlanner(planner);

        const ProblemDefinitionPtr pdef = ss->getProblemDefinition();

        const auto start = std::chrono::steady_clock::now();
        const auto elapsedSeconds = [&start]() {
            return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        };

        /* Anytime planners report every improved solution here. Only optimizing planners
         * call it, which is why the termination condition also polls hasExactSolution(). */
        std::vector<SolutionEvent> events;
        pdef->setIntermediateSolutionCallback(
            [&events, &elapsedSeconds](const Planner*, const std::vector<const State*>&,
                                       const Cost cost) {
                events.push_back({elapsedSeconds(), cost.value()});
            });

        const auto toSteady = [](double seconds) {
            return std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(seconds));
        };
        const auto deadline = start + toSteady(options.timeLimit);
        const auto interval = toSteady(options.traceIntervalS);
        auto nextFrame = start;

        std::optional<double> firstSolutionS;
        unsigned int frames = 0;
        unsigned int traceErrors = 0;
        bool truncated = false;

        /* Single-argument PlannerTerminationCondition: evaluated inline on the planner's
         * own thread. ss->solve(double) would route any budget >= 1 s to a std::thread,
         * which Emscripten cannot have without COOP/COEP headers. Running on the planner's
         * thread between iterations is also what makes frame capture lock-free and keeps
         * it from observing a half-mutated graph. */
        PlannerTerminationCondition ptc([&]() {
            const auto now = std::chrono::steady_clock::now();
            const double t = std::chrono::duration<double>(now - start).count();

            if (!firstSolutionS && pdef->hasExactSolution()) {
                firstSolutionS = t;
            }

            if (options.traceSink && now >= nextFrame) {
                if (frames >= options.traceMaxFrames) {
                    truncated = true;
                } else {
                    try {
                        options.traceSink(captureFrame(planner, topSi, t, frames));
                        ++frames;
                    } catch (const ompl::Exception&) {
                        /* A partially-solved hierarchy that refuses to dump is not a run
                         * failure: count it and keep planning. */
                        ++traceErrors;
                    }
                    nextFrame = now + interval;
                }
            }

            return now >= deadline;
        });

        const PlannerStatus status = ss->solve(ptc);
        const auto statusType = static_cast<PlannerStatus::StatusType>(status);

        result.timeSolveS = ss->getLastPlanComputationTime();
        result.status = statusType;
        /* operator bool() is true for approximate solutions too, which is the existing
         * exit-code behaviour; `exact` is the new field that separates the two. */
        result.solved = static_cast<bool>(status);
        result.exact = (statusType == PlannerStatus::EXACT_SOLUTION);

        if (result.solved) {
            const PathPtr solutionPath = pdef->getSolutionPath();
            auto* path = solutionPath->as<og::PathGeometric>();
            result.pathLengthRaw = path->length();

            if (options.simplify) {
                const auto simplifyStart = std::chrono::steady_clock::now();
                ss->getPathSimplifier()->simplifyMax(*path);
                result.timeSimplifyS = std::chrono::duration<double>(
                                           std::chrono::steady_clock::now() - simplifyStart)
                                           .count();
                result.pathLengthSimplified = path->length();
            }

            const StateSpacePtr space = topSi->getStateSpace();
            result.path.reserve(path->getStateCount());
            for (std::size_t i = 0; i < path->getStateCount(); ++i) {
                std::vector<double> q;
                space->copyToReals(q, path->getState(i));
                result.path.push_back(std::move(q));
            }
        }

        PlannerData data(topSi);
        planner->getPlannerData(data);
        result.graphVertices = data.numVertices();
        result.graphEdges = data.numEdges();

        result.stateValidityChecks = counters_.stateValidityChecks;
        result.statesSampled = counters_.statesSampled;

        /* Motion checks live on each level's SpaceInformation, so the top space alone
         * would under-report every multilevel run. */
        const auto addMotionCounts = [&result](const SpaceInformationPtr& si) {
            result.motionChecks += si->getCheckedMotionCount();
            result.motionChecksValid += si->getMotionValidator()->getValidMotionCount();
        };
        addMotionCounts(topSi);
        for (const auto& si : levelSpaceInfos_) {
            addMotionCounts(si);
        }

        result.seed = ompl::RNG::getSeed();
        result.timeFirstSolutionS = firstSolutionS;
        result.solutionEvents = std::move(events);
        result.traceFrames = frames;
        result.traceTruncated = truncated;
        result.traceErrors = traceErrors;

        return result;
    }

private:
    Scene scene_;
    /* One Environment for every level: each level's obstacles were identical copies
     * anyway, and a member is address-stable for the Solver's lifetime, which the old
     * file-scope vector was not. */
    Environment env_;
    RunCounters counters_;
    std::vector<SpaceInformationPtr> levelSpaceInfos_;

    /* The single place a state space is constructed. */
    std::shared_ptr<KinematicChainSpace> makeChainSpace(unsigned int numLinks) {
        auto space = std::make_shared<KinematicChainSpace>(
            scene_.getR(), numLinks, scene_.getJointLength(), &env_);
        attachSamplerCounter(*space, counters_);
        return space;
    }

    /* SimpleSetup is not copyable, hence the unique_ptr. */
    std::unique_ptr<og::SimpleSetup> buildSetup() {
        auto chain = makeChainSpace(scene_.getN());
        auto ss = std::make_unique<og::SimpleSetup>(chain);

        ss->setStateValidityChecker(std::make_shared<KinematicChainValidityChecker>(
            ss->getSpaceInformation(), scene_.getPinnedPositions(), scene_.getR(),
            scene_.getN(), counters_));

        ScopedState<> start(chain), goal(chain);
        const std::vector<double> startVec = flattenAngles(scene_.getStartAngles());
        const std::vector<double> goalVec = flattenAngles(scene_.getGoalAngles());

        chain->setup();
        chain->copyFromReals(start.get(), startVec);
        chain->copyFromReals(goal.get(), goalVec);
        ss->setStartAndGoalStates(start, goal);

        return ss;
    }

    /* Records every abstraction level's SpaceInformation in levelSpaceInfos_ so the
     * caller can sum motion checks over the whole hierarchy; the top-level `si` is
     * excluded because the caller already holds it. */
    PlannerPtr buildPlanner(const std::string& algo,
                            const std::vector<int>& sequenceLinks,
                            const SpaceInformationPtr& si) {
        /* Single-level planners plan directly on the full chain; building the
         * abstraction spaces for them would be pointless work. */
        if (kSingleLevelAlgorithms.count(algo) > 0) {
            return makeSingleLevelPlanner(algo, si);
        }

        const unsigned int numLinks = static_cast<unsigned int>(scene_.getN());
        const unsigned int numRobots = static_cast<unsigned int>(scene_.getR());
        std::vector<SpaceInformationPtr> si_vec;

        for (unsigned int k = 0; k < sequenceLinks.size(); k++) {
            const unsigned int links = static_cast<unsigned int>(sequenceLinks.at(k) - 1);
            assert(links < numLinks);
            OMPL_INFORM("Create MultiLevel Chain with %d links, and %d robots", links, numRobots);
            auto spaceK = makeChainSpace(links);

            auto siK = std::make_shared<SpaceInformation>(spaceK);
            siK->setStateValidityChecker(std::make_shared<KinematicChainValidityChecker>(
                siK, scene_.getPinnedPositions(), numRobots, links, counters_));
            spaceK->setup();
            si_vec.push_back(siK);
            levelSpaceInfos_.push_back(siK);
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

    /* One mlmp.trace/1 frame. A fresh PlannerData per frame is mandatory:
     * BundleSpaceSequence::getPlannerData throws if handed a populated one. It also
     * tags each vertex with its bundle level and lifts the state into the total space,
     * so every vertex carries a full-dimension configuration regardless of level. */
    Json captureFrame(const PlannerPtr& planner,
                      const SpaceInformationPtr& topSi,
                      double tSeconds,
                      unsigned int index) const {
        PlannerData data(topSi);
        planner->getPlannerData(data);

        Json frame = Json{{"frame", index},
                          {"t_s", tSeconds},
                          {"vertices", Json::array()},
                          {"edges", Json::array()}};
        Json& vertices = frame["vertices"];
        Json& edges = frame["edges"];

        const StateSpacePtr space = topSi->getStateSpace();
        for (unsigned int i = 0; i < data.numVertices(); ++i) {
            const PlannerDataVertex& v = data.getVertex(i);
            const auto* annotated =
                dynamic_cast<const ompl::multilevel::PlannerDataVertexAnnotated*>(&v);

            std::vector<double> q;
            if (v.getState() != nullptr) {
                space->copyToReals(q, v.getState());
            }

            const unsigned int component = annotated ? annotated->getComponent() : 0u;
            vertices.push_back(Json{{"level", annotated ? annotated->getLevel() : 0u},
                                    {"component", component},
                                    {"tree", treeLabel(component)},
                                    {"start", data.isStartVertex(i)},
                                    {"goal", data.isGoalVertex(i)},
                                    {"q", q}});

            std::vector<unsigned int> out;
            data.getEdges(i, out);
            for (unsigned int j : out) {
                edges.push_back(Json::array({i, j}));
            }
        }

        return frame;
    }

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
