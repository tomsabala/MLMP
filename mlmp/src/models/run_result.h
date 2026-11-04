#ifndef MLMP_MODELS_RUN_RESULT_H
#define MLMP_MODELS_RUN_RESULT_H

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>
#include <ompl/base/PlannerStatus.h>
#include <ompl/config.h>

namespace mlmp {

/* ordered_json, not json: the schema documents a key order, and nlohmann::json's
 * std::map backing would silently re-sort every object alphabetically. */
using Json = nlohmann::ordered_json;

using TraceSink = std::function<void(const Json&)>;

struct RunOptions {
    std::string algo = "BiQRRT";
    std::string sceneName;        // as given on the CLI, e.g. "scene1.json"
    std::string scenePath;        // resolved path actually opened
    double timeLimit = 1.0;       // seconds
    bool simplify = true;
    TraceSink traceSink;          // empty => no tracing, zero overhead
    std::string traceFile;        // echoed into the result; informational
    double traceIntervalS = 0.05;
    unsigned int traceMaxFrames = 400;
};

struct SolutionEvent {
    double tSeconds;
    double cost;
};

struct RunResult {
    std::string planner;
    std::string sceneName;
    std::string scenePath;
    int robots = 0;
    int links = 0;
    /* "planar" | "urdf". robots/links keep their planar meanings; for a URDF scene
     * robots is 1 and links is the actuated joint count, so dof == links there. */
    std::string model = "planar";
    int dof = 0;
    std::optional<std::string> robotModel;     // URDF <robot name>, null for planar
    std::optional<std::string> robotUrdf;      // resolved URDF path, null for planar
    std::uint_fast32_t seed = 0;
    double timeLimitS = 0.0;

    bool solved = false;
    bool exact = false;
    ompl::base::PlannerStatus::StatusType status = ompl::base::PlannerStatus::UNKNOWN;

    double timeSolveS = 0.0;
    std::optional<double> timeFirstSolutionS;
    double timeSimplifyS = 0.0;
    std::vector<SolutionEvent> solutionEvents;

    std::optional<double> pathLengthRaw;
    std::optional<double> pathLengthSimplified;
    std::vector<std::vector<double>> path;   // joint vectors, final (simplified) path

    std::uint64_t graphVertices = 0;
    std::uint64_t graphEdges = 0;
    std::uint64_t stateValidityChecks = 0;
    std::uint64_t statesSampled = 0;
    std::uint64_t motionChecks = 0;
    std::uint64_t motionChecksValid = 0;

    bool traced = false;
    std::string traceFile;
    unsigned int traceFrames = 0;
    bool traceTruncated = false;
    unsigned int traceErrors = 0;
};

inline const char* statusToString(ompl::base::PlannerStatus::StatusType s) {
    switch (s) {
        case ompl::base::PlannerStatus::INVALID_START: return "invalid_start";
        case ompl::base::PlannerStatus::INVALID_GOAL: return "invalid_goal";
        case ompl::base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE: return "unrecognized_goal_type";
        case ompl::base::PlannerStatus::TIMEOUT: return "timeout";
        case ompl::base::PlannerStatus::APPROXIMATE_SOLUTION: return "approximate_solution";
        case ompl::base::PlannerStatus::EXACT_SOLUTION: return "exact_solution";
        case ompl::base::PlannerStatus::CRASH: return "crash";
        case ompl::base::PlannerStatus::ABORT: return "abort";
        case ompl::base::PlannerStatus::INFEASIBLE: return "infeasible";
        default: return "unknown";
    }
}

/* Vertex tagging written by BiQRRTImpl::getPlannerData; 0 for planners that do not tag. */
inline const char* treeLabel(unsigned int component) {
    switch (component) {
        case 1: return "start";
        case 2: return "goal";
        case 3: return "section";
        default: return "untagged";
    }
}

namespace detail {
inline Json optionalToJson(const std::optional<double>& v) {
    return v ? Json(*v) : Json(nullptr);
}
inline Json optionalToJson(const std::optional<std::string>& v) {
    return v ? Json(*v) : Json(nullptr);
}
}

/* The mlmp.run/1 wire format. Consumers index by key; the order below is for humans
 * reading a raw line. */
inline Json toJson(const RunResult& r) {
    Json events = Json::array();
    for (const auto& e : r.solutionEvents) {
        events.push_back(Json{{"t_s", e.tSeconds}, {"cost", e.cost}});
    }

    Json trace = nullptr;
    if (r.traced) {
        trace = Json{{"file", r.traceFile},
                     {"frames", r.traceFrames},
                     {"truncated", r.traceTruncated},
                     {"errors", r.traceErrors}};
    }

    return Json{
        {"schema", "mlmp.run/1"},
        {"planner", r.planner},
        {"scene", r.sceneName},
        {"scene_path", r.scenePath},
        {"robots", r.robots},
        {"links", r.links},
        {"model", r.model},
        {"dof", r.dof},
        {"robot_model", detail::optionalToJson(r.robotModel)},
        {"robot_urdf", detail::optionalToJson(r.robotUrdf)},
        {"seed", static_cast<std::uint64_t>(r.seed)},
        {"time_limit_s", r.timeLimitS},
        {"ompl_version", OMPL_VERSION},
        {"solved", r.solved},
        {"exact", r.exact},
        {"status", statusToString(r.status)},
        {"time_solve_s", r.timeSolveS},
        {"time_first_solution_s", detail::optionalToJson(r.timeFirstSolutionS)},
        {"time_simplify_s", r.timeSimplifyS},
        {"solution_events", events},
        {"path_length_raw", detail::optionalToJson(r.pathLengthRaw)},
        {"path_length_simplified", detail::optionalToJson(r.pathLengthSimplified)},
        {"path_states", r.path.size()},
        {"path", r.path},
        {"graph_vertices", r.graphVertices},
        {"graph_edges", r.graphEdges},
        {"state_validity_checks", r.stateValidityChecks},
        {"states_sampled", r.statesSampled},
        {"motion_checks", r.motionChecks},
        {"motion_checks_valid", r.motionChecksValid},
        {"trace", trace},
    };
}

/* `kind` is one of "usage", "io", "runtime". A crash emits no stdout at all, which is
 * how a harness tells the fourth failure mode apart from these three. */
inline Json errorJson(const std::string& kind, const std::string& message) {
    return Json{{"schema", "mlmp.run/1"},
                {"error", Json{{"kind", kind}, {"message", message}}}};
}

}

#endif
