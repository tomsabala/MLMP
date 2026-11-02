#ifndef MLMP_COMMON_COUNTERS_H
#define MLMP_COMMON_COUNTERS_H

#include <cstdint>

namespace mlmp {

/* Per-run tallies. Incremented from the planner's own thread only: the solver
 * drives a non-threaded PlannerTerminationCondition and every supported planner
 * is single-threaded, so plain integers are correct and atomics would be waste. */
struct RunCounters {
    std::uint64_t stateValidityChecks = 0;
    std::uint64_t statesSampled = 0;
};

}

#endif
