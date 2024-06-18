#ifndef OMPL_TOOLS_SELF_CONFIG_
#define OMPL_TOOLS_SELF_CONFIG_

#include "ompl/base/Goal.h"
#include "ompl/base/Planner.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/datastructures/NearestNeighborsSqrtApprox.h"
#include "ompl/datastructures/NearestNeighborsGNAT.h"
#include "ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h"
#include <mutex>
#include <iostream>
#include <string>

namespace ompl
{
    /** \brief Includes various tools such as self config, benchmarking, etc. */
    namespace tools
    {
        /** \brief This class contains methods that automatically
            configure various parameters for motion planning. If expensive
            computation is performed, the results are cached. */
        class SelfConfig
        {
        public:
            /** \brief Construct an instance that can configure the space
                encapsulated by \e si. Any information printed to the
                console is prefixed by \e context */
            SelfConfig(const base::SpaceInformationPtr &si, const std::string &context = std::string());

            ~SelfConfig();

            /** \brief Get the probability of a sampled state being valid (calls
             * base::SpaceInformation::probabilityOfValidState())*/
            double getProbabilityOfValidState();

            /** \brief Get the probability of a sampled state being valid (calls
             * base::SpaceInformation::averageValidMotionLength())*/
            double getAverageValidMotionLength();

            /** \brief Instances of base::ValidStateSampler need a number of attempts to be specified -- the maximum
               number of times
                a new sample is selected and checked to be valid. This function computes a number of \e attempts such
               that the probability
                of obtaining a valid sample is 90\% */
            void configureValidStateSamplingAttempts(unsigned int &attempts);

            /** \brief Compute what a good length for motion segments is */
            void configurePlannerRange(double &range);

            /** \brief If \e proj is undefined, it is set to the default
                projection reported by base::StateSpace::getDefaultProjection().
                If no default projection is available either, an exception is thrown. */
            void configureProjectionEvaluator(base::ProjectionEvaluatorPtr &proj);

            /** \brief Print the computed configuration parameters */
            void print(std::ostream &out = std::cout) const;

            /** \brief Select a default nearest neighbor datastructure for the given space
             *
             * The default depends on the planning algorithm and the space the planner operates in:
             * - If the space is a metric space and the planner is single-threaded,
             *   then the default is ompl::NearestNeighborsGNATNoThreadSafety.
             * - If the space is a metric space and the planner is multi-threaded,
             *   then the default is ompl::NearestNeighborsGNAT.
             * - If the space is a not a metric space,
             *   then the default is ompl::NearestNeighborsSqrtApprox.
             */
            template <typename _T>
            static NearestNeighbors<_T> *getDefaultNearestNeighbors(const base::Planner *planner)
            {
                const base::StateSpacePtr &space = planner->getSpaceInformation()->getStateSpace();
                const base::PlannerSpecs &specs = planner->getSpecs();
                if (space->isMetricSpace())
                {
                    if (specs.multithreaded)
                        return new NearestNeighborsGNAT<_T>();
                    return new NearestNeighborsGNATNoThreadSafety<_T>();
                }
                return new NearestNeighborsSqrtApprox<_T>();
            }

            /** \brief Given a goal specification, decide on a planner for that goal */
            static base::PlannerPtr getDefaultPlanner(const base::GoalPtr &goal);

        private:
            /// @cond IGNORE
            class SelfConfigImpl;

            SelfConfigImpl *impl_;
            std::string context_;
            static std::mutex staticConstructorLock_;
            /// @endcond
        };
    }
}

#endif