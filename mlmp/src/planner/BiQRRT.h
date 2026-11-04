#ifndef OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_BIQRRT_
#define OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_BIQRRT_
#include <ompl/multilevel/datastructures/BundleSpaceSequence.h>
#include <planner/BiQRRTImpl.h>

namespace ompl
{
    namespace multilevel
    {
        /** \brief Bidirectional bundle-space RRT.
         *
         * A real class rather than an alias for BundleSpaceSequence<BiQRRTImpl> for one
         * reason: the per-level planners live in the protected member bundleSpaces_ and
         * PlannerMultiLevel exposes no public accessor, so propagating a step size to
         * every level has to happen from inside the sequence type. */
        class BiQRRT : public BundleSpaceSequence<BiQRRTImpl>
        {
            using BaseT = BundleSpaceSequence<BiQRRTImpl>;

        public:
            BiQRRT(std::vector<base::SpaceInformationPtr> &siVec, std::string type = "BiQRRT")
              : BaseT(siVec, type)
            {
                Planner::declareParam<double>("range", this, &BiQRRT::setRange, &BiQRRT::getRange,
                                              "0.:0.1:10000.");
            }

            /** \brief Propagate the step size to every bundle level. bundleSpaces_ is
             * filled by the BundleSpaceSequence constructor, so this reaches every level
             * when called straight after construction. */
            void setRange(double distance)
            {
                range_ = distance;
                for (auto *bundleSpace : bundleSpaces_)
                    bundleSpace->setRange(distance);
            }

            double getRange() const
            {
                return range_;
            }

        private:
            double range_{0.1};
        };

    }  // namespace multilevel
}  // namespace ompl

#endif
