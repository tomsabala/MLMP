#ifndef OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_BIQRRTIMPL_
#define OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_BIQRRTIMPL_
#include <ompl/multilevel/datastructures/BundleSpaceGraph.h>
#include <ompl/datastructures/PDF.h>

namespace ompl
{
    namespace base
    {
        OMPL_CLASS_FORWARD(OptimizationObjective);
    }
    namespace multilevel
    {
        /** \brief Implementation of BundleSpace Rapidly-Exploring Random Trees Algorithm*/
        class BiQRRTImpl : public ompl::multilevel::BundleSpaceGraph
        {
            using BaseT = BundleSpaceGraph;

        public:
            BiQRRTImpl(const ompl::base::SpaceInformationPtr &si, BundleSpace *parent_);
            ~BiQRRTImpl() override;

            /** \brief One iteration of RRT with adjusted sampling function */
            void grow() override;

            void init() override;
            void setup() override;
            void clear() override;
            void getPlannerData(ompl::base::PlannerData &data) const override;

        protected:
            using TreeData = std::shared_ptr<NearestNeighbors<Configuration *>>;
            TreeData treeStart_;
            TreeData treeGoal_;
            double distanceBetweenTrees_;

            bool activeInitialTree_{true};
        };
    }  // namespace multilevel
}  // namespace ompl

#endif