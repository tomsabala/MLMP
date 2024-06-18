#ifndef OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_BIQRRT_
#define OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_BIQRRT_
#include <ompl/multilevel/datastructures/BundleSpaceSequence.h>
#include <BiQRRTImpl.h>

namespace ompl
{
    namespace multilevel
    {
        using BiQRRT = BundleSpaceSequence<BiQRRTImpl>;

    }  // namespace multilevel
}  // namespace ompl

#endif