#ifndef MLMP_STATE_COUNTING_STATE_SAMPLER_H
#define MLMP_STATE_COUNTING_STATE_SAMPLER_H

#include <memory>
#include <utility>

#include <ompl/base/StateSampler.h>
#include <ompl/base/StateSpace.h>

#include <common/counters.h>

namespace mlmp {

/* Forwards every call to the space's default sampler, so the RNG draw sequence is
 * byte-identical to the unwrapped run and --seed reproducibility is preserved. */
class CountingStateSampler : public ompl::base::StateSampler {
public:
    CountingStateSampler(const ompl::base::StateSpace* space,
                         ompl::base::StateSamplerPtr inner,
                         RunCounters& counters)
        : ompl::base::StateSampler(space), inner_(std::move(inner)), counters_(&counters) {}

    void sampleUniform(ompl::base::State* state) override {
        ++counters_->statesSampled;
        inner_->sampleUniform(state);
    }

    void sampleUniformNear(ompl::base::State* state, const ompl::base::State* near,
                           double distance) override {
        ++counters_->statesSampled;
        inner_->sampleUniformNear(state, near, distance);
    }

    void sampleGaussian(ompl::base::State* state, const ompl::base::State* mean,
                        double stdDev) override {
        ++counters_->statesSampled;
        inner_->sampleGaussian(state, mean, stdDev);
    }

private:
    ompl::base::StateSamplerPtr inner_;
    RunCounters* counters_;
};

/* `counters` must outlive every sampler allocated from `space`; in practice both are
 * owned by the same Solver instance.
 *
 * The hook is StateSpace-level because SpaceInformation::allocStateSampler() forwards
 * straight to stateSpace_->allocStateSampler(), which honours this allocator; there is
 * no SpaceInformation-level hook for plain state samplers. */
inline void attachSamplerCounter(ompl::base::StateSpace& space, RunCounters& counters) {
    space.setStateSamplerAllocator(
        [&counters](const ompl::base::StateSpace* s) -> ompl::base::StateSamplerPtr {
            /* allocDefaultStateSampler, not allocStateSampler: the latter would route
             * back through this allocator and recurse infinitely. */
            return std::make_shared<CountingStateSampler>(s, s->allocDefaultStateSampler(), counters);
        });
}

}

#endif
