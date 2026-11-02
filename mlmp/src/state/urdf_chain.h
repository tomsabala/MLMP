#ifndef MLMP_STATE_URDF_CHAIN_H
#define MLMP_STATE_URDF_CHAIN_H

#ifdef MLMP_ENABLE_URDF

#include <memory>
#include <string>

#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <models/urdf_robot.h>

/* A plain R^n joint space for the first `numJoints` joints of a URDF chain.
 *
 * Deliberately overrides nothing: distance is RealVectorStateSpace's weighted L2 and the
 * projection is the base class default (which is what LBKPIECE needs). Staying a plain
 * R^n is also what lets ompl::multilevel::Projection_RN_RM apply unchanged, so level k is
 * literally the first k coordinates of level k+1 and the whole bundle hierarchy works
 * without a custom projection.
 *
 * Consequence accepted, not fixed: a `continuous` joint is treated as bounded rather than
 * wrapping, so a planner may take the long way round a +/-pi boundary. The UR5's joints
 * are bounded at +/-2pi upstream, so this costs the committed robot nothing. */
class UrdfChainSpace : public ompl::base::RealVectorStateSpace {
public:
    UrdfChainSpace(std::shared_ptr<const mlmp::UrdfRobot> robot, unsigned int numJoints)
        : ompl::base::RealVectorStateSpace(numJoints)
        , robot_(std::move(robot))
        , numJoints_(numJoints) {
        ompl::base::RealVectorBounds bounds(numJoints);
        for (unsigned int i = 0; i < numJoints; ++i) {
            bounds.setLow(i, robot_->joints[i].lower);
            bounds.setHigh(i, robot_->joints[i].upper);
        }
        setBounds(bounds);
        setName("UrdfChain" + std::to_string(numJoints));
    }

    const mlmp::UrdfRobot& robot() const { return *robot_; }
    unsigned int numJoints() const { return numJoints_; }

private:
    std::shared_ptr<const mlmp::UrdfRobot> robot_;
    unsigned int numJoints_;
};

#endif

#endif
