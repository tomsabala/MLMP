#ifndef MLMP_STATE_URDF_VALIDITY_CHECKER_H
#define MLMP_STATE_URDF_VALIDITY_CHECKER_H

#ifdef MLMP_ENABLE_URDF

#include <cstdlib>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/collision_object.h>
#include <fcl/narrowphase/collision_request.h>
#include <fcl/narrowphase/collision_result.h>

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <common/counters.h>
#include <models/scene_types.h>
#include <models/urdf_robot.h>
#include <state/urdf_chain.h>

/* FCL narrowphase collision for one URDF arm: self-collision between non-adjacent links,
 * then every link against every workspace obstacle.
 *
 * Only shapes belonging to the first `numJoints` joints are kept, which is the
 * truncated-arm rule that makes an abstraction level's geometry match its DOF -- exactly
 * as a planar level with k links carries no geometry past link k. */
class UrdfValidityChecker : public ompl::base::StateValidityChecker {
public:
    UrdfValidityChecker(const ompl::base::SpaceInformationPtr& si,
                        std::shared_ptr<const mlmp::UrdfRobot> robot,
                        unsigned int numJoints,
                        const Eigen::Isometry3d& base,
                        const std::vector<mlmp::Obstacle3d>& obstacles,
                        const std::set<std::pair<std::string, std::string>>& disabled,
                        mlmp::RunCounters& counters)
        : ompl::base::StateValidityChecker(si)
        , robot_(std::move(robot))
        , numJoints_(numJoints)
        , base_(base)
        , counters_(&counters) {
        for (const auto& shape : robot_->shapes) {
            if (shape.jointIndex < static_cast<int>(numJoints_)) {
                shapes_.push_back(&shape);
                robotObjects_.push_back(
                    std::make_unique<fcl::CollisionObjectd>(shape.geometry));
            }
        }

        for (const auto& obstacle : obstacles) {
            std::shared_ptr<fcl::CollisionGeometryd> geometry;
            if (obstacle.type == "box") {
                geometry = std::make_shared<fcl::Boxd>(obstacle.size[0], obstacle.size[1],
                                                       obstacle.size[2]);
            } else if (obstacle.type == "sphere") {
                geometry = std::make_shared<fcl::Sphered>(obstacle.radius);
            } else if (obstacle.type == "cylinder") {
                geometry = std::make_shared<fcl::Cylinderd>(obstacle.radius, obstacle.length);
            } else {
                throw std::runtime_error("unknown obstacle3d type: " + obstacle.type);
            }

            auto object = std::make_unique<fcl::CollisionObjectd>(geometry);
            object->setTransform(mlmp::poseFromXyzRpy(obstacle.xyz, obstacle.rpy));
            object->computeAABB();
            obstacleObjects_.push_back(std::move(object));
        }

        /* Adjacent links always touch at their shared joint, and a link cannot collide
         * with itself, so both are excluded once here rather than per state. */
        for (std::size_t a = 0; a < shapes_.size(); ++a) {
            for (std::size_t b = a + 1; b < shapes_.size(); ++b) {
                if (shapes_[a]->linkName == shapes_[b]->linkName) {
                    continue;
                }
                if (std::abs(shapes_[a]->jointIndex - shapes_[b]->jointIndex) <= 1) {
                    continue;
                }
                if (disabled.count({shapes_[a]->linkName, shapes_[b]->linkName}) > 0) {
                    continue;
                }
                selfPairs_.emplace_back(a, b);
            }
        }

        frames_.reserve(numJoints_ + 1);
    }

    bool isValid(const ompl::base::State* state) const override {
        ++counters_->stateValidityChecks;

        const auto* s = state->as<ompl::base::RealVectorStateSpace::StateType>();
        robot_->linkFrames(s->values, numJoints_, base_, frames_);

        for (std::size_t i = 0; i < shapes_.size(); ++i) {
            robotObjects_[i]->setTransform(frames_[shapes_[i]->jointIndex + 1] *
                                           shapes_[i]->linkToShape);
            robotObjects_[i]->computeAABB();
        }

        for (const auto& pair : selfPairs_) {
            if (collides(*robotObjects_[pair.first], *robotObjects_[pair.second])) {
                return false;
            }
        }

        for (const auto& robotObject : robotObjects_) {
            for (const auto& obstacleObject : obstacleObjects_) {
                if (collides(*robotObject, *obstacleObject)) {
                    return false;
                }
            }
        }

        return true;
    }

private:
    bool collides(const fcl::CollisionObjectd& a, const fcl::CollisionObjectd& b) const {
        result_.clear();
        return fcl::collide(&a, &b, request_, result_) > 0;
    }

    std::shared_ptr<const mlmp::UrdfRobot> robot_;
    unsigned int numJoints_;
    Eigen::Isometry3d base_;
    std::vector<const mlmp::UrdfShape*> shapes_;
    std::vector<std::pair<std::size_t, std::size_t>> selfPairs_;

    /* Mutable scratch: isValid is const, and none of this is observable state. Keeping
     * the objects and the frame buffer as members is what makes the hot loop allocate
     * nothing. */
    mutable std::vector<std::unique_ptr<fcl::CollisionObjectd>> robotObjects_;
    std::vector<std::unique_ptr<fcl::CollisionObjectd>> obstacleObjects_;
    mutable std::vector<Eigen::Isometry3d> frames_;
    mutable fcl::CollisionRequestd request_;
    mutable fcl::CollisionResultd result_;

    /* Pointer, not reference, mirroring KinematicChainValidityChecker. */
    mlmp::RunCounters* counters_;
};

#endif

#endif
