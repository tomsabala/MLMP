#ifndef OMPL_DEMO_KINEMATIC_CHAIN_
#define OMPL_DEMO_KINEMATIC_CHAIN_

#include <cmath>
#include <limits>
#include <vector>

#include <boost/math/constants/constants.hpp>

#include <ompl/base/spaces/RealVectorStateSpace.h>

#include "common/geometry.h"

using Environment = std::vector<mlmp::common::Segment>;

namespace {
    constexpr double PI = boost::math::constants::pi<double>();
    constexpr double TWO_PI = 2.0 * boost::math::constants::pi<double>();
}

class KinematicChainProjector : public ompl::base::ProjectionEvaluator {
public:
    explicit KinematicChainProjector(const ompl::base::StateSpace* space)
        : ompl::base::ProjectionEvaluator(space) {
        int dimension = std::max(2, static_cast<int>(ceil(log(static_cast<double>(space->getDimension())))));
        projectionMatrix_.computeRandom(space->getDimension(), dimension);
    }

    unsigned int getDimension() const override {
        return projectionMatrix_.mat.rows();
    }

    void project(const ompl::base::State* state, Eigen::Ref<Eigen::VectorXd> projection) const override {
        std::vector<double> v(space_->getDimension());
        space_->copyToReals(v, state);
        projectionMatrix_.project(&v[0], projection);
    }

protected:
    ompl::base::ProjectionMatrix projectionMatrix_;
};

class KinematicChainSpace : public ompl::base::RealVectorStateSpace {
public:
    KinematicChainSpace(unsigned int numRobots,
                        unsigned int numLinks,
                        double linkLength,
                        Environment* env = nullptr)
        : ompl::base::RealVectorStateSpace(numRobots * numLinks)
        , numRobots_(numRobots)
        , numLinks_(numLinks)
        , linkLength_(linkLength)
        , environment_(env) {
        ompl::base::RealVectorBounds bounds(numRobots * numLinks);
        bounds.setLow(-PI);
        bounds.setHigh(PI);
        setBounds(bounds);
    }

    void registerProjections() override {
        registerDefaultProjection(std::make_shared<KinematicChainProjector>(this));
    }

    double distance(const ompl::base::State* state1, const ompl::base::State* state2) const override {
        const auto* cstate1 = state1->as<StateType>();
        const auto* cstate2 = state2->as<StateType>();
        double theta1 = 0.0;
        double theta2 = 0.0;
        double dx = 0.0;
        double dy = 0.0;
        double dist = 0.0;

        for (unsigned int i = 0; i < dimension_; ++i) {
            theta1 += cstate1->values[i];
            theta2 += cstate2->values[i];
            dx += cos(theta1) - cos(theta2);
            dy += sin(theta1) - sin(theta2);
            dist += sqrt(dx * dx + dy * dy);
        }

        return dist * linkLength_;
    }

    void enforceBounds(ompl::base::State* state) const override {
        auto* statet = state->as<StateType>();

        for (unsigned int i = 0; i < dimension_; ++i) {
            double v = fmod(statet->values[i], TWO_PI);
            if (v < -PI) {
                v += TWO_PI;
            } else if (v >= PI) {
                v -= TWO_PI;
            }
            statet->values[i] = v;
        }
    }

    bool equalStates(const ompl::base::State* state1, const ompl::base::State* state2) const override {
        const auto* cstate1 = state1->as<StateType>();
        const auto* cstate2 = state2->as<StateType>();
        constexpr double epsilon = std::numeric_limits<double>::epsilon() * 2.0;

        for (unsigned int i = 0; i < dimension_; ++i) {
            if (fabs(cstate1->values[i] - cstate2->values[i]) >= epsilon) {
                return false;
            }
        }
        return true;
    }

    void interpolate(const ompl::base::State* from,
                     const ompl::base::State* to,
                     double t,
                     ompl::base::State* state) const override {
        const auto* fromt = from->as<StateType>();
        const auto* tot = to->as<StateType>();
        auto* statet = state->as<StateType>();

        for (unsigned int i = 0; i < dimension_; ++i) {
            double diff = tot->values[i] - fromt->values[i];
            if (fabs(diff) <= PI) {
                statet->values[i] = fromt->values[i] + diff * t;
            } else {
                if (diff > 0.0) {
                    diff = TWO_PI - diff;
                } else {
                    diff = -TWO_PI - diff;
                }

                statet->values[i] = fromt->values[i] - diff * t;
                if (statet->values[i] > PI) {
                    statet->values[i] -= TWO_PI;
                } else if (statet->values[i] < -PI) {
                    statet->values[i] += TWO_PI;
                }
            }
        }
    }

    double linkLength() const { return linkLength_; }
    const Environment* environment() const { return environment_; }

protected:
    double linkLength_;
    unsigned int numRobots_;
    unsigned int numLinks_;
    Environment* environment_;
};

class KinematicChainValidityChecker : public ompl::base::StateValidityChecker {
public:
    KinematicChainValidityChecker(const ompl::base::SpaceInformationPtr& si,
                                   std::vector<mlmp::common::Point> pinnedPositions,
                                   unsigned int numRobots,
                                   unsigned int numLinks)
        : ompl::base::StateValidityChecker(si)
        , pinnedPositions_(std::move(pinnedPositions))
        , numRobots_(numRobots)
        , numLinks_(numLinks) {
    }

    bool isValid(const ompl::base::State* state) const override {
        const auto* space = si_->getStateSpace()->as<KinematicChainSpace>();
        const auto* s = state->as<KinematicChainSpace::StateType>();
        return isValidImpl(space, s);
    }

protected:
    bool isValidImpl(const KinematicChainSpace* space, const KinematicChainSpace::StateType* s) const {
        double linkLength = space->linkLength();
        std::vector<Environment> robotSegments = computeRobotSegments(s, linkLength);

        for (const auto& segments : robotSegments) {
            if (!selfIntersectionTest(segments)) {
                return false;
            }
        }

        for (unsigned int i = 0; i < numRobots_; ++i) {
            for (unsigned int j = i + 1; j < numRobots_; ++j) {
                if (!noIntersectionBetween(robotSegments[i], robotSegments[j])) {
                    return false;
                }
            }
        }

        Environment allRobotSegments;
        for (const auto& segments : robotSegments) {
            allRobotSegments.insert(allRobotSegments.end(), segments.begin(), segments.end());
        }

        return noIntersectionBetween(allRobotSegments, *space->environment());
    }

    std::vector<Environment> computeRobotSegments(const KinematicChainSpace::StateType* s,
                                                   double linkLength) const {
        std::vector<Environment> robotSegments;
        robotSegments.reserve(numRobots_);

        for (unsigned int i = 0; i < numRobots_; ++i) {
            Environment segments;
            double theta = 0.0;
            double x = pinnedPositions_[i].x;
            double y = pinnedPositions_[i].y;

            for (unsigned int j = 0; j < numLinks_; ++j) {
                theta += s->values[j * numRobots_ + i];
                double xNext = x + cos(theta) * linkLength;
                double yNext = y + sin(theta) * linkLength;
                segments.emplace_back(x, y, xNext, yNext);
                x = xNext;
                y = yNext;
            }
            robotSegments.push_back(std::move(segments));
        }

        return robotSegments;
    }

    bool selfIntersectionTest(const Environment& segments) const {
        for (unsigned int i = 0; i < segments.size(); ++i) {
            for (unsigned int j = i + 1; j < segments.size(); ++j) {
                if (segmentsIntersect(segments[i], segments[j])) {
                    return false;
                }
            }
        }
        return true;
    }

    bool noIntersectionBetween(const Environment& env0, const Environment& env1) const {
        for (const auto& seg0 : env0) {
            for (const auto& seg1 : env1) {
                if (segmentsIntersect(seg0, seg1)) {
                    return false;
                }
            }
        }
        return true;
    }

    bool segmentsIntersect(const mlmp::common::Segment& s0, const mlmp::common::Segment& s1) const {
        // Line segment intersection algorithm
        // Source: http://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect/1201356#1201356
        double s10_x = s0.x1 - s0.x0;
        double s10_y = s0.y1 - s0.y0;
        double s32_x = s1.x1 - s1.x0;
        double s32_y = s1.y1 - s1.y0;
        double denom = s10_x * s32_y - s32_x * s10_y;

        if (fabs(denom) < std::numeric_limits<double>::epsilon()) {
            return false;
        }

        bool denomPositive = denom > 0;

        double s02_x = s0.x0 - s1.x0;
        double s02_y = s0.y0 - s1.y0;
        double s_numer = s10_x * s02_y - s10_y * s02_x;

        if ((s_numer < std::numeric_limits<float>::epsilon()) == denomPositive) {
            return false;
        }

        double t_numer = s32_x * s02_y - s32_y * s02_x;

        if ((t_numer < std::numeric_limits<float>::epsilon()) == denomPositive) {
            return false;
        }

        if (((s_numer - denom > -std::numeric_limits<float>::epsilon()) == denomPositive) ||
            ((t_numer - denom > std::numeric_limits<float>::epsilon()) == denomPositive)) {
            return false;
        }

        return true;
    }

    std::vector<mlmp::common::Point> pinnedPositions_;
    unsigned int numRobots_;
    unsigned int numLinks_;
};

#endif
