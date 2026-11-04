#ifndef MLMP_MODELS_URDF_ROBOT_H
#define MLMP_MODELS_URDF_ROBOT_H

#ifdef MLMP_ENABLE_URDF

#include <array>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include <fcl/geometry/shape/box.h>
#include <fcl/geometry/shape/cylinder.h>
#include <fcl/geometry/shape/sphere.h>
#include <fcl/narrowphase/collision_object.h>

#include <urdf_model/joint.h>
#include <urdf_model/link.h>
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

#include <models/scene_types.h>

namespace mlmp {

/* Thrown for a URDF this build can read but not plan over: a branched tree, an
 * unsupported joint type, a mesh collision. main.cpp maps it to error.kind == "usage";
 * an unreadable or malformed file stays std::runtime_error and maps to "io". */
struct UnsupportedUrdf : std::runtime_error {
    using std::runtime_error::runtime_error;
};

/* rpy is fixed-axis roll-pitch-yaw, the URDF convention. See scene_types.h. */
inline Eigen::Isometry3d poseFromXyzRpy(const std::array<double, 3>& xyz,
                                        const std::array<double, 3>& rpy) {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.linear() = (Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
                     Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
                     Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX()))
                        .toRotationMatrix();
    pose.translation() = Eigen::Vector3d(xyz[0], xyz[1], xyz[2]);
    return pose;
}

struct UrdfJoint {
    std::string name;
    std::string childLink;
    Eigen::Isometry3d parentToJoint = Eigen::Isometry3d::Identity();
    Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
    double lower = -M_PI;
    double upper = M_PI;
};

struct UrdfShape {
    std::string linkName;
    int jointIndex = -1;                 // -1: root link; else index into joints
    Eigen::Isometry3d linkToShape = Eigen::Isometry3d::Identity();
    std::shared_ptr<fcl::CollisionGeometryd> geometry;
};

/* A URDF flattened into a single serial chain of actuated joints. Fixed joints are
 * folded into the neighbouring transforms, so `joints` is exactly the DOF vector the
 * planner sees and `shapes` carries every collision primitive tagged with the joint
 * whose child frame it rides on. */
struct UrdfRobot {
    std::string name;
    std::vector<UrdfJoint> joints;       // root -> tip, actuated only
    std::vector<UrdfShape> shapes;

    static std::shared_ptr<const UrdfRobot> load(const std::string& path);

    /* frames[0] is the root link frame (== base); frames[i+1] is the child-link frame of
     * joints[i]. Only the first numJoints joints are articulated; the vector is sized
     * numJoints+1 and resized in place so the hot path allocates nothing. */
    void linkFrames(const double* q, unsigned int numJoints,
                    const Eigen::Isometry3d& base,
                    std::vector<Eigen::Isometry3d>& frames) const {
        frames.resize(numJoints + 1);
        frames[0] = base;
        for (unsigned int i = 0; i < numJoints; ++i) {
            frames[i + 1] = frames[i] * joints[i].parentToJoint *
                            Eigen::AngleAxisd(q[i], joints[i].axis);
        }
    }
};

namespace detail {

inline Eigen::Isometry3d toIsometry(const urdf::Pose& pose) {
    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
    double qw = 1.0;
    pose.rotation.getQuaternion(qx, qy, qz, qw);
    Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
    out.linear() = Eigen::Quaterniond(qw, qx, qy, qz).normalized().toRotationMatrix();
    out.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
    return out;
}

inline std::shared_ptr<fcl::CollisionGeometryd> toFclGeometry(const urdf::Geometry& geom,
                                                              const std::string& linkName) {
    switch (geom.type) {
        case urdf::Geometry::BOX: {
            /* URDF <box size> and fcl::Boxd both take full extents. */
            const auto& box = static_cast<const urdf::Box&>(geom);
            return std::make_shared<fcl::Boxd>(box.dim.x, box.dim.y, box.dim.z);
        }
        case urdf::Geometry::SPHERE: {
            const auto& sphere = static_cast<const urdf::Sphere&>(geom);
            return std::make_shared<fcl::Sphered>(sphere.radius);
        }
        case urdf::Geometry::CYLINDER: {
            /* Both URDF and FCL put the cylinder axis on local +z, so no extra rotation. */
            const auto& cyl = static_cast<const urdf::Cylinder&>(geom);
            return std::make_shared<fcl::Cylinderd>(cyl.radius, cyl.length);
        }
        case urdf::Geometry::MESH:
        default:
            throw UnsupportedUrdf("mesh collision geometry is not supported: " + linkName);
    }
}

}

inline std::shared_ptr<const UrdfRobot> UrdfRobot::load(const std::string& path) {
    urdf::ModelInterfaceSharedPtr model = urdf::parseURDFFile(path);
    if (!model) {
        throw std::runtime_error("unable to parse URDF: " + path);
    }

    auto robot = std::make_shared<UrdfRobot>();
    robot->name = model->getName();

    /* Offset accumulated across fixed joints since the last actuated joint. It prefixes
     * both the next actuated joint's origin and any collision shape met on the way, so a
     * fixed joint changes geometry placement without creating a DOF. */
    Eigen::Isometry3d pending = Eigen::Isometry3d::Identity();
    int jointIndex = -1;

    urdf::LinkConstSharedPtr link = model->getRoot();
    while (link) {
        const auto addShape = [&](const urdf::CollisionSharedPtr& collision) {
            if (!collision || !collision->geometry) {
                return;
            }
            UrdfShape shape;
            shape.linkName = link->name;
            shape.jointIndex = jointIndex;
            shape.linkToShape = pending * detail::toIsometry(collision->origin);
            shape.geometry = detail::toFclGeometry(*collision->geometry, link->name);
            robot->shapes.push_back(std::move(shape));
        };

        /* A link with no <collision> contributes no shape; that is legal, not an error. */
        if (!link->collision_array.empty()) {
            for (const auto& collision : link->collision_array) {
                addShape(collision);
            }
        } else {
            addShape(link->collision);
        }

        if (link->child_joints.size() > 1) {
            throw UnsupportedUrdf("branched URDF is not supported at link " + link->name);
        }
        if (link->child_joints.empty()) {
            break;
        }

        const urdf::JointSharedPtr joint = link->child_joints[0];
        const Eigen::Isometry3d origin =
            detail::toIsometry(joint->parent_to_joint_origin_transform);

        if (joint->mimic) {
            throw UnsupportedUrdf("mimic joints are not supported: " + joint->name);
        }

        switch (joint->type) {
            case urdf::Joint::FIXED:
                pending = pending * origin;
                break;
            case urdf::Joint::REVOLUTE:
            case urdf::Joint::CONTINUOUS: {
                UrdfJoint uj;
                uj.name = joint->name;
                uj.childLink = joint->child_link_name;
                uj.parentToJoint = pending * origin;
                /* URDF does not require a unit axis, and AngleAxisd with a non-unit axis
                 * silently scales the rotation. */
                uj.axis = Eigen::Vector3d(joint->axis.x, joint->axis.y, joint->axis.z);
                if (uj.axis.norm() < 1e-9) {
                    throw UnsupportedUrdf("degenerate joint axis for " + joint->name);
                }
                uj.axis.normalize();
                if (joint->type == urdf::Joint::CONTINUOUS) {
                    uj.lower = -M_PI;
                    uj.upper = M_PI;
                } else {
                    if (!joint->limits) {
                        throw UnsupportedUrdf("revolute joint without <limit>: " + joint->name);
                    }
                    uj.lower = joint->limits->lower;
                    uj.upper = joint->limits->upper;
                    /* setBounds would otherwise reject these deep inside OMPL with
                     * "Bounds for real vector space seem to be incorrect"; a malformed
                     * asset is a usage condition and deserves our own message. */
                    if (!(uj.lower < uj.upper)) {
                        throw UnsupportedUrdf("inverted joint limits for " + joint->name);
                    }
                }
                robot->joints.push_back(std::move(uj));
                pending = Eigen::Isometry3d::Identity();
                ++jointIndex;
                break;
            }
            default:
                throw UnsupportedUrdf("unsupported joint type for " + joint->name);
        }

        link = model->getLink(joint->child_link_name);
    }

    if (robot->joints.empty()) {
        throw UnsupportedUrdf("URDF has no actuated joints: " + path);
    }
    /* With no shapes at all, both loops in UrdfValidityChecker::isValid are empty and it
     * returns true unconditionally — a robot buried in an obstacle would report an exact
     * solution. A single link without <collision> stays legal; a robot-wide zero is not. */
    if (robot->shapes.empty()) {
        throw UnsupportedUrdf("URDF has no <collision> geometry: " + path);
    }

    return robot;
}

}

#endif

#endif
