#ifndef SCENE_H
#define SCENE_H

#include <array>
#include <cfloat>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include <ompl/base/SpaceInformation.h>
#include <ompl/util/Console.h>

#include <common/geometry.h>
#include <models/planar_robot_arm.h>
#include <models/scene_types.h>
#include <models/urdf_robot.h>

namespace ob = ompl::base;

namespace mlmp {

using Robot = PlanarRobotArm;

struct FaceInfo {
    bool isObstacle;
    std::string info;
};

class Scene {
public:
    void loadScene(const std::string& filename) {
        OMPL_DEBUG("------On scene::load-scene------");
        std::ifstream file(filename);
        if (!file.is_open()) {
            OMPL_DEBUG("Unable to open file: %s", filename.c_str());
            throw std::runtime_error("Unable to open file: " + filename);
        }

        nlohmann::json json;
        try {
            file >> json;
        } catch (const nlohmann::json::parse_error& e) {
            OMPL_DEBUG("JSON parse error: %s", std::string(e.what()).c_str());
            throw std::runtime_error("JSON parse error: " + std::string(e.what()));
        }

        const std::string model = json.contains("metadata")
                                      ? json["metadata"].value("model", std::string("planar"))
                                      : std::string("planar");
        if (model == "planar") {
            model_ = RobotModel::Planar;
        } else if (model == "urdf") {
            model_ = RobotModel::Urdf;
        } else {
            throw std::runtime_error("unknown metadata.model: " + model);
        }

        if (model_ == RobotModel::Urdf) {
            loadUrdfScene(json, filename);
            OMPL_DEBUG("------END load-scene------");
            return;
        }

        n_ = json["metadata"]["n"].get<int>();

        double minX = DBL_MAX;
        double minY = DBL_MAX;
        double maxX = -DBL_MAX;
        double maxY = -DBL_MAX;

        loadRobots(json, minX, minY, maxX, maxY);
        loadObstacles(json, minX, minY, maxX, maxY);
        addBoundarySegments(minX, minY, maxX, maxY);

        OMPL_DEBUG("------END load-scene------");
    }

    /* Planar: links per robot. URDF: actuated joint count. Solver's level sequence
     * (irange(2, getN() + 1)) is therefore correct for both models with no branch. */
    int getN() const { return n_; }
    /* robots_ stays empty for a URDF scene, which supports exactly one robot. */
    int getR() const {
        return model_ == RobotModel::Urdf ? 1 : static_cast<int>(robots_.size());
    }
    int getDof() const { return model_ == RobotModel::Urdf ? n_ : getR() * getN(); }
    RobotModel getModel() const { return model_; }
    double getJointLength() const { return robots_[0].jointLength; }

    const std::string& getUrdfPath() const { return urdfPath_; }
    const std::vector<Obstacle3d>& getObstacles3d() const { return obstacles3d_; }
    const std::array<double, 3>& getBaseXyz() const { return baseXyz_; }
    const std::array<double, 3>& getBaseRpy() const { return baseRpy_; }
    const std::vector<double>& getUrdfStart() const { return urdfStart_; }
    const std::vector<double>& getUrdfGoal() const { return urdfGoal_; }
    const std::set<std::pair<std::string, std::string>>& getDisabledPairs() const {
        return disabledPairs_;
    }
#ifdef MLMP_ENABLE_URDF
    std::shared_ptr<const UrdfRobot> getUrdfRobot() const { return urdfRobot_; }
#endif

    std::vector<common::Segment> getObstaclesSegments() const {
        return obstacleSegments_;
    }

    std::vector<Robot> getRobots() const { return robots_; }

    std::vector<common::Point> getPinnedPositions() const {
        std::vector<common::Point> points;
        points.reserve(robots_.size());
        for (const auto& robot : robots_) {
            points.emplace_back(robot.cx, robot.cy);
        }
        return points;
    }

    std::vector<std::vector<double>> getStartAngles() const { return startAngles_; }
    std::vector<std::vector<double>> getGoalAngles() const { return goalAngles_; }
    double getStartAngleAt(int r, int j) const { return startAngles_[r][j]; }
    double getGoalAngleAt(int r, int j) const { return goalAngles_[r][j]; }

private:
    std::vector<Robot> robots_;
    std::vector<std::vector<double>> startAngles_;
    std::vector<std::vector<double>> goalAngles_;
    std::vector<common::Segment> obstacleSegments_;
    int n_ = 0;

    RobotModel model_ = RobotModel::Planar;
    std::string urdfPath_;
    std::array<double, 3> baseXyz_{};
    std::array<double, 3> baseRpy_{};
    std::vector<double> urdfStart_;
    std::vector<double> urdfGoal_;
    std::vector<Obstacle3d> obstacles3d_;
    /* Each disabled pair is stored in both orders so lookup needs no sorting. */
    std::set<std::pair<std::string, std::string>> disabledPairs_;
#ifdef MLMP_ENABLE_URDF
    std::shared_ptr<const UrdfRobot> urdfRobot_;
#endif

    /* True when `candidate` is `base` itself or lies beneath it. Compared component by
     * component, so "/opt/mlmp/scenes-evil" does not pass for base "/opt/mlmp/scenes".
     * Both arguments must already be canonical. */
    static bool isInsideBase(const std::filesystem::path& base,
                             const std::filesystem::path& candidate) {
        auto b = base.begin();
        auto c = candidate.begin();
        for (; b != base.end(); ++b, ++c) {
            if (c == candidate.end() || *c != *b) {
                return false;
            }
        }
        return true;
    }

    /* Resolves `ref` under one base directory. Returns an empty string when the asset is
     * absent or when it escapes `base` (`..` segments, or a symlink pointing out of it).
     * The caller turns every such outcome into the same message, so nothing here leaks
     * whether a rejected path exists on the host. */
    static std::string resolveUnderBase(const std::filesystem::path& base,
                                        const std::filesystem::path& ref) {
        namespace fs = std::filesystem;
        std::error_code ec;
        const fs::path canonicalBase = fs::weakly_canonical(base, ec);
        if (ec) {
            return {};
        }
        const fs::path candidate = fs::weakly_canonical(canonicalBase / ref, ec);
        if (ec || !isInsideBase(canonicalBase, candidate)) {
            return {};
        }
        return fs::exists(candidate, ec) && !ec ? candidate.string() : std::string();
    }

    /* Resolution order for a URDF or other asset referenced from a scene file: relative
     * to the scene's own directory, then $MLMP_SCENE_DIR. The dev tree hits the first,
     * the published image the second ($MLMP_SCENE_DIR=/opt/mlmp/scenes, ref
     * "robots/ur5.urdf").
     *
     * Scene JSON is untrusted input — solver.h commits to a future Emscripten entry
     * point — so a reference may not name an absolute path and may not climb out of the
     * base it resolves against. Every rejection shares one message: distinguishing
     * "exists but is not a URDF" from "unable to resolve" would be a file-existence
     * oracle over the whole host filesystem. */
    static std::string resolveAssetPath(const std::string& sceneFile, const std::string& ref) {
        namespace fs = std::filesystem;
        const fs::path refPath(ref);
        if (refPath.is_absolute()) {
            throw std::runtime_error("asset path must be relative: " + ref);
        }

        const fs::path sceneDir = fs::path(sceneFile).parent_path();
        if (!sceneDir.empty()) {
            const std::string resolved = resolveUnderBase(sceneDir, refPath);
            if (!resolved.empty()) {
                return resolved;
            }
        }

        if (const char* root = std::getenv("MLMP_SCENE_DIR")) {
            const std::string resolved = resolveUnderBase(fs::path(root), refPath);
            if (!resolved.empty()) {
                return resolved;
            }
        }

        throw std::runtime_error("unable to resolve asset path: " + ref);
    }

    static std::array<double, 3> readXyz(const nlohmann::json& json, const char* key,
                                         const std::array<double, 3>& fallback) {
        if (!json.contains(key)) {
            return fallback;
        }
        const auto& values = json[key];
        if (values.size() != 3) {
            throw std::runtime_error(std::string("expected 3 numbers for ") + key);
        }
        return {values[0].get<double>(), values[1].get<double>(), values[2].get<double>()};
    }

    /* The 3D path. Angles are radians here, not the planar loader's integer degrees. */
    void loadUrdfScene(const nlohmann::json& json, const std::string& filename) {
        if (!json.contains("robots") || json["robots"].size() != 1) {
            throw std::runtime_error("urdf scenes support exactly one robot");
        }
        const auto& robotJson = json["robots"][0];

        urdfPath_ = resolveAssetPath(filename, robotJson.at("urdf").get<std::string>());

        if (robotJson.contains("basePose")) {
            baseXyz_ = readXyz(robotJson["basePose"], "xyz", {0.0, 0.0, 0.0});
            baseRpy_ = readXyz(robotJson["basePose"], "rpy", {0.0, 0.0, 0.0});
        }

        for (const auto& angle : robotJson.at("startAngles")) {
            urdfStart_.push_back(angle.get<double>());
        }
        for (const auto& angle : robotJson.at("goalAngles")) {
            urdfGoal_.push_back(angle.get<double>());
        }

        for (const auto& pair : robotJson.value("disableCollisions", nlohmann::json::array())) {
            if (pair.size() != 2) {
                throw std::runtime_error("disableCollisions entries must be link-name pairs");
            }
            const auto a = pair[0].get<std::string>();
            const auto b = pair[1].get<std::string>();
            disabledPairs_.emplace(a, b);
            disabledPairs_.emplace(b, a);
        }

        loadObstacles3d(json);

#ifdef MLMP_ENABLE_URDF
        urdfRobot_ = UrdfRobot::load(urdfPath_);
        n_ = static_cast<int>(urdfRobot_->joints.size());
        if (urdfStart_.size() != urdfRobot_->joints.size() ||
            urdfGoal_.size() != urdfRobot_->joints.size()) {
            throw std::runtime_error("startAngles/goalAngles length does not match URDF DOF");
        }
#endif
    }

    /* A non-positive or non-finite extent becomes a degenerate FCL shape that collides
     * with nothing, so an obstacle the scene declared would be silently unenforced. */
    static void requirePositiveExtent(double value, const std::string& what) {
        if (!std::isfinite(value) || value <= 0.0) {
            throw std::runtime_error("obstacle3d requires a finite positive " + what);
        }
    }

    void loadObstacles3d(const nlohmann::json& json) {
        for (const auto& entry : json.value("obstacles3d", nlohmann::json::array())) {
            Obstacle3d obstacle;
            obstacle.type = entry.at("type").get<std::string>();

            const auto& pose = entry.at("pose");
            obstacle.xyz = readXyz(pose, "xyz", {0.0, 0.0, 0.0});
            obstacle.rpy = readXyz(pose, "rpy", {0.0, 0.0, 0.0});

            if (obstacle.type == "box") {
                obstacle.size = readXyz(entry, "size", {0.0, 0.0, 0.0});
                if (!entry.contains("size")) {
                    throw std::runtime_error("box obstacle3d requires size");
                }
                requirePositiveExtent(obstacle.size[0], "box size x");
                requirePositiveExtent(obstacle.size[1], "box size y");
                requirePositiveExtent(obstacle.size[2], "box size z");
            } else if (obstacle.type == "sphere") {
                obstacle.radius = entry.at("radius").get<double>();
                requirePositiveExtent(obstacle.radius, "sphere radius");
            } else if (obstacle.type == "cylinder") {
                obstacle.radius = entry.at("radius").get<double>();
                obstacle.length = entry.at("length").get<double>();
                requirePositiveExtent(obstacle.radius, "cylinder radius");
                requirePositiveExtent(obstacle.length, "cylinder length");
            } else {
                throw std::runtime_error("unknown obstacle3d type: " + obstacle.type);
            }

            obstacles3d_.push_back(std::move(obstacle));
        }
    }

    void loadRobots(const nlohmann::json& json,
                    double& minX, double& minY,
                    double& maxX, double& maxY) {
        for (const auto& robotJson : json["robots"]) {
            Robot robot(
                robotJson["id"].get<int>(),
                robotJson["pinnedPosition"]["x"].get<double>(),
                robotJson["pinnedPosition"]["y"].get<double>(),
                robotJson["jointLength"].get<double>()
            );

            double reach = robot.jointLength * n_;
            minY = fmin(minY, robot.cy - reach);
            minX = fmin(minX, robot.cx - reach);
            maxY = fmax(maxY, robot.cy + reach);
            maxX = fmax(maxX, robot.cx + reach);

            robots_.push_back(robot);

            std::vector<double> startAngs;
            std::vector<double> goalAngs;
            for (const auto& angle : robotJson["startAngles"]) {
                startAngs.push_back((M_PI * angle.get<int>()) / 180.0);
            }
            for (const auto& angle : robotJson["goalAngles"]) {
                goalAngs.push_back((M_PI * angle.get<int>()) / 180.0);
            }
            startAngles_.push_back(startAngs);
            goalAngles_.push_back(goalAngs);
        }
    }

    void loadObstacles(const nlohmann::json& json,
                       double& minX, double& minY,
                       double& maxX, double& maxY) {
        for (const auto& obstacle : json["obstacles"]) {
            const auto& points = obstacle["points"];
            size_t numPoints = points.size();

            for (size_t i = 0; i < numPoints; ++i) {
                size_t next = (i + 1) % numPoints;
                obstacleSegments_.emplace_back(
                    points[i]["x"].get<double>(),
                    points[i]["y"].get<double>(),
                    points[next]["x"].get<double>(),
                    points[next]["y"].get<double>()
                );

                minY = fmin(minY, points[i]["y"].get<double>());
                minX = fmin(minX, points[i]["x"].get<double>());
                maxY = fmax(maxY, points[i]["y"].get<double>());
                maxX = fmax(maxX, points[i]["x"].get<double>());
            }
        }
    }

    void addBoundarySegments(double minX, double minY, double maxX, double maxY) {
        double padding = 1.0;
        double left = minX - padding;
        double right = maxX + padding;
        double bottom = minY - padding;
        double top = maxY + padding;

        obstacleSegments_.emplace_back(left, bottom, right, bottom);
        obstacleSegments_.emplace_back(right, bottom, right, top);
        obstacleSegments_.emplace_back(right, top, left, top);
        obstacleSegments_.emplace_back(left, top, left, bottom);
    }
};

}

#endif
