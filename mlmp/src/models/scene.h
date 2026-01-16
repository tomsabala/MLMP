#ifndef SCENE_H
#define SCENE_H

#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include <ompl/base/SpaceInformation.h>
#include <ompl/util/Console.h>

#include <common/geometry.h>
#include <planar_robot_arm.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

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

        n_ = json["metadata"]["n"].get<int>();

        double minX = DBL_MAX;
        double minY = DBL_MAX;
        double maxX = DBL_MIN;
        double maxY = DBL_MIN;

        loadRobots(json, minX, minY, maxX, maxY);
        loadObstacles(json, minX, minY, maxX, maxY);
        addBoundarySegments(minX, minY, maxX, maxY);

        OMPL_DEBUG("------END load-scene------");
    }

    int getN() const { return n_; }
    int getR() const { return static_cast<int>(robots_.size()); }
    double getJointLength() const { return robots_[0].jointLength; }

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
