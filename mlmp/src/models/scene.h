#include <iostream>
#include <cmath>
#include <vector>
#include <fstream>

#include <planar_robot_arm.h>
#include <common/geometry.h>

#include <nlohmann/json.hpp>

// #include <CGAL/Exact_predicates_exact_constructions_kernel.h>
// #include <CGAL/Constrained_Delaunay_triangulation_2.h>
// #include <CGAL/Constrained_triangulation_plus_2.h>
// #include <CGAL/Triangulation_face_base_with_info_2.h>
// #include <CGAL/intersections.h>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRT.h>


#ifndef SCENE_H
#define SCENE_H

struct FaceInfo {
    bool isObstacle;
    std::string info;
};

typedef mlmp::PlanarRobotArm Robot;

namespace ob = ompl::base;
namespace og = ompl::geometric;

// typedef CGAL::Exact_predicates_inexact_constructions_kernel K;

// typedef CGAL::Triangulation_vertex_base_2<K> Vb;
// typedef CGAL::Triangulation_face_base_with_info_2<FaceInfo, K> Fb;
// typedef CGAL::Constrained_triangulation_face_base_2<K, Fb> FbC;
// typedef CGAL::Triangulation_data_structure_2<Vb, FbC> Tds;
// // typedef CGAL::Exact_predicates_tag Itag;
// typedef CGAL::Constrained_Delaunay_triangulation_2<K, Tds> CDT;
// // typedef CGAL::Constrained_triangulation_plus_2<CDT> CDT_plus;
// typedef CDT::Face_handle Face_handle;
// typedef CDT::Edge_iterator Edge_iterator;

// typedef CGAL::Polygon_2<K> Polygon;
// typedef K::Point_2 Point;
// typedef K::Segment_2 Segment;
// typedef CDT::Edge Edge;


namespace mlmp {
    class Scene {
        std::vector<Robot> robots;
        std::vector<std::vector<double>> startAngles;
        std::vector<std::vector<double>> goalAngles;
        std::vector<common::Segment> obstacleSegments;
        // CDT cdt;
        int n;

        // void addObstacle(const Polygon& polygon) {
        //     auto prev = polygon.vertices_end();
        //     for (auto it = polygon.vertices_begin(); it != polygon.vertices_end(); ++it) {
        //         if (prev != polygon.vertices_end()) {
        //             cdt.insert_constraint(*prev, *it);
        //         }
        //         prev = it;
        //     }
        //     cdt.insert_constraint(*prev, *polygon.vertices_begin());
        // }

        void addRobot(const Robot& robot) {
            robots.emplace_back(robot);
        }

        void setN(int value) {
            n=value;
        }

        // std::vector<Segment> computeSegments(const std::vector<double> &jointAngles, const Point pinnedPosition, const double jointLength) const {
        //     if (verbose) {
        //         std::cout<<"------On scene::compute-segments------\nGot {";
        //         for (const auto& i: jointAngles)
        //             std::cout << (180*i)/M_PI << ' ';
        //         std::cout<<"}, with "<<pinnedPosition<<std::endl;
        //     }
            
        //     std::vector<Segment> segments;
        //     auto currentStart = pinnedPosition;
        //     double currentAngle = 0.0;

        //     for (size_t i = 0; i < jointAngles.size(); ++i) {
        //         currentAngle = jointAngles[i];
        //         Point currentEnd(currentStart.x() + jointLength * cos(currentAngle),
        //                         currentStart.y() + jointLength * sin(currentAngle));
        //         segments.emplace_back(Segment(currentStart, currentEnd));
        //         currentStart = currentEnd;
        //     }

        //     if (verbose) {
        //         std::cout<<"------"<<std::endl;
        //     }
        //     return segments;
        // }

        // Function to check if a segment is valid by walking along it and checking face validity
        // bool isArmValid(const std::vector<Segment> &arm) const {
        //     if (verbose) {
        //         std::cout<<"------On scene::is-arm-valid------"<<std::endl;
        //     }
        //     for (const auto& segment : arm) {
        //         if (verbose) {
        //             std::cout<<"seg: " << segment<< "\n";
        //         }
        //         // Walk through the segment
        //         Face_handle face = cdt.locate(segment.source());
        //         std::set<Point> setOfIntersection;

        //         if (face != nullptr) {
        //             do {
        //                 if (verbose) {
        //                     std::cout<<"** (In inner loop) "<< face->info().info << std::endl;
        //                 }

        //                 if (face->info().isObstacle) {
        //                     return false;
        //                 }

        //                 // Find the next face intersected by the segment
        //                 boost::optional<std::pair<Point, Face_handle>> intersection;
        //                 for (int i = 0; i < 3; ++i) {
        //                     Edge edge(face, i);
        //                     Point p1 = edge.first->vertex((i + 1) % 3)->point();
        //                     Point p2 = edge.first->vertex((i + 2) % 3)->point();
        //                     Segment edge_segment(p1, p2);
        //                     if (verbose) {
        //                         std::cout<<"** (In inner loop) edge: "<<edge_segment<<"\n"; 
        //                     }
        //                     auto result = CGAL::intersection(segment, edge_segment);
        //                     if (result) {
        //                         if (const Point* ipoint = boost::get<Point>(&*result)) {
        //                            if (verbose) {
        //                                 std::cout<<"** (In inner loop) found intersection, with "<< *ipoint<<std::endl;
        //                             }
        //                             if(setOfIntersection.find(*ipoint) != setOfIntersection.end()) {
        //                                 continue;
        //                             }
        //                             if (!intersection || CGAL::compare_distance_to_point(segment.source(), *ipoint, intersection->first) == CGAL::SMALLER) {
        //                                 Face_handle neighbor = face->neighbor(i);
        //                                 if (cdt.is_constrained(edge)) { // Check if the edge is not constrained
        //                                     if (verbose) {
        //                                         std::cout<<"------On scene::is-arm-valid------\nFoundIntersection"<<std::endl;
        //                                     }
        //                                     return false;
        //                                 } else {
        //                                     intersection = std::make_pair(*ipoint, neighbor);
        //                                     setOfIntersection.insert(*ipoint);
        //                                 }
        //                             }
        //                         }
        //                     }
        //                 }
        //                 if (!intersection) break;
        //                 face = intersection->second;
        //             } while (face != nullptr && !cdt.is_infinite(face));
        //         }

        //     }
        //     if (verbose) {
        //         std::cout<<"------"<<std::endl;
        //     }
        //     return true;
        // }

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


            // Load metadata
            setN(json["metadata"]["n"].get<int>());
            
            double minY, minX=DBL_MAX;
            double maxY, maxX=DBL_MIN;
            
            // Load robots
            for (const auto& robot : json["robots"]) {
                Robot _robot = Robot(
                    robot["id"].get<int>(),
                    robot["pinnedPosition"]["x"].get<double>(),
                    robot["pinnedPosition"]["y"].get<double>(),
                    robot["jointLength"]
                );

                minY = fmin(minY, _robot.cy - _robot.jointLength*n);
                minX = fmin(minX, _robot.cx - _robot.jointLength*n);
                maxY = fmax(maxY, _robot.cy + _robot.jointLength*n);
                maxX = fmax(maxX, _robot.cx + _robot.jointLength*n);

                addRobot(_robot);
                std::vector<double> _sa;
                std::vector<double> _ga;
                for (const auto& angle : robot["startAngles"]) {
                    _sa.emplace_back((M_PI*angle.get<int>())/180);
                }
                for (const auto& angle : robot["goalAngles"]) {
                    _ga.emplace_back((M_PI*angle.get<int>())/180);
                }
                startAngles.emplace_back(_sa);
                goalAngles.emplace_back(_ga);
            }

            // // Load obstacles
            // std::vector<Polygon> obstacles;
            
            
            for (const auto& obstacle : json["obstacles"]) {
                int numPoints = obstacle["points"].size();
                // Polygon polygon;
                for (auto i=0; i<obstacle["points"].size(); ++i) {
                    obstacleSegments.push_back(common::Segment(obstacle["points"][i]["x"].get<double>(), obstacle["points"][i]["y"].get<double>(),
                                                       obstacle["points"][(i+1)%numPoints]["x"].get<double>(), obstacle["points"][(i+1)%numPoints]["y"].get<double>()));

                    minY = fmin(minY, obstacle["points"][i]["y"].get<double>());
                    minX = fmin(minX, obstacle["points"][i]["x"].get<double>());
                    maxY = fmax(maxY, obstacle["points"][i]["y"].get<double>());
                    maxX = fmax(maxX, obstacle["points"][i]["x"].get<double>());    
                }
            }

            obstacleSegments.emplace_back(common::Segment(minX-1, minY-1, maxX+1, minY-1));
            obstacleSegments.emplace_back(common::Segment(maxX+1, minY-1, maxX+1, maxY+1));
            obstacleSegments.emplace_back(common::Segment(maxX+1, maxY+1, minX-1, maxY+1));
            obstacleSegments.emplace_back(common::Segment(minX-1, maxY+1, minX-1, minY-1));

            OMPL_DEBUG("------END load-scene------");
        }

        // bool isStateValid(const ob::State *state, const int& r, const int& j) const {
        //     // Compute the robot arm segments based on the given joint angles
        //     const auto *_angles = state->as<ob::RealVectorStateSpace::StateType>();
        //     std::vector<std::vector<Segment>> robotArms(r);
        //     if (verbose) {
        //         std::cout<<"------On scene::is-state-valid------"<<std::endl;
        //         std::cout<<"robots "<<r<<" | joints "<<j<<std::endl;
        //     }
        //     for (unsigned int i = 0; i < r; ++i) {
        //         std::vector<double> values(j);
        //         for (unsigned int k = 0; k < j; ++k) {
        //             values[k] = _angles->values[i*j + k];
        //         }
        //         robotArms[i] = computeSegments(values, Point(robots[i].cx, robots[i].cy), robots[i].jointLength);
        //     }
            
        //     for (const auto &arm : robotArms) 
        //         if (!isArmValid(arm)) 
        //             return false;    
            
        //     if (verbose) {
        //         std::cout<<"check for self intersections"<<std::endl;
        //     }

        //     for (auto i = 0; i < r; ++i) 
        //         for (auto j1 = 0; j1 < j; ++j1) 
        //             for (auto j2 = j1 + 1; j2 < j; ++j2) {
        //                 if (j2 == j1 + 1) {
        //                     if (std::abs(M_PI - std::abs(_angles->values[i*j + j1] - _angles->values[i*j+j2])) <= 0.001) {
        //                         if (verbose) 
        //                             std::cout<<"** (In inner loop) found intesection for robot "<<i<<" joints "<< j1<< " "<< j2 <<std::endl;
        //                         return false;
        //                     }
        //                 } else {
        //                     const auto result = intersection(robotArms[i][j1], robotArms[i][j2]);
        //                     if (result) {
        //                         if (verbose) 
        //                             std::cout<<"** (In inner loop) found intesection for robot "<<i<<" joints "<< j1<< " "<< j2 <<std::endl;
        //                         return false; 
        //                     }
        //                 }
        //             }
                        

                             
        //     if (verbose) {
        //         std::cout<<"check for dual intersections"<<std::endl;
        //     }

        //     for (auto r1 = 0; r1 < r; ++r1) 
        //         for (auto r2 = r1 + 1; r2 < r; ++r2)
        //             for (auto j1 = 0; j1 < j; ++j1) 
        //                 for (auto j2 = 0; j2 < j; ++j2) {
        //                     const auto result = intersection(robotArms[r1][j1], robotArms[r2][j2]);
        //                     if (result) {
        //                         if (verbose) 
        //                             std::cout<<"** (In inner loop) found intesection for robot "<<r1<<" "<< r2 <<" joints "<< j1<< " "<< j2 <<std::endl;
        //                         return false;
        //                     }
        //                 }
        //     if (verbose) {
        //         std::cout<<"------END scene::is-state-valid------"<<std::endl;
        //     }
        //     return true; 
        // }

        int getN() const { return n;}

        int getR() const {return robots.size();}    

        std::vector<common::Segment> getObstaclesSegments() {
            return obstacleSegments;
        }

        double getJointLength() {
            return robots[0].jointLength;
        }

        std::vector<Robot> getRobots() {return robots;}

        std::vector<common::Point> getPinnedPositions() {
            std::vector<common::Point> points;
            for (auto r : robots) {
                points.emplace_back(common::Point(r.cx, r.cy));
            }
            return points;
        }

        std::vector<std::vector<double>> getStartAngles() { return startAngles; }
        std::vector<std::vector<double>> getGoalAngles() { return goalAngles; }
        double getStartAngleAt(int r, int j) { return startAngles[r][j]; }
        double getGoalAngleAt(int r, int j) { return goalAngles[r][j]; }
    };
} 

#endif