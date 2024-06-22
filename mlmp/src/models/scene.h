#include <iostream>
#include <cmath>
#include <vector>
#include <fstream>

#include <planar_robot_arm.h>
#include <abstraction_type.h>

#include <nlohmann/json.hpp>

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Constrained_triangulation_plus_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/intersections.h>

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
typedef mlmp::abstraction::AbstractionType AbstractionLevel;

namespace ob = ompl::base;
namespace og = ompl::geometric;

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;

typedef CGAL::Triangulation_vertex_base_2<K> Vb;
typedef CGAL::Triangulation_face_base_with_info_2<FaceInfo, K> Fb;
typedef CGAL::Constrained_triangulation_face_base_2<K, Fb> FbC;
typedef CGAL::Triangulation_data_structure_2<Vb, FbC> Tds;
// typedef CGAL::Exact_predicates_tag Itag;
typedef CGAL::Constrained_Delaunay_triangulation_2<K, Tds> CDT;
// typedef CGAL::Constrained_triangulation_plus_2<CDT> CDT_plus;
typedef CDT::Face_handle Face_handle;

typedef CGAL::Polygon_2<K> Polygon;
typedef K::Point_2 Point;
typedef K::Segment_2 Segment;
typedef CDT::Edge Edge;


namespace mlmp {
    class Scene {
        std::vector<Robot> robots;
        std::vector<std::vector<double>> startAngles;
        std::vector<std::vector<double>> goalAngles;
        CDT cdt;
        int n;
        bool verbose;

        void addObstacle(const Polygon& polygon) {
            auto prev = polygon.vertices_end();
            for (auto it = polygon.vertices_begin(); it != polygon.vertices_end(); ++it) {
                if (prev != polygon.vertices_end()) {
                    cdt.insert_constraint(*prev, *it);
                }
                prev = it;
            }
            cdt.insert_constraint(*prev, *polygon.vertices_begin());
        }

        void addRobot(const Robot& robot) {
            robots.emplace_back(robot);
        }

        void setN(int value) {
            n=value;
        }

        std::vector<Segment> computeSegments(const std::vector<double> &jointAngles, const Point pinnedPosition, const double jointLength) const {
            if (verbose) {
                std::cout<<"------On scene::compute-segments------\nGot {";
                for (const auto& i: jointAngles)
                    std::cout << (180*i)/M_PI << ' ';
                std::cout<<"}, with "<<pinnedPosition<<std::endl;
            }
            
            std::vector<Segment> segments;
            auto currentStart = pinnedPosition;
            double currentAngle = 0.0;

            for (size_t i = 0; i < jointAngles.size(); ++i) {
                currentAngle = jointAngles[i];
                Point currentEnd(currentStart.x() + jointLength * cos(currentAngle),
                                currentStart.y() + jointLength * sin(currentAngle));
                segments.emplace_back(Segment(currentStart, currentEnd));
                currentStart = currentEnd;
            }

            if (verbose) {
                std::cout<<"------"<<std::endl;
            }
            return segments;
        }

        Polygon computeBoundingBox(const std::vector<Polygon> obstacles) {
            double minY, minX=DBL_MAX;
            double maxY, maxX=DBL_MIN;

            for (const auto poly : obstacles) {
                for (const auto vertex : poly.vertices()) {
                    minY = fmin(minY, vertex.y());
                    minX = fmin(minX, vertex.x());
                    maxY = fmax(maxY, vertex.y());
                    maxX = fmax(maxX, vertex.x());    
                }
            }

            for (const auto robot : robots) {
                minY = fmin(minY, robot.cy - robot.jointLength*n);
                minX = fmin(minX, robot.cx - robot.jointLength*n);
                maxY = fmax(maxY, robot.cy + robot.jointLength*n);
                maxX = fmax(maxX, robot.cx + robot.jointLength*n);
            }

            Polygon res;
            res.push_back(Point(minX-1, minY-1));
            res.push_back(Point(maxX+1, minY-1));
            res.push_back(Point(maxX+1, maxY+1));
            res.push_back(Point(minX-1, maxY+1));
            return res;
        }

        // Function to check if a segment is valid by walking along it and checking face validity
        bool isArmValid(const std::vector<Segment> &arm) const {
            if (verbose) {
                std::cout<<"------On scene::is-arm-valid------"<<std::endl;
            }
            for (const auto& segment : arm) {
                if (verbose) {
                    std::cout<<"seg: " << segment<< "\n";
                }
                // Walk through the segment
                Face_handle face = cdt.locate(segment.source());
                std::set<Point> setOfIntersection;

                if (face != nullptr) {
                    do {
                        if (verbose) {
                            std::cout<<"** (In inner loop) "<< face->info().info << std::endl;
                        }

                        if (face->info().isObstacle) {
                            return false;
                        }

                        // Find the next face intersected by the segment
                        boost::optional<std::pair<Point, Face_handle>> intersection;
                        for (int i = 0; i < 3; ++i) {
                            Edge edge(face, i);
                            Point p1 = edge.first->vertex((i + 1) % 3)->point();
                            Point p2 = edge.first->vertex((i + 2) % 3)->point();
                            Segment edge_segment(p1, p2);
                            if (verbose) {
                                std::cout<<"** (In inner loop) edge: "<<edge_segment<<"\n"; 
                            }
                            auto result = CGAL::intersection(segment, edge_segment);
                            if (result) {
                                if (const Point* ipoint = boost::get<Point>(&*result)) {
                                   if (verbose) {
                                        std::cout<<"** (In inner loop) found intersection, with "<< *ipoint<<std::endl;
                                    }
                                    if(setOfIntersection.find(*ipoint) != setOfIntersection.end()) {
                                        continue;
                                    }
                                    if (!intersection || CGAL::compare_distance_to_point(segment.source(), *ipoint, intersection->first) == CGAL::SMALLER) {
                                        Face_handle neighbor = face->neighbor(i);
                                        if (cdt.is_constrained(edge)) { // Check if the edge is not constrained
                                            if (verbose) {
                                                std::cout<<"------On scene::is-arm-valid------\nFoundIntersection"<<std::endl;
                                            }
                                            return false;
                                        } else {
                                            intersection = std::make_pair(*ipoint, neighbor);
                                            setOfIntersection.insert(*ipoint);
                                        }
                                    }
                                }
                            }
                        }
                        if (!intersection) break;
                        face = intersection->second;
                    } while (face != nullptr && !cdt.is_infinite(face));
                }

            }
            if (verbose) {
                std::cout<<"------"<<std::endl;
            }
            return true;
        }

        public:
    
        void loadScene(const std::string& filename, bool debug) {
            if (verbose) {
                std::cout<<"------On scene::load-scene------"<<std::endl;
            }
            verbose=debug;
            std::ifstream file(filename);
            if (!file.is_open()) {
                throw std::runtime_error("Unable to open file: " + filename);
            }

            nlohmann::json json;
            try {
                file >> json;
            } catch (const nlohmann::json::parse_error& e) {
                throw std::runtime_error("JSON parse error: " + std::string(e.what()));
            }

                        // Load robots
            for (const auto& robot : json["robots"]) {
                Robot _robot = Robot(
                    robot["id"].get<int>(),
                    robot["pinnedPosition"]["x"].get<double>(),
                    robot["pinnedPosition"]["y"].get<double>(),
                    robot["jointLength"]
                );
                addRobot(_robot);
                std::vector<double> _sa;
                std::vector<double> _ga;
                for (const auto& angle : robot["startAngles"]) {
                    _sa.emplace_back((M_PI*angle.get<int>())/180);
                }
                for (const auto& angle : robot["goalAngles"]) {
                    _ga.emplace_back((M_PI*angle.get<int>())/180);
                }
                if (verbose) {
                    std::cout<<"** (In inner loop) start angles: ";
                    for (const auto& a : _sa) {
                        std::cout<<a <<" ";
                    }
                    std::cout<<std::endl;
                    std::cout<<"** (In inner loop) goal angles: ";
                    for (const auto& a : _ga) {
                        std::cout<<a <<" ";
                    }
                    std::cout<<std::endl;
                }
                startAngles.emplace_back(_sa);
                goalAngles.emplace_back(_ga);
            }

            // Load obstacles
            std::vector<Polygon> obstacles;
            for (const auto& obstacle : json["obstacles"]) {
                Polygon polygon;
                for (const auto& point : obstacle["points"]) {
                    polygon.push_back(Point(point["x"].get<double>(), point["y"].get<double>()));
                }
                addObstacle(polygon);
                obstacles.emplace_back(polygon);
            }
            Polygon boundingBox = computeBoundingBox(obstacles);
            addObstacle(boundingBox);
            // Mark faces as belonging to obstacles
            for (auto face = cdt.all_faces_begin(); face != cdt.all_faces_end(); ++face) {
                face->info().isObstacle = false;
                face->info().info = "free space";
            }
            for (size_t i=0; i<obstacles.size(); ++i) {
                auto poly = obstacles[i];
                for (auto face = cdt.all_faces_begin(); face != cdt.all_faces_end(); ++face) {
                    if (poly.bounded_side(face->vertex(0)->point()) == CGAL::ON_BOUNDED_SIDE &&
                        poly.bounded_side(face->vertex(1)->point()) == CGAL::ON_BOUNDED_SIDE &&
                        poly.bounded_side(face->vertex(2)->point()) == CGAL::ON_BOUNDED_SIDE) {
                        face->info().isObstacle = true;
                        face->info().info = std::to_string(i);
                    }
                }
            }
        
            // Load metadata
            setN(json["metadata"]["n"].get<int>());
        }

        bool isStateValid(const ob::State *state, const int& r, const int& j) const {
            // Compute the robot arm segments based on the given joint angles
            const auto *_angles = state->as<ob::RealVectorStateSpace::StateType>();
            std::vector<std::vector<Segment>> robotArms(r);
            if (verbose) {
                std::cout<<"------On scene::is-state-valid------"<<std::endl;
                std::cout<<"robots "<<r<<" | joints "<<j<<std::endl;
            }
            for (unsigned int i = 0; i < r; ++i) {
                std::vector<double> values(j);
                for (unsigned int k = 0; k < j; ++k) {
                    values[k] = _angles->values[i*j + k];
                }
                robotArms[i] = computeSegments(values, Point(robots[i].cx, robots[i].cy), robots[i].jointLength);
            }
            
            for (const auto &arm : robotArms) 
                if (!isArmValid(arm)) 
                    return false;    
            
            if (verbose) {
                std::cout<<"check for self intersections"<<std::endl;
            }

            for (auto i = 0; i < r; ++i) 
                for (auto j1 = 0; j1 < j; ++j1) 
                    for (auto j2 = j1 + 1; j2 < j; ++j2) {
                        if (j2 == j1 + 1) {
                            if (std::abs(M_PI - std::abs(_angles->values[i*j + j1] - _angles->values[i*j+j2])) <= 0.001) {
                                if (verbose) 
                                    std::cout<<"** (In inner loop) found intesection for robot "<<i<<" joints "<< j1<< " "<< j2 <<std::endl;
                                return false;
                            }
                        } else {
                            const auto result = intersection(robotArms[i][j1], robotArms[i][j2]);
                            if (result) {
                                if (verbose) 
                                    std::cout<<"** (In inner loop) found intesection for robot "<<i<<" joints "<< j1<< " "<< j2 <<std::endl;
                                return false; 
                            }
                        }
                    }
                        

                             
            if (verbose) {
                std::cout<<"check for dual intersections"<<std::endl;
            }

            for (auto r1 = 0; r1 < r; ++r1) 
                for (auto r2 = r1 + 1; r2 < r; ++r2)
                    for (auto j1 = 0; j1 < j; ++j1) 
                        for (auto j2 = 0; j2 < j; ++j2) {
                            const auto result = intersection(robotArms[r1][j1], robotArms[r2][j2]);
                            if (result) {
                                if (verbose) 
                                    std::cout<<"** (In inner loop) found intesection for robot "<<r1<<" "<< r2 <<" joints "<< j1<< " "<< j2 <<std::endl;
                                return false;
                            }
                        }
            if (verbose) {
                std::cout<<"------END scene::is-state-valid------"<<std::endl;
            }
            return true; 
        }

        int getN() const { return n;}

        int getR() const {return robots.size();}    

        std::vector<Robot> getRobots() {return robots;}

        std::vector<std::vector<double>> getStartAngles() { return startAngles; }
        std::vector<std::vector<double>> getGoalAngles() { return goalAngles; }
        double getStartAngleAt(int r, int j) { return startAngles[r][j]; }
        double getGoalAngleAt(int r, int j) { return goalAngles[r][j]; }
    };
} 

#endif