# include <vector>

#ifndef PLANAR_ROBOT_ARM_H
#define PLANAR_ROBOT_ARM_H

namespace mlmp {
    class PlanarRobotArm {
        
        public:
        
        int id;
        double cx, cy;
        double jointLength;
        
        PlanarRobotArm(int id, double x, double y, double l) : id(id), cx(x), cy(y), jointLength(l) {}
    };
}

#endif