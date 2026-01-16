#ifndef PLANAR_ROBOT_ARM_H
#define PLANAR_ROBOT_ARM_H

namespace mlmp {

class PlanarRobotArm {
public:
    int id;
    double cx, cy;
    double jointLength;

    PlanarRobotArm(int robotId, double x, double y, double length)
        : id(robotId), cx(x), cy(y), jointLength(length) {
    }
};

}

#endif
