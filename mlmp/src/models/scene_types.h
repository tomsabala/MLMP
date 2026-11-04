#ifndef MLMP_MODELS_SCENE_TYPES_H
#define MLMP_MODELS_SCENE_TYPES_H

#include <array>
#include <string>

namespace mlmp {

/* Which geometry/validity path a scene drives. Chosen once at load time from
 * metadata.model; the planar path must stay byte-identical to what it was before
 * URDF support existed, so the two never share a code path below Solver. */
enum class RobotModel { Planar, Urdf };

/* A workspace obstacle for the 3D (URDF) path. `rpy` is the URDF convention:
 * fixed-axis roll, then pitch, then yaw, i.e.
 *   AngleAxisd(rpy[2], UnitZ()) * AngleAxisd(rpy[1], UnitY()) * AngleAxisd(rpy[0], UnitX()).
 * Every rpy->rotation conversion in this project (obstacles, basePose, URDF origins and
 * the renderer's geom quaternions) uses exactly that order; disagreeing in one place
 * puts the renderer and the collision checker out of sync and looks like a planner bug. */
struct Obstacle3d {
    std::string type;                 // "box" | "sphere" | "cylinder"
    std::array<double, 3> size{};     // full size for a box, not half-extents
    double radius = 0.0;
    double length = 0.0;
    std::array<double, 3> xyz{};
    std::array<double, 3> rpy{};
};

}

#endif
