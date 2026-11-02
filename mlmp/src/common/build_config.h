#ifndef MLMP_COMMON_BUILD_CONFIG_H
#define MLMP_COMMON_BUILD_CONFIG_H

namespace mlmp {

/* Whether this binary was built with urdfdom + FCL. A URDF scene loads either way
 * (the JSON is parsed, the robot is not), so main.cpp can turn an unsupported build
 * into a clean usage error instead of a crash or a silent no-op. */
#ifdef MLMP_ENABLE_URDF
inline constexpr bool kUrdfSupported = true;
#else
inline constexpr bool kUrdfSupported = false;
#endif

}

#endif
