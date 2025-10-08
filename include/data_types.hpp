#pragma once

#include "math_utils.hpp"

#include <cstddef>

namespace gg {

struct ImuData {
    double timestamp{0.0};  // seconds
    Vec3 accel;             // m/s^2 in body frame (specific force measurement)
    Vec3 gyro;              // rad/s in body frame
};

struct ProcessedPoint {
    double timestamp{0.0};
    Vec3 accelNav;            // CG acceleration expressed in navigation frame (m/s^2)
    Vec3 accelBody;           // CG acceleration expressed in body frame (m/s^2)
    Vec3 omegaBody;           // Angular velocity body frame (rad/s)
    Vec3 alphaBody;           // Angular acceleration body frame (rad/s^2)
    double longitudinalVelocity{0.0};  // Forward velocity in navigation X (m/s)
    double yawRate{0.0};                  // rad/s
    double yawAcceleration{0.0};          // rad/s^2
    double gx{0.0};                       // Longitudinal acceleration in g-units
    double gy{0.0};                       // Lateral acceleration in g-units
};

struct Point2D {
    double x{0.0};
    double y{0.0};
};

}  // namespace gg

