#include "attitude_estimator.hpp"

#include <algorithm>

namespace gg {

AttitudeEstimator::AttitudeEstimator(double accelCorrectionGain)
    : accelCorrectionGain_(accelCorrectionGain) {}

void AttitudeEstimator::initializeFromAccelerometer(const Vec3& accel) {
    Vec3 gravityBody = normalized(-accel);
    double roll = std::atan2(gravityBody.y, gravityBody.z);
    double pitch = std::atan2(-gravityBody.x,
                              std::sqrt(gravityBody.y * gravityBody.y + gravityBody.z * gravityBody.z));
    orientation_ = Quaternion::fromEuler(roll, pitch, 0.0);
    orientation_.normalize();
}

Quaternion AttitudeEstimator::update(double timestamp, const Vec3& gyro, const Vec3& accel) {
    if (!lastTimestamp_) {
        initializeFromAccelerometer(accel);
        lastTimestamp_ = timestamp;
        return orientation_;
    }

    double dt = timestamp - *lastTimestamp_;
    if (dt <= 0.0) {
        dt = 1e-3;  // fallback to keep integration stable
    }

    Vec3 omega = gyro - gyroBias_;

    double accelMagnitude = norm(accel);
    if (accelMagnitude > 1e-3) {
        Vec3 measuredGravity = normalized(-accel);
        Vec3 predictedGravity = normalized(orientation_.rotateInverse(Vec3{0.0, 0.0, -kGravity}));
        Vec3 correction = cross(predictedGravity, measuredGravity);
        omega += correction * accelCorrectionGain_;
    }

    Quaternion delta = Quaternion::fromAngularVelocity(omega, dt);
    orientation_ = (orientation_ * delta).normalized();

    lastTimestamp_ = timestamp;
    return orientation_;
}

}  // namespace gg

