#pragma once

#include "data_types.hpp"

#include <optional>

namespace gg {

class AttitudeEstimator {
public:
    explicit AttitudeEstimator(double accelCorrectionGain = 0.02);

    void setGyroBias(const Vec3& bias) { gyroBias_ = bias; }

    const Quaternion& orientation() const { return orientation_; }

    Quaternion update(double timestamp, const Vec3& gyro, const Vec3& accel);

private:
    void initializeFromAccelerometer(const Vec3& accel);

    std::optional<double> lastTimestamp_;
    Quaternion orientation_ = Quaternion::identity();
    Vec3 gyroBias_{0.0, 0.0, 0.0};
    double accelCorrectionGain_{0.02};
};

}  // namespace gg

