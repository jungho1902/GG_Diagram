#pragma once

#include "attitude_estimator.hpp"

#include <optional>

namespace gg {

class GgProcessor {
public:
    struct Config {
        Vec3 accelBias{0.0, 0.0, 0.0};
        Vec3 gyroBias{0.0, 0.0, 0.0};
        Vec3 leverArm{0.0, 0.0, 0.0};  // Body frame IMU position relative to CG (m)
        double velocityDamping{0.0};   // Optional exponential decay to prevent drift (1/s)
    };

    explicit GgProcessor(AttitudeEstimator attitudeEstimator);
    GgProcessor(AttitudeEstimator attitudeEstimator, Config config);

    ProcessedPoint process(const ImuData& sample);

    const AttitudeEstimator& attitude() const { return attitudeEstimator_; }

private:
    AttitudeEstimator attitudeEstimator_;
    Config config_;

    std::optional<Vec3> lastOmega_;
    std::optional<double> lastTimestamp_;
    double longitudinalVelocity_{0.0};
};

}  // namespace gg
