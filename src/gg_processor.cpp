#include "gg_processor.hpp"

#include <cmath>

namespace gg {

GgProcessor::GgProcessor(AttitudeEstimator attitudeEstimator)
    : GgProcessor(std::move(attitudeEstimator), Config{}) {}

GgProcessor::GgProcessor(AttitudeEstimator attitudeEstimator, Config config)
    : attitudeEstimator_(std::move(attitudeEstimator)), config_(config) {
    attitudeEstimator_.setGyroBias(config_.gyroBias);
}

ProcessedPoint GgProcessor::process(const ImuData& sample) {
    Vec3 correctedAccel = sample.accel - config_.accelBias;
    Quaternion orientation = attitudeEstimator_.update(sample.timestamp, sample.gyro, correctedAccel);

    Vec3 omega = sample.gyro - config_.gyroBias;

    double dt = 0.0;
    Vec3 alpha{0.0, 0.0, 0.0};
    if (lastTimestamp_) {
        dt = sample.timestamp - *lastTimestamp_;
        if (dt > 1e-6 && lastOmega_) {
            alpha = (omega - *lastOmega_) / dt;
        }
    }

    Vec3 leverTerm = cross(alpha, config_.leverArm) + cross(omega, cross(omega, config_.leverArm));
    Vec3 gravityBody = orientation.rotateInverse(Vec3{0.0, 0.0, -kGravity});
    Vec3 accelBody = correctedAccel - leverTerm + gravityBody;
    Vec3 accelNav = orientation.rotate(accelBody);

    if (dt > 1e-6) {
        if (config_.velocityDamping > 0.0) {
            double decay = std::exp(-config_.velocityDamping * dt);
            longitudinalVelocity_ = longitudinalVelocity_ * decay + accelNav.x * dt;
        } else {
            longitudinalVelocity_ += accelNav.x * dt;
        }
    }

    ProcessedPoint output;
    output.timestamp = sample.timestamp;
    output.accelNav = accelNav;
    output.accelBody = accelBody;
    output.omegaBody = omega;
    output.alphaBody = alpha;
    output.longitudinalVelocity = longitudinalVelocity_;
    output.yawRate = omega.z;
    output.yawAcceleration = alpha.z;
    output.gx = accelNav.x / kGravity;
    output.gy = accelNav.y / kGravity;

    lastOmega_ = omega;
    lastTimestamp_ = sample.timestamp;

    return output;
}

}  // namespace gg
