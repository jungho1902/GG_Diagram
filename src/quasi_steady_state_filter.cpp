#include "quasi_steady_state_filter.hpp"

#include <algorithm>
#include <cmath>

namespace gg {

QuasiSteadyStateFilter::QuasiSteadyStateFilter()
    : QuasiSteadyStateFilter(Thresholds{}) {}

QuasiSteadyStateFilter::QuasiSteadyStateFilter(Thresholds thresholds)
    : thresholds_(thresholds) {}

void QuasiSteadyStateFilter::prune(double currentTime) {
    while (!history_.empty() && currentTime - history_.front().timestamp > thresholds_.windowDuration) {
        history_.pop_front();
    }
}

bool QuasiSteadyStateFilter::update(const ProcessedPoint& point) {
    history_.push_back(point);
    prune(point.timestamp);
    if (history_.size() < 2) {
        return false;
    }
    return isSteadyState(history_);
}

bool QuasiSteadyStateFilter::isSteadyState(const std::deque<ProcessedPoint>& window) const {
    if (window.size() < 2) {
        return false;
    }

    double maxJerk = 0.0;
    double maxAngularAccel = 0.0;
    double sumDvdt2 = 0.0;
    double sumDrdt2 = 0.0;
    std::size_t derivativeSamples = 0;

    for (const auto& sample : window) {
        maxAngularAccel = std::max(maxAngularAccel, norm(sample.alphaBody));
    }

    for (std::size_t i = 1; i < window.size(); ++i) {
        const auto& prev = window[i - 1];
        const auto& curr = window[i];
        double dt = curr.timestamp - prev.timestamp;
        if (dt <= 1e-6) {
            continue;
        }

        Vec3 jerkVec = (curr.accelNav - prev.accelNav) / dt;
        double jerkSup = std::max({std::fabs(jerkVec.x), std::fabs(jerkVec.y), std::fabs(jerkVec.z)});
        maxJerk = std::max(maxJerk, jerkSup);

        double dvdt = (curr.longitudinalVelocity - prev.longitudinalVelocity) / dt;
        double drdt = (curr.yawRate - prev.yawRate) / dt;
        sumDvdt2 += dvdt * dvdt;
        sumDrdt2 += drdt * drdt;
        ++derivativeSamples;
    }

    if (derivativeSamples == 0) {
        return false;
    }

    double rmsDvdt = std::sqrt(sumDvdt2 / derivativeSamples);
    double rmsDrdt = std::sqrt(sumDrdt2 / derivativeSamples);

    return maxJerk <= thresholds_.jerk &&
           maxAngularAccel <= thresholds_.angularAcceleration &&
           rmsDvdt <= thresholds_.velocityDerivativeRms &&
           rmsDrdt <= thresholds_.yawDerivativeRms;
}

}  // namespace gg
