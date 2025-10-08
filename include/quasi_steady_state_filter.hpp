#pragma once

#include "data_types.hpp"

#include <deque>

namespace gg {

class QuasiSteadyStateFilter {
public:
    struct Thresholds {
        double jerk{0.3};  // m/s^3
        double angularAcceleration{5.0 * 3.14159265358979323846 / 180.0};  // rad/s^2
        double velocityDerivativeRms{0.5};  // m/s^3
        double yawDerivativeRms{2.0 * 3.14159265358979323846 / 180.0};    // rad/s^2 RMS
        double windowDuration{0.2};  // seconds
    };

    QuasiSteadyStateFilter();
    explicit QuasiSteadyStateFilter(Thresholds thresholds);

    bool update(const ProcessedPoint& point);

    const std::deque<ProcessedPoint>& history() const { return history_; }

private:
    void prune(double currentTime);
    bool isSteadyState(const std::deque<ProcessedPoint>& window) const;

    Thresholds thresholds_;
    std::deque<ProcessedPoint> history_;
};

}  // namespace gg
