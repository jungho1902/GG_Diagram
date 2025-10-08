#pragma once

#include "data_types.hpp"

#include <array>
#include <vector>

namespace gg {

struct QualityMetrics {
    double meanSamplingInterval{0.0};
    double samplingJitter{0.0};
    double saturationAccelRate{0.0};
    double saturationGyroRate{0.0};
    double missingDataRate{0.0};
    double nominalFrequency{0.0};
    std::size_t sampleCount{0};
};

struct TimeSyncResult {
    double offset{0.0};
    double peakCorrelation{0.0};
    bool valid{false};
};

struct AllanDeviationSeries {
    std::vector<double> tau;            // averaging time [s]
    std::vector<double> sigmaX;         // X-axis Allan deviation
    std::vector<double> sigmaY;
    std::vector<double> sigmaZ;
    double sampleInterval{0.0};
};

struct AnalyticsResults {
    QualityMetrics quality;
    bool hasQuality{false};

    TimeSyncResult timeSync;
    bool hasTimeSync{false};

    AllanDeviationSeries accelAllan;
    AllanDeviationSeries gyroAllan;
    bool hasAllan{false};
};

QualityMetrics computeQualityMetrics(const std::vector<ImuData>& samples,
                                     double saturationAccelThreshold = 50.0,
                                     double saturationGyroThreshold = 20.0);

TimeSyncResult estimateTimeOffset(const std::vector<ProcessedPoint>& processed,
                                  std::size_t maxLagSamples = 500);

AllanDeviationSeries computeAllanDeviation(const std::vector<ImuData>& samples,
                                          bool accelerometer,
                                          double maxTauSeconds = 10.0,
                                          std::size_t tauSteps = 10);

}  // namespace gg
