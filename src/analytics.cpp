#include "analytics.hpp"

#include "math_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace gg {

namespace {

template <typename T>
double mean(const std::vector<T>& values) {
    if (values.empty()) {
        return 0.0;
    }
    double sum = std::accumulate(values.begin(), values.end(), 0.0);
    return sum / static_cast<double>(values.size());
}

template <typename T>
double standardDeviation(const std::vector<T>& values, double avg) {
    if (values.size() < 2) {
        return 0.0;
    }
    double accum = 0.0;
    for (const auto& v : values) {
        double diff = static_cast<double>(v) - avg;
        accum += diff * diff;
    }
    return std::sqrt(accum / static_cast<double>(values.size() - 1));
}

}  // namespace

QualityMetrics computeQualityMetrics(const std::vector<ImuData>& samples,
                                     double saturationAccelThreshold,
                                     double saturationGyroThreshold) {
    QualityMetrics metrics;
    if (samples.size() < 2) {
        return metrics;
    }

    std::vector<double> intervals;
    intervals.reserve(samples.size() - 1);
    std::size_t gaps = 0;
    std::size_t accelSaturated = 0;
    std::size_t gyroSaturated = 0;

    for (std::size_t i = 1; i < samples.size(); ++i) {
        double dt = samples[i].timestamp - samples[i - 1].timestamp;
        if (dt <= 0.0) {
            continue;
        }
        intervals.push_back(dt);
    }

    if (intervals.empty()) {
        return metrics;
    }

    double avgDt = mean(intervals);
    double jitter = standardDeviation(intervals, avgDt);
    double nominalFreq = avgDt > 0.0 ? 1.0 / avgDt : 0.0;

    for (double dt : intervals) {
        if (dt > 1.5 * avgDt) {
            ++gaps;
        }
    }

    for (const auto& sample : samples) {
        double accelMax = std::max({std::fabs(sample.accel.x), std::fabs(sample.accel.y), std::fabs(sample.accel.z)});
        double gyroMax = std::max({std::fabs(sample.gyro.x), std::fabs(sample.gyro.y), std::fabs(sample.gyro.z)});
        if (accelMax >= saturationAccelThreshold) {
            ++accelSaturated;
        }
        if (gyroMax >= saturationGyroThreshold) {
            ++gyroSaturated;
        }
    }

    metrics.meanSamplingInterval = avgDt;
    metrics.samplingJitter = jitter;
    metrics.nominalFrequency = nominalFreq;
    metrics.sampleCount = samples.size();
    metrics.missingDataRate = static_cast<double>(gaps) / static_cast<double>(intervals.size());
    metrics.saturationAccelRate = static_cast<double>(accelSaturated) / static_cast<double>(samples.size());
    metrics.saturationGyroRate = static_cast<double>(gyroSaturated) / static_cast<double>(samples.size());
    return metrics;
}

TimeSyncResult estimateTimeOffset(const std::vector<ProcessedPoint>& processed,
                                  std::size_t maxLagSamples) {
    TimeSyncResult result;
    if (processed.size() < 10) {
        return result;
    }

    std::vector<double> signalA;
    std::vector<double> signalB;
    signalA.reserve(processed.size());
    signalB.reserve(processed.size());

    for (const auto& pt : processed) {
        signalA.push_back(pt.accelNav.y);
        signalB.push_back(pt.longitudinalVelocity * pt.yawRate);
    }

    double meanA = mean(signalA);
    double meanB = mean(signalB);

    std::vector<double> centeredA(signalA.size());
    std::vector<double> centeredB(signalB.size());
    for (std::size_t i = 0; i < signalA.size(); ++i) {
        centeredA[i] = signalA[i] - meanA;
        centeredB[i] = signalB[i] - meanB;
    }

    double denomA = std::sqrt(std::inner_product(centeredA.begin(), centeredA.end(), centeredA.begin(), 0.0));
    double denomB = std::sqrt(std::inner_product(centeredB.begin(), centeredB.end(), centeredB.begin(), 0.0));
    if (denomA == 0.0 || denomB == 0.0) {
        return result;
    }

    std::size_t maxLag = std::min(maxLagSamples, processed.size() - 1);
    double bestCorr = -1.0;
    int bestLag = 0;

    for (int lag = -static_cast<int>(maxLag); lag <= static_cast<int>(maxLag); ++lag) {
        double sum = 0.0;
        std::size_t count = 0;
        for (std::size_t i = 0; i < centeredA.size(); ++i) {
            int j = static_cast<int>(i) + lag;
            if (j < 0 || j >= static_cast<int>(centeredB.size())) {
                continue;
            }
            sum += centeredA[i] * centeredB[j];
            ++count;
        }
        if (count < 5) {
            continue;
        }
        double corr = sum / (denomA * denomB);
        if (corr > bestCorr) {
            bestCorr = corr;
            bestLag = lag;
        }
    }

    if (bestCorr > 0.0) {
        result.valid = true;
        result.peakCorrelation = bestCorr;
        double dt = 0.0;
        if (processed.size() >= 2) {
            dt = processed[1].timestamp - processed[0].timestamp;
        }
        if (dt <= 0.0) {
            double tSpan = processed.back().timestamp - processed.front().timestamp;
            dt = tSpan / static_cast<double>(processed.size());
        }
        result.offset = static_cast<double>(bestLag) * dt;
    }

    return result;
}

AllanDeviationSeries computeAllanDeviation(const std::vector<ImuData>& samples,
                                          bool accelerometer,
                                          double maxTauSeconds,
                                          std::size_t tauSteps) {
    AllanDeviationSeries result;
    if (samples.size() < 100 || tauSteps == 0) {
        return result;
    }

    std::vector<double> intervals;
    intervals.reserve(samples.size() - 1);
    for (std::size_t i = 1; i < samples.size(); ++i) {
        double dt = samples[i].timestamp - samples[i - 1].timestamp;
        if (dt > 0.0) {
            intervals.push_back(dt);
        }
    }
    if (intervals.empty()) {
        return result;
    }

    double sampleInterval = mean(intervals);
    result.sampleInterval = sampleInterval;

    std::vector<int> mValues;
    mValues.reserve(tauSteps);
    double maxM = maxTauSeconds > 0.0 ? std::max(1.0, maxTauSeconds / sampleInterval) : samples.size() / 10.0;
    double logMin = std::log10(1.0);
    double logMax = std::log10(maxM);
    double step = (logMax - logMin) / static_cast<double>(tauSteps);
    if (!std::isfinite(step) || step <= 0.0) {
        step = 0.3;
    }
    for (std::size_t i = 0; i < tauSteps; ++i) {
        int m = static_cast<int>(std::round(std::pow(10.0, logMin + step * i)));
        if (m < 1) {
            m = 1;
        }
        if (!mValues.empty() && m == mValues.back()) {
            continue;
        }
        mValues.push_back(m);
    }

    auto extractAxisData = [&](int axis) {
        std::vector<double> data;
        data.reserve(samples.size());
        for (const auto& s : samples) {
            if (accelerometer) {
                if (axis == 0) data.push_back(s.accel.x);
                else if (axis == 1) data.push_back(s.accel.y);
                else data.push_back(s.accel.z);
            } else {
                if (axis == 0) data.push_back(s.gyro.x);
                else if (axis == 1) data.push_back(s.gyro.y);
                else data.push_back(s.gyro.z);
            }
        }
        return data;
    };

    std::vector<double> axisData[3] = {
        extractAxisData(0),
        extractAxisData(1),
        extractAxisData(2)
    };

    auto computeSigma = [&](const std::vector<double>& data, int m) -> double {
        int n = static_cast<int>(data.size());
        if (n < 2 * m) {
            return std::numeric_limits<double>::quiet_NaN();
        }
        int k = n / m;
        if (k < 2) {
            return std::numeric_limits<double>::quiet_NaN();
        }
        std::vector<double> avg(k, 0.0);
        for (int i = 0; i < k; ++i) {
            double sum = 0.0;
            for (int j = 0; j < m; ++j) {
                sum += data[i * m + j];
            }
            avg[i] = sum / static_cast<double>(m);
        }
        double accum = 0.0;
        for (int i = 0; i < k - 1; ++i) {
            double diff = avg[i + 1] - avg[i];
            accum += diff * diff;
        }
        double variance = accum / (2.0 * static_cast<double>(k - 1));
        return std::sqrt(std::max(variance, 0.0));
    };

    for (int m : mValues) {
        double tau = static_cast<double>(m) * sampleInterval;
        double sigmaX = computeSigma(axisData[0], m);
        double sigmaY = computeSigma(axisData[1], m);
        double sigmaZ = computeSigma(axisData[2], m);
        if (std::isnan(sigmaX) && std::isnan(sigmaY) && std::isnan(sigmaZ)) {
            continue;
        }
        result.tau.push_back(tau);
        result.sigmaX.push_back(sigmaX);
        result.sigmaY.push_back(sigmaY);
        result.sigmaZ.push_back(sigmaZ);
    }

    return result;
}

}  // namespace gg
