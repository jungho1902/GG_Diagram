#include "SimulationPipeline.hpp"

#include "math_utils.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <random>
#include <sstream>
#include <string>

namespace gg::gui {

namespace {
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
}

SimulationPipeline::SimulationPipeline()
    : profile_{{{2.0, 2.5, 0.0},
                {2.0, 0.0, 0.0},
                {3.0, 0.0, 0.45},
                {3.0, 0.2, -0.40},
                {2.5, -2.0, 0.0}}},
      rng_(42),
      processorConfig_(buildProcessorConfig(processorParams_)),
      processor_(makeAttitudeEstimator(processorParams_), processorConfig_),
      thresholds_{0.35, 5.0 * kDegToRad, 0.45, 1.6 * kDegToRad, 0.25},
      filter_(thresholds_) {
    envelope_.setAlpha(envelopeAlpha_);
    rebuildProcessor();
}

AttitudeEstimator SimulationPipeline::makeAttitudeEstimator(const ProcessorParams& params) const {
    AttitudeEstimator estimator(0.05);
    estimator.setGyroBias(params.gyroBias);
    return estimator;
}

AttitudeEstimator SimulationPipeline::makeAttitudeEstimator() const {
    return makeAttitudeEstimator(processorParams_);
}

GgProcessor::Config SimulationPipeline::buildProcessorConfig(const ProcessorParams& params) const {
    GgProcessor::Config cfg;
    cfg.accelBias = params.accelBias;
    cfg.gyroBias = params.gyroBias;
    cfg.leverArm = params.leverArm;
    cfg.velocityDamping = params.velocityDamping;
    return cfg;
}

void SimulationPipeline::start() {
    if (useDataset_ && dataset_.empty()) {
        return;
    }
    datasetFinished_ = false;
    running_ = true;
}

void SimulationPipeline::stop() {
    running_ = false;
}

void SimulationPipeline::toggle() {
    running_ = !running_;
}

void SimulationPipeline::reset() {
    stop();
    time_ = 0.0;
    segmentTime_ = 0.0;
    currentSegment_ = 0;
    yaw_ = 0.0;
    yawRate_ = 0.0;
    velocity_ = 0.0;
    timeAccumulator_ = 0.0;
    rng_.seed(42);

    rebuildProcessor();
    filter_ = QuasiSteadyStateFilter(thresholds_);
    envelope_.clear();
    envelope_.setAlpha(envelopeAlpha_);
    datasetAccumulator_ = 0.0;
    datasetIndex_ = 0;
    datasetFinished_ = false;
    if (!dataset_.empty()) {
        datasetStartTime_ = dataset_.front().timestamp;
    }

    clearProcessedData();
}

void SimulationPipeline::update(double deltaTime) {
    if (!running_) {
        return;
    }

    if (!std::isfinite(deltaTime) || deltaTime <= 0.0) {
        deltaTime = dt_;
    }

    bool envelopeDirty = false;

    if (useDataset_ && !dataset_.empty()) {
        datasetAccumulator_ += deltaTime * datasetPlaybackSpeed_;

        while (datasetIndex_ < dataset_.size()) {
            double sampleTime = dataset_[datasetIndex_].timestamp - datasetStartTime_;
            if (sampleTime > datasetAccumulator_ + 1e-9) {
                break;
            }
            processSample(dataset_[datasetIndex_], envelopeDirty);
            ++datasetIndex_;
        }

        if (datasetIndex_ >= dataset_.size()) {
            running_ = false;
            datasetFinished_ = true;
        }
    } else {
        timeAccumulator_ += deltaTime;

        while (timeAccumulator_ >= dt_) {
            timeAccumulator_ -= dt_;
            ImuData sample = generateSample();
            processSample(sample, envelopeDirty);
        }
    }

    if (envelopeDirty) {
        envelopeHull_ = envelope_.calculateEnvelope();
    }
}

void SimulationPipeline::setUseDataset(bool useDataset) {
    if (useDataset_ == useDataset) {
        return;
    }
    useDataset_ = useDataset;
    reset();
}

bool SimulationPipeline::loadDatasetFromCsv(const std::string& path, char delimiter) {
    std::ifstream file(path);
    if (!file.is_open()) {
        return false;
    }

    std::vector<ImuData> data;
    std::string line;
    data.reserve(1024);

    auto isNumericLine = [](const std::string& text) {
        for (char ch : text) {
            if (std::isdigit(static_cast<unsigned char>(ch)) || ch == '-' || ch == '+' || ch == '.' || ch == ',' || ch == ' ' || ch == '\t' || ch == 'e' || ch == 'E') {
                continue;
            }
            return false;
        }
        return true;
    };

    while (std::getline(file, line)) {
        if (line.empty()) {
            continue;
        }

        if (!isNumericLine(line)) {
            continue;  // skip headers or metadata lines
        }

        std::stringstream ss(line);
        std::string token;
        std::vector<double> values;
        while (std::getline(ss, token, delimiter)) {
            try {
                values.push_back(std::stod(token));
            } catch (...) {
                values.clear();
                break;
            }
        }

        if (values.size() < 7) {
            continue;
        }

        ImuData sample;
        sample.timestamp = values[0];
        sample.accel = Vec3{values[1], values[2], values[3]};
        sample.gyro = Vec3{values[4], values[5], values[6]};
        data.push_back(sample);
    }

    if (data.empty()) {
        return false;
    }

    std::sort(data.begin(), data.end(), [](const ImuData& a, const ImuData& b) {
        return a.timestamp < b.timestamp;
    });

    dataset_ = std::move(data);
    datasetStartTime_ = dataset_.front().timestamp;
    datasetIndex_ = 0;
    datasetAccumulator_ = 0.0;
    datasetFinished_ = false;

    clearProcessedData();

    rebuildProcessor();
    filter_ = QuasiSteadyStateFilter(thresholds_);
    envelope_.clear();

    return true;
}

void SimulationPipeline::setDatasetPlaybackSpeed(double speed) {
    datasetPlaybackSpeed_ = std::clamp(speed, 0.01, 10.0);
}

bool SimulationPipeline::computeAnalytics() {
    const std::vector<ImuData>& preferred = !dataset_.empty() ? dataset_ : rawHistory_;
    const std::vector<ImuData>& source = !preferred.empty() ? preferred : rawHistory_;
    analytics_ = AnalyticsResults{};

    if (source.size() < 10) {
        return false;
    }

    analytics_.quality = computeQualityMetrics(source, analyticsParams_.accelSaturation, analyticsParams_.gyroSaturation);
    analytics_.hasQuality = true;

    if (processedHistory_.size() >= 10) {
        analytics_.timeSync = estimateTimeOffset(processedHistory_, analyticsParams_.maxLagSamples);
        analytics_.hasTimeSync = analytics_.timeSync.valid;
    }

    std::size_t allanSteps = std::max<std::size_t>(1, analyticsParams_.allanSteps);
    analytics_.accelAllan = computeAllanDeviation(source, true, analyticsParams_.allanMaxTau, allanSteps);
    analytics_.gyroAllan = computeAllanDeviation(source, false, analyticsParams_.allanMaxTau, allanSteps);
    analytics_.hasAllan = !analytics_.accelAllan.tau.empty() || !analytics_.gyroAllan.tau.empty();

    return analytics_.hasQuality || analytics_.hasTimeSync || analytics_.hasAllan;
}

SimulationStats SimulationPipeline::stats() const {
    SimulationStats stats;
    if (lastProcessed_) {
        stats.gx = lastProcessed_->gx;
        stats.gy = lastProcessed_->gy;
    }
    stats.totalPoints = processedPoints_.size();
    stats.steadyPoints = steadyStatePoints_.size();
    return stats;
}

void SimulationPipeline::setThresholds(const QuasiSteadyStateFilter::Thresholds& thresholds) {
    thresholds_ = thresholds;
    rebuildFilter();
}

void SimulationPipeline::rebuildFilter() {
    filter_ = QuasiSteadyStateFilter(thresholds_);
    envelope_.setAlpha(envelopeAlpha_);
    envelope_.clear();
    steadyStatePoints_.clear();
    envelopeHull_.clear();

    bool envelopeDirty = false;
    for (const auto& processed : processedHistory_) {
        if (filter_.update(processed)) {
            steadyStatePoints_.push_back(Point2D{processed.gx, processed.gy});
            envelope_.addPoint(processed.gx, processed.gy);
            envelopeDirty = true;
        }
    }

    if (envelopeDirty) {
        envelopeHull_ = envelope_.calculateEnvelope();
    }
}

ImuData SimulationPipeline::generateSample() {
    if (profile_.empty()) {
        return ImuData{};
    }

    const auto& segment = profile_[currentSegment_];

    double yawRateError = segment.targetYawRate - yawRate_;
    double yawAcceleration = std::clamp(yawRateError * 2.5, -1.0, 1.0);

    yawRate_ += yawAcceleration * dt_;
    yaw_ += yawRate_ * dt_;

    double ax = segment.longitudinalAccel;
    velocity_ = std::max(0.0, velocity_ + ax * dt_);
    double ay = velocity_ * yawRate_;

    Vec3 accelNav{ax, ay, 0.0};
    Quaternion attitude = Quaternion::fromEuler(0.0, 0.0, yaw_);

    Vec3 accelBody = attitude.rotateInverse(accelNav);
    Vec3 omega{0.0, 0.0, yawRate_};
    Vec3 alpha{0.0, 0.0, yawAcceleration};

    Vec3 gravityBody = attitude.rotateInverse(processorParams_.gravity);

    Vec3 sensorAccelBody = accelBody + cross(alpha, processorParams_.leverArm) + cross(omega, cross(omega, processorParams_.leverArm));
    Vec3 specificForce = sensorAccelBody - gravityBody;

    Vec3 noisyAccel{
        specificForce.x + processorParams_.accelBias.x + accelNoise_(rng_),
        specificForce.y + processorParams_.accelBias.y + accelNoise_(rng_),
        specificForce.z + processorParams_.accelBias.z + accelNoise_(rng_)
    };

    Vec3 noisyGyro{
        omega.x + processorParams_.gyroBias.x + gyroNoise_(rng_),
        omega.y + processorParams_.gyroBias.y + gyroNoise_(rng_),
        omega.z + processorParams_.gyroBias.z + gyroNoise_(rng_)
    };

    ImuData sample{time_, noisyAccel, noisyGyro};

    time_ += dt_;
    segmentTime_ += dt_;
    if (segmentTime_ >= segment.duration) {
        segmentTime_ = 0.0;
        currentSegment_ = (currentSegment_ + 1) % profile_.size();
    }

    return sample;
}

void SimulationPipeline::processSample(const ImuData& sample, bool& envelopeDirty) {
    rawHistory_.push_back(sample);
    ProcessedPoint processed = processor_.process(sample);
    processedHistory_.push_back(processed);
    processedPoints_.push_back(Point2D{processed.gx, processed.gy});
    lastProcessed_ = processed;

    if (filter_.update(processed)) {
        steadyStatePoints_.push_back(Point2D{processed.gx, processed.gy});
        envelope_.addPoint(processed.gx, processed.gy);
        envelopeDirty = true;
    }
}

void SimulationPipeline::clearProcessedData() {
    processedHistory_.clear();
    processedPoints_.clear();
    steadyStatePoints_.clear();
    envelopeHull_.clear();
    lastProcessed_.reset();
    rawHistory_.clear();
    analytics_ = AnalyticsResults{};
    monteCarloSamples_.clear();
    monteCarloEnvelope_.clear();
}

void SimulationPipeline::setEnvelopeAlpha(double alpha) {
    double sanitized = (std::isfinite(alpha) && alpha >= 0.0) ? alpha : 0.0;
    if (std::abs(sanitized - envelopeAlpha_) <= 1e-9) {
        return;
    }

    envelopeAlpha_ = sanitized;
    envelope_.setAlpha(envelopeAlpha_);
    envelopeHull_ = envelope_.calculateEnvelope();

    if (!monteCarloSamples_.empty()) {
        GgEnvelope aggregate(envelopeAlpha_);
        for (const auto& sample : monteCarloSamples_) {
            aggregate.addPoint(sample);
        }
        monteCarloEnvelope_ = aggregate.calculateEnvelope();
    }
}

void SimulationPipeline::rebuildProcessor() {
    processorConfig_ = buildProcessorConfig(processorParams_);
    processor_ = GgProcessor(makeAttitudeEstimator(), processorConfig_);
}

void SimulationPipeline::replayHistory() {
    if (rawHistory_.empty()) {
        processedHistory_.clear();
        processedPoints_.clear();
        steadyStatePoints_.clear();
        envelope_.clear();
        envelope_.setAlpha(envelopeAlpha_);
        envelopeHull_.clear();
        lastProcessed_.reset();
        filter_ = QuasiSteadyStateFilter(thresholds_);
        monteCarloSamples_.clear();
        monteCarloEnvelope_.clear();
        return;
    }

    filter_ = QuasiSteadyStateFilter(thresholds_);
    rebuildProcessor();

    processedHistory_.clear();
    processedPoints_.clear();
    steadyStatePoints_.clear();
    envelope_.clear();
    envelope_.setAlpha(envelopeAlpha_);
    envelopeHull_.clear();
    lastProcessed_.reset();
    monteCarloSamples_.clear();
    monteCarloEnvelope_.clear();

    bool envelopeDirty = false;
    for (const auto& sample : rawHistory_) {
        ProcessedPoint processed = processor_.process(sample);
        processedHistory_.push_back(processed);
        processedPoints_.push_back(Point2D{processed.gx, processed.gy});
        lastProcessed_ = processed;
        if (filter_.update(processed)) {
            steadyStatePoints_.push_back(Point2D{processed.gx, processed.gy});
            envelope_.addPoint(processed.gx, processed.gy);
            envelopeDirty = true;
        }
    }

    if (envelopeDirty) {
        envelopeHull_ = envelope_.calculateEnvelope();
    }
}

void SimulationPipeline::setProcessorParams(const ProcessorParams& params) {
    processorParams_ = params;
    rebuildProcessor();
    replayHistory();
}

void SimulationPipeline::setAnalyticsParams(const AnalyticsParams& params) {
    analyticsParams_ = params;
}

void SimulationPipeline::setMonteCarloParams(const MonteCarloParams& params) {
    monteCarloParams_ = params;
}

bool SimulationPipeline::runMonteCarlo() {
    monteCarloSamples_.clear();
    monteCarloEnvelope_.clear();

    if (monteCarloParams_.samples == 0 || rawHistory_.size() < 2) {
        return false;
    }

    std::default_random_engine rng(monteCarloParams_.seed ? monteCarloParams_.seed : std::random_device{}());
    std::normal_distribution<double> accelBiasNoise(0.0, monteCarloParams_.accelBiasSigma);
    std::normal_distribution<double> gyroBiasNoise(0.0, monteCarloParams_.gyroBiasSigma);
    std::normal_distribution<double> leverNoise(0.0, monteCarloParams_.leverArmSigma);

    GgEnvelope aggregateEnvelope(envelopeAlpha_);

    for (std::size_t i = 0; i < monteCarloParams_.samples; ++i) {
        ProcessorParams perturbed = processorParams_;
        if (monteCarloParams_.accelBiasSigma > 0.0) {
            perturbed.accelBias.x += accelBiasNoise(rng);
            perturbed.accelBias.y += accelBiasNoise(rng);
            perturbed.accelBias.z += accelBiasNoise(rng);
        }
        if (monteCarloParams_.gyroBiasSigma > 0.0) {
            perturbed.gyroBias.x += gyroBiasNoise(rng);
            perturbed.gyroBias.y += gyroBiasNoise(rng);
            perturbed.gyroBias.z += gyroBiasNoise(rng);
        }
        if (monteCarloParams_.leverArmSigma > 0.0) {
            perturbed.leverArm.x += leverNoise(rng);
            perturbed.leverArm.y += leverNoise(rng);
            perturbed.leverArm.z += leverNoise(rng);
        }

        GgProcessor::Config cfg = buildProcessorConfig(perturbed);
        AttitudeEstimator estimator = makeAttitudeEstimator(perturbed);
        GgProcessor tempProcessor(estimator, cfg);
        QuasiSteadyStateFilter tempFilter(thresholds_);

        for (const auto& sample : rawHistory_) {
            ProcessedPoint processed = tempProcessor.process(sample);
            if (tempFilter.update(processed)) {
                Point2D point{processed.gx, processed.gy};
                monteCarloSamples_.push_back(point);
                aggregateEnvelope.addPoint(point);
            }
        }
    }

    monteCarloEnvelope_ = aggregateEnvelope.calculateEnvelope();
    return !monteCarloEnvelope_.empty();
}

}  // namespace gg::gui
