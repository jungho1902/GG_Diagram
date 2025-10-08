#include "SimulationPipeline.hpp"

#include "math_utils.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <random>
#include <sstream>
#include <string>
#include <utility>

namespace gg::gui {

namespace {
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

std::string trim(const std::string& text) {
    std::size_t start = 0;
    while (start < text.size() && std::isspace(static_cast<unsigned char>(text[start]))) {
        ++start;
    }
    std::size_t end = text.size();
    while (end > start && std::isspace(static_cast<unsigned char>(text[end - 1]))) {
        --end;
    }
    return text.substr(start, end - start);
}

std::vector<std::string> splitCsv(const std::string& line) {
    std::vector<std::string> tokens;
    std::string token;
    std::stringstream ss(line);
    while (std::getline(ss, token, ',')) {
        tokens.push_back(token);
    }
    return tokens;
}

double extractNumber(const std::string& source, const std::string& key, double defaultValue) {
    const std::string token = '"' + key + '"';
    std::size_t pos = source.find(token);
    if (pos == std::string::npos) {
        return defaultValue;
    }
    pos = source.find(':', pos);
    if (pos == std::string::npos) {
        return defaultValue;
    }
    pos = source.find_first_of("-0123456789", pos + 1);
    if (pos == std::string::npos) {
        return defaultValue;
    }
    std::size_t end = pos;
    while (end < source.size()) {
        char ch = source[end];
        if (!(std::isdigit(static_cast<unsigned char>(ch)) || ch == '-' || ch == '+' || ch == '.' || ch == 'e' || ch == 'E')) {
            break;
        }
        ++end;
    }
    try {
        return std::stod(source.substr(pos, end - pos));
    } catch (...) {
        return defaultValue;
    }
}

bool extractBool(const std::string& source, const std::string& key, bool defaultValue) {
    const std::string token = '"' + key + '"';
    std::size_t pos = source.find(token);
    if (pos == std::string::npos) {
        return defaultValue;
    }
    pos = source.find(':', pos);
    if (pos == std::string::npos) {
        return defaultValue;
    }
    pos = source.find_first_not_of(" \t\n\r", pos + 1);
    if (pos == std::string::npos) {
        return defaultValue;
    }
    if (source.compare(pos, 4, "true") == 0) {
        return true;
    }
    if (source.compare(pos, 5, "false") == 0) {
        return false;
    }
    return defaultValue;
}

std::size_t extractSize(const std::string& source, const std::string& key, std::size_t defaultValue) {
    double value = extractNumber(source, key, static_cast<double>(defaultValue));
    if (value < 0.0) {
        return defaultValue;
    }
    return static_cast<std::size_t>(value + 0.5);
}

std::string extractObject(const std::string& source, const std::string& key) {
    const std::string token = '"' + key + '"';
    std::size_t pos = source.find(token);
    if (pos == std::string::npos) {
        return {};
    }
    pos = source.find('{', pos);
    if (pos == std::string::npos) {
        return {};
    }
    int depth = 0;
    std::size_t start = pos;
    for (std::size_t i = pos; i < source.size(); ++i) {
        char ch = source[i];
        if (ch == '{') {
            ++depth;
        } else if (ch == '}') {
            --depth;
            if (depth == 0) {
                return source.substr(start, i - start + 1);
            }
        }
    }
    return {};
}

std::string extractArray(const std::string& source, const std::string& key) {
    const std::string token = '"' + key + '"';
    std::size_t pos = source.find(token);
    if (pos == std::string::npos) {
        return {};
    }
    pos = source.find('[', pos);
    if (pos == std::string::npos) {
        return {};
    }
    int depth = 0;
    std::size_t start = pos;
    for (std::size_t i = pos; i < source.size(); ++i) {
        char ch = source[i];
        if (ch == '[') {
            ++depth;
        } else if (ch == ']') {
            --depth;
            if (depth == 0) {
                return source.substr(start, i - start + 1);
            }
        }
    }
    return {};
}

std::vector<double> parseDoubleArray(const std::string& arrayText) {
    std::vector<double> values;
    if (arrayText.empty()) {
        return values;
    }
    std::size_t start = arrayText.find('[');
    std::size_t end = arrayText.find(']', start + 1);
    if (start == std::string::npos || end == std::string::npos || end <= start + 1) {
        return values;
    }
    std::string body = arrayText.substr(start + 1, end - start - 1);
    std::stringstream ss(body);
    std::string token;
    while (std::getline(ss, token, ',')) {
        std::string trimmed = trim(token);
        if (trimmed.empty()) {
            continue;
        }
        try {
            values.push_back(std::stod(trimmed));
        } catch (...) {
            // ignore conversion failures
        }
    }
    return values;
}

std::string readFileToString(const std::filesystem::path& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        return {};
    }
    std::ostringstream ss;
    ss << file.rdbuf();
    return ss.str();
}

std::vector<Point2D> loadPointsCsv(const std::filesystem::path& path) {
    std::vector<Point2D> points;
    std::ifstream file(path);
    if (!file.is_open()) {
        return points;
    }

    std::string header;
    if (!std::getline(file, header)) {
        return points;
    }

    auto headers = splitCsv(header);
    int gxIndex = -1;
    int gyIndex = -1;
    for (std::size_t i = 0; i < headers.size(); ++i) {
        std::string h = trim(headers[i]);
        if (h == "gx" || h == "gx_g") {
            gxIndex = static_cast<int>(i);
        } else if (h == "gy" || h == "gy_g") {
            gyIndex = static_cast<int>(i);
        }
    }
    if (gxIndex < 0 || gyIndex < 0) {
        return points;
    }

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) {
            continue;
        }
        auto tokens = splitCsv(line);
        if (tokens.size() <= static_cast<std::size_t>(std::max(gxIndex, gyIndex))) {
            continue;
        }
        try {
            double gx = std::stod(trim(tokens[gxIndex]));
            double gy = std::stod(trim(tokens[gyIndex]));
            points.push_back(Point2D{gx, gy});
        } catch (...) {
            // skip malformed rows
        }
    }
    return points;
}

std::vector<SimulationPipeline::TrajectorySample> loadTrajectoryCsv(const std::filesystem::path& path) {
    std::vector<SimulationPipeline::TrajectorySample> samples;
    std::ifstream file(path);
    if (!file.is_open()) {
        return samples;
    }

    std::string header;
    if (!std::getline(file, header)) {
        return samples;
    }

    auto headers = splitCsv(header);
    int idxTime = -1;
    int idxPx = -1;
    int idxPy = -1;
    int idxPz = -1;
    int idxVx = -1;
    int idxVy = -1;
    int idxVz = -1;

    for (std::size_t i = 0; i < headers.size(); ++i) {
        std::string h = trim(headers[i]);
        if (h == "timestamp") {
            idxTime = static_cast<int>(i);
        } else if (h == "pos_x") {
            idxPx = static_cast<int>(i);
        } else if (h == "pos_y") {
            idxPy = static_cast<int>(i);
        } else if (h == "pos_z") {
            idxPz = static_cast<int>(i);
        } else if (h == "vel_x") {
            idxVx = static_cast<int>(i);
        } else if (h == "vel_y") {
            idxVy = static_cast<int>(i);
        } else if (h == "vel_z") {
            idxVz = static_cast<int>(i);
        }
    }

    if (idxTime < 0 || idxPx < 0 || idxPy < 0 || idxPz < 0 || idxVx < 0 || idxVy < 0 || idxVz < 0) {
        return samples;
    }

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) {
            continue;
        }
        auto tokens = splitCsv(line);
        if (tokens.size() <= static_cast<std::size_t>(std::max({idxTime, idxPx, idxPy, idxPz, idxVx, idxVy, idxVz}))) {
            continue;
        }
        try {
            SimulationPipeline::TrajectorySample sample;
            sample.timestamp = std::stod(trim(tokens[idxTime]));
            sample.position = Vec3{std::stod(trim(tokens[idxPx])),
                                   std::stod(trim(tokens[idxPy])),
                                   std::stod(trim(tokens[idxPz]))};
            sample.velocity = Vec3{std::stod(trim(tokens[idxVx])),
                                   std::stod(trim(tokens[idxVy])),
                                   std::stod(trim(tokens[idxVz]))};
            samples.push_back(sample);
        } catch (...) {
            // ignore malformed rows
        }
    }

    return samples;
}

}  // namespace

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

bool SimulationPipeline::loadProcessedResults(const std::string& directory) {
    namespace fs = std::filesystem;

    fs::path input(directory);
    fs::path baseDir;
    fs::path resultsPath;

    if (fs::is_directory(input)) {
        baseDir = input;
        resultsPath = baseDir / "results.json";
    } else {
        resultsPath = input;
        baseDir = resultsPath.parent_path();
    }

    if (resultsPath.empty() || !fs::exists(resultsPath)) {
        return false;
    }

    std::vector<Point2D> steadyPoints = loadPointsCsv(baseDir / "steady_points.csv");
    std::vector<Point2D> envelopePoints = loadPointsCsv(baseDir / "envelope.csv");
    std::vector<Point2D> processedPoints = loadPointsCsv(baseDir / "processed_points.csv");

    if (steadyPoints.empty() && processedPoints.empty()) {
        return false;
    }
    if (processedPoints.empty()) {
        processedPoints = steadyPoints;
    }
    if (steadyPoints.empty()) {
        steadyPoints = processedPoints;
    }

    std::string jsonText = readFileToString(resultsPath);
    if (jsonText.empty()) {
        return false;
    }

    stop();
    useDataset_ = false;
    dataset_.clear();
    datasetIndex_ = 0;
    datasetAccumulator_ = 0.0;
    datasetFinished_ = false;

    clearProcessedData();

    processedPoints_ = std::move(processedPoints);
    steadyStatePoints_ = std::move(steadyPoints);

    double alphaFromResults = extractNumber(jsonText, "alpha", envelopeAlpha_);
    envelopeAlpha_ = std::max(0.0, alphaFromResults);
    envelope_.setAlpha(envelopeAlpha_);
    envelope_.clear();
    for (const auto& pt : steadyStatePoints_) {
        envelope_.addPoint(pt.x, pt.y);
    }
    if (!envelopePoints.empty()) {
        envelopeHull_ = std::move(envelopePoints);
    } else {
        envelopeHull_ = envelope_.calculateEnvelope();
    }

    if (!processedPoints_.empty()) {
        ProcessedPoint last{};
        last.gx = processedPoints_.back().x;
        last.gy = processedPoints_.back().y;
        lastProcessed_ = last;
    }

    loadedMetrics_ = LoadedMetrics{};
    loadedMetrics_.hasData = true;
    loadedMetrics_.steadyCount = extractSize(jsonText, "steady_point_count", 0);
    loadedMetrics_.maxGx = extractNumber(jsonText, "max_gx", 0.0);
    loadedMetrics_.maxGy = extractNumber(jsonText, "max_gy", 0.0);
    loadedMetrics_.envelopeArea = extractNumber(jsonText, "envelope_area", 0.0);
    loadedMetrics_.alpha = envelopeAlpha_;

    // Validation metrics
    const std::string validationJson = extractObject(jsonText, "validation");
    if (!validationJson.empty()) {
        loadedMetrics_.rSquared = extractNumber(validationJson, "r_squared", 0.0);
        loadedMetrics_.frictionMu = extractNumber(validationJson, "friction_mu", 0.0);
        loadedMetrics_.frictionViolation = extractNumber(validationJson, "friction_violation_ratio", 0.0);
        loadedMetrics_.frictionMax = extractNumber(validationJson, "friction_max_magnitude", 0.0);
        double haus = extractNumber(validationJson, "hausdorff_distance", std::numeric_limits<double>::quiet_NaN());
        if (std::isfinite(haus)) {
            loadedMetrics_.hausdorff = haus;
        }
    }

    // Quality & analytics
    analytics_ = AnalyticsResults{};
    const std::string qualityJson = extractObject(jsonText, "quality");
    if (!qualityJson.empty()) {
        analytics_.quality.sampleCount = extractSize(qualityJson, "sample_count", 0);
        analytics_.quality.meanSamplingInterval = extractNumber(qualityJson, "mean_dt", 0.0);
        analytics_.quality.nominalFrequency = extractNumber(qualityJson, "nominal_frequency", 0.0);
        analytics_.quality.samplingJitter = extractNumber(qualityJson, "jitter", 0.0);
        analytics_.quality.missingDataRate = extractNumber(qualityJson, "missing_rate", 0.0);
        analytics_.quality.saturationAccelRate = extractNumber(qualityJson, "accel_saturation_rate", 0.0);
        analytics_.quality.saturationGyroRate = extractNumber(qualityJson, "gyro_saturation_rate", 0.0);
        analytics_.hasQuality = true;
    }

    const std::string timeSyncJson = extractObject(jsonText, "time_sync");
    if (!timeSyncJson.empty()) {
        analytics_.timeSync.valid = extractBool(timeSyncJson, "valid", false);
        analytics_.timeSync.offset = extractNumber(timeSyncJson, "offset", 0.0);
        analytics_.timeSync.peakCorrelation = extractNumber(timeSyncJson, "peak_correlation", 0.0);
        analytics_.hasTimeSync = analytics_.timeSync.valid;
    }

    const std::string allanAccelJson = extractObject(jsonText, "allan_accel");
    if (!allanAccelJson.empty()) {
        analytics_.accelAllan.tau = parseDoubleArray(extractArray(allanAccelJson, "tau"));
        analytics_.accelAllan.sigmaX = parseDoubleArray(extractArray(allanAccelJson, "sigma_x"));
        analytics_.accelAllan.sigmaY = parseDoubleArray(extractArray(allanAccelJson, "sigma_y"));
        analytics_.accelAllan.sigmaZ = parseDoubleArray(extractArray(allanAccelJson, "sigma_z"));
        analytics_.accelAllan.sampleInterval = extractNumber(allanAccelJson, "sample_interval", 0.0);
    }

    const std::string allanGyroJson = extractObject(jsonText, "allan_gyro");
    if (!allanGyroJson.empty()) {
        analytics_.gyroAllan.tau = parseDoubleArray(extractArray(allanGyroJson, "tau"));
        analytics_.gyroAllan.sigmaX = parseDoubleArray(extractArray(allanGyroJson, "sigma_x"));
        analytics_.gyroAllan.sigmaY = parseDoubleArray(extractArray(allanGyroJson, "sigma_y"));
        analytics_.gyroAllan.sigmaZ = parseDoubleArray(extractArray(allanGyroJson, "sigma_z"));
        analytics_.gyroAllan.sampleInterval = extractNumber(allanGyroJson, "sample_interval", 0.0);
    }

    analytics_.hasAllan = !analytics_.accelAllan.tau.empty() || !analytics_.gyroAllan.tau.empty();

    const std::string mcJson = extractObject(jsonText, "monte_carlo");
    if (!mcJson.empty()) {
        loadedMetrics_.monteCarloEnabled = extractBool(mcJson, "enabled", false);
        loadedMetrics_.monteCarloSamples = extractSize(mcJson, "samples", 0);
        loadedMetrics_.mcMeanGx = extractNumber(mcJson, "mean_gx", 0.0);
        loadedMetrics_.mcMeanGy = extractNumber(mcJson, "mean_gy", 0.0);
        loadedMetrics_.mcMeanArea = extractNumber(mcJson, "mean_area", 0.0);

        auto gxCI = parseDoubleArray(extractArray(mcJson, "gx_ci_95"));
        if (gxCI.size() == 2) {
            loadedMetrics_.mcGxCI = {gxCI[0], gxCI[1]};
        }
        auto gyCI = parseDoubleArray(extractArray(mcJson, "gy_ci_95"));
        if (gyCI.size() == 2) {
            loadedMetrics_.mcGyCI = {gyCI[0], gyCI[1]};
        }
        auto areaCI = parseDoubleArray(extractArray(mcJson, "area_ci_95"));
        if (areaCI.size() == 2) {
            loadedMetrics_.mcAreaCI = {areaCI[0], areaCI[1]};
        }
    } else {
        loadedMetrics_.monteCarloEnabled = false;
    }

    loadedTrajectory_ = loadTrajectoryCsv(baseDir / "trajectory.csv");

    return true;
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
    loadedMetrics_ = LoadedMetrics{};
    loadedTrajectory_.clear();
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
