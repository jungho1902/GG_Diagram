#include "analytics.hpp"
#include "gg_envelope.hpp"
#include "gg_processor.hpp"
#include "quasi_steady_state_filter.hpp"
#include "vision/dataset.hpp"
#include "vision/vio_stub.hpp"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <optional>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <limits>

namespace {

struct Options {
    std::string inputPath;
    std::string outputJson{"results.json"};
    std::string envelopeCsv{"envelope.csv"};
    std::string steadyCsv{"steady_points.csv"};
    std::string processedCsv{"processed_points.csv"};
    std::string cameraMeta;
    std::string cameraRoot;
    std::string vioTrajectoryPath{"trajectory.csv"};
    std::string compareEnvelopePath;
    double alpha{0.0};
    gg::Vec3 accelBias{0.0, 0.0, 0.0};
    gg::Vec3 gyroBias{0.0, 0.0, 0.0};
    gg::Vec3 leverArm{0.0, 0.0, 0.0};
    double velocityDamping{0.0};
    double frictionMu{0.0};
    bool writeProcessed{false};
    bool useVio{false};

    std::size_t monteCarloSamples{0};
    unsigned int monteCarloSeed{42};
    double mcAccelSigma{0.0};
    double mcGyroSigma{0.0};
    double mcLeverSigma{0.0};
};

struct CsvColumnMap {
    int timestamp{-1};
    int timestampUs{-1};
    int ax{-1};
    int ay{-1};
    int az{-1};
    int gx{-1};
    int gy{-1};
    int gz{-1};
};

void printUsage() {
    std::cerr << "Usage: gg_pipeline_cli --input dataset.csv [options]\n"
              << "Options:\n"
              << "  --output-json path          Output JSON summary (default results.json)\n"
              << "  --envelope-csv path         Alpha-shape envelope CSV (default envelope.csv)\n"
              << "  --steady-csv path           Steady-state points CSV (default steady_points.csv)\n"
              << "  --processed-csv path        Full processed history CSV\n"
              << "  --alpha value               Alpha parameter (0 = convex hull)\n"
              << "  --accel-bias ax ay az       Accelerometer bias [m/s^2]\n"
              << "  --gyro-bias gx gy gz        Gyroscope bias [rad/s]\n"
              << "  --lever-arm lx ly lz        Lever arm IMU→CG [m]\n"
              << "  --velocity-damping value    Longitudinal velocity damping (1/s)\n"
              << "  --friction-mu value         Static friction coefficient for envelope check\n"
              << "  --camera-meta path          ESP32-CAM meta.csv path\n"
              << "  --camera-root path          Root directory for camera frames\n"
              << "  --use-vio                   Enable pseudo VIO logging/output\n"
              << "  --vio-trajectory path       Output CSV for VIO states (default trajectory.csv)\n"
              << "  --compare-envelope path     Reference envelope CSV for Hausdorff distance\n"
              << "  --monte-carlo N             Monte Carlo sample count\n"
              << "  --mc-seed value             Monte Carlo random seed\n"
              << "  --mc-accel-sigma value      Monte Carlo accel bias sigma [m/s^2]\n"
              << "  --mc-gyro-sigma value       Monte Carlo gyro bias sigma [rad/s]\n"
              << "  --mc-lever-sigma value      Monte Carlo lever-arm sigma [m]\n";
}

bool parseDouble(const std::string& token, double& out) {
    try {
        size_t idx = 0;
        double value = std::stod(token, &idx);
        if (idx != token.size()) {
            return false;
        }
        out = value;
        return true;
    } catch (...) {
        return false;
    }
}

bool parseVec3(int argc, char** argv, int& i, gg::Vec3& out) {
    if (i + 3 >= argc) {
        return false;
    }
    double x, y, z;
    if (!parseDouble(argv[++i], x) || !parseDouble(argv[++i], y) || !parseDouble(argv[++i], z)) {
        return false;
    }
    out = gg::Vec3{x, y, z};
    return true;
}

Options parseOptions(int argc, char** argv) {
    Options opts;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--input" && i + 1 < argc) {
            opts.inputPath = argv[++i];
        } else if (arg == "--output-json" && i + 1 < argc) {
            opts.outputJson = argv[++i];
        } else if (arg == "--envelope-csv" && i + 1 < argc) {
            opts.envelopeCsv = argv[++i];
        } else if (arg == "--steady-csv" && i + 1 < argc) {
            opts.steadyCsv = argv[++i];
        } else if (arg == "--processed-csv" && i + 1 < argc) {
            opts.processedCsv = argv[++i];
            opts.writeProcessed = true;
        } else if (arg == "--alpha" && i + 1 < argc) {
            parseDouble(argv[++i], opts.alpha);
        } else if (arg == "--accel-bias") {
            if (!parseVec3(argc, argv, i, opts.accelBias)) {
                throw std::runtime_error("Invalid --accel-bias arguments");
            }
        } else if (arg == "--gyro-bias") {
            if (!parseVec3(argc, argv, i, opts.gyroBias)) {
                throw std::runtime_error("Invalid --gyro-bias arguments");
            }
        } else if (arg == "--lever-arm") {
            if (!parseVec3(argc, argv, i, opts.leverArm)) {
                throw std::runtime_error("Invalid --lever-arm arguments");
            }
        } else if (arg == "--velocity-damping" && i + 1 < argc) {
            if (!parseDouble(argv[++i], opts.velocityDamping)) {
                throw std::runtime_error("Invalid --velocity-damping value");
            }
        } else if (arg == "--friction-mu" && i + 1 < argc) {
            if (!parseDouble(argv[++i], opts.frictionMu)) {
                throw std::runtime_error("Invalid --friction-mu value");
            }
        } else if (arg == "--camera-meta" && i + 1 < argc) {
            opts.cameraMeta = argv[++i];
        } else if (arg == "--camera-root" && i + 1 < argc) {
            opts.cameraRoot = argv[++i];
        } else if (arg == "--use-vio") {
            opts.useVio = true;
        } else if (arg == "--vio-trajectory" && i + 1 < argc) {
            opts.vioTrajectoryPath = argv[++i];
        } else if (arg == "--compare-envelope" && i + 1 < argc) {
            opts.compareEnvelopePath = argv[++i];
        } else if (arg == "--monte-carlo" && i + 1 < argc) {
            opts.monteCarloSamples = static_cast<std::size_t>(std::stoul(argv[++i]));
        } else if (arg == "--mc-seed" && i + 1 < argc) {
            opts.monteCarloSeed = static_cast<unsigned int>(std::stoul(argv[++i]));
        } else if (arg == "--mc-accel-sigma" && i + 1 < argc) {
            if (!parseDouble(argv[++i], opts.mcAccelSigma)) {
                throw std::runtime_error("Invalid --mc-accel-sigma value");
            }
        } else if (arg == "--mc-gyro-sigma" && i + 1 < argc) {
            if (!parseDouble(argv[++i], opts.mcGyroSigma)) {
                throw std::runtime_error("Invalid --mc-gyro-sigma value");
            }
        } else if (arg == "--mc-lever-sigma" && i + 1 < argc) {
            if (!parseDouble(argv[++i], opts.mcLeverSigma)) {
                throw std::runtime_error("Invalid --mc-lever-sigma value");
            }
        } else {
            std::cerr << "Unknown or incomplete argument: " << arg << "\n";
            printUsage();
            throw std::runtime_error("Invalid arguments");
        }
    }

    if (opts.inputPath.empty()) {
        printUsage();
        throw std::runtime_error("--input is required");
    }

    return opts;
}

std::vector<std::string> splitCsv(const std::string& line) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream ss(line);
    while (std::getline(ss, token, ',')) {
        tokens.push_back(token);
    }
    return tokens;
}

std::string trim(const std::string& s) {
    std::size_t start = 0;
    while (start < s.size() && std::isspace(static_cast<unsigned char>(s[start]))) {
        ++start;
    }
    std::size_t end = s.size();
    while (end > start && std::isspace(static_cast<unsigned char>(s[end - 1]))) {
        --end;
    }
    return s.substr(start, end - start);
}

CsvColumnMap mapColumns(const std::vector<std::string>& headers) {
    CsvColumnMap map;
    for (std::size_t i = 0; i < headers.size(); ++i) {
        std::string h = trim(headers[i]);
        if (h == "timestamp_us") {
            map.timestampUs = static_cast<int>(i);
        } else if (h == "timestamp" || h == "time") {
            map.timestamp = static_cast<int>(i);
        } else if (h == "ax" || h == "ax_mps2") {
            map.ax = static_cast<int>(i);
        } else if (h == "ay" || h == "ay_mps2") {
            map.ay = static_cast<int>(i);
        } else if (h == "az" || h == "az_mps2") {
            map.az = static_cast<int>(i);
        } else if (h == "gx" || h == "gx_rads") {
            map.gx = static_cast<int>(i);
        } else if (h == "gy" || h == "gy_rads") {
            map.gy = static_cast<int>(i);
        } else if (h == "gz" || h == "gz_rads") {
            map.gz = static_cast<int>(i);
        }
    }
    if ((map.timestamp < 0 && map.timestampUs < 0) || map.ax < 0 || map.ay < 0 || map.az < 0 ||
        map.gx < 0 || map.gy < 0 || map.gz < 0) {
        throw std::runtime_error("CSV missing required columns (timestamp[_us], ax, ay, az, gx, gy, gz)");
    }
    return map;
}

std::vector<gg::ImuData> loadCsv(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open input file: " + path);
    }

    std::string headerLine;
    if (!std::getline(file, headerLine)) {
        throw std::runtime_error("Empty CSV file: " + path);
    }

    auto headers = splitCsv(headerLine);
    CsvColumnMap columns = mapColumns(headers);

    std::vector<gg::ImuData> data;
    data.reserve(10000);
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) {
            continue;
        }
        auto tokens = splitCsv(line);
        if (tokens.size() < headers.size()) {
            tokens.resize(headers.size());
        }

        auto readValue = [&](int index) -> double {
            if (index < 0 || index >= static_cast<int>(tokens.size())) {
                throw std::runtime_error("Malformed CSV row: " + line);
            }
            double value;
            if (!parseDouble(trim(tokens[index]), value)) {
                throw std::runtime_error("Failed to parse numeric value from: " + tokens[index]);
            }
            return value;
        };

        gg::ImuData sample;
        if (columns.timestampUs >= 0) {
            sample.timestamp = readValue(columns.timestampUs) * 1e-6;
        } else {
            sample.timestamp = readValue(columns.timestamp);
        }
        sample.accel = gg::Vec3{readValue(columns.ax), readValue(columns.ay), readValue(columns.az)};
        sample.gyro = gg::Vec3{readValue(columns.gx), readValue(columns.gy), readValue(columns.gz)};
        data.push_back(sample);
    }

    std::sort(data.begin(), data.end(), [](const gg::ImuData& a, const gg::ImuData& b) {
        return a.timestamp < b.timestamp;
    });

    return data;
}

void writePointsCsv(const std::string& path, const std::vector<gg::Point2D>& points) {
    std::ofstream out(path);
    if (!out.is_open()) {
        throw std::runtime_error("Failed to open output file: " + path);
    }
    out << "gx,gy\n";
    out << std::setprecision(10);
    for (const auto& p : points) {
        out << p.x << ',' << p.y << '\n';
    }
}

void writeProcessedCsv(const std::string& path, const std::vector<gg::ProcessedPoint>& points) {
    std::ofstream out(path);
    if (!out.is_open()) {
        throw std::runtime_error("Failed to open output file: " + path);
    }
    out << "timestamp,gx,gy,longitudinal_velocity,yaw_rate\n";
    out << std::setprecision(10);
    for (const auto& p : points) {
        out << p.timestamp << ',' << p.gx << ',' << p.gy << ','
            << p.longitudinalVelocity << ',' << p.yawRate << '\n';
    }
}

double polygonArea(const std::vector<gg::Point2D>& polygon) {
    if (polygon.size() < 3) {
        return 0.0;
    }
    double area = 0.0;
    for (std::size_t i = 0; i < polygon.size(); ++i) {
        const auto& p1 = polygon[i];
        const auto& p2 = polygon[(i + 1) % polygon.size()];
        area += p1.x * p2.y - p2.x * p1.y;
    }
    return 0.5 * std::fabs(area);
}

double computeRSquared(const std::vector<gg::ProcessedPoint>& processed) {
    if (processed.size() < 2) {
        return 0.0;
    }
    std::vector<double> measured;
    std::vector<double> predicted;
    measured.reserve(processed.size());
    predicted.reserve(processed.size());

    for (const auto& p : processed) {
        double aLat = p.accelBody.y;
        double estimate = p.longitudinalVelocity * p.yawRate;
        measured.push_back(aLat);
        predicted.push_back(estimate);
    }

    double meanMeasured = std::accumulate(measured.begin(), measured.end(), 0.0) / measured.size();
    double ssTot = 0.0;
    double ssRes = 0.0;
    for (std::size_t i = 0; i < measured.size(); ++i) {
        double diff = measured[i] - predicted[i];
        ssRes += diff * diff;
        double dev = measured[i] - meanMeasured;
        ssTot += dev * dev;
    }
    if (ssTot == 0.0) {
        return 0.0;
    }
    return 1.0 - ssRes / ssTot;
}

double hausdorffDistance(const std::vector<gg::Point2D>& A, const std::vector<gg::Point2D>& B) {
    if (A.empty() || B.empty()) {
        return 0.0;
    }

    auto pointToSet = [](const gg::Point2D& p, const std::vector<gg::Point2D>& set) {
        double minDist = std::numeric_limits<double>::max();
        for (const auto& q : set) {
            double dx = p.x - q.x;
            double dy = p.y - q.y;
            double dist = std::sqrt(dx * dx + dy * dy);
            minDist = std::min(minDist, dist);
        }
        return minDist;
    };

    double maxAB = 0.0;
    for (const auto& p : A) {
        maxAB = std::max(maxAB, pointToSet(p, B));
    }

    double maxBA = 0.0;
    for (const auto& q : B) {
        maxBA = std::max(maxBA, pointToSet(q, A));
    }

    return std::max(maxAB, maxBA);
}

double frictionViolationRatio(const std::vector<gg::Point2D>& points, double mu, double& maxMagnitude) {
    if (mu <= 0.0) {
        maxMagnitude = 0.0;
        return 0.0;
    }

    std::size_t violations = 0;
    maxMagnitude = 0.0;
    double muSq = mu * mu;
    for (const auto& p : points) {
        double g2 = p.x * p.x + p.y * p.y;
        if (g2 > muSq) {
            ++violations;
        }
        maxMagnitude = std::max(maxMagnitude, std::sqrt(g2));
    }
    return points.empty() ? 0.0 : static_cast<double>(violations) / static_cast<double>(points.size());
}

struct MonteCarloOutputs {
    std::vector<gg::Point2D> samples;
    std::vector<gg::Point2D> envelope;
    std::vector<double> gxMax;
    std::vector<double> gyMax;
    std::vector<double> envelopeArea;
};

MonteCarloOutputs runMonteCarlo(const Options& opts, const std::vector<gg::ImuData>& data) {
    MonteCarloOutputs outputs;
    if (opts.monteCarloSamples == 0 || data.size() < 2) {
        return outputs;
    }

    std::mt19937 rng(opts.monteCarloSeed);
    std::normal_distribution<double> accelNoise(0.0, opts.mcAccelSigma);
    std::normal_distribution<double> gyroNoise(0.0, opts.mcGyroSigma);
    std::normal_distribution<double> leverNoise(0.0, opts.mcLeverSigma);

    gg::GgEnvelope aggregate(opts.alpha);

    for (std::size_t i = 0; i < opts.monteCarloSamples; ++i) {
        gg::Vec3 accelBias = opts.accelBias;
        gg::Vec3 gyroBias = opts.gyroBias;
        gg::Vec3 leverArm = opts.leverArm;

        if (opts.mcAccelSigma > 0.0) {
            accelBias.x += accelNoise(rng);
            accelBias.y += accelNoise(rng);
            accelBias.z += accelNoise(rng);
        }
        if (opts.mcGyroSigma > 0.0) {
            gyroBias.x += gyroNoise(rng);
            gyroBias.y += gyroNoise(rng);
            gyroBias.z += gyroNoise(rng);
        }
        if (opts.mcLeverSigma > 0.0) {
            leverArm.x += leverNoise(rng);
            leverArm.y += leverNoise(rng);
            leverArm.z += leverNoise(rng);
        }

        gg::AttitudeEstimator estimator;
        gg::GgProcessor::Config config;
        config.accelBias = accelBias;
        config.gyroBias = gyroBias;
        config.leverArm = leverArm;
        config.velocityDamping = opts.velocityDamping;

        gg::GgProcessor processor(std::move(estimator), config);
        gg::QuasiSteadyStateFilter filter;
        gg::GgEnvelope envelope(opts.alpha);

        double gxMax = 0.0;
        double gyMax = 0.0;

        for (const auto& sample : data) {
            gg::ProcessedPoint point = processor.process(sample);
            if (filter.update(point)) {
                gg::Point2D pt{point.gx, point.gy};
                envelope.addPoint(pt);
                aggregate.addPoint(pt);
                outputs.samples.push_back(pt);
                gxMax = std::max(gxMax, std::fabs(point.gx));
                gyMax = std::max(gyMax, std::fabs(point.gy));
            }
        }

        auto hull = envelope.calculateEnvelope();
        outputs.gxMax.push_back(gxMax);
        outputs.gyMax.push_back(gyMax);
        outputs.envelopeArea.push_back(polygonArea(hull));
    }

    outputs.envelope = aggregate.calculateEnvelope();
    return outputs;
}

std::vector<gg::Point2D> loadEnvelopeCsv(const std::string& path) {
    std::vector<gg::Point2D> pts;
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open envelope CSV: " + path);
    }
    std::string header;
    if (!std::getline(file, header)) {
        return pts;
    }
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        auto tokens = splitCsv(line);
        if (tokens.size() < 2) continue;
        gg::Point2D p;
        parseDouble(trim(tokens[0]), p.x);
        parseDouble(trim(tokens[1]), p.y);
        pts.push_back(p);
    }
    return pts;
}

std::string toJson(const Options& opts,
                   const std::vector<gg::ProcessedPoint>& processed,
                   const std::vector<gg::Point2D>& steady,
                   const std::vector<gg::Point2D>& envelope,
                   const gg::QualityMetrics& quality,
                   const gg::TimeSyncResult& timeSync,
                   const gg::AllanDeviationSeries& allanAccel,
                   const gg::AllanDeviationSeries& allanGyro,
                   const MonteCarloOutputs& mc,
                   double rSquared,
                   double frictionRatio,
                   double frictionMax,
                   std::optional<double> hausdorff) {
    double area = polygonArea(envelope);
    double maxGx = 0.0;
    double maxGy = 0.0;
    for (const auto& p : steady) {
        maxGx = std::max(maxGx, std::fabs(p.x));
        maxGy = std::max(maxGy, std::fabs(p.y));
    }

    auto dumpAllan = [](std::ostringstream& json, const char* key, const gg::AllanDeviationSeries& s) {
        json << "  \"" << key << "\": {\n";
        json << "    \"tau\": [";
        for (std::size_t i = 0; i < s.tau.size(); ++i) {
            if (i > 0) json << ',';
            json << s.tau[i];
        }
        json << "],\n    \"sigma_x\": [";
        for (std::size_t i = 0; i < s.sigmaX.size(); ++i) {
            if (i > 0) json << ',';
            json << s.sigmaX[i];
        }
        json << "],\n    \"sigma_y\": [";
        for (std::size_t i = 0; i < s.sigmaY.size(); ++i) {
            if (i > 0) json << ',';
            json << s.sigmaY[i];
        }
        json << "],\n    \"sigma_z\": [";
        for (std::size_t i = 0; i < s.sigmaZ.size(); ++i) {
            if (i > 0) json << ',';
            json << s.sigmaZ[i];
        }
        json << "],\n    \"sample_interval\": " << s.sampleInterval << "\n";
        json << "  },\n";
    };

    std::ostringstream json;
    json << std::setprecision(10);
    json << "{\n";
    json << "  \"steady_point_count\": " << steady.size() << ",\n";
    json << "  \"max_gx\": " << maxGx << ",\n";
    json << "  \"max_gy\": " << maxGy << ",\n";
    json << "  \"envelope_area\": " << area << ",\n";
    json << "  \"alpha\": " << opts.alpha << ",\n";
    json << "  \"validation\": {\n"
         << "    \"r_squared\": " << rSquared << ",\n"
         << "    \"friction_mu\": " << opts.frictionMu << ",\n"
         << "    \"friction_violation_ratio\": " << frictionRatio << ",\n"
         << "    \"friction_max_magnitude\": " << frictionMax;
    if (hausdorff) {
        json << ",\n    \"hausdorff_distance\": " << *hausdorff << "\n  },\n";
    } else {
        json << "\n  },\n";
    }

    json << "  \"quality\": {\n"
         << "    \"sample_count\": " << quality.sampleCount << ",\n"
         << "    \"mean_dt\": " << quality.meanSamplingInterval << ",\n"
         << "    \"nominal_frequency\": " << quality.nominalFrequency << ",\n"
         << "    \"jitter\": " << quality.samplingJitter << ",\n"
         << "    \"missing_rate\": " << quality.missingDataRate << ",\n"
         << "    \"accel_saturation_rate\": " << quality.saturationAccelRate << ",\n"
         << "    \"gyro_saturation_rate\": " << quality.saturationGyroRate << "\n"
         << "  },\n";

    json << "  \"time_sync\": {\n"
         << "    \"valid\": " << (timeSync.valid ? "true" : "false") << ",\n"
         << "    \"offset\": " << timeSync.offset << ",\n"
         << "    \"peak_correlation\": " << timeSync.peakCorrelation << "\n"
         << "  },\n";

    dumpAllan(json, "allan_accel", allanAccel);
    dumpAllan(json, "allan_gyro", allanGyro);

    json << "  \"monte_carlo\": {\n";
    if (!mc.gxMax.empty()) {
        auto percentile = [](std::vector<double> values, double p) {
            if (values.empty()) return 0.0;
            std::sort(values.begin(), values.end());
            double idx = p * (values.size() - 1);
            std::size_t i = static_cast<std::size_t>(idx);
            double frac = idx - i;
            if (i + 1 < values.size()) {
                return values[i] * (1.0 - frac) + values[i + 1] * frac;
            }
            return values.back();
        };

        double meanGx = std::accumulate(mc.gxMax.begin(), mc.gxMax.end(), 0.0) / mc.gxMax.size();
        double meanGy = std::accumulate(mc.gyMax.begin(), mc.gyMax.end(), 0.0) / mc.gyMax.size();
        double meanArea = std::accumulate(mc.envelopeArea.begin(), mc.envelopeArea.end(), 0.0) / mc.envelopeArea.size();

        json << "    \"enabled\": true,\n"
             << "    \"samples\": " << mc.gxMax.size() << ",\n"
             << "    \"mean_gx\": " << meanGx << ",\n"
             << "    \"mean_gy\": " << meanGy << ",\n"
             << "    \"mean_area\": " << meanArea << ",\n"
             << "    \"gx_ci_95\": [" << percentile(mc.gxMax, 0.025) << ',' << percentile(mc.gxMax, 0.975) << "],\n"
             << "    \"gy_ci_95\": [" << percentile(mc.gyMax, 0.025) << ',' << percentile(mc.gyMax, 0.975) << "],\n"
             << "    \"area_ci_95\": [" << percentile(mc.envelopeArea, 0.025) << ',' << percentile(mc.envelopeArea, 0.975) << "]\n";
    } else {
        json << "    \"enabled\": false\n";
    }
    json << "  }\n";
    json << "}\n";
    return json.str();
}

}  // namespace

int main(int argc, char** argv) {
    try {
        Options opts = parseOptions(argc, argv);
        auto data = loadCsv(opts.inputPath);
        if (data.size() < 2) {
            throw std::runtime_error("Dataset too small to process");
        }

        gg::AttitudeEstimator estimator;
        gg::GgProcessor::Config config;
        config.accelBias = opts.accelBias;
        config.gyroBias = opts.gyroBias;
        config.leverArm = opts.leverArm;
        config.velocityDamping = opts.velocityDamping;

        gg::GgProcessor processor(std::move(estimator), config);
        gg::QuasiSteadyStateFilter filter;
        gg::GgEnvelope envelope(opts.alpha);

        std::vector<gg::ProcessedPoint> processed;
        processed.reserve(data.size());
        std::vector<gg::Point2D> steadyPoints;

        for (const auto& sample : data) {
            gg::ProcessedPoint point = processor.process(sample);
            processed.push_back(point);
            if (filter.update(point)) {
                gg::Point2D pt{point.gx, point.gy};
                steadyPoints.push_back(pt);
                envelope.addPoint(pt);
            }
        }

        auto envelopeHull = envelope.calculateEnvelope();
        writePointsCsv(opts.envelopeCsv, envelopeHull);
        writePointsCsv(opts.steadyCsv, steadyPoints);
        if (opts.writeProcessed) {
            writeProcessedCsv(opts.processedCsv, processed);
        }

        // Optional pseudo VIO logging
        if (opts.useVio && !opts.cameraMeta.empty()) {
            std::string root = opts.cameraRoot.empty() ? std::filesystem::path(opts.cameraMeta).parent_path().string() : opts.cameraRoot;
            auto frames = vision::loadCameraFrames(opts.cameraMeta, root);
            vision::VioConfig vioCfg;
            vioCfg.processorConfig = config;
            vioCfg.gravity = gg::Vec3{0.0, 0.0, -gg::kGravity};
            auto states = vision::runPseudoVio(data, frames, vioCfg);
            if (!states.empty()) {
                vision::writeTrajectoryCsv(states, opts.vioTrajectoryPath);
            }
        }

        gg::QualityMetrics quality = gg::computeQualityMetrics(data);
        gg::TimeSyncResult timeSync = gg::estimateTimeOffset(processed);
        gg::AllanDeviationSeries allanAccel = gg::computeAllanDeviation(data, true);
        gg::AllanDeviationSeries allanGyro = gg::computeAllanDeviation(data, false);

        double rSquared = computeRSquared(processed);
        double frictionMax = 0.0;
        double frictionRatio = frictionViolationRatio(steadyPoints, opts.frictionMu, frictionMax);

        std::optional<double> hausdorff;
        if (!opts.compareEnvelopePath.empty()) {
            auto reference = loadEnvelopeCsv(opts.compareEnvelopePath);
            if (!reference.empty()) {
                hausdorff = hausdorffDistance(envelopeHull, reference);
            }
        }

        MonteCarloOutputs mc = runMonteCarlo(opts, data);

        std::string json = toJson(opts, processed, steadyPoints, envelopeHull,
                                  quality, timeSync, allanAccel, allanGyro,
                                  mc, rSquared, frictionRatio, frictionMax, hausdorff);

        std::ofstream out(opts.outputJson);
        if (!out.is_open()) {
            throw std::runtime_error("Failed to open output JSON: " + opts.outputJson);
        }
        out << json;
        out.close();

        std::cout << "Processed " << processed.size() << " samples" << std::endl;
        std::cout << "Steady-state points: " << steadyPoints.size() << std::endl;
        if (!mc.gxMax.empty()) {
            std::cout << "Monte Carlo samples: " << mc.gxMax.size() << std::endl;
        }

        return EXIT_SUCCESS;
    } catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << '\n';
        return EXIT_FAILURE;
    }
}
