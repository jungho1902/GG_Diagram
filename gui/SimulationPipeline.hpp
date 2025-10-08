#pragma once

#include "analytics.hpp"
#include "gg_envelope.hpp"
#include "gg_processor.hpp"
#include "quasi_steady_state_filter.hpp"

#include <optional>
#include <random>
#include <string>
#include <vector>

namespace gg::gui {

struct SimulationStats {
    double gx{0.0};
    double gy{0.0};
    std::size_t totalPoints{0};
    std::size_t steadyPoints{0};
};

class SimulationPipeline {
public:
    SimulationPipeline();

    void start();
    void stop();
    void toggle();
    void reset();

    void update(double deltaTime);

    bool isRunning() const { return running_; }

    void setUseDataset(bool useDataset);
    bool useDataset() const { return useDataset_; }

    bool loadDatasetFromCsv(const std::string& path, char delimiter = ',');
    bool hasDataset() const { return !dataset_.empty(); }
    std::size_t datasetSize() const { return dataset_.size(); }
    bool datasetFinished() const { return datasetFinished_; }

    void setDatasetPlaybackSpeed(double speed);
    double datasetPlaybackSpeed() const { return datasetPlaybackSpeed_; }

    bool computeAnalytics();
    const AnalyticsResults& analytics() const { return analytics_; }

    struct ProcessorParams {
        Vec3 accelBias{0.05, -0.04, 0.02};
        Vec3 gyroBias{0.002, -0.001, 0.0005};
        Vec3 leverArm{0.30, 0.02, 0.12};
        double velocityDamping{0.05};
        Vec3 gravity{0.0, 0.0, -kGravity};
    };

    struct AnalyticsParams {
        double accelSaturation{50.0};
        double gyroSaturation{20.0};
        std::size_t maxLagSamples{500};
        double allanMaxTau{20.0};
        std::size_t allanSteps{15};
    };

    struct MonteCarloParams {
        std::size_t samples{0};
        double accelBiasSigma{0.0};
        double gyroBiasSigma{0.0};
        double leverArmSigma{0.0};
        unsigned int seed{42};
    };

    const ProcessorParams& processorParams() const { return processorParams_; }
    void setProcessorParams(const ProcessorParams& params);

    const AnalyticsParams& analyticsParams() const { return analyticsParams_; }
    void setAnalyticsParams(const AnalyticsParams& params);

    const MonteCarloParams& monteCarloParams() const { return monteCarloParams_; }
    void setMonteCarloParams(const MonteCarloParams& params);
    bool runMonteCarlo();
    const std::vector<Point2D>& monteCarloEnvelope() const { return monteCarloEnvelope_; }
    const std::vector<Point2D>& monteCarloSamples() const { return monteCarloSamples_; }

    SimulationStats stats() const;

    const std::vector<Point2D>& processedPoints() const { return processedPoints_; }
    const std::vector<Point2D>& steadyStatePoints() const { return steadyStatePoints_; }
    const std::vector<Point2D>& envelopeHull() const { return envelopeHull_; }

    QuasiSteadyStateFilter::Thresholds thresholds() const { return thresholds_; }
    void setThresholds(const QuasiSteadyStateFilter::Thresholds& thresholds);

    double envelopeAlpha() const { return envelopeAlpha_; }
    void setEnvelopeAlpha(double alpha);

private:
    struct Segment {
        double duration;
        double longitudinalAccel;
        double targetYawRate;
    };

    ImuData generateSample();
    void rebuildFilter();
    void processSample(const ImuData& sample, bool& envelopeDirty);
    void clearProcessedData();
    void rebuildProcessor();
    void replayHistory();
    AttitudeEstimator makeAttitudeEstimator(const ProcessorParams& params) const;
    GgProcessor::Config buildProcessorConfig(const ProcessorParams& params) const;

    const double dt_{0.01};

    AttitudeEstimator makeAttitudeEstimator() const;

    std::vector<Segment> profile_;

    std::default_random_engine rng_;
    std::normal_distribution<double> accelNoise_{0.0, 0.05};
    std::normal_distribution<double> gyroNoise_{0.0, 0.002};

    ProcessorParams processorParams_;
    AnalyticsParams analyticsParams_;
    MonteCarloParams monteCarloParams_;
    GgProcessor::Config processorConfig_;
    GgProcessor processor_;

    QuasiSteadyStateFilter::Thresholds thresholds_;
    QuasiSteadyStateFilter filter_;
    double envelopeAlpha_{1.5};
    GgEnvelope envelope_;

    double time_{0.0};
    double segmentTime_{0.0};
    std::size_t currentSegment_{0};
    double yaw_{0.0};
    double yawRate_{0.0};
    double velocity_{0.0};

    bool running_{false};
    double timeAccumulator_{0.0};

    std::optional<ProcessedPoint> lastProcessed_;

    std::vector<ProcessedPoint> processedHistory_;
    std::vector<Point2D> processedPoints_;
    std::vector<Point2D> steadyStatePoints_;
    std::vector<Point2D> envelopeHull_;

    std::vector<Point2D> monteCarloSamples_;
    std::vector<Point2D> monteCarloEnvelope_;

    bool useDataset_{false};
    std::vector<ImuData> dataset_;
    std::size_t datasetIndex_{0};
    double datasetStartTime_{0.0};
    double datasetAccumulator_{0.0};
    double datasetPlaybackSpeed_{1.0};
    bool datasetFinished_{false};

    std::vector<ImuData> rawHistory_;
    AnalyticsResults analytics_;
};

}  // namespace gg::gui
