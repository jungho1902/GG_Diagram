#include "vision/vio_stub.hpp"

#include "attitude_estimator.hpp"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <stdexcept>

namespace vision {

std::vector<VioState> runPseudoVio(const std::vector<gg::ImuData>& imu,
                                   const std::vector<CameraFrame>& /*frames*/,
                                   const VioConfig& config) {
    std::vector<VioState> states;
    states.reserve(imu.size());

    gg::AttitudeEstimator estimator;
    gg::GgProcessor processor(estimator, config.processorConfig);

    gg::Vec3 positionNav{0.0, 0.0, 0.0};
    gg::Vec3 velocityNav{0.0, 0.0, 0.0};

    double lastTimestamp = imu.empty() ? 0.0 : imu.front().timestamp;

    for (const auto& sample : imu) {
        gg::ProcessedPoint processed = processor.process(sample);

        double dt = sample.timestamp - lastTimestamp;
        if (dt < 0.0) {
            dt = 0.0;
        }

        velocityNav.x = processed.longitudinalVelocity;
        velocityNav.y += processed.accelNav.y * dt;
        velocityNav.z += processed.accelNav.z * dt;

        positionNav.x += velocityNav.x * dt;
        positionNav.y += velocityNav.y * dt;
        positionNav.z += velocityNav.z * dt;

        VioState state;
        state.timestamp = sample.timestamp;
        state.position = positionNav;
        state.velocity = velocityNav;
        state.orientation = processor.attitude().orientation();
        states.push_back(state);

        lastTimestamp = sample.timestamp;
    }

    return states;
}

void writeTrajectoryCsv(const std::vector<VioState>& states, const std::string& path) {
    std::ofstream out(path);
    if (!out.is_open()) {
        throw std::runtime_error("Failed to open VIO trajectory output: " + path);
    }

    out << "timestamp,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,quat_w,quat_x,quat_y,quat_z\n";
    out << std::setprecision(10);
    for (const auto& state : states) {
        out << state.timestamp << ','
            << state.position.x << ',' << state.position.y << ',' << state.position.z << ','
            << state.velocity.x << ',' << state.velocity.y << ',' << state.velocity.z << ','
            << state.orientation.w << ',' << state.orientation.x << ','
            << state.orientation.y << ',' << state.orientation.z << '\n';
    }
}

}  // namespace vision
