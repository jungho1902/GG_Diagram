#pragma once

#include "data_types.hpp"
#include "gg_processor.hpp"
#include "vision/dataset.hpp"

#include <string>
#include <vector>

namespace vision {

struct VioState {
    double timestamp{0.0};
    gg::Vec3 position;      // meters in navigation frame
    gg::Vec3 velocity;      // m/s in navigation frame
    gg::Quaternion orientation;  // quaternion rotating nav -> body
};

struct VioConfig {
    gg::GgProcessor::Config processorConfig;
    gg::Vec3 gravity{0.0, 0.0, -gg::kGravity};
};

std::vector<VioState> runPseudoVio(const std::vector<gg::ImuData>& imu,
                                   const std::vector<CameraFrame>& frames,
                                   const VioConfig& config);

void writeTrajectoryCsv(const std::vector<VioState>& states, const std::string& path);

}  // namespace vision
