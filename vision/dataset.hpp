#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace vision {

struct CameraFrame {
    std::uint64_t ppsTimestampUs{0};
    std::uint64_t localCaptureUs{0};
    std::uint64_t offsetUs{0};
    std::string relativePath;  // relative to camera dataset root
};

std::vector<CameraFrame> loadCameraFrames(const std::string& metaCsvPath,
                                          const std::string& cameraRootDir);

}  // namespace vision
