#include "vision/dataset.hpp"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace vision {

namespace {

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

std::vector<std::string> splitCsv(const std::string& line) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream ss(line);
    while (std::getline(ss, token, ',')) {
        tokens.push_back(token);
    }
    return tokens;
}

}  // namespace

std::vector<CameraFrame> loadCameraFrames(const std::string& metaCsvPath,
                                          const std::string& cameraRootDir) {
    namespace fs = std::filesystem;

    if (!fs::exists(metaCsvPath)) {
        throw std::runtime_error("Camera meta CSV not found: " + metaCsvPath);
    }

    std::ifstream file(metaCsvPath);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open camera meta CSV: " + metaCsvPath);
    }

    std::string headerLine;
    if (!std::getline(file, headerLine)) {
        throw std::runtime_error("Camera meta CSV is empty: " + metaCsvPath);
    }

    auto headers = splitCsv(headerLine);
    int colPps = -1;
    int colLocal = -1;
    int colOffset = -1;
    int colPath = -1;

    for (std::size_t i = 0; i < headers.size(); ++i) {
        std::string h = trim(headers[i]);
        if (h == "pps_timestamp_us") {
            colPps = static_cast<int>(i);
        } else if (h == "local_capture_us") {
            colLocal = static_cast<int>(i);
        } else if (h == "offset_us") {
            colOffset = static_cast<int>(i);
        } else if (h == "path") {
            colPath = static_cast<int>(i);
        }
    }

    if (colPath < 0) {
        throw std::runtime_error("Camera meta CSV missing 'path' column: " + metaCsvPath);
    }

    std::vector<CameraFrame> frames;
    frames.reserve(1000);

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) {
            continue;
        }
        auto tokens = splitCsv(line);
        if (tokens.size() <= static_cast<std::size_t>(colPath)) {
            continue;
        }

        CameraFrame frame;
        if (colPps >= 0) {
            frame.ppsTimestampUs = std::stoull(trim(tokens[colPps]));
        }
        if (colLocal >= 0) {
            frame.localCaptureUs = std::stoull(trim(tokens[colLocal]));
        }
        if (colOffset >= 0) {
            frame.offsetUs = std::stoull(trim(tokens[colOffset]));
        }
        frame.relativePath = trim(tokens[colPath]);

        if (!frame.relativePath.empty()) {
            fs::path rel(frame.relativePath);
            fs::path root(cameraRootDir);
            fs::path full = root / rel;
            if (!fs::exists(full)) {
                // keep relative path but warn via exception to signal missing frame
                throw std::runtime_error("Camera frame missing: " + full.string());
            }
        }
        frames.push_back(frame);
    }

    std::sort(frames.begin(), frames.end(), [](const CameraFrame& a, const CameraFrame& b) {
        return a.localCaptureUs < b.localCaptureUs;
    });

    return frames;
}

}  // namespace vision
