#include "trajectory_file.h"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace {
std::string trimCopy(const std::string& value) {
    const auto first = value.find_first_not_of(" \t\r\n#;");
    if (first == std::string::npos) return {};
    const auto last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

bool readTimingHeader(const std::string& line,
                      double& segmentSeconds,
                      double& moveTimeSeconds) {
    std::string text = trimCopy(line);
    const auto eq = text.find('=');
    if (eq == std::string::npos) return false;

    std::string key = trimCopy(text.substr(0, eq));
    std::string val = trimCopy(text.substr(eq + 1));
    std::transform(key.begin(), key.end(), key.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

    try {
        const double parsed = std::stod(val);
        if (key == "segment_period_s" || key == "segment_seconds") {
            segmentSeconds = parsed;
            return true;
        }
        if (key == "segment_period_ms" || key == "segment_ms") {
            segmentSeconds = parsed / 1000.0;
            return true;
        }
        if (key == "segment_period_us" || key == "segment_us") {
            segmentSeconds = parsed / 1000000.0;
            return true;
        }
        if (key == "movetimevalue" || key == "move_time" || key == "move_time_s") {
            moveTimeSeconds = parsed;
            return true;
        }
    } catch (...) {
        return false;
    }

    return false;
}
}

bool readMappedPositionFile(const std::string& filename,
                            AxisVectors& positions,
                            double& segmentSeconds,
                            double& moveTimeSeconds) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cout << "Cannot open file: " << filename << std::endl;
        return false;
    }

    for (auto& axis : positions) axis.clear();
    segmentSeconds = -1.0;
    moveTimeSeconds = -1.0;

    std::string line;
    while (std::getline(file, line)) {
        const auto firstNonSpace = line.find_first_not_of(" \t\r\n");
        if (firstNonSpace == std::string::npos) continue;

        const char first = line[firstNonSpace];
        if (first == '#' || first == ';') {
            readTimingHeader(line, segmentSeconds, moveTimeSeconds);
            continue;
        }

        std::stringstream ss(line);
        AxisPositions point{};
        for (size_t i = 0; i < AXIS_COUNT; ++i) {
            if (!(ss >> point[i])) {
                std::cout << "Format error at line: " << line << std::endl;
                return false;
            }
        }
        for (size_t i = 0; i < AXIS_COUNT; ++i) {
            positions[i].push_back(point[i]);
        }
    }

    return !positions[0].empty();
}

bool writeTrajectoryFile(const std::string& filename,
                         const Trajectory& trajectory) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cout << "Cannot open file for write: " << filename << std::endl;
        return false;
    }

    file << "# trajectory_format=time_ms positions velocities\n";
    for (const auto& point : trajectory) {
        file << point.time_ms;
        for (size_t i = 0; i < AXIS_COUNT; ++i) {
            file << ' ' << point.positions[i];
        }
        for (size_t i = 0; i < AXIS_COUNT; ++i) {
            file << ' ' << point.velocities[i];
        }
        file << '\n';
    }
    return true;
}

bool readTrajectoryFile(const std::string& filename,
                        Trajectory& trajectory) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cout << "Cannot open trajectory file: " << filename << std::endl;
        return false;
    }

    trajectory.clear();
    std::string line;
    while (std::getline(file, line)) {
        const auto firstNonSpace = line.find_first_not_of(" \t\r\n");
        if (firstNonSpace == std::string::npos) continue;
        const char first = line[firstNonSpace];
        if (first == '#' || first == ';') continue;

        std::stringstream ss(line);
        TrajectoryPoint point{};
        if (!(ss >> point.time_ms)) {
            std::cout << "Trajectory format error at line: " << line << std::endl;
            return false;
        }
        for (size_t i = 0; i < AXIS_COUNT; ++i) {
            if (!(ss >> point.positions[i])) {
                std::cout << "Trajectory position format error at line: " << line << std::endl;
                return false;
            }
        }
        for (size_t i = 0; i < AXIS_COUNT; ++i) {
            if (!(ss >> point.velocities[i])) {
                std::cout << "Trajectory velocity format error at line: " << line << std::endl;
                return false;
            }
        }
        trajectory.push_back(point);
    }

    return !trajectory.empty();
}
