#define AXIS_CONTROLLER_OVERRIDE_MODE
#include "axis_controller.h"
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <limits>

namespace {
constexpr auto kCommandSpacing = std::chrono::microseconds(1500);
constexpr double kDefaultSegmentSeconds = 0.05;
constexpr unsigned char kAccelerationTimeParameter = 3;
constexpr unsigned char kDecelerationTimeParameter = 4;
constexpr int kStreamingAccelTimeMs = 5;
constexpr int kStreamingDecelTimeMs = 5;
constexpr double kMinCornerSpeedScale = 0.25;
constexpr double kStraightCosThreshold = 0.98;
constexpr size_t DI_INDEX = 0;
constexpr size_t DO_INDEX = 0;
constexpr size_t FILE_RELAY_COLUMN = 6;
constexpr auto kRelaySettleDelay = std::chrono::milliseconds(500);
constexpr auto kMotionPollInterval = std::chrono::milliseconds(10);
constexpr auto kHomingTimeout = std::chrono::seconds(45);
constexpr auto kFinalSettleTimeout = std::chrono::seconds(20);

std::vector<int> g_recordedRelayStates;
std::vector<int> g_savedRelayStates;

std::chrono::microseconds g_segment_period =
    std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::duration<double>(kDefaultSegmentSeconds));

AxisVectorsEx makeExtendedPaths(const AxisVectors& positions,
                                const std::vector<int>& relayStates) {
    AxisVectorsEx out;
    const size_t steps = positions[0].size();

    for (size_t a = 0; a < AXIS_COUNT; ++a) {
        out[a] = positions[a];
    }

    out[AXIS_COUNT].reserve(steps);
    for (size_t i = 0; i < steps; ++i) {
        out[AXIS_COUNT].push_back(i < relayStates.size() ? relayStates[i] : 0);
    }

    return out;
}

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

void setSegmentPeriodSeconds(double seconds) {
    if (seconds <= 0.0) return;
    g_segment_period = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::duration<double>(seconds));
}

AxisVelocities computeSegmentVelocities(const AxisPositions& current,
                                        const AxisPositions& targets,
                                        std::chrono::microseconds segmentPeriod) {
    AxisVelocities velocities{};
    velocities.fill(1);

    double seconds = static_cast<double>(segmentPeriod.count()) / 1000000.0;
    if (seconds <= 0.0) {
        seconds = kDefaultSegmentSeconds;
    }

    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        const auto distance =
            std::llabs(static_cast<long long>(targets[i]) - static_cast<long long>(current[i]));
        if (distance == 0) {
            velocities[i] = 1;
            continue;
        }

        const double speed = std::ceil(static_cast<double>(distance) / seconds);
        if (speed > static_cast<double>(std::numeric_limits<int>::max())) {
            velocities[i] = std::numeric_limits<int>::max();
        } else {
            velocities[i] = std::max(1, static_cast<int>(speed));
        }
    }

    return velocities;
}

double vectorLength(const AxisPositions& from, const AxisPositions& to) {
    double sum = 0.0;
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        const double d = static_cast<double>(to[i]) - static_cast<double>(from[i]);
        sum += d * d;
    }
    return std::sqrt(sum);
}

double cornerSpeedScale(const AxisPositions& before,
                        const AxisPositions& corner,
                        const AxisPositions& after) {
    const double lenA = vectorLength(before, corner);
    const double lenB = vectorLength(corner, after);
    if (lenA <= 0.0 || lenB <= 0.0) {
        return 1.0;
    }

    double dot = 0.0;
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        const double a = static_cast<double>(corner[i]) - static_cast<double>(before[i]);
        const double b = static_cast<double>(after[i]) - static_cast<double>(corner[i]);
        dot += a * b;
    }

    const double cosTheta = std::max(-1.0, std::min(1.0, dot / (lenA * lenB)));
    if (cosTheta >= kStraightCosThreshold) {
        return 1.0;
    }

    return std::max(kMinCornerSpeedScale, (1.0 + cosTheta) * 0.5);
}

std::chrono::microseconds scaledSegmentPeriod(std::chrono::microseconds basePeriod,
                                              double speedScale) {
    const double safeScale = std::max(kMinCornerSpeedScale, std::min(1.0, speedScale));
    const double scaledSeconds =
        (static_cast<double>(basePeriod.count()) / 1000000.0) / safeScale;
    return std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::duration<double>(scaledSeconds));
}

bool setAllMotionTiming(const AxisPorts& ports,
                        const AxisSlaves& slaves,
                        unsigned char paramNo,
                        int value,
                        const char* label) {
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        const int ret = FAS_SetParameter(ports[i], slaves[i], paramNo, value);
        if (ret != FMM_OK) {
            std::cout << "Failed to set " << label << " for axis " << (i + 1)
                      << ": " << ret << std::endl;
            return false;
        }
    }
    return true;
}

bool readAllMotionTiming(const AxisPorts& ports,
                         const AxisSlaves& slaves,
                         unsigned char paramNo,
                         AxisPositions& values,
                         const char* label) {
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        const int ret = FAS_GetParameter(ports[i], slaves[i], paramNo, &values[i]);
        if (ret != FMM_OK) {
            std::cout << "Failed to read " << label << " for axis " << (i + 1)
                      << ": " << ret << std::endl;
            return false;
        }
    }
    return true;
}

bool restoreAllMotionTiming(const AxisPorts& ports,
                            const AxisSlaves& slaves,
                            unsigned char paramNo,
                            const AxisPositions& values,
                            const char* label) {
    bool ok = true;
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        const int ret = FAS_SetParameter(ports[i], slaves[i], paramNo, values[i]);
        if (ret != FMM_OK) {
            std::cout << "Failed to restore " << label << " for axis " << (i + 1)
                      << ": " << ret << std::endl;
            ok = false;
        }
    }
    return ok;
}

void stopAllAxes(const AxisPorts& ports, const AxisSlaves& slaves) {
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        const int ret = FAS_MoveStop(ports[i], slaves[i]);
        if (ret != FMM_OK) {
            std::cout << "Failed to stop axis " << (i + 1) << ": " << ret << std::endl;
        }
    }
}

bool hasMotionFault(const EZISERVO_AXISSTATUS& st) {
    return st.FFLAG_ERRORALL ||
           st.FFLAG_HWPOSILMT ||
           st.FFLAG_HWNEGALMT ||
           st.FFLAG_SWPOGILMT ||
           st.FFLAG_SWNEGALMT ||
           st.FFLAG_ERRPOSOVERFLOW ||
           st.FFLAG_ERROVERCURRENT ||
           st.FFLAG_ERROVERSPEED ||
           st.FFLAG_ERRPOSTRACKING ||
           st.FFLAG_ERROVERLOAD ||
           st.FFLAG_ERROVERHEAT ||
           st.FFLAG_ERRBACKEMF ||
           st.FFLAG_ERRMOTORPOWER ||
           st.FFLAG_ERRINPOSITION ||
           st.FFLAG_EMGSTOP;
}

void printMotionStatus(size_t axis, const EZISERVO_AXISSTATUS& st) {
    std::cout << "Axis " << (axis + 1)
              << " status=0x" << std::hex << st.dwValue << std::dec
              << " servo=" << st.FFLAG_SERVOON
              << " motion=" << st.FFLAG_MOTIONING
              << " error=" << st.FFLAG_ERRORALL
              << " hw+=" << st.FFLAG_HWPOSILMT
              << " hw-=" << st.FFLAG_HWNEGALMT
              << " sw+=" << st.FFLAG_SWPOGILMT
              << " sw-=" << st.FFLAG_SWNEGALMT
              << " posOverflow=" << st.FFLAG_ERRPOSOVERFLOW
              << " overSpeed=" << st.FFLAG_ERROVERSPEED
              << " posTracking=" << st.FFLAG_ERRPOSTRACKING
              << " overload=" << st.FFLAG_ERROVERLOAD
              << " inPosition=" << st.FFLAG_ERRINPOSITION
              << " emg=" << st.FFLAG_EMGSTOP
              << std::endl;
}

std::chrono::milliseconds estimateMoveTimeout(const AxisPositions& current,
                                              const AxisPositions& targets,
                                              const AxisVelocities& velocities,
                                              const AxisBools& hasCommand) {
    double longestSeconds = 0.0;
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        if (!hasCommand[i] || velocities[i] <= 0) continue;
        const auto distance =
            std::llabs(static_cast<long long>(targets[i]) - static_cast<long long>(current[i]));
        longestSeconds = std::max(longestSeconds,
                                  static_cast<double>(distance) / velocities[i]);
    }

    const auto estimated = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(longestSeconds * 3.0 + 3.0));
    return std::max(std::chrono::milliseconds(estimated), std::chrono::milliseconds(5000));
}

bool waitForMotionComplete(const AxisPorts& ports,
                           const AxisSlaves& slaves,
                           const AxisBools& activeAxes,
                           std::chrono::milliseconds timeout,
                           const char* label) {
    AxisStatuses statuses{};
    const auto deadline = std::chrono::steady_clock::now() + timeout;

    while (true) {
        bool allDone = true;

        for (size_t i = 0; i < AXIS_COUNT; ++i) {
            if (!activeAxes[i]) continue;

            const int ret = FAS_GetAxisStatus(ports[i], slaves[i], &(statuses[i].dwValue));
            if (ret != FMM_OK) {
                std::cout << label << " failed to read status for axis " << (i + 1)
                          << ": " << ret << std::endl;
                stopAllAxes(ports, slaves);
                return false;
            }

            if (hasMotionFault(statuses[i])) {
                std::cout << label << " stopped because axis " << (i + 1)
                          << " reported a fault/limit." << std::endl;
                printMotionStatus(i, statuses[i]);
                stopAllAxes(ports, slaves);
                return false;
            }

            if (statuses[i].FFLAG_MOTIONING) {
                allDone = false;
            }
        }

        if (allDone) return true;

        if (std::chrono::steady_clock::now() >= deadline) {
            std::cout << label << " timeout while waiting for driver running bit to clear."
                      << std::endl;
            for (size_t i = 0; i < AXIS_COUNT; ++i) {
                if (activeAxes[i]) printMotionStatus(i, statuses[i]);
            }
            stopAllAxes(ports, slaves);
            return false;
        }

        std::this_thread::sleep_for(kMotionPollInterval);
    }
}
}
//hihi
//hihiiiiiii
AxisController::AxisController() {
    int portDefaults[6] = {0, 0, 0, 0, 0, 0};
    unsigned char slaveDefaults[6] = {0, 1, 2, 3, 4, 5};
    const double gearDefaults[6] = {40.0, 81.0, 5.0, 10.0, 50.0, 1.0};
    gear_ratios_.fill(1.0);
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        nPortIDs_[i] = portDefaults[i];
        iSlaveNos_[i] = slaveDefaults[i];
        gear_ratios_[i] = gearDefaults[i];
    }
}

AxisController::~AxisController() {
    stopRecordingThread();
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        FAS_Close(nPortIDs_[i]);
    }
}

constexpr size_t MAX_BUFFERS = 8;
// AxisVectors recorded_positions;
std::mutex rec_mtx;
std::atomic<bool> recording{false};
std::thread rec_thread;

std::string AxisController::axisName(size_t idx) const {
    return "Motor " + std::to_string(idx + 1);
}

using RecordBuffers = std::array<AxisVectors, MAX_BUFFERS>;

RecordBuffers recorded_positions_buffers;

std::atomic<size_t> currentRecordingBuffer{MAX_BUFFERS - 1};
std::atomic<size_t> bufferCount{0};

AxisVectors downsampleBuffer(
    const AxisVectors& in,
    size_t step
) {
    AxisVectors out;
    if (in[0].empty()) return out;

    size_t len = in[0].size();
    for (size_t i = 0; i < len; i += step) {
        for (size_t a = 0; a < AXIS_COUNT; ++a) {
            out[a].push_back(in[a][i]);
        }
    }
    return out;
}

struct GoPlanner {
    std::vector<AxisPositions> steps;
    size_t cursor = 0;

    void clear() {
        steps.clear();
        cursor = 0;
    }

    bool ready() const {
        return !steps.empty();
    }

    void reset() {
        cursor = 0;
    }

    bool next(AxisPositions& out) {
        if (cursor >= steps.size()) return false;
        out = steps[cursor++];
        return true;
    }
};

GoPlanner goPlanner;
std::atomic<bool> plannerReady{false};

bool AxisController::isRecording() const {
    return recording_.load();
}

void AxisController::setModeSave() {
    capture_mode_.store(static_cast<int>(CaptureMode::ManualSave));
    std::cout << "Capture mode switched to MANUAL SAVE." << std::endl;
}

void AxisController::setModeRecord() {
    capture_mode_.store(static_cast<int>(CaptureMode::AutoRecord));
    std::cout << "Capture mode switched to AUTO RECORD." << std::endl;
}
bool AxisController::isSaveMode() const {
    return capture_mode_.load() == static_cast<int>(CaptureMode::ManualSave);
}

void AxisController::setModeFile() {
    capture_mode_.store(static_cast<int>(CaptureMode::FileMode));
    std::cout << "Capture mode switched to FILE." << std::endl;
}

bool AxisController::isFileMode() const {
    return capture_mode_.load() == static_cast<int>(CaptureMode::FileMode);
}

std::string AxisController::modeName() const {
    return isSaveMode() ? "MANUAL SAVE"
         : isFileMode() ? "FILE"
         : "AUTO RECORD";
}

bool AxisController::readAxisStatuses(AxisStatuses& statuses) {
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        if (FAS_GetAxisStatus(nPortIDs_[i], iSlaveNos_[i], &(statuses[i].dwValue)) != FMM_OK) {
            return false;
        }
    }
    return true;
}

bool AxisController::readActualPositions(AxisPositions& positions) {
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        if (FAS_GetActualPos(nPortIDs_[i], iSlaveNos_[i], &positions[i]) != FMM_OK) {
            return false;
        }
    }
    return true;
}

AxisVelocities AxisController::computeVelocities(const AxisPositions& current,
                                                 const AxisPositions& targets,
                                                 const AxisBools& hasCommand) const {
    AxisVelocities velocities{};
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        velocities[i] = static_cast<int>(baseVelocityForAxis(i));
    }

    int maxDistance = 0;
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        if (!hasCommand[i]) continue;
        int distance = std::abs(targets[i] - current[i]);
        if (distance > maxDistance) maxDistance = distance;
    }

    if (maxDistance > 0) {
        for (size_t i = 0; i < AXIS_COUNT; ++i) {
            if (!hasCommand[i]) continue;
            int distance = std::abs(targets[i] - current[i]);
            if (distance > 0) {
                const unsigned int axisBaseVelocity = baseVelocityForAxis(i);
                auto scaled = static_cast<unsigned int>(
                    (static_cast<long long>(distance) * axisBaseVelocity) / maxDistance);
                velocities[i] = static_cast<int>(std::max(min_velocity, scaled));
            }
        }
    }

    return velocities;
}

unsigned int AxisController::baseVelocityForAxis(size_t axisIdx) const {
    const double ratio =
        (axisIdx < gear_ratios_.size() && gear_ratios_[axisIdx] > 0.0)
            ? gear_ratios_[axisIdx]
            : 1.0;
    const double configuredBase = base_velocity_override_.load() > 0.0
        ? base_velocity_override_.load()
        : static_cast<double>(base_velocity);
    const double velocity = std::round(configuredBase * ratio);
    if (velocity > static_cast<double>(std::numeric_limits<unsigned int>::max())) {
        return std::numeric_limits<unsigned int>::max();
    }
    return std::max(1u, static_cast<unsigned int>(velocity));
}

bool AxisController::allServoOn(const AxisStatuses& statuses) const {
    for (const auto& st : statuses) {
        if (!st.FFLAG_SERVOON) return false;
    }
    return true;
}

void AxisController::initializeSystem() {
    const wchar_t* sPort = L"ttyUSB0";
    unsigned int dwBaudRate = 115200;

    bool allOk = true;
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        if (!Driver_Connection(sPort, dwBaudRate, nPortIDs_[i], iSlaveNos_[i])) {
            allOk = false;
            std::cout << axisName(i) << " connection failed." << std::endl;
        }
    }

    if (!allOk) {
        std::cout << "One or more connections failed. GUI will still open."
                  << std::endl;
    }
}

bool AxisController::servoOn() {
    bool allOk = true;
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        bool ok = ServoOn(nPortIDs_[i], iSlaveNos_[i]);
        if (!ok) {
            allOk = false;
            std::cout << axisName(i) << " servo ON failed." << std::endl;
        }
    }

    if (allOk) {
        std::cout << "All " << AXIS_COUNT << " servos ON successfully." << std::endl;
    }
    return allOk;
}

bool AxisController::servoOff() {
    bool allOk = true;
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        bool ok = ServoOff(nPortIDs_[i], iSlaveNos_[i]);
        if (!ok) {
            allOk = false;
            std::cout << axisName(i) << " servo OFF failed." << std::endl;
        }
    }

    if (allOk) {
        std::cout << "All " << AXIS_COUNT << " servos OFF successfully." << std::endl;
    }
    return allOk;
}

bool AxisController::alarmReset() {
    bool allOk = true;
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        const int ret = FAS_ServoAlarmReset(nPortIDs_[i], iSlaveNos_[i]);
        if (ret != FMM_OK) {
            allOk = false;
            std::cout << axisName(i) << " alarm reset failed. Return code: "
                      << ret << std::endl;
        } else {
            std::cout << axisName(i) << " alarm reset command sent." << std::endl;
        }
        std::this_thread::sleep_for(kCommandSpacing);
    }

    if (allOk) {
        std::cout << "All " << AXIS_COUNT << " alarm reset commands sent successfully."
                  << std::endl;
    }
    return allOk;
}

AxisVectorsEx compressBuffer(
    const AxisVectorsEx& in,
    int posEps,
    int minTrendLen
) {
    AxisVectorsEx out;
    if (in[0].empty()) return out;

    size_t N = in[0].size();
    if (N < 2) return in;

    std::vector<bool> keep(N, false);
    keep[0] = true;

    for (size_t a = 0; a < AXIS_COUNT; ++a) {
        int lastDir = 0;
        int trendLen = 0;

        for (size_t i = 1; i < N; ++i) {
            int diff = in[a][i] - in[a][i - 1];
            if (std::abs(diff) < posEps)
                continue;

            int dir = (diff > 0) ? 1 : -1;

            if (dir == lastDir) {
                trendLen++;
            } else {
                if (trendLen >= minTrendLen) {
                    keep[i - 1] = true;
                }
                trendLen = 1;
                lastDir = dir;
            }
        }
        keep[N - 1] = true;
    }

    for (size_t i = 1; i < N; ++i) {
        if (in[AXIS_COUNT][i] != in[AXIS_COUNT][i - 1]) {
            keep[i] = true;
            keep[i - 1] = true;
        }
    }

    for (size_t i = 0; i < N; ++i) {
        if (!keep[i]) continue;
        for (size_t a = 0; a < EXT_AXIS_COUNT; ++a) {
            out[a].push_back(in[a][i]);
        }
    }

    return out;
}
//hi
//hi
bool AxisController::home() {
    AxisStatuses statuses{};
    if (!readAxisStatuses(statuses)) {
        std::cout << "Function(FAS_GetAxisStatus) failed." << std::endl;
        return false;
    }

    if (!allServoOn(statuses)) {
        std::cout << "Some servos are OFF. Turn them ON before homing." << std::endl;
        return false;
    }

    std::cout << "Sending homing commands to all motors..." << std::endl;
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        if (FAS_MoveSingleAxisAbsPos(nPortIDs_[i], iSlaveNos_[i], 0, baseVelocityForAxis(i)) != FMM_OK) {
            std::cout << axisName(i) << " homing command failed." << std::endl;
            return false;
        }
    }

    std::cout << "Waiting for all motors to complete homing..." << std::endl;
    AxisBools activeAxes{};
    activeAxes.fill(true);
    if (!waitForMotionComplete(nPortIDs_, iSlaveNos_, activeAxes, kHomingTimeout,
                               "Homing")) {
        return false;
    }

    AxisPositions finalPos{};
    if (readActualPositions(finalPos)) {
        std::cout << "Homing complete. Final positions: ";
        for (size_t i = 0; i < AXIS_COUNT; ++i) {
            std::cout << axisName(i) << "=" << finalPos[i];
            if (i + 1 < AXIS_COUNT) std::cout << ", ";
        }
        std::cout << std::endl;
    } else {
        std::cout << "Homing complete." << std::endl;
    }
    return true;
}

void AxisController::recordingThread() {
    constexpr int RECORD_PERIOD_MS = 10;
    int printCounter = 0;

    AxisPositions prevPos{};
    bool firstSample = true;

    while (recording_.load()) {
        AxisPositions pos{};
        AxisVelocities vel{};
        std::array<double, AXIS_COUNT> rpm_display{};
        bool inputSignal = false;

        // 1. Đọc vị trí thực tế
        if (!readActualPositions(pos) || !readInputSignal(inputSignal)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(RECORD_PERIOD_MS));
            continue;
        }

        // 2. Tính toán tốc độ thực tế
        if (firstSample) {
            for (size_t i = 0; i < AXIS_COUNT; ++i) {
                vel[i] = 0;
                rpm_display[i] = 0.0;
            }
            firstSample = false;
        } else {
            for (size_t i = 0; i < AXIS_COUNT; ++i) {
                long deltaPos = pos[i] - prevPos[i];
                double pps = std::abs(deltaPos) * (1000.0 / RECORD_PERIOD_MS);
                
                vel[i] = static_cast<unsigned int>(pps); 

                // Lớp bảo vệ chống chia cho 0 gây lỗi toán học (NaN/Inf)
                double current_ppr = (ppr_ > 0) ? ppr_ : 10000.0;
                double current_ratio = (gear_ratios_[i] > 0.0) ? gear_ratios_[i] : 1.0;

                double motor_rpm = (pps / current_ppr) * 60.0;
                rpm_display[i] = motor_rpm / current_ratio;
            }
        }

        prevPos = pos;

        // 3. In ra màn hình bằng C++ chuẩn (thay thế printf)
        printCounter++;
        if (printCounter >= 50) {
            std::cout << "[Recording] ";
            for (size_t i = 0; i < AXIS_COUNT; ++i) {
                std::cout << "M" << i+1 << "(Pos:" << pos[i] << ", RPM:";
                // Dùng setprecision thay cho printf("%.2f")
                std::cout << std::fixed << std::setprecision(2) << rpm_display[i] << ")  ";
            }
            std::cout << std::endl;
            printCounter = 0;
        }

        // 4. Lưu vào buffer
        {
            std::lock_guard<std::mutex> lk(rec_mtx_);
            for (size_t i = 0; i < AXIS_COUNT; ++i) {
                recorded_positions_[i].push_back(pos[i]);
                recorded_velocities_[i].push_back(vel[i]);
                // Nếu bạn có dùng mảng recorded_rpms_ thì bỏ comment dòng dưới:
                // recorded_rpms_[i].push_back(rpm_display[i]);
            }
            g_recordedRelayStates.push_back(inputSignal ? 1 : 0);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(RECORD_PERIOD_MS));
    }
}

void AxisController::record() {
    if (recording_.load()) {
        std::cout << "Already recording." << std::endl;
        return;
    }

    {
        std::lock_guard<std::mutex> lk(rec_mtx_);
        for (auto& v : recorded_positions_) {
            v.clear();
        }
        g_recordedRelayStates.clear();
        for (auto& v : recorded_velocities_) v.clear();
        for (auto& v : recorded_rpms_) v.clear();
    }

    recording_.store(true);
    rec_thread_ = std::thread(&AxisController::recordingThread, this);

    std::cout << "Recording started." << std::endl;
}

void AxisController::stopRecordingThread() {
    if (recording_.load()) {
        recording_.store(false);
    }
    if (rec_thread_.joinable()) rec_thread_.join();
}

void AxisController::stop() {
    if (!recording_.load()) {
        std::cout << "Not recording." << std::endl;
        return;
    }
    stopRecordingThread();

    std::lock_guard<std::mutex> lk(rec_mtx_);
    std::cout << "Recording stopped." << std::endl;

    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        std::cout << axisName(i) << " samples: " << recorded_positions_[i].size() << std::endl;
    }

    // 1. Lưu file Position (record_0.txt, record_1.txt...)
    // std::string filename = "record" + std::to_string(record_file_index_) + ".txt";
    // AxisVectors data = recorded_positions_;
    // if (writeBufferToFile(filename, data, false)) {
    //     std::cout << "Saved record to " << filename << std::endl;
    // }

    // 2. Lưu file Speed thô PPS (speed_0.txt, speed_1.txt...)
    // std::string speedFile = "speed_" + std::to_string(record_file_index_) + ".txt";
    // AxisVectors velData = recorded_velocities_;
    // if (writeBufferToFile(speedFile, velData, false)) {
    //     std::cout << "Saved speed to " << speedFile << std::endl;
    // }

    // ==========================================
    // 3. THÊM MỚI: LƯU FILE RPM (rpm_0.txt, ...)
    // ==========================================
    // std::string rpmFile = "rpm_" + std::to_string(record_file_index_) + ".txt";
    // if (writeRpmToFile(rpmFile, velData)) {
    //     std::cout << "Saved RPM to " << rpmFile << std::endl;
    // } else {
    //     std::cout << "Failed to save RPM file." << std::endl;
    // }

    // Tăng index cho lần record tiếp theo
    record_file_index_++;
}

void AxisController::clear() {
    std::lock_guard<std::mutex> lk(rec_mtx_);
    for (auto& path : recorded_positions_) {
        path.clear();
    }
    g_recordedRelayStates.clear();
    for (auto& v : recorded_rpms_) v.clear();
    for (auto& v : recorded_velocities_) v.clear();
    std::cout << "Cleared recorded positions for all motors." << std::endl;
}

bool AxisController::getPos(AxisPositions& pos) {
    return readActualPositions(pos);
}

bool AxisController::goWithBuffer(const AxisVectors& paths) {
    return goWithBuffer(makeExtendedPaths(paths, {}));
}

bool AxisController::goWithBuffer(const AxisVectorsEx& paths) {
    if (paths[0].empty()) return false;

    AxisStatuses statuses{};
    if (!readAxisStatuses(statuses) || !allServoOn(statuses)) return false;
    const AxisVectorsEx& runPaths = paths;
    size_t steps = runPaths[0].size();
    
    // --- BẬT MỎ HÀN ---

    const auto command_window =
        std::chrono::duration_cast<std::chrono::microseconds>(kCommandSpacing * AXIS_COUNT);
    const auto segment_period_us = g_segment_period;
    if (segment_period_us < command_window) {
        std::cout << "Segment period is shorter than command window. Requested "
                  << segment_period_us.count() << " us, minimum command window "
                  << command_window.count() << " us." << std::endl;
        return false;
    }
    std::cout << "Segment period: " << segment_period_us.count()
              << " us, command window: " << command_window.count() << " us." << std::endl;

    AxisPositions savedAccel{};
    AxisPositions savedDecel{};
    bool timingSaved =
        readAllMotionTiming(nPortIDs_, iSlaveNos_, kAccelerationTimeParameter, savedAccel, "accel time") &&
        readAllMotionTiming(nPortIDs_, iSlaveNos_, kDecelerationTimeParameter, savedDecel, "decel time");
    if (!setAllMotionTiming(nPortIDs_, iSlaveNos_, kAccelerationTimeParameter,
                            kStreamingAccelTimeMs, "streaming accel time") ||
        !setAllMotionTiming(nPortIDs_, iSlaveNos_, kDecelerationTimeParameter,
                            kStreamingDecelTimeMs, "streaming decel time")) {
        if (timingSaved) {
            restoreAllMotionTiming(nPortIDs_, iSlaveNos_, kAccelerationTimeParameter, savedAccel, "accel time");
            restoreAllMotionTiming(nPortIDs_, iSlaveNos_, kDecelerationTimeParameter, savedDecel, "decel time");
        }
        return false;
    }

    auto finishRun = [&](bool ok) {
        if (timingSaved) {
            restoreAllMotionTiming(nPortIDs_, iSlaveNos_, kAccelerationTimeParameter, savedAccel, "accel time");
            restoreAllMotionTiming(nPortIDs_, iSlaveNos_, kDecelerationTimeParameter, savedDecel, "decel time");
        }
        if (ok) {
            std::cout << "GO finished." << std::endl;
        }
        return ok;
    };

    AxisPositions previous{};
    if (!readActualPositions(previous)) {
        return finishRun(false);
    }

    AxisBools axisStreamingStarted{};
    axisStreamingStarted.fill(false);
    bool lastRelayState = false;
    bool firstBlock = true;
    std::cout << "Lookahead mode: slowing before sharp corners. Min corner scale: "
              << kMinCornerSpeedScale << std::endl;

    for (size_t i = 0; i < steps; ++i) {
        const bool relayState = (runPaths[AXIS_COUNT][i] != 0);
        if (firstBlock || relayState != lastRelayState) {
            std::cout << ">> [MOTION] Yeu cau IO thay doi -> "
                      << (relayState ? "ON" : "OFF") << std::endl;

            if (!setRelay(relayState)) {
                return finishRun(false);
            }
            lastRelayState = relayState;
            firstBlock = false;

            // std::cout << ">> [MOTION] Dang cho 500ms de Relay dong ngat an toan..." << std::endl;
            // std::this_thread::sleep_for(kRelaySettleDelay);
        }

        const auto segmentStart = std::chrono::steady_clock::now();
        auto nextCommandTime = segmentStart;
        AxisPositions target{};
        AxisPositions nextTarget{};
        AxisBools hasCommand{};

        for (size_t a = 0; a < AXIS_COUNT; ++a) {
            target[a] = runPaths[a][i];
            nextTarget[a] = (i + 1 < steps) ? runPaths[a][i + 1] : target[a];
            hasCommand[a] = (target[a] != previous[a]);
        }

        const double speedScale =
            (i + 1 < steps) ? cornerSpeedScale(previous, target, nextTarget) : 1.0;
        const auto active_segment_period_us =
            scaledSegmentPeriod(segment_period_us, speedScale);
        AxisVelocities velocities =
            computeSegmentVelocities(previous, target, active_segment_period_us);

        for (size_t a = 0; a < AXIS_COUNT; ++a) {
            if (!hasCommand[a]) {
                continue;
            }
            const auto now = std::chrono::steady_clock::now();
            if (now < nextCommandTime) {
                std::this_thread::sleep_until(nextCommandTime);
            }
#ifdef AXIS_CONTROLLER_OVERRIDE_MODE
            if (!axisStreamingStarted[a]) {
                const int startRet = FAS_MoveSingleAxisAbsPos(
                    nPortIDs_[a],
                    iSlaveNos_[a],
                    target[a],
                    velocities[a]);
                if (startRet == FMM_OK) {
                    axisStreamingStarted[a] = true;
                    nextCommandTime += kCommandSpacing;
                    continue;
                }
                if (startRet != FMP_RUNFAIL) {
                    std::cout << axisName(a) << " initial streaming move failed: "
                              << startRet << std::endl;
                    return finishRun(false);
                }
                axisStreamingStarted[a] = true;
            }

            const int velRet = FAS_VelocityOverride(nPortIDs_[a], iSlaveNos_[a], velocities[a]);
            const int posRet = FAS_PositionAbsOverride(nPortIDs_[a], iSlaveNos_[a], target[a]);
            if (velRet != FMM_OK || posRet != FMM_OK) {
                const int startRet = FAS_MoveSingleAxisAbsPos(
                    nPortIDs_[a],
                    iSlaveNos_[a],
                    target[a],
                    velocities[a]);
                if (startRet != FMM_OK && startRet != FMP_RUNFAIL) {
                    std::cout << axisName(a)
                              << " override failed. vel=" << velRet
                              << ", pos=" << posRet
                              << ", restart=" << startRet << std::endl;
                    return finishRun(false);
                }
            }
            nextCommandTime += kCommandSpacing;
#else
            int ret = FAS_MoveSingleAxisAbsPos(
                nPortIDs_[a],
                iSlaveNos_[a],
                target[a],
                velocities[a]);
            if (ret == FMP_RUNFAIL) {
                const int velRet = FAS_VelocityOverride(nPortIDs_[a], iSlaveNos_[a], velocities[a]);
                const int posRet = FAS_PositionAbsOverride(nPortIDs_[a], iSlaveNos_[a], target[a]);
                if (velRet != FMM_OK || posRet != FMM_OK) {
                    std::cout << axisName(a)
                              << " streaming override failed. vel=" << velRet
                              << ", pos=" << posRet << std::endl;
                    return finishRun(false);
                }
                nextCommandTime += kCommandSpacing;
                continue;
            }
            if (ret != FMM_OK) {
                std::cout << axisName(a) << " streaming move failed: " << ret << std::endl;
                return finishRun(false);
            }
            nextCommandTime += kCommandSpacing;
#endif
        }

        previous = target;
        if (i + 1 < steps) {
            const auto nextSegmentTime = segmentStart + active_segment_period_us;
            const auto now = std::chrono::steady_clock::now();
            if (now < nextSegmentTime) {
                std::this_thread::sleep_until(nextSegmentTime);
            }
        }
    }

    AxisBools activeAxes{};
    activeAxes.fill(true);
    if (!waitForMotionComplete(nPortIDs_, iSlaveNos_, activeAxes, kFinalSettleTimeout,
                               "GO final settle")) {
        return finishRun(false);
    }

    // --- TẮT MỎ HÀN ---
    return finishRun(true);
}

bool AxisController::goFromFile(const std::string& filename)
{
    std::ifstream file(filename);

    if (!file.is_open()) {
        std::cout << "Cannot open file: " << filename << std::endl;
        return false;
    }

    // Lưu tạm vào buffer theo dạng mảng các dòng (array of rows)
    std::vector<AxisPositions> pathBuffer;
    std::vector<int> relayBuffer;
    std::string line;
    double fileSegmentSeconds = -1.0;
    double fileMoveTimeSeconds = -1.0;

    while (std::getline(file, line))
    {
        if (line.empty()) continue;
        const auto firstNonSpace = line.find_first_not_of(" \t\r\n");
        if (firstNonSpace == std::string::npos) continue;
        const char first = line[firstNonSpace];
        if (first == '#' || first == ';') {
            readTimingHeader(line, fileSegmentSeconds, fileMoveTimeSeconds);
            continue;
        }

        std::stringstream ss(line);
        std::vector<int> row;
        int val;

        while (ss >> val) {
            row.push_back(val);
        }

        if (row.size() < AXIS_COUNT) {
            std::cout << "Format error at line: " << line << "\n";
            return false;
        }

        AxisPositions pos{};

        for (size_t i = 0; i < AXIS_COUNT; i++)
        {
            pos[i] = row[i];
        }
        
        // Lưu vào mảng 1 chiều chứa các dòng
        int relayState = 0;
        if (row.size() > FILE_RELAY_COLUMN) {
            relayState = row[FILE_RELAY_COLUMN];
        }
        relayBuffer.push_back(relayState);
        pathBuffer.push_back(pos);
    }

    if (fileSegmentSeconds > 0.0) {
        setSegmentPeriodSeconds(fileSegmentSeconds);
        std::cout << "Segment timing from file header: "
                  << fileSegmentSeconds << " s/step." << std::endl;
    } else if (fileMoveTimeSeconds > 0.0 && pathBuffer.size() > 1) {
        const double segmentSeconds =
            fileMoveTimeSeconds / static_cast<double>(pathBuffer.size() - 1);
        setSegmentPeriodSeconds(segmentSeconds);
        std::cout << "Segment timing from moveTimeValue/Total_Index: "
                  << segmentSeconds << " s/step." << std::endl;
    } else {
        setSegmentPeriodSeconds(kDefaultSegmentSeconds);
        std::cout << "No segment timing header. Using default "
                  << kDefaultSegmentSeconds << " s/step." << std::endl;
    }

    // --- PHẦN BẠN ĐANG THIẾU: Chuyển đổi và gọi hàm ---
    AxisVectorsEx compatiblePaths;
    for (const auto& row : pathBuffer) {
        for (size_t i = 0; i < AXIS_COUNT; i++) {
            compatiblePaths[i].push_back(row[i]);
        }
    }
    compatiblePaths[AXIS_COUNT] = relayBuffer;
    std::string rpmFile = "rpm_" + std::to_string(record_file_index_) + ".txt";

    //saveRpmFromPositions(rpmFile, compatiblePaths); // Hàm này sẽ tính toán RPM từ positions và lưu vào file

    std::cout << "Saved RPM to " << rpmFile << std::endl;

    record_file_index_++;
    // Truyền dữ liệu đã chuyển đổi vào hàm thực thi
    return goWithBuffer(compatiblePaths); 
}

void AxisController::setFileName(const std::string& name)
{
    filename_ = name;
}

std::string AxisController::FileName() const
{
    return filename_;
}

void AxisController::setBaseVelocityOverride(double velocity)
{
    base_velocity_override_.store(velocity > 0.0 ? velocity : -1.0);
}

bool AxisController::go() {
    auto mode = capture_mode_.load();
    if (mode == static_cast<int>(CaptureMode::FileMode)) {
        if (filename_.empty()) {
            std::cout << "File path is empty." << std::endl;
            return false;
        }
        return goFromFile(filename_);
    }
    else if (mode == static_cast<int>(CaptureMode::ManualSave)) {
        AxisVectorsEx paths;
        {
            std::lock_guard<std::mutex> lk(rec_mtx_);
            paths = makeExtendedPaths(saved_positions_, g_savedRelayStates);
        }
        
        if (paths[0].empty()) {
            std::cout << "No manually saved positions to run." << std::endl;
            return false;
        }

        std::cout << "Executing Manual Save positions..." << std::endl;
        return goWithBuffer(paths); 
    }
    AxisVectorsEx paths;
    {
        std::lock_guard<std::mutex> lk(rec_mtx_);
        paths = makeExtendedPaths(recorded_positions_, g_recordedRelayStates);
    }
    return goWithBuffer(paths);
}

bool AxisController::movePos(const AxisPositions& targets) {
    AxisStatuses statuses{};
    if (!readAxisStatuses(statuses)) {
        std::cout << "Function(FAS_GetAxisStatus) failed." << std::endl;
        return false;
    }

    if (!allServoOn(statuses)) {
        std::cout << "Some servos are OFF. Turn them ON before moving." << std::endl;
        return false;
    }

    AxisPositions current{};
    if (!readActualPositions(current)) {
        std::cout << "Failed to get current positions." << std::endl;
        return false;
    }

    AxisBools hasCommand{};
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        hasCommand[i] = (targets[i] != current[i]);
    }

    AxisVelocities velocities = computeVelocities(current, targets, hasCommand);

    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        if (!hasCommand[i]) continue;
        if (FAS_MoveSingleAxisAbsPos(nPortIDs_[i], iSlaveNos_[i], targets[i], velocities[i]) !=
            FMM_OK) {
            std::cout << axisName(i) << " move command failed." << std::endl;
            return false;
        }
    }

    std::cout << "Waiting for motors to complete movement..." << std::endl;
    const auto timeout = estimateMoveTimeout(current, targets, velocities, hasCommand);
    if (!waitForMotionComplete(nPortIDs_, iSlaveNos_, hasCommand, timeout, "Move")) {
        return false;
    }

    AxisPositions finalPos{};
    if (readActualPositions(finalPos)) {
        std::cout << "Movement complete. Final positions: ";
        for (size_t i = 0; i < AXIS_COUNT; ++i) {
            std::cout << axisName(i) << "=" << finalPos[i];
            if (i + 1 < AXIS_COUNT) std::cout << ", ";
        }
        std::cout << std::endl;
    } else {
        std::cout << "Movement complete." << std::endl;
    }
    return true;
}

bool AxisController::goPos() {
    AxisStatuses statuses{};
    if (!readAxisStatuses(statuses)) {
        std::cout << "Function(FAS_GetAxisStatus) failed." << std::endl;
        return false;
    }

    if (!allServoOn(statuses)) {
        std::cout << "Some servos are OFF. Turn them ON before moving." << std::endl;
        return false;
    }

    constexpr unsigned short maxItems = 64;
    std::array<std::vector<ITEM_NODE>, AXIS_COUNT> tableItems;

    for (size_t axis = 0; axis < AXIS_COUNT; ++axis) {
        for (unsigned short wItemNo = 1; wItemNo <= maxItems; ++wItemNo) {
            ITEM_NODE nodeItem;
            if (FAS_PosTableReadItem(nPortIDs_[axis], iSlaveNos_[axis], wItemNo, &nodeItem) !=
                FMM_OK) {
                break;
            }
            bool likelyEmpty = (nodeItem.lPosition == 0 && nodeItem.dwMoveSpd == 0);
            if (!likelyEmpty) {
                tableItems[axis].push_back(nodeItem);
            }
        }
    }

    bool anyItems = false;
    for (const auto& list : tableItems) {
        if (!list.empty()) {
            anyItems = true;
            break;
        }
    }
    if (!anyItems) {
        std::cout << "Position tables have no items to run." << std::endl;
        return false;
    }

    size_t maxSteps = 0;
    for (const auto& list : tableItems) {
        if (list.size() > maxSteps) maxSteps = list.size();
    }

    for (size_t idx = 0; idx < maxSteps; ++idx) {
        AxisBools hasCommand{};
        AxisPositions targets{};
        AxisVelocities velocities{};
        for (size_t axis = 0; axis < AXIS_COUNT; ++axis) {
            velocities[axis] = static_cast<int>(baseVelocityForAxis(axis));
        }

        for (size_t axis = 0; axis < AXIS_COUNT; ++axis) {
            if (idx < tableItems[axis].size()) {
                const ITEM_NODE& node = tableItems[axis][idx];
                targets[axis] = node.lPosition;
                velocities[axis] = (node.dwMoveSpd > 0)
                    ? node.dwMoveSpd
                    : static_cast<int>(baseVelocityForAxis(axis));
                hasCommand[axis] = true;
            }
        }

        AxisPositions current{};
        if (!readActualPositions(current)) {
            std::cout << "Failed to get current positions." << std::endl;
            return false;
        }

        for (size_t axis = 0; axis < AXIS_COUNT; ++axis) {
            if (!hasCommand[axis]) continue;
            if (FAS_MoveSingleAxisAbsPos(nPortIDs_[axis], iSlaveNos_[axis], targets[axis],
                                         velocities[axis]) != FMM_OK) {
                std::cout << axisName(axis) << " move to " << targets[axis] << " failed."
                          << std::endl;
                return false;
            }
        }

        const auto timeout = estimateMoveTimeout(current, targets, velocities, hasCommand);
        if (!waitForMotionComplete(nPortIDs_, iSlaveNos_, hasCommand, timeout, "Go pos")) {
            return false;
        }
    }

    AxisPositions actPos{};
    if (readActualPositions(actPos)) {
        std::cout << "Go pos complete. Final positions - ";
        for (size_t i = 0; i < AXIS_COUNT; ++i) {
            std::cout << axisName(i) << ": " << actPos[i];
            if (i + 1 < AXIS_COUNT) std::cout << ", ";
        }
        std::cout << std::endl;
    } else {
        std::cout << "Go pos complete." << std::endl;
    }
    return true;
}

bool AxisController::savePos() {
    if (!isSaveMode()) {
        std::cout << "Current mode is AUTO RECORD. Switch mode to MANUAL SAVE first."
                  << std::endl;
        return false;
    }
    AxisPositions pos{};
    if (!readActualPositions(pos)) {
        std::cout << "Failed to read current positions. Save pos aborted." << std::endl;
        return false;
    }

    bool inputSignal = false;
    readInputSignal(inputSignal);

    std::lock_guard<std::mutex> lk(rec_mtx_);
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        saved_positions_[i].push_back(pos[i]);
    }
    g_savedRelayStates.push_back(inputSignal ? 1 : 0);

    std::cout << "Saved current position to manual buffer. Total manual points: "
              << saved_positions_[0].size() << std::endl;
    std::cout << "Saved - ";
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        std::cout << axisName(i) << ": " << pos[i];
        if (i + 1 < AXIS_COUNT) std::cout << ", ";
    }
    std::cout << std::endl;
    // --- APPEND TO savePos.txt ---
    AxisVectorsEx temp;

    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        temp[i].push_back(pos[i]);
    }
    // chỉ ghi 1 điểm (pos hiện tại)
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        temp[i].push_back(pos[i]);
    }

    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        temp[i].resize(1);
    }
    temp[AXIS_COUNT].push_back(inputSignal ? 1 : 0);

    if (writeBufferToFile("savePos.txt", temp, true)) {
        std::cout << "Saved to savePos.txt" << std::endl;
    } else {
        std::cout << "Failed to write savePos.txt" << std::endl;
    }
    return true;
}

void AxisController::posTable() {
    AxisVectors paths;
    {
        std::lock_guard<std::mutex> lk(rec_mtx_);
        paths = recorded_positions_;
    }

    bool anyData = false;
    for (const auto& p : paths) {
        if (!p.empty()) {
            anyData = true;
            break;
        }
    }

    if (!anyData) {
        std::cout << "No recorded positions to write. Use 'record' first." << std::endl;
        return;
    }

    constexpr unsigned short startItem = 1;
    std::array<unsigned int, AXIS_COUNT> wrote{};

    for (size_t axis = 0; axis < AXIS_COUNT; ++axis) {
        for (size_t i = 0; i < paths[axis].size(); ++i) {
            unsigned short wItemNo = static_cast<unsigned short>(startItem + i);
            ITEM_NODE nodeItem;
            if (FAS_PosTableReadItem(nPortIDs_[axis], iSlaveNos_[axis], wItemNo, &nodeItem) !=
                FMM_OK) {
                std::cout << axisName(axis) << " FAS_PosTableReadItem failed at item " << wItemNo
                          << "." << std::endl;
                break;
            }

            nodeItem.dwMoveSpd = 1000;
            nodeItem.lPosition = paths[axis][i];
            nodeItem.wBranch = 0;
            nodeItem.wContinuous = 0;

            if (FAS_PosTableWriteItem(nPortIDs_[axis], iSlaveNos_[axis], wItemNo, &nodeItem) !=
                FMM_OK) {
                std::cout << axisName(axis) << " FAS_PosTableWriteItem failed at item " << wItemNo
                          << "." << std::endl;
                break;
            }
            ++wrote[axis];
        }
    }

    std::cout << "Position tables updated." << std::endl;
    for (size_t axis = 0; axis < AXIS_COUNT; ++axis) {
        std::cout << axisName(axis) << " items: " << wrote[axis] << std::endl;
    }
}

void AxisController::printTable() {
    constexpr unsigned short maxItems = 64;

    for (size_t axis = 0; axis < AXIS_COUNT; ++axis) {
        std::cout << "\n=== " << axisName(axis) << " Position Table ===" << std::endl;
        unsigned int shown = 0;
        for (unsigned short wItemNo = 1; wItemNo <= maxItems; ++wItemNo) {
            ITEM_NODE nodeItem;
            if (FAS_PosTableReadItem(nPortIDs_[axis], iSlaveNos_[axis], wItemNo, &nodeItem) !=
                FMM_OK) {
                std::cout << axisName(axis) << " FAS_PosTableReadItem failed at item " << wItemNo
                          << "." << std::endl;
                break;
            }

            bool likelyEmpty = (nodeItem.lPosition == 0 && nodeItem.dwMoveSpd == 0);
            if (likelyEmpty) continue;

            std::cout << axisName(axis) << " Item " << wItemNo << ": Pos=" << nodeItem.lPosition
                      << ", MoveSpd=" << nodeItem.dwMoveSpd << ", Cmd=" << nodeItem.wCommand
                      << ", Wait=" << nodeItem.wWaitTime
                      << ", Cont=" << nodeItem.wContinuous
                      << ", Branch=" << nodeItem.wBranch << std::endl;
            ++shown;
        }
        if (shown == 0) {
            std::cout << "No non-empty items found for " << axisName(axis) << "." << std::endl;
        }
    }
}

void AxisController::setOriginPos() {
    AxisStatuses statuses{};
    if (!readAxisStatuses(statuses)) {
        std::cout << "Function(FAS_GetAxisStatus) failed." << std::endl;
        return;
    }

    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        if (statuses[i].FFLAG_MOTIONING) {
            std::cout << axisName(i) << " is moving. Stop motion before setpos." << std::endl;
            return;
        }
    }
//hihi
    bool allOk = true;
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        if (FAS_SetCommandPos(nPortIDs_[i], iSlaveNos_[i], 0) != FMM_OK) {
            std::cout << axisName(i) << " set command position failed." << std::endl;
            allOk = false;
        }
        if (FAS_SetActualPos(nPortIDs_[i], iSlaveNos_[i], 0) != FMM_OK) {
            std::cout << axisName(i) << " set actual position failed." << std::endl;
            allOk = false;
        }
    }

    if (allOk) {
        std::cout << "All axes positions set to 0." << std::endl;
    }
}

bool AxisController::loadFileToBuffer(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cout << "Cannot open file: " << filename << std::endl;
        return false;
    }

    AxisVectors temp;
    for (auto& v : temp) v.clear();

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) continue;

        std::stringstream ss(line);
        AxisPositions pos{};

        for (size_t i = 0; i < AXIS_COUNT; ++i) {
            if (!(ss >> pos[i])) {
                std::cout << "Format error." << std::endl;
                return false;
            }
        }

        for (size_t i = 0; i < AXIS_COUNT; ++i) {
            temp[i].push_back(pos[i]);
        }
    }

    file_buffer_ = temp;

    std::cout << "File buffer loaded. Steps: "
              << file_buffer_[0].size() << std::endl;

    return true;
}


bool AxisController::setOutputSignal(size_t axisIdx, uint32_t outMask, bool state) {
    if (axisIdx >= AXIS_COUNT) return false;
    
    // Thay thế DWORD bằng uint32_t cho môi trường Linux
    uint32_t dwSetMask = state ? outMask : 0;
    uint32_t dwClearMask = state ? 0 : outMask;
    
    return FAS_SetIOOutput(nPortIDs_[axisIdx], iSlaveNos_[axisIdx], dwSetMask, dwClearMask) == FMM_OK;
}

bool AxisController::isInputActive(size_t axisIdx, uint32_t inMask, bool& active) {
    if (axisIdx >= AXIS_COUNT) return false;
    
    // Thay thế DWORD bằng uint32_t
    uint32_t dwInput = 0;
    
    if (FAS_GetIOInput(nPortIDs_[axisIdx], iSlaveNos_[axisIdx], &dwInput) == FMM_OK) {
        active = (dwInput & inMask) != 0;
        return true;
    }
    return false;
}

bool AxisController::readInputSignal(bool& signal)
{
    uint32_t input = 0;
    if (FAS_GetIOInput(nPortIDs_[0], iSlaveNos_[0], &input) != FMM_OK)
        return false;

    uint32_t bitShift = 26 + DI_INDEX;
    signal = (input & (1u << bitShift)) != 0;
    return true;
}

bool AxisController::setRelay(bool on)
{
    uint32_t bitShift = 15 + DO_INDEX;
    uint32_t mask = (1u << bitShift);

    uint32_t setMask = on ? mask : 0;
    uint32_t clearMask = on ? 0 : mask;

    int res = FAS_SetIOOutput(
        nPortIDs_[0],
        iSlaveNos_[0],
        setMask,
        clearMask
    );

    std::cout << "[IO CONTROL] Driver 0 | Lenh gui di: " << (on ? "ON" : "OFF")
              << " (Mask 0x" << std::hex << mask << std::dec
              << ") | Phan hoi (0=OK): " << res << std::endl;

    return res == FMM_OK;
}

void AxisController::endstopThreadFunc(size_t triggerAxis, uint32_t endstopMask) {
    bool lastState = false;
    while (monitoring_endstop_.load()) {
        bool currentState = false;
        if (isInputActive(triggerAxis, endstopMask, currentState)) {
            if (currentState && !lastState) { // Bắt cạnh lên (vừa nhấn)
                std::cout << "[Endstop] Triggered! Auto-saving position..." << std::endl;
                savePos(); // Gọi hàm lưu vị trí vào saved_positions_
            }
            lastState = currentState;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void AxisController::startEndstopMonitor(size_t triggerAxis, uint32_t endstopMask) {
    if (monitoring_endstop_.load()) return;
    monitoring_endstop_.store(true);
    endstop_thread_ = std::thread(&AxisController::endstopThreadFunc, this, triggerAxis, endstopMask);
    std::cout << "Endstop Monitor started on Axis " << triggerAxis + 1 << std::endl;
}

void AxisController::stopEndstopMonitor() {
    if (monitoring_endstop_.load()) {
        monitoring_endstop_.store(false);
        if (endstop_thread_.joinable()) endstop_thread_.join();
    }
}

bool AxisController::writeBufferToFile(const std::string& filename,
                                       const AxisVectors& data,
                                       bool append)
{
    std::ofstream file;

    if (append)
        file.open(filename, std::ios::app);
    else
        file.open(filename);

    if (!file.is_open()) {
        std::cout << "Cannot open file: " << filename << std::endl;
        return false;
    }

    size_t steps = data[0].size();

    for (size_t i = 0; i < steps; ++i) {
        for (size_t a = 0; a < AXIS_COUNT; ++a) {
            file << data[a][i];
            if (a + 1 < AXIS_COUNT) file << " ";
        }
        file << "\n";
    }

    file.close();
    return true;
}

bool AxisController::writeBufferToFile(const std::string& filename,
                                       const AxisVectorsEx& data,
                                       bool append)
{
    std::ofstream file;

    if (append)
        file.open(filename, std::ios::app);
    else
        file.open(filename);

    if (!file.is_open()) {
        std::cout << "Cannot open file: " << filename << std::endl;
        return false;
    }

    size_t steps = data[0].size();

    for (size_t i = 0; i < steps; ++i) {
        for (size_t a = 0; a < EXT_AXIS_COUNT; ++a) {
            file << data[a][i];
            if (a + 1 < EXT_AXIS_COUNT) file << " ";
        }
        file << "\n";
    }

    file.close();
    return true;
}

/*
void AxisController::saveRpmFromPositions(const std::string& filename, const AxisVectors& positions) {
    if (positions.empty() || positions[0].size() < 2) return;

    std::ofstream file(filename);
    if (!file.is_open()) return;

    size_t steps = positions[0].size();
    // Giả định thời gian giữa các bước (sample rate) là RECORD_PERIOD_MS (10ms)
    // Nếu trong goWithBuffer bạn dùng delay khác, bạn có thể điều chỉnh hằng số này
    constexpr double dt = 0.01; // 10ms = 0.01s

    for (size_t i = 1; i < steps; ++i) {
        for (size_t a = 0; a < AXIS_COUNT; ++a) {
            long deltaPos = positions[a][i] - positions[a][i-1];
            double pps = std::abs(deltaPos) / dt;
            
            double current_ppr = (ppr_ > 0) ? ppr_ : 10000.0;
            double current_ratio = (gear_ratios_[a] > 0.0) ? gear_ratios_[a] : 1.0;
            
            double rpm = (pps / current_ppr) * 60.0 / current_ratio;
            
            file << std::fixed << std::setprecision(2) << rpm;
            if (a + 1 < AXIS_COUNT) file << " ";
        }
        file << "\n";
    }
    file.close();
    std::cout << "RPM log saved to: " << filename << std::endl;
}


bool AxisController::writeRpmToFile(const std::string& filename, const AxisVectors& ppsData) {
    std::ofstream file(filename);
    if (!file.is_open()) return false;
    if (ppsData.empty() || ppsData[0].empty()) return false;

    size_t steps = ppsData[0].size();
    for (size_t i = 0; i < steps; ++i) {
        for (size_t a = 0; a < AXIS_COUNT; ++a) {
            // Lớp bảo vệ chống chia cho 0
            double current_ppr = (ppr_ > 0) ? ppr_ : 10000.0;
            double current_ratio = (gear_ratios_[a] > 0.0) ? gear_ratios_[a] : 1.0;
            
            // Tính RPM từ PPS
            double rpm = (static_cast<double>(ppsData[a][i]) / current_ppr) * 60.0 / current_ratio;
            
            file << std::fixed << std::setprecision(2) << rpm;
            if (a + 1 < AXIS_COUNT) file << " ";
        }
        file << "\n";
    }
    file.close();
    return true;
}
// void AxisController::testAllOutputs() {
//     std::cout << "=== TEST ALL OUTPUTS ===" << std::endl;

//     for (size_t axis = 0; axis < AXIS_COUNT; ++axis) {
//         std::cout << "Axis " << axis + 1 << std::endl;

//         for (int bit = 0; bit < 16; ++bit) {
//             uint32_t mask = (1 << bit);

//             std::cout << "  -> OUT bit " << bit << " ON" << std::endl;
//             setOutputSignal(axis, mask, true);
//             std::this_thread::sleep_for(std::chrono::milliseconds(300));

//             std::cout << "  -> OFF" << std::endl;
//             setOutputSignal(axis, mask, false);
//             std::this_thread::sleep_for(std::chrono::milliseconds(100));
//         }
//     }

//     std::cout << "=== DONE OUTPUT TEST ===" << std::endl;
// }

// void AxisController::monitorAllInputs() {
//     std::cout << "=== MONITOR INPUTS ===" << std::endl;

//     while (true) {
//         for (size_t axis = 0; axis < AXIS_COUNT; ++axis) {
//             uint32_t dwInput = 0;

//             if (FAS_GetIOInput(nPortIDs_[axis], iSlaveNos_[axis], &dwInput) == FMM_OK) {
//                 std::cout << "Axis " << axis + 1 << " Input: ";

//                 for (int bit = 0; bit < 16; ++bit) {
//                     if (dwInput & (1 << bit)) {
//                         std::cout << "[IN" << bit << "=1] ";
//                     }
//                 }
//                 std::cout << std::endl;
//             }
//         }

//         std::this_thread::sleep_for(std::chrono::milliseconds(200));
//     }
// }

// void AxisController::inputToOutputTest() {
//     std::cout << "=== INPUT -> ALL OUTPUT TEST ===" << std::endl;

//     while (true) {
//         for (size_t axis = 0; axis < AXIS_COUNT; ++axis) {
//             uint32_t input = 0;

//             if (FAS_GetIOInput(nPortIDs_[axis], iSlaveNos_[axis], &input) != FMM_OK)
//                 continue;

//             if (input != 0) {
//                 // Nếu có bất kỳ input nào ON → bật toàn bộ output
//                 FAS_SetIOOutput(nPortIDs_[axis], iSlaveNos_[axis], 0xFFFF, 0x0000);
//                 std::cout << "Axis " << axis + 1 << ": INPUT DETECTED -> ALL OUTPUT ON" << std::endl;
//             } else {
//                 // Không có input → tắt toàn bộ
//                 FAS_SetIOOutput(nPortIDs_[axis], iSlaveNos_[axis], 0x0000, 0xFFFF);
//                 std::cout << "Axis " << axis + 1 << ": NO INPUT -> ALL OUTPUT OFF" << std::endl;
//             }
//         }

//         std::this_thread::sleep_for(std::chrono::milliseconds(50));
//     }
// }
*/
