#include "axis_controller.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <thread>
#include <mutex>
#include <atomic>
#include <array>
#include <vector>

// ===== GRBL-style planner buffer =====
constexpr size_t PLANNER_BUFFER_SIZE = 16;
std::atomic<bool> planner_done{false};
struct PlannerBlock {
    AxisPositions target;
    AxisVelocities velocity;
    bool relay_state;
};

struct PlannerBuffer {
    std::mutex mtx;
    std::array<PlannerBlock, PLANNER_BUFFER_SIZE> buffer;
    size_t head = 0;
    size_t tail = 0;

    bool isFullUnsafe() const {
        return ((head + 1) % PLANNER_BUFFER_SIZE) == tail;
    }

    bool isEmptyUnsafe() const {
        return head == tail;
    }

    bool push(const PlannerBlock& b) {
        std::lock_guard<std::mutex> lk(mtx);
        if (isFullUnsafe()) return false;
        buffer[head] = b;
        head = (head + 1) % PLANNER_BUFFER_SIZE;
        return true;
    }

    bool pop(PlannerBlock& b) {
        std::lock_guard<std::mutex> lk(mtx);
        if (isEmptyUnsafe()) return false;
        b = buffer[tail];
        tail = (tail + 1) % PLANNER_BUFFER_SIZE;
        return true;
    }

    bool isEmpty() {
        std::lock_guard<std::mutex> lk(mtx);
        return isEmptyUnsafe();
    }
};

static PlannerBuffer planner;
static std::thread motion_thread;
static std::atomic<bool> motion_running{false};


AxisController::AxisController() {
    int portDefaults[6] = {0, 0, 0, 0, 0, 0};
    unsigned char slaveDefaults[6] = {0, 1, 2, 3, 4, 5};
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        nPortIDs_[i] = portDefaults[i];
        iSlaveNos_[i] = slaveDefaults[i];
    }
}

AxisController::~AxisController() {
    stopRecordingThread();

    motion_running.store(false);
    if (motion_thread.joinable()) motion_thread.join();

    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        FAS_Close(nPortIDs_[i]);
    }
}

constexpr size_t MAX_BUFFERS = 8;
// AxisVectorsEx recorded_positions;
std::mutex rec_mtx;
// std::atomic<bool> recording{false};
std::thread rec_thread;

std::string AxisController::axisName(size_t idx) const {
    return "Motor " + std::to_string(idx + 1);
}

using RecordBuffers = std::array<AxisVectorsEx, MAX_BUFFERS>;

RecordBuffers recorded_positions_buffers;

std::atomic<size_t> currentRecordingBuffer{MAX_BUFFERS - 1};
std::atomic<size_t> bufferCount{0};

AxisVectorsEx downsampleBuffer(
    const AxisVectorsEx& in,
    size_t step
) {
    AxisVectorsEx out;
    if (in[0].empty()) return out;

    size_t len = in[0].size();
    for (size_t i = 0; i < len; i += step) {
        // 👇 FIX: Đổi thành EXT_AXIS_COUNT
        for (size_t a = 0; a < EXT_AXIS_COUNT; ++a) {
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
    velocities.fill(base_velocity);

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
                auto scaled = static_cast<unsigned int>(
                    (static_cast<long long>(distance) * base_velocity) / maxDistance);
                velocities[i] = std::max(min_velocity, scaled);
            }
        }
    }

    return velocities;
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

AxisVectorsEx compressBuffer(
    const AxisVectorsEx& in,
    int posEps,
    int minTrendLen
) {
    AxisVectorsEx out;
    if (in[0].empty()) return out; // Thêm check an toàn
    
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

    // Luôn giữ lại các điểm mà tín hiệu I/O thay đổi trạng thái
    for (size_t i = 1; i < N; ++i) {
        if (in[AXIS_COUNT][i] != in[AXIS_COUNT][i - 1]) {
            keep[i] = true;
            keep[i - 1] = true; 
        }
    }

    for (size_t i = 0; i < N; ++i) {
        if (!keep[i]) continue;
        // 👇 FIX QUAN TRỌNG: Phải lặp đến EXT_AXIS_COUNT
        for (size_t a = 0; a < EXT_AXIS_COUNT; ++a) { 
            out[a].push_back(in[a][i]);
        }
    }

    return out;
}

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
        if (FAS_MoveSingleAxisAbsPos(nPortIDs_[i], iSlaveNos_[i], 0, base_velocity) != FMM_OK) {
            std::cout << axisName(i) << " homing command failed." << std::endl;
            return false;
        }
    }

    AxisBools done{};
    std::cout << "Waiting for all motors to complete homing..." << std::endl;
    while (true) {
        bool allDone = true;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        for (size_t i = 0; i < AXIS_COUNT; ++i) {
            if (done[i]) continue;
            if (FAS_GetAxisStatus(nPortIDs_[i], iSlaveNos_[i], &(statuses[i].dwValue)) ==
                FMM_OK) {
                done[i] = !statuses[i].FFLAG_MOTIONING;
                if (done[i]) {
                    std::cout << axisName(i) << " homing complete." << std::endl;
                }
            }
            allDone = allDone && done[i];
        }
        if (allDone) break;
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
    }

    recording_.store(true);
    rec_thread_ = std::thread(&AxisController::recordingThread, this);

    std::cout << "Recording started." << std::endl;
}

void AxisController::recordingThread() {
    constexpr int RECORD_PERIOD_MS = 10;
    while (recording_.load()) {
        AxisPositions pos{};
        bool inputSignal = false; // <-- Thêm

        // 👇 Cập nhật điều kiện đọc
        if (!readActualPositions(pos) || !readInputSignal(inputSignal)) { 
            std::this_thread::sleep_for(std::chrono::milliseconds(RECORD_PERIOD_MS));
            continue;
        }

        {
            std::lock_guard<std::mutex> lk(rec_mtx_);
            for (size_t i = 0; i < AXIS_COUNT; ++i) {
                recorded_positions_[i].push_back(pos[i]);
            }
            // 👇 Lưu control bit vào mảng
            recorded_positions_[AXIS_COUNT].push_back(inputSignal ? 1 : 0);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(RECORD_PERIOD_MS));
    }
}

// BỔ SUNG: Hàm stopRecordingThread() để khắc phục lỗi undefined reference
void AxisController::stopRecordingThread() {
    if (recording_.load()) {
        recording_.store(false);
    }
    if (rec_thread_.joinable()) {
        rec_thread_.join();
    }
}

void AxisController::stop() {
    if (!recording_.load()) {
        std::cout << "Not recording." << std::endl;
        return;
    }
    stopRecordingThread();
    // --- AUTO SAVE RECORD TO FILE ---
    std::lock_guard<std::mutex> lk(rec_mtx_);
    std::cout << "Recording stopped." << std::endl;
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        std::cout << axisName(i) << " samples: " << recorded_positions_[i].size() << std::endl;
        if (!recorded_positions_[i].empty()) {
            std::cout << axisName(i) << " Positions: [";
            for (size_t j = 0; j < recorded_positions_[i].size(); ++j) {
                std::cout << recorded_positions_[i][j];
                if (j + 1 < recorded_positions_[i].size()) std::cout << ", ";
            }
            std::cout << "]" << std::endl;
        }
    }
    std::string filename = "record" + std::to_string(record_file_index_) + ".txt";
    AxisVectorsEx data = recorded_positions_;
    if (writeBufferToFile(filename, data, false)) {
        std::cout << "Saved record to " << filename << std::endl;
        record_file_index_++;
    } else {
        std::cout << "Failed to save record file." << std::endl;
    }
    
}

void AxisController::clear() {
    std::lock_guard<std::mutex> lk(rec_mtx_);
    for (auto& path : recorded_positions_) {
        path.clear();
    }
    std::cout << "Cleared recorded positions for all motors." << std::endl;
}

bool AxisController::getPos(AxisPositions& pos) {
    return readActualPositions(pos);
}

bool AxisController::goWithBuffer(const AxisVectorsEx& paths)
{
   if (paths[0].empty()) return false;

    // ✅ RESET trạng thái trước khi chạy
    planner_done.store(false);

    motion_running.store(false);
    if (motion_thread.joinable())
        motion_thread.join();

    AxisStatuses statuses{};
    if (!readAxisStatuses(statuses) || !allServoOn(statuses)) return false;

    // bật output
    size_t outAxis = 0;
    uint32_t outMask = 0x01;
    setOutputSignal(outAxis, outMask, true);
    {
        std::lock_guard<std::mutex> lk(planner.mtx);
        planner.head = 0;
        planner.tail = 0;
    }
    planner_done.store(false);

    // ===== GRBL planner =====
    motion_running.store(true);

    std::thread planner_thread([this, paths]() {
        fillPlannerBuffer(paths);
    });

    motion_thread = std::thread(&AxisController::motionThreadFunc, this);
    
    // đợi chạy xong (Đã xóa dòng bool done = false; ở đây để hết bị warning)
    while (true) {
        bool idle = planner.isEmpty();
        bool plannerFinished = planner_done.load();

        AxisStatuses statuses{};
        bool moving = false;

        for (size_t a = 0; a < AXIS_COUNT; ++a) {
            if (FAS_GetAxisStatus(nPortIDs_[a], iSlaveNos_[a], &(statuses[a].dwValue)) == FMM_OK) {
                if (statuses[a].FFLAG_MOTIONING) {
                    moving = true;
                }
            }
        }

        // ✅ FIX CHÍNH Ở ĐÂY
        if (plannerFinished && idle && !moving) break;

        if (!motion_running.load()) break;

        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    motion_running.store(false);
    if (motion_thread.joinable()) motion_thread.join();

    // tắt output
    setOutputSignal(outAxis, outMask, false);

    std::cout << "GO (GRBL-style) finished." << std::endl;
    if (planner_thread.joinable()) planner_thread.join();
    return true;
    
}


bool AxisController::goFromFile(const std::string& filename)
{
    std::ifstream file(filename);

    if (!file.is_open()) {
        std::cout << "Cannot open file: " << filename << std::endl;
        return false;
    }

    AxisVectorsEx compatiblePaths{};
    std::string line;

    while (std::getline(file, line))
    {
        if (line.empty()) continue;

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

        for (size_t i = 0; i < AXIS_COUNT; i++) {
            compatiblePaths[i].push_back(row[i]);
        }
        
        // Nếu file có lưu cột IO thì lấy, không thì mặc định là 0 (để tránh lỗi Segfault)
        if (row.size() >= EXT_AXIS_COUNT) {
            compatiblePaths[AXIS_COUNT].push_back(row[AXIS_COUNT]);
        } else {
            compatiblePaths[AXIS_COUNT].push_back(0);
        }
    }

    compatiblePaths = compressBuffer(compatiblePaths, 5, 6);
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
            paths = saved_positions_;
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
        paths = recorded_positions_;
    }

    paths = compressBuffer(paths, 5, 6);

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

    AxisBools done{};
    std::cout << "Waiting for motors to complete movement..." << std::endl;
    while (true) {
        bool allDone = true;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        for (size_t i = 0; i < AXIS_COUNT; ++i) {
            if (!hasCommand[i] || done[i]) continue;
            if (FAS_GetAxisStatus(nPortIDs_[i], iSlaveNos_[i], &(statuses[i].dwValue)) ==
                FMM_OK) {
                done[i] = !statuses[i].FFLAG_MOTIONING;
                if (done[i]) {
                    std::cout << axisName(i) << " reached target position." << std::endl;
                }
            }
            allDone = allDone && (done[i] || !hasCommand[i]);
        }
        if (allDone) break;
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
        velocities.fill(base_velocity);

        for (size_t axis = 0; axis < AXIS_COUNT; ++axis) {
            if (idx < tableItems[axis].size()) {
                const ITEM_NODE& node = tableItems[axis][idx];
                targets[axis] = node.lPosition;
                velocities[axis] = (node.dwMoveSpd > 0) ? node.dwMoveSpd : base_velocity;
                hasCommand[axis] = true;
            }
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

        AxisBools done{};
        while (true) {
            bool allDone = true;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            for (size_t axis = 0; axis < AXIS_COUNT; ++axis) {
                if (!hasCommand[axis] || done[axis]) continue;
                if (FAS_GetAxisStatus(nPortIDs_[axis], iSlaveNos_[axis],
                                      &(statuses[axis].dwValue)) == FMM_OK) {
                    done[axis] = !statuses[axis].FFLAG_MOTIONING;
                }
                allDone = allDone && (done[axis] || !hasCommand[axis]);
            }
            if (allDone) break;
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

    bool inputSignal = false;         // <-- Thêm
    readInputSignal(inputSignal);

    std::lock_guard<std::mutex> lk(rec_mtx_);
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        saved_positions_[i].push_back(pos[i]);
    }

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

    // // chỉ ghi 1 điểm (pos hiện tại)
    // for (size_t i = 0; i < AXIS_COUNT; ++i) {
    //     temp[i].push_back(pos[i]);
    // }

    if (writeBufferToFile("savePos.txt", temp, true)) {
        std::cout << "Saved to savePos.txt" << std::endl;
    } else {
        std::cout << "Failed to write savePos.txt" << std::endl;
    }
    return true;
}

void AxisController::posTable() {
    AxisVectorsEx paths;
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

    AxisVectorsEx temp;
    for (auto& v : temp) v.clear();

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) continue;

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

        for (size_t i = 0; i < AXIS_COUNT; ++i) {
            temp[i].push_back(row[i]);
        }

        if (row.size() >= EXT_AXIS_COUNT) {
            temp[AXIS_COUNT].push_back(row[AXIS_COUNT]);
        } else {
            temp[AXIS_COUNT].push_back(0);
        }
    }

    file_buffer_ = temp;
    std::cout << "File buffer loaded. Steps: " << file_buffer_[0].size() << std::endl;
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
        for (size_t a = 0; a < AXIS_COUNT; ++a) {
            file << data[a][i];
            if (a + 1 < AXIS_COUNT) file << " ";
        }
        file << "\n";
    }

    file.close();
    return true;
}

// Thay AxisVectorsEx bằng AxisVectorsEx
void AxisController::fillPlannerBuffer(const AxisVectorsEx& paths) 
{
    AxisPositions current{};
    readActualPositions(current);

    for (size_t i = 0; i < paths[0].size(); ++i) {
        while (true) {
            if (!motion_running.load()) return;
            {
                std::lock_guard<std::mutex> lk(planner.mtx);
                if (!planner.isFullUnsafe()) break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        PlannerBlock block;
        AxisBools hasCommand{};

        for (size_t a = 0; a < AXIS_COUNT; ++a) {
            block.target[a] = paths[a][i];
            hasCommand[a] = true;
        }

        // 👇 THÊM: Đọc tín hiệu IO từ paths (cột AXIS_COUNT)
        block.relay_state = (paths[AXIS_COUNT][i] != 0);

        block.velocity = computeVelocities(current, block.target, hasCommand);
        current = block.target;

        planner.push(block);
    }
    planner_done.store(true);
}

void AxisController::motionThreadFunc()
{
    PlannerBlock block;

    while (motion_running.load()) {

        if (!planner.pop(block)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        setRelay(block.relay_state);
        // gửi lệnh
        for (size_t a = 0; a < AXIS_COUNT; ++a) {
            FAS_MoveSingleAxisAbsPos(
                nPortIDs_[a],
                iSlaveNos_[a],
                block.target[a],
                block.velocity[a]
            );
        }

        // chờ NGẮN (giống GRBL polling nhẹ)
        AxisStatuses statuses{};
        bool done = false;

        auto start = std::chrono::steady_clock::now();

        while (!done && motion_running.load()) {
            done = true;

            for (size_t a = 0; a < AXIS_COUNT; ++a) {
                if (FAS_GetAxisStatus(nPortIDs_[a], iSlaveNos_[a], &(statuses[a].dwValue)) == FMM_OK) {
                    if (statuses[a].FFLAG_MOTIONING) {
                        done = false;
                    }
                }
            }

            // ⛔ timeout 5s tránh treo
            if (std::chrono::steady_clock::now() - start > std::chrono::seconds(5)) {
                std::cout << "Motion timeout!" << std::endl;
                break;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
}

bool AxisController::readInputRisingEdge(bool& triggered)
{
    bool current = false;
    if (!readInputSignal(current)) return false;
    // 👇 Sử dụng biến thành viên last_input_signal_ thay vì biến toàn cục
    triggered = (!last_input_signal_ && current);
    last_input_signal_ = current;
    return true;
}



// Các hàm readInputSignal và setRelay giữ nguyên vì đã đúng ý bạn:

bool AxisController::readInputSignal(bool& signal)
{
    DWORD input;
    if (FAS_GetIOInput(nPortIDs_[0], iSlaveNos_[0], &input) != FMM_OK)
        return false;
    signal = (input & (1 << DI_INDEX)) != 0;
    return true;
}

bool AxisController::setRelay(bool on)
{
    if (on)
    {
        return FAS_SetOutput(
            nPortIDs_[0],
            iSlaveNos_[0],
            (1 << DO_INDEX),  // set bit
            0                 // không clear
        ) == FMM_OK;

    }
    else
    {
        return FAS_SetOutput(
            nPortIDs_[0],
            iSlaveNos_[0],
            0,                // không set
            (1 << DO_INDEX)   // clear bit
        ) == FMM_OK;
    }
}