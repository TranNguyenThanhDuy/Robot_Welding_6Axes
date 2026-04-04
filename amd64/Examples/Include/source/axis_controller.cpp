#include "axis_controller.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <deque>

AxisController::AxisController() {
    int portDefaults[6] = {0, 0, 0, 0, 0, 0};
    unsigned char slaveDefaults[6] = {0, 1, 2, 3, 4, 5};
    gear_ratios_.fill(1.0);
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        nPortIDs_[i] = portDefaults[i];
        iSlaveNos_[i] = slaveDefaults[i];
    }
}

AxisController::~AxisController() {
    stopRecordingThread();
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        FAS_Close(nPortIDs_[i]);
    }
}

constexpr size_t MAX_BUFFERS = 8;
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

// Tính toán góc Vector giữa 3 điểm để xác định đường thẳng
double computeCosAngle(const AxisPositions& p0,
                       const AxisPositions& p1,
                       const AxisPositions& p2)
{
    double dot = 0, mag1 = 0, mag2 = 0;

    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        double v1 = p1[i] - p0[i];
        double v2 = p2[i] - p1[i];

        dot += v1 * v2;
        mag1 += v1 * v1;
        mag2 += v2 * v2;
    }

    if (mag1 == 0 || mag2 == 0) return 1.0;
    return dot / (std::sqrt(mag1) * std::sqrt(mag2));
}

// 🔥 THUẬT TOÁN LỌC QUỸ ĐẠO THÔNG MINH (TẠO BUFFER MỚI GỌN HƠN)
AxisVectors compressBuffer(const AxisVectors& in, int minDist) {
    AxisVectors out;
    size_t N = in[0].size();
    if (N < 2) return in;

    // Luôn giữ điểm bắt đầu
    for (size_t a = 0; a < AXIS_COUNT; ++a) out[a].push_back(in[a][0]);

    size_t lastKept = 0;

    for (size_t i = 1; i < N - 1; ++i) {
        // 1. Kiểm tra khoảng cách (Bỏ qua các điểm nhiễu li ti)
        int maxDist = 0;
        for (size_t a = 0; a < AXIS_COUNT; ++a) {
            int dist = std::abs(in[a][i] - in[a][lastKept]);
            if (dist > maxDist) maxDist = dist;
        }
        if (maxDist < minDist) continue;

        // 2. Kiểm tra tính thẳng hàng (Collinearity)
        AxisPositions p0, p1, p2;
        for (size_t a = 0; a < AXIS_COUNT; ++a) {
            p0[a] = in[a][lastKept];
            p1[a] = in[a][i];
            p2[a] = in[a][i + 1];
        }

        double cosA = computeCosAngle(p0, p1, p2);

        // Nếu cosA > 0.995 (tức là 3 điểm tạo thành 1 góc < 5.7 độ -> gần như là đường thẳng)
        // Vứt bỏ điểm hiện tại, để motor kéo dài đường chạy mà không cần dừng lại.
        if (cosA > 0.995) continue;

        // Nếu là góc cua, giữ lại điểm này để motor quẹo
        for (size_t a = 0; a < AXIS_COUNT; ++a) out[a].push_back(in[a][i]);
        lastKept = i;
    }

    // Luôn giữ điểm kết thúc
    for (size_t a = 0; a < AXIS_COUNT; ++a) out[a].push_back(in[a][N - 1]);
    
    std::cout << "[Optimizer] Khung duong da gop: tu " << N << " diem xuong " << out[0].size() << " diem." << std::endl;
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
                                     
    int maxDistance = 0;
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        if (!hasCommand[i]) continue;
        int distance = std::abs(targets[i] - current[i]);
        if (distance > maxDistance) maxDistance = distance;
    } 
    
    // 🔥 Căn chuẩn tỷ lệ thuận của GRBL để 6 trục đến đích cùng lúc (Tránh cong quỹ đạo)
    unsigned int masterVel = std::max((unsigned int)1000, (unsigned int)base_velocity);

    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        if (!hasCommand[i] || targets[i] == current[i]) {
            velocities[i] = 0;
            continue;
        }

        int distance = std::abs(targets[i] - current[i]);

        unsigned int targetVel = (maxDistance > 0)
            ? (distance * masterVel) / maxDistance
            : 1;

        // Vận tốc thấp nhất = 1 để tránh lỗi Driver từ chối lệnh
        velocities[i] = std::max((unsigned int)1, targetVel);
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

void AxisController::recordingThread() {
    constexpr int RECORD_PERIOD_MS = 10;
    AxisPositions prevPos{};
    bool firstSample = true;

    while (recording_.load()) {
        AxisPositions pos{};
        AxisVelocities vel{};

        if (!readActualPositions(pos)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(RECORD_PERIOD_MS));
            continue;
        }

        prevPos = pos;

        {
            std::lock_guard<std::mutex> lk(rec_mtx_);
            for (size_t i = 0; i < AXIS_COUNT; ++i) {
                recorded_positions_[i].push_back(pos[i]);
                recorded_velocities_[i].push_back(vel[i]);
            }
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
        for (auto& v : recorded_velocities_) v.clear();
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

    std::string filename = "record" + std::to_string(record_file_index_) + ".txt";
    AxisVectors data = recorded_positions_;
    if (writeBufferToFile(filename, data, false)) {
        std::cout << "Saved record to " << filename << std::endl;
    }

    std::string speedFile = "speed_" + std::to_string(record_file_index_) + ".txt";
    AxisVectors velData = recorded_velocities_;
    if (writeBufferToFile(speedFile, velData, false)) {
        std::cout << "Saved speed to " << speedFile << std::endl;
    }
    record_file_index_++;
}

void AxisController::clear() {
    std::lock_guard<std::mutex> lk(rec_mtx_);
    for (auto& path : recorded_positions_) {
        path.clear();
    }
    for (auto& v : recorded_velocities_) v.clear();
    std::cout << "Cleared recorded positions for all motors." << std::endl;
}

bool AxisController::getPos(AxisPositions& pos) {
    return readActualPositions(pos);
}

bool AxisController::goWithBuffer(const AxisVectors& paths) {
    if (paths[0].empty()) return false;
    
    AxisStatuses statuses{};
    if (!readAxisStatuses(statuses) || !allServoOn(statuses)) {
        std::cout << "[ERROR] Cannot read status or servos are OFF." << std::endl;
        return false;
    }

    // Lọc nhiễu quỹ đạo: bỏ qua các điểm quá gần (100 xung trở xuống ~ rất mịn)
    // Tránh việc nạp buffer liên tục làm giật motor và chạy quá chậm
    AxisVectors cleanPaths = compressBuffer(paths, 100);
    if (cleanPaths[0].empty()) return false;
    size_t steps = cleanPaths[0].size();

    AxisPositions lastTarget{};
    readActualPositions(lastTarget); // Chỉ dùng 1 lần ở đây để mốc

    size_t outAxis = 0;
    uint32_t outMask = 0x01;
    setOutputSignal(outAxis, outMask, true);

    startPlanner();

    for (size_t i = 0; i < steps; ++i) {
        PlannerBlock block;
        bool anyMove = false;
        AxisBools hasCommand{};

        for (size_t a = 0; a < AXIS_COUNT; ++a) {
            block.target[a] = cleanPaths[a][i];
            
            // 🔥 BẮT BUG Ở ĐÂY: Quãng đường di chuyển phải được tính từ lastTarget
            // Không được dùng readActualPositions(current) vì motor vật lý có thể đang bị trễ.
            if (block.target[a] != lastTarget[a]) {
                hasCommand[a] = true;
                anyMove = true;
            } else {
                hasCommand[a] = false;
            }
        }

        if (!anyMove) continue;

        double speedScale = 1.0;

        if (i > 0 && i < steps - 1) {
            AxisPositions p0, p1, p2;

            for (size_t a = 0; a < AXIS_COUNT; ++a) {
                p0[a] = cleanPaths[a][i - 1];
                p1[a] = cleanPaths[a][i];
                p2[a] = cleanPaths[a][i + 1];
            }

            double cosA = computeCosAngle(p0, p1, p2);
            double junctionFactor = (1.0 - cosA); 
            speedScale = std::clamp(1.0 - junctionFactor, 0.3, 1.0);
        }
        
        // Truyền lastTarget vào để nội suy khoảng cách chính xác theo chuẩn vector
        block.velocity = computeVelocities(lastTarget, block.target, hasCommand);
        
        for (size_t a = 0; a < AXIS_COUNT; ++a) {
            if (block.velocity[a] > 0) {
                block.velocity[a] = std::max((unsigned int)1, static_cast<unsigned int>(block.velocity[a] * speedScale));
            }
        }
        
        // Neo lại vị trí vừa tính toán cho block tiếp theo
        lastTarget = block.target;

        while (true) {
            {
                std::lock_guard<std::mutex> lk(planner_mtx_);
                if (plannerBuffer_.size() < PLANNER_BUFFER_SIZE) {
                    plannerBuffer_.push_back(block);
                    break;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(2)); 
        }
    }

    auto last_activity = std::chrono::steady_clock::now();
    size_t last_buffer_size = PLANNER_BUFFER_SIZE + 1;

    while (true) {
        bool empty;
        size_t current_size;
        {
            std::lock_guard<std::mutex> lk(planner_mtx_);
            empty = plannerBuffer_.empty();
            current_size = plannerBuffer_.size();
        }

        AxisStatuses st{};
        bool busy = false;

        for (size_t a = 0; a < AXIS_COUNT; ++a) {
            if (FAS_GetAxisStatus(nPortIDs_[a], iSlaveNos_[a], &st[a].dwValue) == FMM_OK) {
                busy |= st[a].FFLAG_MOTIONING;
            }
        }

        if (empty && !busy) break;

        if (current_size != last_buffer_size) {
            last_activity = std::chrono::steady_clock::now();
            last_buffer_size = current_size;
        }

        // Tăng timeout lên 60 giây để chờ block đầu chạy hết (Rất dài)
        if (std::chrono::steady_clock::now() - last_activity > std::chrono::seconds(60)) {
            std::cout << "[ERROR] Timeout 60s waiting planner finish! Force Exit." << std::endl;
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    stopPlanner();
    setOutputSignal(outAxis, outMask, false);
    std::cout << "GO (planner mode) finished." << std::endl;
    return true;
}

bool AxisController::goFromFile(const std::string& filename)
{
    std::ifstream file(filename);

    if (!file.is_open()) {
        std::cout << "Cannot open file: " << filename << std::endl;
        return false;
    }

    std::vector<AxisPositions> pathBuffer; 
    std::string line;

    while (std::getline(file, line))
    {
        if (line.empty()) continue;

        std::stringstream ss(line);
        AxisPositions pos{};

        for (size_t i = 0; i < AXIS_COUNT; i++)
        {
            if (!(ss >> pos[i]))
            {
                std::cout << "Format error at line: " << line << "\n";
                return false;
            }
        }
        
        pathBuffer.push_back(pos);
    }

    AxisVectors compatiblePaths;
    for (const auto& row : pathBuffer) {
        for (size_t i = 0; i < AXIS_COUNT; i++) {
            compatiblePaths[i].push_back(row[i]);
        }
    }
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
        AxisVectors paths;
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
    AxisVectors paths;
    {
        std::lock_guard<std::mutex> lk(rec_mtx_);
        paths = recorded_positions_;
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
            int res = FAS_MoveSingleAxisAbsPos(nPortIDs_[axis], iSlaveNos_[axis], targets[axis], velocities[axis]);
            if (res != FMM_OK) {
                std::cout << "[ERROR] " << axisName(axis) << " move to " << targets[axis] << " failed. Code: " << res << std::endl;
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

    AxisVectors temp;

    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        temp[i].push_back(pos[i]);
    }
    for (auto& v : temp) v.clear();

    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        temp[i].push_back(pos[i]);
    }

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

// 🛑 HÀM NÀY ĐÃ ĐƯỢC KHÔI PHỤC HOÀN TOÀN TỪ CODE CŨ
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
    
    uint32_t dwSetMask = state ? outMask : 0;
    uint32_t dwClearMask = state ? 0 : outMask;
    
    return FAS_SetIOOutput(nPortIDs_[axisIdx], iSlaveNos_[axisIdx], dwSetMask, dwClearMask) == FMM_OK;
}

bool AxisController::isInputActive(size_t axisIdx, uint32_t inMask, bool& active) {
    if (axisIdx >= AXIS_COUNT) return false;
    
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
            if (currentState && !lastState) {
                std::cout << "[Endstop] Triggered! Auto-saving position..." << std::endl;
                savePos();
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

// 🛑 HÀM NÀY ĐÃ ĐƯỢC KHÔI PHỤC HOÀN TOÀN TỪ CODE CŨ
bool AxisController::readActualVelocities(AxisVelocities& velocities) {
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        int currentPPS = 0; 
        
        if (FAS_GetActualVel(nPortIDs_[i], iSlaveNos_[i], &currentPPS) != FMM_OK) {
            return false;
        }
        
        velocities[i] = std::abs(currentPPS);
    }
    return true;
}

void AxisController::executeBlock(const PlannerBlock& block) {
    for (size_t a = 0; a < AXIS_COUNT; ++a) {
        if (block.velocity[a] == 0) continue; 

        int res = FAS_MoveSingleAxisAbsPos(
            nPortIDs_[a],
            iSlaveNos_[a],
            block.target[a],
            block.velocity[a]
        );
        if (res != FMM_OK) {
            std::cout << "[ERROR] Axis " << a + 1 << " Move Failed. Code: " << res << std::endl;
        }
    }
}   

void AxisController::plannerThreadFunc() {
    while (planner_running_.load()) {
        PlannerBlock block;
        bool hasBlock = false;

        {
            std::lock_guard<std::mutex> lk(planner_mtx_);
            if (!plannerBuffer_.empty()) {
                block = plannerBuffer_.front();
                plannerBuffer_.pop_front();
                hasBlock = true;
            }
        }

        if (!hasBlock) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        executeBlock(block);

        std::this_thread::sleep_for(std::chrono::milliseconds(20));

        auto startWait = std::chrono::steady_clock::now();
        while (planner_running_.load()) {
            bool isMoving = false;
            bool readOk = false;
            AxisStatuses statuses{};
            
            // Xử lý chống nhiễu giao tiếp RS485 (Tránh lọt block khi đang nhiễu)
            for (int retry = 0; retry < 3; ++retry) {
                if (readAxisStatuses(statuses)) {
                    readOk = true;
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }

            if (readOk) {
                for (size_t a = 0; a < AXIS_COUNT; ++a) {
                    if (block.velocity[a] > 0 && statuses[a].FFLAG_MOTIONING) {
                        isMoving = true;
                        break;
                    }
                }
            } else {
                // Đọc lỗi liên tiếp => Cứ giả định là đang chạy để tránh nạp ép sinh lỗi 133
                isMoving = true; 
            }

            if (!isMoving) break; // Đã xong hoàn toàn.

            // Timeout được tăng lên 60 giây ở đây để các block khởi động thật dài không bị Timeout
            if (std::chrono::steady_clock::now() - startWait > std::chrono::seconds(60)) {
                std::cout << "[Planner] Lỗi: Timeout 60s. Xóa bộ đệm và dừng Motor để bảo vệ hệ thống." << std::endl;
                stopPlanner(); // Dừng ngay lập tức!
                return;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
}


void AxisController::stopPlanner() {
    planner_running_.store(false);

    if (planner_thread_.joinable())
        planner_thread_.join();

    std::lock_guard<std::mutex> lk(planner_mtx_);
    plannerBuffer_.clear();
}

// 🛑 HÀM NÀY ĐÃ ĐƯỢC KHÔI PHỤC HOÀN TOÀN TỪ CODE CŨ
int AxisController::rampVelocity(int last, int target) const {
    int step = 100;
    if (target > last)
        return std::min(last + step, target);
    else
        return std::max(last - step, target);
}

void AxisController::startPlanner() {
    if (planner_running_.load()) return;

    planner_running_.store(true);

    planner_thread_ = std::thread(&AxisController::plannerThreadFunc, this);

    std::cout << "[Planner] Started\n";
}