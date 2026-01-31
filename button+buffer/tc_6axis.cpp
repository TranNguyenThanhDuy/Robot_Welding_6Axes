#include "DriverConnection.h"
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cmath>
#include <iostream>
#include <limits>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// ------------------------------------------------------------
// Axis selection
// Define exactly one of AXIS_1, AXIS_2, AXIS_5, AXIS_6
#if defined(AXIS_1)
constexpr size_t AXIS_COUNT = 1;
#elif defined(AXIS_2)
constexpr size_t AXIS_COUNT = 2;
#elif defined(AXIS_5)
constexpr size_t AXIS_COUNT = 5;
#elif defined(AXIS_6)
constexpr size_t AXIS_COUNT = 6;
#else
#error "Define one of AXIS_1, AXIS_2, AXIS_5, AXIS_6 before building."
#endif
// ------------------------------------------------------------

constexpr unsigned int base_velocity = 5000;
constexpr unsigned int min_velocity = 100; 

using AxisPorts = std::array<int, AXIS_COUNT>;
using AxisSlaves = std::array<unsigned char, AXIS_COUNT>;
using AxisPositions = std::array<int, AXIS_COUNT>;
using AxisStatuses = std::array<EZISERVO_AXISSTATUS, AXIS_COUNT>;
using AxisVelocities = std::array<unsigned int, AXIS_COUNT>;
using AxisVectors = std::array<std::vector<int>, AXIS_COUNT>;
using AxisBools = std::array<bool, AXIS_COUNT>;


AxisPorts nPortIDs = []() {
    AxisPorts ports{};
    int defaults[6] = {0, 0, 0, 0, 0, 0};
    for (size_t i = 0; i < AXIS_COUNT; ++i) ports[i] = defaults[i];
    return ports;
}();

AxisSlaves iSlaveNos = []() {
    AxisSlaves slaves{};
    unsigned char defaults[6] = {0, 1, 2, 3, 4, 5};
    for (size_t i = 0; i < AXIS_COUNT; ++i) slaves[i] = defaults[i];
    return slaves;
}();

// AxisVectors recorded_positions;
std::mutex rec_mtx;
std::atomic<bool> recording{false};
std::thread rec_thread;

std::string axisName(size_t idx) {
    return "Motor " + std::to_string(idx + 1);
}

constexpr size_t MAX_BUFFERS = 8;

// buffers[buffer_index][axis_index][positions...]
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

bool readAxisStatuses(AxisStatuses& statuses) {
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        if (FAS_GetAxisStatus(nPortIDs[i], iSlaveNos[i], &(statuses[i].dwValue)) != FMM_OK) {
            return false;
        }
    }
    return true;
}

bool readActualPositions(AxisPositions& positions) {
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        if (FAS_GetActualPos(nPortIDs[i], iSlaveNos[i], &positions[i]) != FMM_OK) {
            return false;
        }
    }
    return true;
}

AxisVelocities computeVelocities(const AxisPositions& current,
                                 const AxisPositions& targets,
                                 const AxisBools& hasCommand) {
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

bool allServoOn(const AxisStatuses& statuses) {
    for (const auto& st : statuses) {
        if (!st.FFLAG_SERVOON) return false;
    }
    return true;
}

void initializeSystem();
void showCommands();
bool handleServoOn();
bool handleServoOff();
bool handleHome();
void handleRecord();
void handleStop();
void handleClear();
void handleGetPos();
void handleGo();
void handleMovePos();
void handleGoPos();
void handlePosTable();
void handlePrintTable();
void recordingThread();
void cleanup();

int main() {
    initializeSystem();
    showCommands();

    std::string line;
    while (true) {
        if (!std::getline(std::cin, line)) break;
        std::string cmd = line;
        std::transform(cmd.begin(), cmd.end(), cmd.begin(),
                       [](unsigned char c) { return std::tolower(c); });

        if (cmd == "q") break;
        if (cmd.empty()) continue;

        if (cmd == "on") {
            handleServoOn();
        } else if (cmd == "off") {
            handleServoOff();
        } else if (cmd == "home") {
            handleHome();
        } else if (cmd == "record") {
            handleRecord();
        } else if (cmd == "stop") {
            handleStop();
        } else if (cmd == "clear") {
            handleClear();
        } else if (cmd == "getpos") {
            handleGetPos();
        } else if (cmd == "go") {
            handleGo();
        } else if (cmd == "movepos" || cmd == "move pos") {
            handleMovePos();
        } else if (cmd == "go pos" || cmd == "gopos") {
            handleGoPos();
        } else if (cmd == "postable") {
            handlePosTable();
        } else if (cmd == "print table" || cmd == "printtable") {
            handlePrintTable();
        } else {
            std::cout << "Unknown command. Use 'on', 'off', 'home', 'record', 'stop', 'clear', "
                         "'getpos', 'go', 'movepos', 'go pos', 'postable', 'print table', or 'q'."
                      << std::endl;
        }
    }

    cleanup();
    return 0;
}

void initializeSystem() {
    const wchar_t* sPort = L"ttyUSB0";
    unsigned int dwBaudRate = 115200;

    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        Driver_Connection(sPort, dwBaudRate, nPortIDs[i], iSlaveNos[i]);
    }
}

void showCommands() {
    std::cout << "=== " << AXIS_COUNT << "-AXIS SERVO CONTROL ===" << std::endl;
    std::cout << "on: Servo on (" << AXIS_COUNT << " motors)" << std::endl;
    std::cout << "off: Servo off (" << AXIS_COUNT << " motors)" << std::endl;
    std::cout << "home: Homing all motors (Only works in servo on)" << std::endl;
    std::cout << "record: Start recording all motors (Only works in servo off)" << std::endl;
    std::cout << "stop: Stop recording" << std::endl;
    std::cout << "clear: Clear buffer" << std::endl;
    std::cout << "getpos: Print current positions of all motors" << std::endl;
    std::cout << "go: Go with positions in buffer (Only works in servo on)" << std::endl;
    std::cout << "movepos: Move motors to specified positions (Only works in servo on)" << std::endl;
    std::cout << "go pos: Run items saved in position table (Only works in servo on)" << std::endl;
    std::cout << "postable: Save record to position table" << std::endl;
    std::cout << "print table: Print position table" << std::endl;
    std::cout << "q: Quit" << std::endl;
}

std::string printBuffer(const char* buffer, int n)
{
    return std::string(buffer, n);
}


bool handleServoOn() {
    bool allOk = true;
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        bool ok = ServoOn(nPortIDs[i], iSlaveNos[i]);
        if (!ok) {
            allOk = false;
            printf("%s servo ON failed.\n", axisName(i).c_str());
        }
    }

    if (allOk) {
        printf("All %zu servos ON successfully.\n", AXIS_COUNT);
    }
    return allOk;
}

bool handleServoOff() {
    bool allOk = true;
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        bool ok = ServoOff(nPortIDs[i], iSlaveNos[i]);
        if (!ok) {
            allOk = false;
            printf("%s servo OFF failed.\n", axisName(i).c_str());
        }
    }

    if (allOk) {
        printf("All %zu servos OFF successfully.\n", AXIS_COUNT);
    }
    return allOk;
}

bool handleHome() {
    AxisStatuses statuses{};
    if (!readAxisStatuses(statuses)) {
        printf("Function(FAS_GetAxisStatus) failed.\n");
        return false;
    }

    if (!allServoOn(statuses)) {
        std::cout << "Some servos are OFF. Turn them ON before homing." << std::endl;
        return false;
    }

    std::cout << "Sending homing commands to all motors..." << std::endl;
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        if (FAS_MoveSingleAxisAbsPos(nPortIDs[i], iSlaveNos[i], 0, base_velocity) != FMM_OK) {
            printf("%s homing command failed.\n", axisName(i).c_str());
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
            if (FAS_GetAxisStatus(nPortIDs[i], iSlaveNos[i], &(statuses[i].dwValue)) == FMM_OK) {
                done[i] = !statuses[i].FFLAG_MOTIONING;
                if (done[i]) {
                    printf("%s homing complete.\n", axisName(i).c_str());
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

void recordingThread() {
    constexpr int RECORD_PERIOD_MS = 10;

    while (recording.load()) {
        AxisPositions pos{};

        if (!readActualPositions(pos)) {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(RECORD_PERIOD_MS)
            );
            continue;
        }

        size_t buf;
        {
            std::lock_guard<std::mutex> lk(rec_mtx);
            buf = currentRecordingBuffer.load();

            // === 1 FRAME = SNAPSHOT TOÀN ROBOT ===
            for (size_t i = 0; i < AXIS_COUNT; ++i) {
                recorded_positions_buffers[buf][i].push_back(pos[i]);
            }
        }

        std::this_thread::sleep_for(
            std::chrono::milliseconds(RECORD_PERIOD_MS)
        );
    }
}

void handleRecord() {
    if (recording.load()) {
        std::cout << "Already recording.\n";
        return;
    }

    {
        std::lock_guard<std::mutex> lk(rec_mtx);

        size_t next =
            (currentRecordingBuffer.load() + 1) % MAX_BUFFERS;

        currentRecordingBuffer.store(next);

        // clear đúng buffer sẽ ghi
        for (auto& v : recorded_positions_buffers[next])
            v.clear();
    }


    plannerReady.store(false);

    recording.store(true);
    rec_thread = std::thread(recordingThread);
    std::cout << "Recording started to buffer " << currentRecordingBuffer.load() 
              << " (" << (bufferCount.load() + 1) << "/" << MAX_BUFFERS << ").\n";
}

void handleStop() {
    {
    std::lock_guard<std::mutex> lk(rec_mtx);

    size_t buf = currentRecordingBuffer.load();
    std::cout << "\n========== FULL RECORD BUFFER " << buf << " ==========\n";

    for (size_t a = 0; a < AXIS_COUNT; ++a) {
        auto& v = recorded_positions_buffers[buf][a];
        std::cout << "Axis " << a + 1 << " (" << v.size() << " points):\n";

        for (size_t i = 0; i < v.size(); ++i) {
            std::cout << "  [" << i << "] " << v[i] << "\n";
        }
        std::cout << "\n";
    }

    std::cout << "===============================================\n";
}

    if (!recording.load()) {
        std::cout << "Not recording.\n";
        return;
    }

    recording.store(false);
    if (rec_thread.joinable()) rec_thread.join();

    // Increment buffer count
    size_t currentCount = bufferCount.fetch_add(1) + 1;
    
    std::cout << "Recording stopped. Buffer " << currentRecordingBuffer.load() 
              << " saved (" << currentCount << "/" << MAX_BUFFERS << " buffers used).\n";
}

void handleClear() {
    std::lock_guard<std::mutex> lk(rec_mtx);

    for (auto& buffer : recorded_positions_buffers) {
        for (auto& axisVec : buffer) {
            axisVec.clear();
        }
    }

    bufferCount.store(0);
    currentRecordingBuffer.store(MAX_BUFFERS - 1);

    std::cout << "All buffers cleared.\n";
}

void handleGetPos() {
    AxisPositions pos{};
    if (!readActualPositions(pos)) {
        std::cout << "Failed to read current positions." << std::endl;
        return;
    }

    std::cout << "Current positions - ";
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        std::cout << axisName(i) << ": " << pos[i];
        if (i + 1 < AXIS_COUNT) std::cout << ", ";
    }
    std::cout << std::endl;
}

AxisVectors compressBuffer(
    const AxisVectors& in,
    int posEps,
    int minTrendLen
) {
    AxisVectors out;
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

    for (size_t i = 0; i < N; ++i) {
        if (!keep[i]) continue;
        for (size_t a = 0; a < AXIS_COUNT; ++a) {
            out[a].push_back(in[a][i]);
        }
    }

    return out;
}




void handleGo() {

    if (bufferCount.load() == 0) {
        std::cout << "No buffer.\n";
        return;
    }

    AxisStatuses statuses{};
    if (!readAxisStatuses(statuses) || !allServoOn(statuses)) {
        std::cout << "Servo OFF.\n";
        return;
    }

    AxisVectors paths;
    size_t bufIdx;

    {
        std::lock_guard<std::mutex> lk(rec_mtx);
        bufIdx = currentRecordingBuffer.load();
        paths  = recorded_positions_buffers[bufIdx];
    }

    // === COMPRESS BUFFER Ở ĐÂY ===
    paths = compressBuffer(paths, 5 , 6);

// // ===== IN TOÀN BỘ BUFFER (SAU RECORD, TRƯỚC GO) =====
//     std::cout << "\n========== FULL PLANNER BUFFER ==========\n";

//     for (size_t a = 0; a < AXIS_COUNT; ++a) {
//         const auto& v = paths[a];
//         std::cout << "Axis " << a + 1 << " (" << v.size() << " points):\n";

//         for (size_t i = 0; i < v.size(); ++i) {
//             std::cout << "  [" << i << "] " << v[i] << "\n";
//         }
//         std::cout << "\n";
//     }




    std::cout << "========================================\n";

    // Kiểm tra buffer có dữ liệu không
    bool anyData = false;
    for (const auto& v : paths) {
        if (!v.empty()) {
            anyData = true;
            break;
        }
    }

    if (!anyData) {
        std::cout << "Selected buffer is empty.\n";
        return;
    }

    std::cout << "GO start (smooth buffer planner)\n";

    // --------------------------------------------------
    std::array<size_t, AXIS_COUNT> idx{};   // index riêng cho từng trục
    AxisPositions lastPos{};
    AxisBools hasLast{};

    // --------------------------------------------------
    size_t steps = paths[0].size();

    for (size_t i = 0; i < steps; ++i) {
        AxisPositions target{};
        AxisBools hasCommand{};

        for (size_t a = 0; a < AXIS_COUNT; ++a) {
            target[a] = paths[a][i];
            hasCommand[a] = true;
        }

        AxisPositions current{};
        if (!readActualPositions(current)) return;

        AxisVelocities velocities =
            computeVelocities(current, target, hasCommand);

        for (size_t a = 0; a < AXIS_COUNT; ++a) {
            FAS_MoveSingleAxisAbsPos(
                nPortIDs[a],
                iSlaveNos[a],
                target[a],
                velocities[a]
            );
        }

        AxisStatuses statuses{};
        while (true) {
            bool done = true;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            for (size_t a = 0; a < AXIS_COUNT; ++a) {
                FAS_GetAxisStatus(
                    nPortIDs[a],
                    iSlaveNos[a],
                    &statuses[a].dwValue
                );
                done &= !statuses[a].FFLAG_MOTIONING;
            }
            if (done) break;
        }
    }


    std::cout << "GO finished (smooth buffer)\n";
}

void handleListBuffers() {
    std::lock_guard<std::mutex> lk(rec_mtx);
    
    size_t count = bufferCount.load();
    std::cout << "Available buffers: " << count << "/" << MAX_BUFFERS << "\n";
    
    for (size_t b = 0; b < count; ++b) {
        size_t steps = 0;
        if (!recorded_positions_buffers[b].empty()) {
            steps = recorded_positions_buffers[b][0].size();
        }
        std::cout << "  Buffer " << b << ": " << steps << " steps\n";
    }
}

void handleMovePos() {
    AxisStatuses statuses{};
    if (!readAxisStatuses(statuses)) {
        printf("Function(FAS_GetAxisStatus) failed.\n");
        return;
    }

    if (!allServoOn(statuses)) {
        std::cout << "Some servos are OFF. Turn them ON before moving." << std::endl;
        return;
    }

    AxisPositions targets{};
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        std::cout << "Enter target position for " << axisName(i) << ": ";
        if (!(std::cin >> targets[i])) {
            std::cout << "Invalid input for " << axisName(i) << " position." << std::endl;
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            return;
        }
    }
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    AxisPositions current{};
    if (!readActualPositions(current)) {
        printf("Failed to get current positions.\n");
        return;
    }

    AxisBools hasCommand{};
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        hasCommand[i] = (targets[i] != current[i]);
    }

    AxisVelocities velocities = computeVelocities(current, targets, hasCommand);

    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        if (!hasCommand[i]) continue;
        if (FAS_MoveSingleAxisAbsPos(nPortIDs[i], iSlaveNos[i], targets[i], velocities[i]) !=
            FMM_OK) {
            printf("%s move command failed.\n", axisName(i).c_str());
            return;
        }
    }

    AxisBools done{};
    std::cout << "Waiting for motors to complete movement..." << std::endl;
    while (true) {
        bool allDone = true;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        for (size_t i = 0; i < AXIS_COUNT; ++i) {
            if (!hasCommand[i] || done[i]) continue;
            if (FAS_GetAxisStatus(nPortIDs[i], iSlaveNos[i], &(statuses[i].dwValue)) == FMM_OK) {
                done[i] = !statuses[i].FFLAG_MOTIONING;
                if (done[i]) {
                    printf("%s reached target position.\n", axisName(i).c_str());
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
}

void handleGoPos() {
    AxisStatuses statuses{};
    if (!readAxisStatuses(statuses)) {
        printf("Function(FAS_GetAxisStatus) failed.\n");
        return;
    }

    if (!allServoOn(statuses)) {
        std::cout << "Some servos are OFF. Turn them ON before moving." << std::endl;
        return;
    }

    constexpr unsigned short maxItems = 64;
    std::array<std::vector<ITEM_NODE>, AXIS_COUNT> tableItems;

    for (size_t axis = 0; axis < AXIS_COUNT; ++axis) {
        for (unsigned short wItemNo = 1; wItemNo <= maxItems; ++wItemNo) {
            ITEM_NODE nodeItem;
            if (FAS_PosTableReadItem(nPortIDs[axis], iSlaveNos[axis], wItemNo, &nodeItem) !=
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
        return;
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
            if (FAS_MoveSingleAxisAbsPos(nPortIDs[axis], iSlaveNos[axis], targets[axis],
                                         velocities[axis]) != FMM_OK) {
                printf("%s move to %d failed.\n", axisName(axis).c_str(), targets[axis]);
                return;
            }
        }

        AxisBools done{};
        while (true) {
            bool allDone = true;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            for (size_t axis = 0; axis < AXIS_COUNT; ++axis) {
                if (!hasCommand[axis] || done[axis]) continue;
                if (FAS_GetAxisStatus(nPortIDs[axis], iSlaveNos[axis], &(statuses[axis].dwValue)) ==
                    FMM_OK) {
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
}

void handlePosTable() {
    AxisVectors paths;
{
    std::lock_guard<std::mutex> lk(rec_mtx);
    paths = recorded_positions_buffers[currentRecordingBuffer.load()];
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
            if (FAS_PosTableReadItem(nPortIDs[axis], iSlaveNos[axis], wItemNo, &nodeItem) !=
                FMM_OK) {
                printf("%s FAS_PosTableReadItem failed at item %u.\n", axisName(axis).c_str(),
                       wItemNo);
                break;
            }

            nodeItem.dwMoveSpd = 1000;
            nodeItem.lPosition = paths[axis][i];
            nodeItem.wBranch = 0;
            nodeItem.wContinuous = 0;

            if (FAS_PosTableWriteItem(nPortIDs[axis], iSlaveNos[axis], wItemNo, &nodeItem) !=
                FMM_OK) {
                printf("%s FAS_PosTableWriteItem failed at item %u.\n", axisName(axis).c_str(),
                       wItemNo);
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

void handlePrintTable() {
    constexpr unsigned short maxItems = 64;

    for (size_t axis = 0; axis < AXIS_COUNT; ++axis) {
        std::cout << "\n=== " << axisName(axis) << " Position Table ===" << std::endl;
        unsigned int shown = 0;
        for (unsigned short wItemNo = 1; wItemNo <= maxItems; ++wItemNo) {
            ITEM_NODE nodeItem;
            if (FAS_PosTableReadItem(nPortIDs[axis], iSlaveNos[axis], wItemNo, &nodeItem) !=
                FMM_OK) {
                printf("%s FAS_PosTableReadItem failed at item %u.\n", axisName(axis).c_str(),
                       wItemNo);
                break;
            }

            bool likelyEmpty = (nodeItem.lPosition == 0 && nodeItem.dwMoveSpd == 0);
            if (likelyEmpty) continue;

            printf("%s Item %u: Pos=%d, MoveSpd=%u, Cmd=%u, Wait=%u, Cont=%u, Branch=%u\n",
                   axisName(axis).c_str(), wItemNo, nodeItem.lPosition, nodeItem.dwMoveSpd,
                   nodeItem.wCommand, nodeItem.wWaitTime, nodeItem.wContinuous, nodeItem.wBranch);
            ++shown;
        }
        if (shown == 0) {
            std::cout << "No non-empty items found for " << axisName(axis) << "." << std::endl;
        }
    }
}

void cleanup() {
    if (recording.load()) {
        recording.store(false);
        if (rec_thread.joinable()) rec_thread.join();
    }
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        FAS_Close(nPortIDs[i]);
    }
}
