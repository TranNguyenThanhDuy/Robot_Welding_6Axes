#include "axis_controller.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>

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
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        FAS_Close(nPortIDs_[i]);
    }
}

std::string AxisController::axisName(size_t idx) const {
    return "Motor " + std::to_string(idx + 1);
}

bool AxisController::isRecording() const {
    return recording_.load();
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
    AxisPositions lastPos{};
    AxisBools hasLast{};

    while (recording_.load()) {
        AxisPositions pos{};
        if (!readActualPositions(pos)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        bool changed = false;
        for (size_t i = 0; i < AXIS_COUNT; ++i) {
            if (!hasLast[i] || pos[i] != lastPos[i]) {
                changed = true;
                break;
            }
        }

        if (changed) {
            std::lock_guard<std::mutex> lk(rec_mtx_);
            for (size_t i = 0; i < AXIS_COUNT; ++i) {
                recorded_positions_[i].push_back(pos[i]);
                lastPos[i] = pos[i];
                hasLast[i] = true;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void AxisController::record() {
    if (recording_.load()) {
        std::cout << "Already recording." << std::endl;
        return;
    }
    recording_.store(true);
    rec_thread_ = std::thread(&AxisController::recordingThread, this);
    std::cout << "Recording started for all motors (capturing positions only). Type 'stop' to end."
              << std::endl;
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
        if (!recorded_positions_[i].empty()) {
            std::cout << axisName(i) << " Positions: [";
            for (size_t j = 0; j < recorded_positions_[i].size(); ++j) {
                std::cout << recorded_positions_[i][j];
                if (j + 1 < recorded_positions_[i].size()) std::cout << ", ";
            }
            std::cout << "]" << std::endl;
        }
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

bool AxisController::go() {
    AxisStatuses statuses{};
    if (!readAxisStatuses(statuses)) {
        std::cout << "Function(FAS_GetAxisStatus) failed." << std::endl;
        return false;
    }

    if (!allServoOn(statuses)) {
        std::cout << "Some servos are OFF. Turn them ON before moving." << std::endl;
        return false;
    }

    AxisVectors paths;
    {
        std::lock_guard<std::mutex> lk(rec_mtx_);
        paths = recorded_positions_;
    }

    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        if (paths[i].empty()) {
            std::cout << axisName(i) << " has no recorded positions. Use 'record' first."
                      << std::endl;
            return false;
        }
    }

    size_t maxSteps = 0;
    for (const auto& p : paths) {
        if (p.size() > maxSteps) maxSteps = p.size();
    }

    for (size_t step = 0; step < maxSteps; ++step) {
        AxisPositions current{};
        readActualPositions(current);

        AxisPositions targets{};
        AxisBools hasCommand{};
        for (size_t i = 0; i < AXIS_COUNT; ++i) {
            if (step < paths[i].size()) {
                targets[i] = paths[i][step];
                hasCommand[i] = true;
            }
        }

        AxisVelocities velocities = computeVelocities(current, targets, hasCommand);

        for (size_t i = 0; i < AXIS_COUNT; ++i) {
            if (!hasCommand[i]) continue;
            if (FAS_MoveSingleAxisAbsPos(nPortIDs_[i], iSlaveNos_[i], targets[i],
                                         velocities[i]) != FMM_OK) {
                std::cout << axisName(i) << " move to " << targets[i] << " failed." << std::endl;
                return false;
            }
        }

        AxisBools done{};
        while (true) {
            bool allDone = true;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            for (size_t i = 0; i < AXIS_COUNT; ++i) {
                if (!hasCommand[i] || done[i]) continue;
                if (FAS_GetAxisStatus(nPortIDs_[i], iSlaveNos_[i], &(statuses[i].dwValue)) ==
                    FMM_OK) {
                    done[i] = !statuses[i].FFLAG_MOTIONING;
                }
                allDone = allDone && (done[i] || !hasCommand[i]);
            }
            if (allDone) break;
        }
    }

    AxisPositions finalPos{};
    if (readActualPositions(finalPos)) {
        std::cout << "Go complete. Final positions - ";
        for (size_t i = 0; i < AXIS_COUNT; ++i) {
            std::cout << axisName(i) << ": " << finalPos[i];
            if (i + 1 < AXIS_COUNT) std::cout << ", ";
        }
        std::cout << std::endl;
    } else {
        std::cout << "Go complete." << std::endl;
    }
    return true;
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
