#pragma once

#include "DriverConnection.h"
#include <array>
#include <atomic>
#include <cstddef>
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

class AxisController {
public:
    AxisController();
    ~AxisController();

    void initializeSystem();
    bool servoOn();
    bool servoOff();
    bool home();
    void record();
    void stop();
    void clear();
    bool getPos(AxisPositions& pos);
    bool go();
    bool movePos(const AxisPositions& targets);
    bool goPos();
    void posTable();
    void printTable();
    void setOriginPos();
    std::string axisName(size_t idx) const;
    bool isRecording() const;

private:
    bool readAxisStatuses(AxisStatuses& statuses);
    bool readActualPositions(AxisPositions& positions);
    AxisVelocities computeVelocities(const AxisPositions& current,
                                     const AxisPositions& targets,
                                     const AxisBools& hasCommand) const;
    bool allServoOn(const AxisStatuses& statuses) const;
    void recordingThread();
    void stopRecordingThread();

    AxisPorts nPortIDs_{};
    AxisSlaves iSlaveNos_{};
    AxisVectors recorded_positions_{};
    std::mutex rec_mtx_{};
    std::atomic<bool> recording_{false};
    std::thread rec_thread_{};
};
