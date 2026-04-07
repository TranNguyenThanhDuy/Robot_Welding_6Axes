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

constexpr unsigned int default_base_velocity = 20000;
constexpr unsigned int default_min_velocity = 10;
constexpr unsigned short default_accel_time = 100;
constexpr unsigned short default_decel_time = 100;

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
    bool savePos();
    void setModeRecord();
    void setModeSave();
    bool isFileMode() const;
    bool isSaveMode() const;
    std::string modeName() const;
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
    bool goFromFile(const std::string& filename);
    void setModeFile();
    void setFileName(const std::string& name);
    std::string FileName() const;
    void setBaseVelocity(unsigned int value);
    void setMinVelocity(unsigned int value);
    void setAccelTime(unsigned short value);
    void setDecelTime(unsigned short value);
    unsigned int baseVelocity() const;
    unsigned int minVelocity() const;
    unsigned short accelTime() const;
    unsigned short decelTime() const;

private:
    enum class CaptureMode : int {
        FileMode = 0,
        ManualSave = 1,
        AutoRecord = 2
    };

    bool readAxisStatuses(AxisStatuses& statuses);
    bool readActualPositions(AxisPositions& positions);
    bool moveSingleAxisAbsPosProfiled(size_t axisIndex,
                                      int targetPosition,
                                      unsigned int velocity) const;
    AxisVelocities computeVelocities(const AxisPositions& current,
                                     const AxisPositions& targets,
                                     const AxisBools& hasCommand) const;
    bool allServoOn(const AxisStatuses& statuses) const;
    void recordingThread();
    void stopRecordingThread();
    bool goWithBuffer(const AxisVectors& paths);
    bool loadFileToBuffer(const std::string& filename);
   
    AxisVectors file_buffer_;
    AxisPorts nPortIDs_{};
    AxisSlaves iSlaveNos_{};
    AxisVectors recorded_positions_{};
    AxisVectors saved_positions_{};
    std::mutex rec_mtx_{};
    std::atomic<int> capture_mode_{static_cast<int>(CaptureMode::FileMode)};
    std::atomic<bool> recording_{false};
    std::thread rec_thread_{};
    unsigned int base_velocity_ = default_base_velocity;
    unsigned int min_velocity_ = default_min_velocity;
    unsigned short accel_time_ = default_accel_time;
    unsigned short decel_time_ = default_decel_time;

    std::string filename_;
};
