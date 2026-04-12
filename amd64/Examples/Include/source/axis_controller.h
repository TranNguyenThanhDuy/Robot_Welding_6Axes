#pragma once

#include "DriverConnection.h"
#include <array>
#include <atomic>
#include <cstddef>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <cstdint> // Thêm thư viện này

// Thêm định nghĩa DWORD cho môi trường Linux
typedef uint32_t DWORD; 

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

// Thêm hằng số mở rộng để chứa thêm 1 cột tín hiệu IO
constexpr size_t EXT_AXIS_COUNT = AXIS_COUNT + 1; 

constexpr unsigned int base_velocity = 20000;
constexpr unsigned int min_velocity = 10;

// Các chân IO mặc định (có thể đổi số kênh tương ứng trên board của bạn)
constexpr int DI_INDEX = 0; 
constexpr int DO_INDEX = 0; 

using AxisPorts = std::array<int, AXIS_COUNT>;
using AxisSlaves = std::array<unsigned char, AXIS_COUNT>;
using AxisPositions = std::array<int, AXIS_COUNT>;
using AxisStatuses = std::array<EZISERVO_AXISSTATUS, AXIS_COUNT>;
using AxisVelocities = std::array<unsigned int, AXIS_COUNT>;
using AxisVectors = std::array<std::vector<int>, AXIS_COUNT>;
using AxisBools = std::array<bool, AXIS_COUNT>;

// Kiểu dữ liệu mở rộng có chứa IO
using AxisVectorsEx = std::array<std::vector<int>, EXT_AXIS_COUNT>; 
using AxisPositionsEx = std::array<int, EXT_AXIS_COUNT>;

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

    // Các hàm giao tiếp I/O cơ bản (mới và cũ kết hợp)
    bool setOutputSignal(size_t axisIdx, uint32_t outMask, bool state);
    bool isInputActive(size_t axisIdx, uint32_t inMask, bool& active);
    
    // Các hàm IO của module cũ dùng cho Relay
    bool readInputSignal(bool& signal);
    bool readInputRisingEdge(bool& triggered);
    bool setRelay(bool on);

    // Chức năng theo dõi Endstop
    void startEndstopMonitor(size_t triggerAxis, uint32_t endstopMask);
    void stopEndstopMonitor();

    // SỬA: Thay AxisVectors bằng AxisVectorsEx
    bool writeBufferToFile(const std::string& filename,
                           const AxisVectorsEx& data,
                           bool append);

private:
    enum class CaptureMode : int {
        FileMode = 0,
        ManualSave = 1,
        AutoRecord = 2
    };

    // SỬA: Thay AxisVectors bằng AxisVectorsEx
    void fillPlannerBuffer(const AxisVectorsEx& paths);
    void motionThreadFunc();

    std::thread endstop_thread_;
    std::atomic<bool> monitoring_endstop_{false};
    void endstopThreadFunc(size_t triggerAxis, uint32_t endstopMask);

    bool readAxisStatuses(AxisStatuses& statuses);
    bool readActualPositions(AxisPositions& positions);
    AxisVelocities computeVelocities(const AxisPositions& current,
                                     const AxisPositions& targets,
                                     const AxisBools& hasCommand) const;
    bool allServoOn(const AxisStatuses& statuses) const;
    void recordingThread();
    void stopRecordingThread();
    
    // SỬA: Thay AxisVectors bằng AxisVectorsEx
    bool goWithBuffer(const AxisVectorsEx& paths);
    bool loadFileToBuffer(const std::string& filename);
   
    AxisPorts nPortIDs_{};
    AxisSlaves iSlaveNos_{};
    
    // SỬA: Chuyển các buffer lưu dữ liệu sang dạng Ex (thêm cột IO)
    AxisVectorsEx file_buffer_{};
    AxisVectorsEx recorded_positions_{};
    AxisVectorsEx saved_positions_{};
    
    std::mutex rec_mtx_{};
    std::atomic<int> capture_mode_{static_cast<int>(CaptureMode::FileMode)};
    std::atomic<bool> recording_{false};
    std::thread rec_thread_{};
    int record_file_index_ = 1;
    std::string filename_;
    
    // THÊM: Biến lưu trữ trạng thái I/O trước đó cho hàm RisingEdge
    bool last_input_signal_ = false;
};
