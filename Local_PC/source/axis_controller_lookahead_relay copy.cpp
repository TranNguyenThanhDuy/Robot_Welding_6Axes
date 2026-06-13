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
// --- CÁC HẰNG SỐ CẤU HÌNH HỆ THỐNG ---
constexpr auto kCommandSpacing = std::chrono::microseconds(1500);          // Khoảng thời gian giãn cách giữa lệnh gửi tới các trục (1.5ms)
constexpr double kDefaultSegmentSeconds = 0.05;                            // Thời gian mặc định cho một phân đoạn quỹ đạo (50ms)
constexpr unsigned char kAccelerationTimeParameter = 3;                   // Mã tham số thời gian tăng tốc trong EEPROM của Ezi-Servo
constexpr unsigned char kDecelerationTimeParameter = 4;                   // Mã tham số thời gian giảm tốc trong EEPROM của Ezi-Servo
constexpr int kStreamingAccelTimeMs = 5;                                   // Thời gian tăng tốc siêu ngắn (5ms) dùng khi chạy chế độ Stream
constexpr int kStreamingDecelTimeMs = 5;                                   // Thời gian giảm tốc siêu ngắn (5ms) dùng khi chạy chế độ Stream
constexpr double kMinCornerSpeedScale = 0.25;                              // Hệ số giảm tốc tối thiểu khi đi vào góc cua gắt (Giảm max còn 25%)
constexpr double kStraightCosThreshold = 0.98;                             // Ngưỡng Cosin để xác định đường thẳng (Cos > 0.98 -> Không giảm tốc)
constexpr size_t DI_INDEX = 0;                                             // Chỉ số ngõ vào số (Digital Input) dùng đọc cảm biến ngoại vi
constexpr size_t DO_INDEX = 0;                                             // Chỉ số ngõ ra số (Digital Output) điều khiển Rơ-le mỏ hàn
constexpr size_t FILE_RELAY_COLUMN = 6;                                    // Cột thứ 7 (index 6) trong file text chứa trạng thái On/Off mỏ hàn
constexpr auto kRelaySettleDelay = std::chrono::milliseconds(500);         // Thời gian chờ rơ-le cơ khí ổn định tiếp điểm (500ms)
constexpr auto kMotionPollInterval = std::chrono::milliseconds(10);        // Chu kỳ quét kiểm tra trạng thái động cơ dừng hẳn (10ms)
constexpr auto kHomingTimeout = std::chrono::seconds(45);                  // Thời gian tối đa cho phép robot chạy về gốc tọa độ O (45 giây)
constexpr auto kFinalSettleTimeout = std::chrono::seconds(20);             // Thời gian chờ tối đa khi kết thúc toàn bộ quỹ đạo (20 giây)

// --- BIẾN TOÀN CỤC CHỨA TRẠNG THÁI NGOẠI VI ---
std::vector<int> g_recordedRelayStates;                                    // Lưu lịch sử trạng thái mỏ hàn khi Record tự động
std::vector<int> g_savedRelayStates;                                       // Lưu lịch sử trạng thái mỏ hàn khi lưu thủ công (Manual Save)

// Thời gian chu kỳ phân đoạn chuyển động (mặc định chuyển từ giây sang Microseconds)
std::chrono::microseconds g_segment_period =
    std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::duration<double>(kDefaultSegmentSeconds));

/**
 * @brief Hàm gộp ma trận tọa độ các trục và mảng trạng thái rơ-le thành một cấu trúc dữ liệu mở rộng
 */
AxisVectorsEx makeExtendedPaths(const AxisVectors& positions, const std::vector<int>& relayStates) {
    AxisVectorsEx out;
    const size_t steps = positions[0].size();

    // Sao chép tọa độ di chuyển của tất cả các trục cơ khí (0 tới AXIS_COUNT-1)
    for (size_t a = 0; a < AXIS_COUNT; ++a) {
        out[a] = positions[a];
    }

    // Điền trạng thái rơ-le mỏ hàn vào kênh mở rộng cuối cùng (Index = AXIS_COUNT)
    out[AXIS_COUNT].reserve(steps);
    for (size_t i = 0; i < steps; ++i) {
        out[AXIS_COUNT].push_back(i < relayStates.size() ? relayStates[i] : 0);
    }

    return out;
}

/**
 * @brief Hàm xóa bỏ khoảng trắng thừa và ký tự comment (#, ;) ở đầu/cuối chuỗi text
 */
std::string trimCopy(const std::string& value) {
    const auto first = value.find_first_not_of(" \t\r\n#;");
    if (first == std::string::npos) return {};
    const auto last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

/**
 * @brief Hàm đọc dữ liệu cấu hình thời gian (Header) ở các dòng đầu tiên của file quỹ đạo
 */
bool readTimingHeader(const std::string& line, double& segmentSeconds, double& moveTimeSeconds) {
    std::string text = trimCopy(line);
    const auto eq = text.find('=');
    if (eq == std::string::npos) return false;

    std::string key = trimCopy(text.substr(0, eq));
    std::string val = trimCopy(text.substr(eq + 1));
    // Chuyển toàn bộ ký tự key sang chữ thường để không phân biệt hoa thường
    std::transform(key.begin(), key.end(), key.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

    try {
        const double parsed = std::stod(val);
        // Kiểm tra nếu file định nghĩa thời gian một phân đoạn bằng Giây
        if (key == "segment_period_s" || key == "segment_seconds") {
            segmentSeconds = parsed;
            return true;
        }
        // Kiểm tra nếu file định nghĩa bằng Mili-giây
        if (key == "segment_period_ms" || key == "segment_ms") {
            segmentSeconds = parsed / 1000.0;
            return true;
        }
        // Kiểm tra nếu file định nghĩa bằng Micro-giây
        if (key == "segment_period_us" || key == "segment_us") {
            segmentSeconds = parsed / 1000000.0;
            return true;
        }
        // Kiểm tra nếu file định nghĩa tổng thời gian của cả quá trình chuyển động
        if (key == "movetimevalue" || key == "move_time" || key == "move_time_s") {
            moveTimeSeconds = parsed;
            return true;
        }
    } catch (...) {
        return false;
    }

    return false;
}

/**
 * @brief Cài đặt thời gian chu kỳ phân đoạn (đơn vị: Giây)
 */
void setSegmentPeriodSeconds(double seconds) {
    if (seconds <= 0.0) return;
    g_segment_period = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::duration<double>(seconds));
}

/**
 * @brief THUẬT TOÁN ĐỒNG TỐC: Tính toán vận tốc cho từng trục dựa trên quãng đường để tất cả các trục đến đích cùng lúc
 */
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
        // Tính khoảng cách di chuyển delta (sử dụng llabs tránh tràn số khi trừ tọa độ lớn)
        const auto distance =
            std::llabs(static_cast<long long>(targets[i]) - static_cast<long long>(current[i]));
        if (distance == 0) {
            velocities[i] = 1; // Giữ mức 1 để tránh lỗi driver nhận tốc độ bằng 0
            continue;
        }

        // Vận tốc = Quãng đường / Thời gian (Làm tròn lên bằng hàm std::ceil)
        const double speed = std::ceil(static_cast<double>(distance) / seconds);
        if (speed > static_cast<double>(std::numeric_limits<int>::max())) {
            velocities[i] = std::numeric_limits<int>::max();
        } else {
            velocities[i] = std::max(1, static_cast<int>(speed));
        }
    }

    return velocities;
}

/**
 * @brief Tính độ dài hình học (Độ lớn Vector) giữa hai điểm trong không gian cấu hình đa trục
 */
double vectorLength(const AxisPositions& from, const AxisPositions& to) {
    double sum = 0.0;
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        const double d = static_cast<double>(to[i]) - static_cast<double>(from[i]);
        sum += d * d; // Bình phương khoảng cách từng trục
    }
    return std::sqrt(sum); // Căn bậc hai tổng bình phương (Khoảng cách Euclid)
}

/**
 * @brief THUẬT TOÁN LOOKAHEAD (NỘI SUY TRƯỚC GÓC CUA): Tính toán giảm tốc độ dựa trên tích vô hướng của góc cua phần cứng
 */
double cornerSpeedScale(const AxisPositions& before,
                        const AxisPositions& corner,
                        const AxisPositions& after) {
    const double lenA = vectorLength(before, corner); // Chiều dài vector hướng đi tới
    const double lenB = vectorLength(corner, after);  // Chiều dài vector hướng đi tiếp theo
    if (lenA <= 0.0 || lenB <= 0.0) {
        return 1.0; // Nếu robot đứng yên tại chỗ, không cần giảm tốc scale
    }

    // Tính tích vô hướng (Dot Product) giữa hai Vector di chuyển liên tiếp
    double dot = 0.0;
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        const double a = static_cast<double>(corner[i]) - static_cast<double>(before[i]);
        const double b = static_cast<double>(after[i]) - static_cast<double>(corner[i]);
        dot += a * b;
    }

    // Tính Cosin của góc kẹp giữa hai vector: cos(Theta) = (A.B) / (|A|*|B|)
    const double cosTheta = std::max(-1.0, std::min(1.0, dot / (lenA * lenB)));
    
    // Nếu góc cua rất phẳng (gần như đường thẳng), giữ nguyên 100% tốc độ
    if (cosTheta >= kStraightCosThreshold) {
        return 1.0;
    }

    // Nếu góc cua gắt, giảm tốc mềm theo công thức nội suy (Tránh khựng giật cơ khí)
    return std::max(kMinCornerSpeedScale, (1.0 + cosTheta) * 0.5);
}

/**
 * @brief Tính chu kỳ thời gian phân đoạn mới sau khi đã áp dụng tỷ lệ giảm tốc từ thuật toán Lookahead
 */
std::chrono::microseconds scaledSegmentPeriod(std::chrono::microseconds basePeriod, double speedScale) {
    const double safeScale = std::max(kMinCornerSpeedScale, std::min(1.0, speedScale));
    // Tốc độ giảm đi đồng nghĩa với việc kéo dài thời gian thực thi phân đoạn đó ra
    const double scaledSeconds = (static_cast<double>(basePeriod.count()) / 1000000.0) / safeScale;
    return std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::duration<double>(scaledSeconds));
}

/**
 * @brief Ghi thông số cấu hình thời gian tăng/giảm tốc chung xuống toàn bộ Driver Ezi-Servo
 */
bool setAllMotionTiming(const AxisPorts& ports, const AxisSlaves& slaves, unsigned char paramNo, int value, const char* label) {
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        const int ret = FAS_SetParameter(ports[i], slaves[i], paramNo, value);
        if (ret != FMM_OK) {
            std::cout << "Failed to set " << label << " for axis " << (i + 1) << ": " << ret << std::endl;
            return false;
        }
    }
    return true;
}

/**
 * @brief Đọc thông số cấu hình thời gian tăng/giảm tốc hiện tại của Driver để sao lưu trước khi nạp đè dữ liệu mới
 */
bool readAllMotionTiming(const AxisPorts& ports, const AxisSlaves& slaves, unsigned char paramNo, AxisPositions& values, const char* label) {
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        const int ret = FAS_GetParameter(ports[i], slaves[i], paramNo, &values[i]);
        if (ret != FMM_OK) {
            std::cout << "Failed to read " << label << " for axis " << (i + 1) << ": " << ret << std::endl;
            return false;
        }
    }
    return true;
}

/**
 * @brief Khôi phục lại trạng thái thông số cấu hình cũ của Driver sau khi chu trình Stream quỹ đạo kết thúc
 */
bool restoreAllMotionTiming(const AxisPorts& ports, const AxisSlaves& slaves, unsigned char paramNo, const AxisPositions& values, const char* label) {
    bool ok = true;
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        const int ret = FAS_SetParameter(ports[i], slaves[i], paramNo, values[i]);
        if (ret != FMM_OK) {
            std::cout << "Failed to restore " << label << " for axis " << (i + 1) << ": " << ret << std::endl;
            ok = false;
        }
    }
    return ok;
}

/**
 * @brief Lệnh dừng khẩn cấp lập tức toàn bộ các trục động cơ đang quay
 */
void stopAllAxes(const AxisPorts& ports, const AxisSlaves& slaves) {
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        const int ret = FAS_MoveStop(ports[i], slaves[i]);
        if (ret != FMM_OK) {
            std::cout << "Failed to stop axis " << (i + 1) << ": " << ret << std::endl;
        }
    }
}

/**
 * @brief Bộ lọc kiểm tra lỗi trạng thái phần cứng (Quá dòng, quá nhiệt, đụng giới hạn hành trình, mất vị trí...)
 */
bool hasMotionFault(const EZISERVO_AXISSTATUS& st) {
    return st.FFLAG_ERRORALL ||
           st.FFLAG_HWPOSILMT || st.FFLAG_HWNEGALMT || // Công tắc hành trình phần cứng (+/-)
           st.FFLAG_SWPOGILMT || st.FFLAG_SWNEGALMT || // Giới hạn phần mềm cài đặt sẵn
           st.FFLAG_ERRPOSOVERFLOW ||                  // Tràn bộ đếm xung vị trí
           st.FFLAG_ERROVERCURRENT ||                  // Driver quá dòng (Chập tải/kẹt cơ)
           st.FFLAG_ERROVERSPEED   ||                  // Motor quay quá tốc độ quy định
           st.FFLAG_ERRPOSTRACKING ||                  // Sai lệch vị trí thực tế so với tập lệnh quá lớn
           st.FFLAG_ERROVERLOAD    ||                  // Quá tải lực đầu trục
           st.FFLAG_ERROVERHEAT    ||                  // Driver/Motor quá nóng
           st.FFLAG_ERRBACKEMF     ||                  // Lỗi dòng điện ngược dòng
           st.FFLAG_ERRMOTORPOWER  ||                  // Mất nguồn động lực cấp cho motor
           st.FFLAG_ERRINPOSITION  || 
           st.FFLAG_EMGSTOP;                           // Nút nhấn dừng khẩn cấp được kích hoạt
}

/**
 * @brief In chi tiết mã lỗi Hex và giải nghĩa trạng thái hoạt động hiện tại của motor ra console
 */
void printMotionStatus(size_t axis, const EZISERVO_AXISSTATUS& st) {
    std::cout << "Axis " << (axis + 1) << " status=0x" << std::hex << st.dwValue << std::dec
              << " servo=" << st.FFLAG_SERVOON << " motion=" << st.FFLAG_MOTIONING
              << " error=" << st.FFLAG_ERRORALL << " hw+=" << st.FFLAG_HWPOSILMT
              << " hw-=" << st.FFLAG_HWNEGALMT << " sw+=" << st.FFLAG_SWPOGILMT
              << " sw-=" << st.FFLAG_SWNEGALMT << " posOverflow=" << st.FFLAG_ERRPOSOVERFLOW
              << " overSpeed=" << st.FFLAG_ERROVERSPEED << " posTracking=" << st.FFLAG_ERRPOSTRACKING
              << " overload=" << st.FFLAG_ERROVERLOAD << " inPosition=" << st.FFLAG_ERRINPOSITION
              << " emg=" << st.FFLAG_EMGSTOP << std::endl;
}

/**
 * @brief Ước lượng thời gian Timeout an toàn dựa trên khoảng cách di chuyển thực tế và vận tốc chạy
 */
std::chrono::milliseconds estimateMoveTimeout(const AxisPositions& current,
                                              const AxisPositions& targets,
                                              const AxisVelocities& velocities,
                                              const AxisBools& hasCommand) {
    double longestSeconds = 0.0;
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        if (!hasCommand[i] || velocities[i] <= 0) continue;
        const auto distance = std::llabs(static_cast<long long>(targets[i]) - static_cast<long long>(current[i]));
        longestSeconds = std::max(longestSeconds, static_cast<double>(distance) / velocities[i]);
    }
    // Nhân hệ số an toàn 3 lần + cộng bù 3 giây hệ thống phòng hờ driver tăng tốc chậm
    const auto estimated = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(longestSeconds * 3.0 + 3.0));
    return std::max(std::chrono::milliseconds(estimated), std::chrono::milliseconds(5000));
}

/**
 * @brief Vòng lặp giám sát phần cứng thời gian thực cho tới khi tất cả các trục hoàn thành chuyển động
 */
bool waitForMotionComplete(const AxisPorts& ports, const AxisSlaves& slaves, const AxisBools& activeAxes, std::chrono::milliseconds timeout, const char* label) {
    AxisStatuses statuses{};
    const auto deadline = std::chrono::steady_clock::now() + timeout;

    while (true) {
        bool allDone = true;

        for (size_t i = 0; i < AXIS_COUNT; ++i) {
            if (!activeAxes[i]) continue;

            // Đọc liên tục bit trạng thái thanh ghi Driver từ thư viện DLL/SO của Fastech
            const int ret = FAS_GetAxisStatus(ports[i], slaves[i], &(statuses[i].dwValue));
            if (ret != FMM_OK) {
                std::cout << label << " failed to read status for axis " << (i + 1) << ": " << ret << std::endl;
                stopAllAxes(ports, slaves);
                return false;
            }

            // Nếu phát hiện bất kỳ lỗi phần cứng nào nguy hiểm, ngay lập tức ngắt toàn bộ hệ thống
            if (hasMotionFault(statuses[i])) {
                std::cout << label << " stopped because axis " << (i + 1) << " reported a fault/limit." << std::endl;
                printMotionStatus(i, statuses[i]);
                stopAllAxes(ports, slaves);
                return false;
            }

            // Nếu motor vẫn đang quay (bit MOTIONING tích cực), đánh dấu chưa hoàn thành vòng lặp
            if (statuses[i].FFLAG_MOTIONING) {
                allDone = false;
            }
        }

        if (allDone) return true; // Tất cả động cơ dừng an toàn tại đích lý tưởng

        // Kiểm tra nếu vượt quá thời gian Timeout cho phép chuyển động tự động
        if (std::chrono::steady_clock::now() >= deadline) {
            std::cout << label << " timeout while waiting for driver running bit to clear." << std::endl;
            for (size_t i = 0; i < AXIS_COUNT; ++i) {
                if (activeAxes[i]) printMotionStatus(i, statuses[i]);
            }
            stopAllAxes(ports, slaves);
            return false;
        }

        std::this_thread::sleep_for(kMotionPollInterval); // Nghỉ 10ms tránh nghẽn CPU luồng giám sát
    }
}
} // KẾT THÚC NAMESPACE ẨN

// --- CÁC HÀM THÀNH VIÊN LỚP AXISCONTROLLER ---

AxisController::AxisController() {
    int portDefaults[6] = {0, 0, 0, 0, 0, 0};
    unsigned char slaveDefaults[6] = {0, 1, 2, 3, 4, 5};
    const double gearDefaults[6] = {40.0, 81.0, 5.0, 10.0, 50.0, 1.0}; // Cấu hình tỷ số truyền hộp số cho từng motor robot
    gear_ratios_.fill(1.0);
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        nPortIDs_[i] = portDefaults[i];
        iSlaveNos_[i] = slaveDefaults[i];
        gear_ratios_[i] = gearDefaults[i];
    }
}

AxisController::~AxisController() {
    stopRecordingThread(); // Bảo đảm luồng record ngầm được giải phóng an toàn khi hủy lớp
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        FAS_Close(nPortIDs_[i]); // Đóng các cổng vật lý kết nối Comport/Serial
    }
}

// Định danh tên cơ cấu Motor hiển thị trên giao diện người dùng
std::string AxisController::axisName(size_t idx) const {
    return "Motor " + std::to_string(idx + 1);
}

/**
 * @brief THUẬT TOÁN NÉN QUỸ ĐẠO: Loại bỏ bớt điểm trùng, lọc bớt điểm nhiễu trên đường thẳng để giảm dung lượng bộ đệm
 */
AxisVectorsEx compressBuffer(const AxisVectorsEx& in, int posEps, int minTrendLen) {
    AxisVectorsEx out;
    if (in[0].empty()) return out;
    size_t N = in[0].size();
    if (N < 2) return in;

    std::vector<bool> keep(N, false);
    keep[0] = true; // Luôn giữ điểm bắt đầu quỹ đạo đầu tiên

    for (size_t a = 0; a < AXIS_COUNT; ++a) {
        int lastDir = 0;
        int trendLen = 0;

        for (size_t i = 1; i < N; ++i) {
            int diff = in[a][i] - in[a][i - 1];
            if (std::abs(diff) < posEps) continue; // Bỏ qua nếu độ lệch vị trí nhỏ hơn sai số cấu hình Cắt nhiễu (posEps)

            int dir = (diff > 0) ? 1 : -1; // Xác định hướng chuyển động cơ học (Tiến/Lùi)

            if (dir == lastDir) {
                trendLen++; // Nếu đang chuyển động thẳng đều cùng hướng, tích lũy độ dài chuỗi ổn định
            } else {
                if (trendLen >= minTrendLen) {
                    keep[i - 1] = true; // Lưu lại điểm biên bẻ lái bước ngoặt đổi hướng cơ khí
                }
                trendLen = 1;
                lastDir = dir;
            }
        }
        keep[N - 1] = true; // Luôn giữ điểm chốt chặn cuối cùng quỹ đạo
    }

    // ĐIỀU KIỆN CÔNG NGHỆ: Tuyệt đối không được nén nếu tại thời điểm đó có lệnh chuyển trạng thái Đóng/Ngắt mỏ hàn
    for (size_t i = 1; i < N; ++i) {
        if (in[AXIS_COUNT][i] != in[AXIS_COUNT][i - 1]) {
            keep[i] = true;
            keep[i - 1] = true;
        }
    }

    // Đẩy dữ liệu đã nén sạch vào bộ đệm đầu ra
    for (size_t i = 0; i < N; ++i) {
        if (!keep[i]) continue;
        for (size_t a = 0; a < EXT_AXIS_COUNT; ++a) {
            out[a].push_back(in[a][i]);
        }
    }
    return out;
}

// --- QUẢN LÝ CHẾ ĐỘ HOẠT ĐỘNG (CAPTURE MODE) ---
bool AxisController::isRecording() const { return recording_.load(); }
void AxisController::setModeSave() { capture_mode_.store(static_cast<int>(CaptureMode::ManualSave)); }
void AxisController::setModeRecord() { capture_mode_.store(static_cast<int>(CaptureMode::AutoRecord)); }
bool AxisController::isSaveMode() const { return capture_mode_.load() == static_cast<int>(CaptureMode::ManualSave); }
void AxisController::setModeFile() { capture_mode_.store(static_cast<int>(CaptureMode::FileMode)); }
bool AxisController::isFileMode() const { return capture_mode_.load() == static_cast<int>(CaptureMode::FileMode); }
std::string AxisController::modeName() const {
    return isSaveMode() ? "MANUAL SAVE" : isFileMode() ? "FILE" : "AUTO RECORD";
}

// Đọc đồng loạt thanh ghi trạng thái phần cứng của tất cả các trục động cơ
bool AxisController::readAxisStatuses(AxisStatuses& statuses) {
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        if (FAS_GetAxisStatus(nPortIDs_[i], iSlaveNos_[i], &(statuses[i].dwValue)) != FMM_OK) return false;
    }
    return true;
}

// Đọc giá trị tọa độ thực tế từ Encoder phản hồi về Driver
bool AxisController::readActualPositions(AxisPositions& positions) {
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        if (FAS_GetActualPos(nPortIDs_[i], iSlaveNos_[i], &positions[i]) != FMM_OK) return false;
    }
    return true;
}

/**
 * @brief Tính toán vận tốc phân phối thủ công cho chuyển động điểm-điểm thông thường (không chạy stream)
 */
AxisVelocities AxisController::computeVelocities(const AxisPositions& current, const AxisPositions& targets, const AxisBools& hasCommand) const {
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
                auto scaled = static_cast<unsigned int>((static_cast<long long>(distance) * axisBaseVelocity) / maxDistance);
                velocities[i] = static_cast<int>(std::max(min_velocity, scaled));
            }
        }
    }
    return velocities;
}

/**
 * @brief Trả về vận tốc cơ sở của từng trục sau khi tính toán bù trừ qua Tỷ số truyền và giá trị Override cấu hình từ GUI
 */
unsigned int AxisController::baseVelocityForAxis(size_t axisIdx) const {
    const double ratio = (axisIdx < gear_ratios_.size() && gear_ratios_[axisIdx] > 0.0) ? gear_ratios_[axisIdx] : 1.0;
    const double configuredBase = base_velocity_override_.load() > 0.0 ? base_velocity_override_.load() : static_cast<double>(base_velocity);
    const double velocity = std::round(configuredBase * ratio);
    if (velocity > static_cast<double>(std::numeric_limits<unsigned int>::max())) {
        return std::numeric_limits<unsigned int>::max();
    }
    return std::max(1u, static_cast<unsigned int>(velocity));
}

// Kiểm tra xem tất cả động cơ đã kích hoạt trạng thái giữ dòng (Servo kích hoạt khóa trục thành công) hay chưa
bool AxisController::allServoOn(const AxisStatuses& statuses) const {
    for (const auto& st : statuses) {
        if (!st.FFLAG_SERVOON) return false;
    }
    return true;
}

/**
 * @brief Khởi tạo hệ thống: Thử kết nối vật lý với toàn bộ cổng COM Driver thiết bị ngõ USB Linux
 */
void AxisController::initializeSystem() {
    const wchar_t* sPort = L"ttyUSB0"; // Cấu hình port kết nối USB Serial truyền thông RS485 đa điểm
    unsigned int dwBaudRate = 115200;

    bool allOk = true;
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        if (!Driver_Connection(sPort, dwBaudRate, nPortIDs_[i], iSlaveNos_[i])) {
            allOk = false;
            std::cout << axisName(i) << " connection failed." << std::endl;
        }
    }
    if (!allOk) {
        std::cout << "One or more connections failed. GUI will still open." << std::endl;
    }
}

// Bật dòng lực giữ từ từ cho toàn bộ hệ thống motor
bool AxisController::servoOn() {
    bool allOk = true;
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        if (!ServoOn(nPortIDs_[i], iSlaveNos_[i])) {
            allOk = false;
            std::cout << axisName(i) << " servo ON failed." << std::endl;
        }
    }
    if (allOk) std::cout << "All " << AXIS_COUNT << " servos ON successfully." << std::endl;
    return allOk;
}

// Tắt dòng lực giữ (thả tự do trục động cơ để đẩy/kéo bằng tay dễ dàng)
bool AxisController::servoOff() {
    bool allOk = true;
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        if (!ServoOff(nPortIDs_[i], iSlaveNos_[i])) {
            allOk = false;
            std::cout << axisName(i) << " servo OFF failed." << std::endl;
        }
    }
    if (allOk) std::cout << "All " << AXIS_COUNT << " servos OFF successfully." << std::endl;
    return allOk;
}

/**
 * @brief Hàm điều khiển Robot quay đồng loạt về tọa độ gốc O (vị trí Abs = 0)
 */
bool AxisController::home() {
    AxisStatuses statuses{};
    if (!readAxisStatuses(statuses) || !allServoOn(statuses)) {
        std::cout << "Some servos are OFF. Turn them ON before homing." << std::endl;
        return false;
    }

    std::cout << "Sending homing commands to all motors..." << std::endl;
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        // Gửi lệnh dịch chuyển tuyệt đối đồng bộ đưa toàn bộ trục về tọa độ 0
        if (FAS_MoveSingleAxisAbsPos(nPortIDs_[i], iSlaveNos_[i], 0, baseVelocityForAxis(i)) != FMM_OK) {
            std::cout << axisName(i) << " homing command failed." << std::endl;
            return false;
        }
    }

    std::cout << "Waiting for all motors to complete homing..." << std::endl;
    AxisBools activeAxes{};
    activeAxes.fill(true);
    if (!waitForMotionComplete(nPortIDs_, iSlaveNos_, activeAxes, kHomingTimeout, "Homing")) return false;

    AxisPositions finalPos{};
    if (readActualPositions(finalPos)) {
        std::cout << "Homing complete. Final positions: ";
        for (size_t i = 0; i < AXIS_COUNT; ++i) {
            std::cout << axisName(i) << "=" << finalPos[i] << ((i + 1 < AXIS_COUNT) ? ", " : "");
        }
        std::cout << std::endl;
    }
    return true;
}

/**
 * @brief LUỒNG CHẠY NGẦM (THREAD): Liên tục quét dữ liệu vị trí cơ học, tính toán thời gian thực vận tốc RPM động cơ
 */
void AxisController::recordingThread() {
    constexpr int RECORD_PERIOD_MS = 10; // Chu kỳ lấy mẫu chuỗi dữ liệu (10ms)
    int printCounter = 0;

    AxisPositions prevPos{};
    bool firstSample = true;

    while (recording_.load()) {
        AxisPositions pos{};
        AxisVelocities vel{};
        std::array<double, AXIS_COUNT> rpm_display{};
        bool inputSignal = false;

        // 1. Đọc vị trí thực tế của encoder và tín hiệu từ cảm biến ngoại vi chân IO rơ-le
        if (!readActualPositions(pos) || !readInputSignal(inputSignal)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(RECORD_PERIOD_MS));
            continue;
        }

        // 2. Tính toán tốc độ thực tế từ độ lệch góc di chuyển
        if (firstSample) {
            for (size_t i = 0; i < AXIS_COUNT; ++i) {
                vel[i] = 0; rpm_display[i] = 0.0;
            }
            firstSample = false;
        } else {
            for (size_t i = 0; i < AXIS_COUNT; ++i) {
                long deltaPos = pos[i] - prevPos[i];
                // Tính toán tần số xung pps (Pulses Per Second) thực tế
                double pps = std::abs(deltaPos) * (1000.0 / RECORD_PERIOD_MS);
                vel[i] = static_cast<unsigned int>(pps); 

                // Lớp bảo vệ chống chia cho số 0 gây lỗi sập toán học lõi vi xử lý (NaN/Inf)
                double current_ppr = (ppr_ > 0) ? ppr_ : 10000.0;
                double current_ratio = (gear_ratios_[i] > 0.0) ? gear_ratios_[i] : 1.0;

                // Công thức tính vận tốc vòng/phút trước và sau hộp số giảm tốc cơ học
                double motor_rpm = (pps / current_ppr) * 60.0;
                rpm_display[i] = motor_rpm / current_ratio;
            }
        }

        prevPos = pos;

        // 3. In thông số giám sát chuyển động ra màn hình Console bằng luồng C++ chuẩn định dạng an toàn
        printCounter++;
        if (printCounter >= 50) { // Cứ sau 500ms (50 mẫu * 10ms) thì cập nhật log một lần tránh tràn console
            std::cout << "[Recording] ";
            for (size_t i = 0; i < AXIS_COUNT; ++i) {
                std::cout << "M" << i+1 << "(Pos:" << pos[i] << ", RPM:" 
                          << std::fixed << std::setprecision(2) << rpm_display[i] << ")  ";
            }
            std::cout << std::endl;
            printCounter = 0;
        }

        // 4. Khóa Mutex an toàn luồng, nạp toàn bộ mảng dữ liệu vào bộ đệm lưu trữ
        {
            std::lock_guard<std::mutex> lk(rec_mtx_);
            for (size_t i = 0; i < AXIS_COUNT; ++i) {
                recorded_positions_[i].push_back(pos[i]);
                recorded_velocities_[i].push_back(vel[i]);
            }
            g_recordedRelayStates.push_back(inputSignal ? 1 : 0);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(RECORD_PERIOD_MS));
    }
}

// Bắt đầu kích hoạt luồng ghi quỹ đạo tự động ngầm
void AxisController::record() {
    if (recording_.load()) { std::cout << "Already recording." << std::endl; return; }
    {
        std::lock_guard<std::mutex> lk(rec_mtx_);
        for (auto& v : recorded_positions_) v.clear();
        g_recordedRelayStates.clear();
        for (auto& v : recorded_velocities_) v.clear();
        for (auto& v : recorded_rpms_) v.clear();
    }
    recording_.store(true);
    rec_thread_ = std::thread(&AxisController::recordingThread, this);
    std::cout << "Recording started." << std::endl;
}

// Ngắt luồng luân chuyển lấy mẫu ghi dữ liệu
void AxisController::stopRecordingThread() {
    if (recording_.load()) recording_.store(false);
    if (rec_thread_.joinable()) rec_thread_.join();
}

// Dừng quá trình ghi tự động và tự động lũy tiến số thứ tự File ghi lưu trữ tiếp theo
void AxisController::stop() {
    if (!recording_.load()) { std::cout << "Not recording." << std::endl; return; }
    stopRecordingThread();

    std::lock_guard<std::mutex> lk(rec_mtx_);
    std::cout << "Recording stopped." << std::endl;
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        std::cout << axisName(i) << " samples: " << recorded_positions_[i].size() << std::endl;
    }
    record_file_index_++;
}

// Xóa sạch bộ nhớ toàn bộ các bộ đệm dữ liệu đang lưu trong RAM
void AxisController::clear() {
    std::lock_guard<std::mutex> lk(rec_mtx_);
    for (auto& path : recorded_positions_) path.clear();
    g_recordedRelayStates.clear();
    for (auto& v : recorded_rpms_) v.clear();
    for (auto& v : recorded_velocities_) v.clear();
    std::cout << "Cleared recorded positions for all motors." << std::endl;
}

bool AxisController::getPos(AxisPositions& pos) { return readActualPositions(pos); }
bool AxisController::goWithBuffer(const AxisVectors& paths) { return goWithBuffer(makeExtendedPaths(paths, {})); }

/**
 * @brief TRÁI TIM BỘ ĐIỀU KHIỂN (HÀM PHÁT QUỸ ĐẠO STREAMING OVERRIDE CHẠY REALTIME)
 */
bool AxisController::goWithBuffer(const AxisVectorsEx& paths) {
    if (paths[0].empty()) return false;

    AxisStatuses statuses{};
    if (!readAxisStatuses(statuses) || !allServoOn(statuses)) return false;
    const AxisVectorsEx& runPaths = paths;
    size_t steps = runPaths[0].size();
    
    // Kiểm tra tính hợp lệ của băng thông lệnh (Tránh gửi dồn dập vượt quá khả năng xử lý phần cứng)
    const auto command_window = std::chrono::duration_cast<std::chrono::microseconds>(kCommandSpacing * AXIS_COUNT);
    const auto segment_period_us = g_segment_period;
    if (segment_period_us < command_window) {
        std::cout << "Segment period is shorter than command window. Minimum requested error!" << std::endl;
        return false;
    }

    // --- BẬT ĐÈ THỜI GIAN ĐÁP ỨNG DRIVER ---
    // Sao lưu thời gian gia tốc gốc trong EEPROM của động cơ để tránh làm mất cấu hình chuẩn của máy
    AxisPositions savedAccel{}; AxisPositions savedDecel{};
    bool timingSaved = readAllMotionTiming(nPortIDs_, iSlaveNos_, kAccelerationTimeParameter, savedAccel, "accel time") &&
                       readAllMotionTiming(nPortIDs_, iSlaveNos_, kDecelerationTimeParameter, savedDecel, "decel time");
    
    // Nạp đè cấu hình gia tốc cực ngắn (5ms) để motor đáp ứng đổi quỹ đạo lập tức theo lệnh Stream
    if (!setAllMotionTiming(nPortIDs_, iSlaveNos_, kAccelerationTimeParameter, kStreamingAccelTimeMs, "streaming accel time") ||
        !setAllMotionTiming(nPortIDs_, iSlaveNos_, kDecelerationTimeParameter, kStreamingDecelTimeMs, "streaming decel time")) {
        if (timingSaved) {
            restoreAllMotionTiming(nPortIDs_, iSlaveNos_, kAccelerationTimeParameter, savedAccel, "accel time");
            restoreAllMotionTiming(nPortIDs_, iSlaveNos_, kDecelerationTimeParameter, savedDecel, "decel time");
        }
        return false;
    }

    // Biểu thức Lambda giải phóng tài nguyên hệ thống, khôi phục cấu hình Driver khi thoát hàm kết thúc chạy quỹ đạo
    auto finishRun = [&](bool ok) {
        if (timingSaved) {
            restoreAllMotionTiming(nPortIDs_, iSlaveNos_, kAccelerationTimeParameter, savedAccel, "accel time");
            restoreAllMotionTiming(nPortIDs_, iSlaveNos_, kDecelerationTimeParameter, savedDecel, "decel time");
        }
        if (ok) std::cout << "GO finished." << std::endl;
        return ok;
    };

    AxisPositions previous{};
    if (!readActualPositions(previous)) return finishRun(false);

    AxisBools axisStreamingStarted{}; axisStreamingStarted.fill(false);
    bool lastRelayState = false; bool firstBlock = true;

    // VÒNG LẶP CHÍNH DỊCH CHUYỂN QUỸ ĐẠO THEO TỪNG ĐIỂM (PHÂN ĐOẠN)
    for (size_t i = 0; i < steps; ++i) {
        // --- XỬ LÝ ĐỒNG BỘ NGOẠI VI (MỎ HÀN/RƠ-LE) ---
        const bool relayState = (runPaths[AXIS_COUNT][i] != 0);
        if (firstBlock || relayState != lastRelayState) {
            std::cout << ">> [MOTION] Yeu cau IO thay doi -> " << (relayState ? "ON" : "OFF") << std::endl;
            if (!setRelay(relayState)) return finishRun(false); // Bật/Tắt mỏ hàn công nghệ đồng bộ với điểm di chuyển
            lastRelayState = relayState; firstBlock = false;
        }

        const auto segmentStart = std::chrono::steady_clock::now();
        auto nextCommandTime = segmentStart;
        AxisPositions target{}; AxisPositions nextTarget{}; AxisBools hasCommand{};

        for (size_t a = 0; a < AXIS_COUNT; ++a) {
            target[a] = runPaths[a][i];
            nextTarget[a] = (i + 1 < steps) ? runPaths[a][i + 1] : target[a];
            hasCommand[a] = (target[a] != previous[a]);
        }

        // Thực hiện thuật toán nội suy bo góc cua Lookahead phối hợp với chu kỳ thời gian thực
        const double speedScale = (i + 1 < steps) ? cornerSpeedScale(previous, target, nextTarget) : 1.0;
        const auto active_segment_period_us = scaledSegmentPeriod(segment_period_us, speedScale);
        AxisVelocities velocities = computeSegmentVelocities(previous, target, active_segment_period_us);

        // GỬI LỆNH ĐIỀU KHIỂN PHẦN CỨNG XUỐNG TỪNG TRỤC MOTOR
        for (size_t a = 0; a < AXIS_COUNT; ++a) {
            if (!hasCommand[a]) continue;
            const auto now = std::chrono::steady_clock::now();
            if (now < nextCommandTime) std::this_thread::sleep_until(nextCommandTime); // Giãn cách phân bổ các tập lệnh RS485 1.5ms

#ifdef AXIS_CONTROLLER_OVERRIDE_MODE
            // NẾU CHƯA KÍCH HOẠT: Kích hoạt lệnh di chuyển gốc khởi tạo trạng thái Motioning cho Driver
            if (!axisStreamingStarted[a]) {
                const int startRet = FAS_MoveSingleAxisAbsPos(nPortIDs_[a], iSlaveNos_[a], target[a], velocities[a]);
                if (startRet == FMM_OK) {
                    axisStreamingStarted[a] = true; nextCommandTime += kCommandSpacing; continue;
                }
                if (startRet != FMP_RUNFAIL) {
                    std::cout << axisName(a) << " initial streaming move failed: " << startRet << std::endl;
                    return finishRun(false);
                }
                axisStreamingStarted[a] = true;
            }

            // NẾU ĐANG QUAY: Liên tục nạp đè cưỡng bức Vận tốc & Vị trí đích mới ngay khi Motor đang quay (Không dừng cơ khí)
            const int velRet = FAS_VelocityOverride(nPortIDs_[a], iSlaveNos_[a], velocities[a]);
            const int posRet = FAS_PositionAbsOverride(nPortIDs_[a], iSlaveNos_[a], target[a]);
            if (velRet != FMM_OK || posRet != FMM_OK) {
                // Thử khởi động lại tập lệnh gốc nếu lệnh override tạm thời bị rớt khung truyền thông
                const int startRet = FAS_MoveSingleAxisAbsPos(nPortIDs_[a], iSlaveNos_[a], target[a], velocities[a]);
                if (startRet != FMM_OK && startRet != FMP_RUNFAIL) return finishRun(false);
            }
            nextCommandTime += kCommandSpacing;
#else
            // Chế độ dự phòng chạy nội suy cứng phân mảnh lệnh trực tiếp thông thường
            int ret = FAS_MoveSingleAxisAbsPos(nPortIDs_[a], iSlaveNos_[a], target[a], velocities[a]);
            if (ret == FMP_RUNFAIL) {
                const int velRet = FAS_VelocityOverride(nPortIDs_[a], iSlaveNos_[a], velocities[a]);
                const int posRet = FAS_PositionAbsOverride(nPortIDs_[a], iSlaveNos_[a], target[a]);
                if (velRet != FMM_OK || posRet != FMM_OK) return finishRun(false);
                nextCommandTime += kCommandSpacing; continue;
            }
            if (ret != FMM_OK) return finishRun(false);
            nextCommandTime += kCommandSpacing;
#endif
        }

        previous = target;
        // Chờ đồng bộ thời gian hoàn thành phân đoạn trước khi nạp điểm tiếp theo trong chuỗi quỹ đạo
        if (i + 1 < steps) {
            const auto nextSegmentTime = segmentStart + active_segment_period_us;
            const auto now = std::chrono::steady_clock::now();
            if (now < nextSegmentTime) std::this_thread::sleep_until(nextSegmentTime);
        }
    }

    // Chờ cho robot giảm tốc ổn định dừng hẳn cơ học hoàn toàn tại điểm cuối cùng
    AxisBools activeAxes{}; activeAxes.fill(true);
    if (!waitForMotionComplete(nPortIDs_, iSlaveNos_, activeAxes, kFinalSettleTimeout, "GO final settle")) {
        return finishRun(false);
    }

    return finishRun(true);
}

/**
 * @brief ĐỌC QUỸ ĐẠO TỪ FILE SẴN CÓ VÀ PHÁT CHUYỂN ĐỘNG CHO ROBOT
 */
bool AxisController::goFromFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) { std::cout << "Cannot open file: " << filename << std::endl; return false; }

    std::vector<AxisPositions> pathBuffer;
    std::vector<int> relayBuffer;
    std::string line;
    double fileSegmentSeconds = -1.0; double fileMoveTimeSeconds = -1.0;

    // Vòng lặp giải mã dữ liệu text từng dòng một của file cấu hình tọa độ CNC/Robot
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        const auto firstNonSpace = line.find_first_not_of(" \t\r\n");
        if (firstNonSpace == std::string::npos) continue;
        const char first = line[firstNonSpace];
        
        // Phát hiện dòng chứa thông tin cấu hình Header của tập lệnh điều khiển máy
        if (first == '#' || first == ';') {
            readTimingHeader(line, fileSegmentSeconds, fileMoveTimeSeconds); continue;
        }

        std::stringstream ss(line);
        std::vector<int> row; int val;
        while (ss >> val) row.push_back(val);

        if (row.size() < AXIS_COUNT) {
            std::cout << "Format error at line: " << line << "\n"; return false;
        }

        AxisPositions pos{};
        for (size_t i = 0; i < AXIS_COUNT; i++) pos[i] = row[i];
        
        // Giải mã cột quy định kích hoạt linh kiện phụ mỏ hàn đi kèm tọa độ di chuyển
        int relayState = 0;
        if (row.size() > FILE_RELAY_COLUMN) relayState = row[FILE_RELAY_COLUMN];
        relayBuffer.push_back(relayState);
        pathBuffer.push_back(pos);
    }

    // Áp dụng thuật toán tính chu kỳ định thì phân phối bước di chuyển tối ưu dựa vào Header file dữ liệu
    if (fileSegmentSeconds > 0.0) {
        setSegmentPeriodSeconds(fileSegmentSeconds);
    } else if (fileMoveTimeSeconds > 0.0 && pathBuffer.size() > 1) {
        const double segmentSeconds = fileMoveTimeSeconds / static_cast<double>(pathBuffer.size() - 1);
        setSegmentPeriodSeconds(segmentSeconds);
    } else {
        setSegmentPeriodSeconds(kDefaultSegmentSeconds);
    }

    // Chuyển đổi định dạng mảng dòng sang định dạng mảng trục mở rộng tương thích đầu vào của lõi Stream
    AxisVectorsEx compatiblePaths;
    for (const auto& row : pathBuffer) {
        for (size_t i = 0; i < AXIS_COUNT; i++) compatiblePaths[i].push_back(row[i]);
    }
    compatiblePaths[AXIS_COUNT] = relayBuffer;
    std::string rpmFile = "rpm_" + std::to_string(record_file_index_) + ".txt";

    std::cout << "Saved RPM to " << rpmFile << std::endl;
    record_file_index_++;
    
    // Đẩy toàn bộ cấu trúc dữ liệu hoàn chỉnh giải mã từ file trực tiếp vào lõi xử lý phần cứng chuyển động
    return goWithBuffer(compatiblePaths); 
}

void AxisController::setFileName(const std::string& name) { filename_ = name; }
std::string AxisController::FileName() const { return filename_; }
void AxisController::setBaseVelocityOverride(double velocity) { base_velocity_override_.store(velocity > 0.0 ? velocity : -1.0); }

/**
 * @brief Hàm kích hoạt vận hành chuyển động thông minh tự động lựa chọn nguồn dữ liệu theo chế độ cấu hình GUI
 */
bool AxisController::go() {
    auto mode = capture_mode_.load();
    if (mode == static_cast<int>(CaptureMode::FileMode)) {
        if (filename_.empty()) { std::cout << "File path is empty." << std::endl; return false; }
        return goFromFile(filename_); // Luồng di chuyển đọc nạp dữ liệu từ tệp tin lưu trữ CNC
    }
    else if (mode == static_cast<int>(CaptureMode::ManualSave)) {
        AxisVectorsEx paths;
        {
            std::lock_guard<std::mutex> lk(rec_mtx_);
            paths = makeExtendedPaths(saved_positions_, g_savedRelayStates);
        }
        if (paths[0].empty()) { std::cout << "No manually saved positions to run." << std::endl; return false; }
        std::cout << "Executing Manual Save positions..." << std::endl;
        return goWithBuffer(paths); // Luồng chuyển động theo các nút bấm ghi tọa độ thủ công của người vận hành máy
    }
    AxisVectorsEx paths;
    {
        std::lock_guard<std::mutex> lk(rec_mtx_);
        paths = makeExtendedPaths(recorded_positions_, g_recordedRelayStates);
    }
    return goWithBuffer(paths); // Mặc định chạy lại chuỗi vết chuyển động đã tự động ghi (Record) trước đó
}

/**
 * @brief Điều khiển chuyển động Point-to-Point (Điểm tới Điểm) đơn lẻ thông thường
 */
bool AxisController::movePos(const AxisPositions& targets) {
    AxisStatuses statuses{};
    if (!readAxisStatuses(statuses) || !allServoOn(statuses)) {
        std::cout << "Some servos are OFF. Turn them ON before moving." << std::endl; return false;
    }

    AxisPositions current{};
    if (!readActualPositions(current)) return false;

    AxisBools hasCommand{};
    for (size_t i = 0; i < AXIS_COUNT; ++i) hasCommand[i] = (targets[i] != current[i]);

    AxisVelocities velocities = computeVelocities(current, targets, hasCommand);

    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        if (!hasCommand[i]) continue;
        if (FAS_MoveSingleAxisAbsPos(nPortIDs_[i], iSlaveNos_[i], targets[i], velocities[i]) != FMM_OK) {
            std::cout << axisName(i) << " move command failed." << std::endl; return false;
        }
    }

    std::cout << "Waiting for motors to complete movement..." << std::endl;
    const auto timeout = estimateMoveTimeout(current, targets, velocities, hasCommand);
    if (!waitForMotionComplete(nPortIDs_, iSlaveNos_, hasCommand, timeout, "Move")) return false;

    AxisPositions finalPos{};
    if (readActualPositions(finalPos)) {
        std::cout << "Movement complete. Final positions: ";
        for (size_t i = 0; i < AXIS_COUNT; ++i) {
            std::cout << axisName(i) << "=" << finalPos[i] << ((i + 1 < AXIS_COUNT) ? ", " : "");
        }
        std::cout << std::endl;
    }
    return true;
}

/**
 * @brief Hàm vận hành robot theo dữ liệu bảng vị trí lưu nội tại trong EEPROM của Ezi-Servo (Position Table Mode)
 */
bool AxisController::goPos() {
    AxisStatuses statuses{};
    if (!readAxisStatuses(statuses) || !allServoOn(statuses)) return false;

    constexpr unsigned short maxItems = 64;
    std::array<std::vector<ITEM_NODE>, AXIS_COUNT> tableItems;

    // Đọc đồng loạt mảng dữ liệu Position Table lưu sẵn dưới Driver phần cứng
    for (size_t axis = 0; axis < AXIS_COUNT; ++axis) {
        for (unsigned short wItemNo = 1; wItemNo <= maxItems; ++wItemNo) {
            ITEM_NODE nodeItem;
            if (FAS_PosTableReadItem(nPortIDs_[axis], iSlaveNos_[axis], wItemNo, &nodeItem) != FMM_OK) break;
            bool likelyEmpty = (nodeItem.lPosition == 0 && nodeItem.dwMoveSpd == 0);
            if (!likelyEmpty) tableItems[axis].push_back(nodeItem);
        }
    }

    bool anyItems = false;
    for (const auto& list : tableItems) { if (!list.empty()) { anyItems = true; break; } }
    if (!anyItems) { std::cout << "Position tables have no items to run." << std::endl; return false; }

    size_t maxSteps = 0;
    for (const auto& list : tableItems) { if (list.size() > maxSteps) maxSteps = list.size(); }

    // Duyệt tuần tự để phát và kích hoạt lệnh chạy từng dòng phần tử trong Position Table
    for (size_t idx = 0; idx < maxSteps; ++idx) {
        AxisBools hasCommand{}; AxisPositions targets{}; AxisVelocities velocities{};
        for (size_t axis = 0; axis < AXIS_COUNT; ++axis) velocities[axis] = static_cast<int>(baseVelocityForAxis(axis));

        for (size_t axis = 0; axis < AXIS_COUNT; ++axis) {
            if (idx < tableItems[axis].size()) {
                const ITEM_NODE& node = tableItems[axis][idx];
                targets[axis] = node.lPosition;
                velocities[axis] = (node.dwMoveSpd > 0) ? node.dwMoveSpd : static_cast<int>(baseVelocityForAxis(axis));
                hasCommand[axis] = true;
            }
        }

        AxisPositions current{};
        if (!readActualPositions(current)) return false;

        for (size_t axis = 0; axis < AXIS_COUNT; ++axis) {
            if (!hasCommand[axis]) continue;
            if (FAS_MoveSingleAxisAbsPos(nPortIDs_[axis], iSlaveNos_[axis], targets[axis], velocities[axis]) != FMM_OK) return false;
        }

        const auto timeout = estimateMoveTimeout(current, targets, velocities, hasCommand);
        if (!waitForMotionComplete(nPortIDs_, iSlaveNos_, hasCommand, timeout, "Go pos")) return false;
    }
    return true;
}

/**
 * @brief GHI LƯU THỦ CÔNG (MANUAL SAVE): Đọc điểm hiện tại robot đang đứng và xuất nối tiếp (append) vào file dữ liệu lưu trữ
 */
bool AxisController::savePos() {
    if (!isSaveMode()) {
        std::cout << "Current mode is AUTO RECORD. Switch mode to MANUAL SAVE first." << std::endl; return false;
    }
    AxisPositions pos{};
    if (!readActualPositions(pos)) { std::cout << "Failed to read current positions. Save pos aborted." << std::endl; return false; }

    bool inputSignal = false; readInputSignal(inputSignal);

    // Lưu tọa độ điểm thủ công hiện tại vào RAM hệ thống phục vụ chuyển động nội tại tức thời
    {
        std::lock_guard<std::mutex> lk(rec_mtx_);
        for (size_t i = 0; i < AXIS_COUNT; ++i) saved_positions_[i].push_back(pos[i]);
        g_savedRelayStates.push_back(inputSignal ? 1 : 0);
    }

    std::cout << "Saved current position to manual buffer. Total manual points: " << saved_positions_[0].size() << std::endl;

    // --- XUẤT NỐI TIẾP VÀO FILE savePos.txt LƯU TRỮ TRÊN Ổ CỨNG MÁY TÍNH ---
    AxisVectorsEx temp;
    for (size_t i = 0; i < AXIS_COUNT; ++i) temp[i].push_back(pos[i]);
    for (size_t i = 0; i < AXIS_COUNT; ++i) temp[i].push_back(pos[i]);
    for (size_t i = 0; i < AXIS_COUNT; ++i) temp[i].resize(1);
    temp[AXIS_COUNT].push_back(inputSignal ? 1 : 0);

    // Kích hoạt ghi file ở chế độ append = true để không xóa dữ liệu các điểm đã bấm lưu trước đó
    if (writeBufferToFile("savePos.txt", temp, true)) {
        std::cout << "Saved to savePos.txt" << std::endl;
    } else {
        std::cout << "Failed to write savePos.txt" << std::endl;
    }
    return true;
}

/**
 * @brief Hàm truyền và đồng bộ dữ liệu quỹ đạo bộ nhớ RAM xuống thanh ghi Position Table vật lý của linh kiện Ezi-Servo
 */
void AxisController::posTable() {
    AxisVectors paths;
    { std::lock_guard<std::mutex> lk(rec_mtx_); paths = recorded_positions_; }

    bool anyData = false;
    for (const auto& p : paths) { if (!p.empty()) { anyData = true; break; } }
    if (!anyData) { std::cout << "No recorded positions to write. Use 'record' first." << std::endl; return; }

    constexpr unsigned short startItem = 1;
    std::array<unsigned int, AXIS_COUNT> wrote{};

    for (size_t axis = 0; axis < AXIS_COUNT; ++axis) {
        for (size_t i = 0; i < paths[axis].size(); ++i) {
            unsigned short wItemNo = static_cast<unsigned short>(startItem + i);
            ITEM_NODE nodeItem;
            if (FAS_PosTableReadItem(nPortIDs_[axis], iSlaveNos_[axis], wItemNo, &nodeItem) != FMM_OK) break;

            nodeItem.dwMoveSpd = 1000;          // Cấu hình vận tốc mặc định chạy bảng vị trí là 1000 pps
            nodeItem.lPosition = paths[axis][i]; // Nạp tọa độ ghi lưu
            nodeItem.wBranch = 0;
            nodeItem.wContinuous = 0;

            if (FAS_PosTableWriteItem(nPortIDs_[axis], iSlaveNos_[axis], wItemNo, &nodeItem) != FMM_OK) break;
            ++wrote[axis];
        }
    }
    std::cout << "Position tables updated." << std::endl;
}

/**
 * @brief In toàn bộ nội dung cấu hình lưu trữ bên trong Position Table của Driver phần cứng ra Console phục vụ Debug lỗi máy
 */
void AxisController::printTable() {
    constexpr unsigned short maxItems = 64;
    for (size_t axis = 0; axis < AXIS_COUNT; ++axis) {
        std::cout << "\n=== " << axisName(axis) << " Position Table ===" << std::endl;
        unsigned int shown = 0;
        for (unsigned short wItemNo = 1; wItemNo <= maxItems; ++wItemNo) {
            ITEM_NODE nodeItem;
            if (FAS_PosTableReadItem(nPortIDs_[axis], iSlaveNos_[axis], wItemNo, &nodeItem) != FMM_OK) break;

            bool likelyEmpty = (nodeItem.lPosition == 0 && nodeItem.dwMoveSpd == 0);
            if (likelyEmpty) continue; // Bỏ qua các hàng trống không chứa dữ liệu công nghệ CNC

            std::cout << axisName(axis) << " Item " << wItemNo << ": Pos=" << nodeItem.lPosition
                      << ", MoveSpd=" << nodeItem.dwMoveSpd << ", Cmd=" << nodeItem.wCommand
                      << ", Wait=" << nodeItem.wWaitTime << ", Cont=" << nodeItem.wContinuous
                      << ", Branch=" << nodeItem.wBranch << std::endl;
            ++shown;
        }
        if (shown == 0) std::cout << "No non-empty items found for " << axisName(axis) << "." << std::endl;
    }
}

/**
 * @brief Hàm Reset thiếp lập phần mềm cưỡng bức: Ép tọa độ vị trí hiện tại cơ học của robot thành gốc O ảo (Tọa độ bằng 0)
 */
void AxisController::setOriginPos() {
    AxisStatuses statuses{};
    if (!readAxisStatuses(statuses)) return;

    // Chỉ cho phép định nghĩa lại gốc tọa độ khi toàn bộ cơ cấu máy đứng yên hoàn toàn
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        if (statuses[i].FFLAG_MOTIONING) {
            std::cout << axisName(i) << " is moving. Stop motion before setpos." << std::endl; return;
        }
    }

    bool allOk = true;
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        // Đồng thời thiết lập lại cả thanh ghi tọa độ lệnh (Command) lẫn tọa độ thực từ Encoder phản hồi (Actual) về mức số 0
        if (FAS_SetCommandPos(nPortIDs_[i], iSlaveNos_[i], 0) != FMM_OK) allOk = false;
        if (FAS_SetActualPos(nPortIDs_[i], iSlaveNos_[i], 0) != FMM_OK) allOk = false;
    }
    if (allOk) std::cout << "All axes positions set to 0." << std::endl;
}

// Nạp dữ liệu thô từ file txt lưu tọa độ trực tiếp vào bộ đệm RAM của máy tính
bool AxisController::loadFileToBuffer(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) { std::cout << "Cannot open file: " << filename << std::endl; return false; }

    AxisVectors temp;
    for (auto& v : temp) v.clear();

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line); AxisPositions pos{};

        for (size_t i = 0; i < AXIS_COUNT; ++i) {
            if (!(ss >> pos[i])) { std::cout << "Format error." << std::endl; return false; }
        }
        for (size_t i = 0; i < AXIS_COUNT; ++i) temp[i].push_back(pos[i]);
    }
    file_buffer_ = temp;
    std::cout << "File buffer loaded. Steps: " << file_buffer_[0].size() << std::endl;
    return true;
}

/**
 * @brief Điều khiển kích hoạt trạng thái logic (High/Low) cho một chân Output vật lý bất kỳ trên Driver
 */
bool AxisController::setOutputSignal(size_t axisIdx, uint32_t outMask, bool state) {
    if (axisIdx >= AXIS_COUNT) return false;
    uint32_t dwSetMask = state ? outMask : 0;
    uint32_t dwClearMask = state ? 0 : outMask;
    return FAS_SetIOOutput(nPortIDs_[axisIdx], iSlaveNos_[axisIdx], dwSetMask, dwClearMask) == FMM_OK;
}

/**
 * @brief Đọc trạng thái tín hiệu điện (Có điện/Cách điện) hiện thời của chân Input cảm biến vật lý trên Driver
 */
bool AxisController::isInputActive(size_t axisIdx, uint32_t inMask, bool& active) {
    if (axisIdx >= AXIS_COUNT) return false;
    uint32_t dwInput = 0;
    if (FAS_GetIOInput(nPortIDs_[axisIdx], iSlaveNos_[axisIdx], &dwInput) == FMM_OK) {
        active = (dwInput & inMask) != 0; return true;
    }
    return false;
}

/**
 * @brief Hàm công nghệ: Đọc chân Digital Input chuyên dụng gán cho cảm biến ngoại vi mỏ hàn cơ khí
 */
bool AxisController::readInputSignal(bool& signal) {
    uint32_t input = 0;
    if (FAS_GetIOInput(nPortIDs_[0], iSlaveNos_[0], &input) != FMM_OK) return false;

    // Chân DI trên Ezi-Servo bắt đầu ánh xạ từ bit thứ 26 trong thanh ghi I/O 32-bit
    uint32_t bitShift = 26 + DI_INDEX;
    signal = (input & (1u << bitShift)) != 0;
    return true;
}

/**
 * @brief Hàm công nghệ: Kích hoạt đóng/ngắt Rơ-le điều khiển mỏ hàn thông qua chân vật lý Digital Output trên Driver 0
 */
bool AxisController::setRelay(bool on) {
    // Chân DO trên Ezi-Servo bắt đầu ánh xạ từ bit thứ 15 trong thanh ghi điều khiển Output
    uint32_t bitShift = 15 + DO_INDEX;
    uint32_t mask = (1u << bitShift);

    uint32_t setMask = on ? mask : 0;
    uint32_t clearMask = on ? 0 : mask;

    int res = FAS_SetIOOutput(nPortIDs_[0], iSlaveNos_[0], setMask, clearMask);

    std::cout << "[IO CONTROL] Driver 0 | Lenh gui di: " << (on ? "ON" : "OFF")
              << " (Mask 0x" << std::hex << mask << std::dec << ") | Phan hoi (0=OK): " << res << std::endl;

    return res == FMM_OK;
}

/**
 * @brief LUỒNG CHẠY NGẦM NGẮT KHẨN (THREAD): Giám sát công tắc hành trình cơ học để tự động lưu tọa độ khẩn cấp bảo vệ máy khi va chạm
 */
void AxisController::endstopThreadFunc(size_t triggerAxis, uint32_t endstopMask) {
    bool lastState = false;
    while (monitoring_endstop_.load()) {
        bool currentState = false;
        if (isInputActive(triggerAxis, endstopMask, currentState)) {
            if (currentState && !lastState) { // Thuật toán bắt Cạnh lên (Rising Edge - Khoảnh khắc vừa chạm công tắc)
                std::cout << "[Endstop] Triggered! Auto-saving position..." << std::endl;
                savePos(); // Gọi khẩn cấp hàm ghi lại tọa độ va chạm phục vụ công tác giám định cơ khí
            }
            lastState = currentState;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Tránh chiếm dụng luồng vô tận CPU
    }
}

// Bắt đầu kích hoạt luồng giám sát an toàn công tắc giới hạn máy
void AxisController::startEndstopMonitor(size_t triggerAxis, uint32_t endstopMask) {
    if (monitoring_endstop_.load()) return;
    monitoring_endstop_.store(true);
    endstop_thread_ = std::thread(&AxisController::endstopThreadFunc, this, triggerAxis, endstopMask);
    std::cout << "Endstop Monitor started on Axis " << triggerAxis + 1 << std::endl;
}

// Hủy giải phóng luồng giám sát công tắc hành trình giới hạn cơ học
void AxisController::stopEndstopMonitor() {
    if (monitoring_endstop_.load()) {
        monitoring_endstop_.store(false);
        if (endstop_thread_.joinable()) endstop_thread_.join();
    }
}

// --- CÁC HÀM XUẤT/GHI FILE DỮ LIỆU TỌA ĐỘ VÀ CÔNG NGHỆ ---
bool AxisController::writeBufferToFile(const std::string& filename, const AxisVectors& data, bool append) {
    std::ofstream file;
    if (append) file.open(filename, std::ios::app); else file.open(filename);
    if (!file.is_open()) { std::cout << "Cannot open file: " << filename << std::endl; return false; }

    size_t steps = data[0].size();
    for (size_t i = 0; i < steps; ++i) {
        for (size_t a = 0; a < AXIS_COUNT; ++a) {
            file << data[a][i] << ((a + 1 < AXIS_COUNT) ? " " : "");
        }
        file << "\n";
    }
    file.close(); return true;
}

bool AxisController::writeBufferToFile(const std::string& filename, const AxisVectorsEx& data, bool append) {
    std::ofstream file;
    if (append) file.open(filename, std::ios::app); else file.open(filename);
    if (!file.is_open()) { std::cout << "Cannot open file: " << filename << std::endl; return false; }

    size_t steps = data[0].size();
    for (size_t i = 0; i < steps; ++i) {
        for (size_t a = 0; a < EXT_AXIS_COUNT; ++a) {
            file << data[a][i] << ((a + 1 < EXT_AXIS_COUNT) ? " " : "");
        }
        file << "\n";
    }
    file.close(); return true;
}