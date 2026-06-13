#include "serial_controller.h"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>

namespace {
constexpr int kReadBufferSize = 256;
}

SerialController::SerialController() = default;

SerialController::~SerialController() {
    stop();
    closePort();
}

bool SerialController::openPort(const std::string& device, int baudRate) {
    closePort();

    // Mở cổng ở chế độ Blocking chuẩn Linux
    int fd = ::open(device.c_str(), O_RDWR | O_NOCTTY);
    if (fd < 0) {
        std::lock_guard<std::mutex> lk(errMtx_);
        lastError_ = "open failed: " + std::string(std::strerror(errno));
        return false;
    }

    struct termios tty {};
    if (tcgetattr(fd, &tty) != 0) {
        std::lock_guard<std::mutex> lk(errMtx_);
        lastError_ = "tcgetattr failed: " + std::string(std::strerror(errno)); // <-- ĐÃ SỬA THÀNH lastError_
        ::close(fd);
        return false;
    }

    // Thiết lập cấu hình Raw Mode cơ bản để nhận dữ liệu ký tự thô
    cfmakeraw(&tty);
    speed_t speed = baudToTermios(baudRate);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    // Cấu hình khung truyền dữ liệu tiêu chuẩn (8-N-1)
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    // TẮT HOÀN TOÀN FLOW CONTROL (Khắc phục lỗi đơ cổng trên Linux/WSL/Raspberry Pi)
    tty.c_cflag &= ~CRTSCTS;                // Tắt Hardware Flow Control (RTS/CTS)
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Tắt Software Flow Control (XON/XOFF)

    // Cấu hình cơ chế chặn luồng thông minh kèm Timeout đọc dữ liệu
    tty.c_cc[VMIN] = 1;   // Trả về ngay khi nhận được tối thiểu 1 byte
    tty.c_cc[VTIME] = 1;  // Timeout giữ luồng đợi byte kế tiếp trong 0.1 giây (100ms)

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::lock_guard<std::mutex> lk(errMtx_);
        lastError_ = "tcsetattr failed: " + std::string(std::strerror(errno));
        ::close(fd);
        return false;
    }

    {
        std::lock_guard<std::mutex> lk(ioMtx_);
        fd_ = fd;
    }

    {
        std::lock_guard<std::mutex> lk(errMtx_);
        lastError_.clear();
    }
    return true;
}

void SerialController::closePort() {
    std::lock_guard<std::mutex> lk(ioMtx_);
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool SerialController::isOpen() const {
    std::lock_guard<std::mutex> lk(ioMtx_);
    return fd_ >= 0;
}

void SerialController::start(LineHandler handler) {
    if (running_.load()) {
        return;
    }
    handler_ = std::move(handler);
    running_.store(true);
    worker_ = std::thread(&SerialController::readLoop, this);
}

void SerialController::stop() {
    running_.store(false);
    if (worker_.joinable()) {
        worker_.join();
    }
}

bool SerialController::isRunning() const {
    return running_.load();
}

bool SerialController::sendLine(const std::string& line) {
    std::lock_guard<std::mutex> lk(ioMtx_);
    if (fd_ < 0) {
        std::lock_guard<std::mutex> elk(errMtx_);
        lastError_ = "send failed: port is not open";
        return false;
    }

    std::string payload = line;
    if (payload.empty() || payload.back() != '\n') {
        payload.push_back('\n');
    }

    ssize_t wrote = ::write(fd_, payload.data(), payload.size());
    if (wrote < 0 || static_cast<size_t>(wrote) != payload.size()) {
        std::lock_guard<std::mutex> elk(errMtx_);
        lastError_ = "write failed: " + std::string(std::strerror(errno));
        return false;
    }
    return true;
}

std::string SerialController::lastError() const {
    std::lock_guard<std::mutex> lk(errMtx_);
    return lastError_;
}

speed_t SerialController::baudToTermios(int baudRate) {
    switch (baudRate) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200:
        default:     return B115200;
    }
}

void SerialController::readLoop() {
    std::string lineBuffer;
    lineBuffer.reserve(256);
    std::cerr << "[SerialController] Read loop started\n";

    while (running_.load()) {
        int fd = -1;
        {
            std::lock_guard<std::mutex> lk(ioMtx_);
            fd = fd_;
        }

        if (fd < 0) {
            usleep(10000);
            continue;
        }

        char buf[kReadBufferSize];
        // Điểm chặn luồng tự động giải phóng CPU khi không có dữ liệu truyền lên
        ssize_t n = ::read(fd, buf, sizeof(buf));
        
        if (n < 0) {
            std::lock_guard<std::mutex> lk(errMtx_);
            lastError_ = "read failed: " + std::string(std::strerror(errno));
            std::cerr << "[SerialController] Read error: " << lastError_ << "\n";
            usleep(10000);
            continue;
        }
        if (n == 0) {
            continue;
        }
        
        std::cerr << "[SerialController] Read " << n << " bytes\n";

        for (ssize_t i = 0; i < n; ++i) {
            const char c = buf[i];
            
            // Chấp nhận linh hoạt mọi ký tự ngắt dòng (\r, \n hoặc \r\n) từ STM32 gửi lên
            if (c == '\r' || c == '\n') {
                if (!lineBuffer.empty()) {
                    std::cerr << "[SerialController] Invoking handler with: " << lineBuffer << "\n";
                    if (handler_) {
                        handler_(lineBuffer);
                    }
                    lineBuffer.clear();
                }
                continue;
            }
            lineBuffer.push_back(c);
        }
    }
}