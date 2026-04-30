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

    int fd = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        std::lock_guard<std::mutex> lk(errMtx_);
        lastError_ = "open failed: " + std::string(std::strerror(errno));
        return false;
    }

    struct termios tty {};
    if (tcgetattr(fd, &tty) != 0) {
        std::lock_guard<std::mutex> lk(errMtx_);
        lastError_ = "tcgetattr failed: " + std::string(std::strerror(errno));
        ::close(fd);
        return false;
    }

    cfmakeraw(&tty);
    speed_t speed = baudToTermios(baudRate);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

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
        case 9600:
            return B9600;
        case 19200:
            return B19200;
        case 38400:
            return B38400;
        case 57600:
            return B57600;
        case 115200:
        default:
            return B115200;
    }
}

void SerialController::readLoop() {
    std::string lineBuffer;
    lineBuffer.reserve(256);

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
        ssize_t n = ::read(fd, buf, sizeof(buf));
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                usleep(5000);
                continue;
            }
            std::lock_guard<std::mutex> lk(errMtx_);
            lastError_ = "read failed: " + std::string(std::strerror(errno));
            usleep(10000);
            continue;
        }
        if (n == 0) {
            usleep(5000);
            continue;
        }

        for (ssize_t i = 0; i < n; ++i) {
            const char c = buf[i];
            if (c == '\r') continue;
            if (c == '\n') {
                if (!lineBuffer.empty() && handler_) {
                    handler_(lineBuffer);
                }
                lineBuffer.clear();
                continue;
            }
            lineBuffer.push_back(c);
        }
    }
}

