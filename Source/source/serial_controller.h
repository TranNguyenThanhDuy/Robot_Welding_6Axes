#pragma once

#include <atomic>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <termios.h>

class SerialController {
public:
    using LineHandler = std::function<void(const std::string&)>;

    SerialController();
    ~SerialController();

    bool openPort(const std::string& device, int baudRate = 115200);
    void closePort();
    bool isOpen() const;

    void start(LineHandler handler);
    void stop();
    bool isRunning() const;

    bool sendLine(const std::string& line);
    std::string lastError() const;

private:
    static speed_t baudToTermios(int baudRate);
    void readLoop();

    int fd_ = -1;
    std::atomic<bool> running_{false};
    std::thread worker_{};
    LineHandler handler_{};

    mutable std::mutex ioMtx_{};
    mutable std::mutex errMtx_{};
    std::string lastError_{};
};
