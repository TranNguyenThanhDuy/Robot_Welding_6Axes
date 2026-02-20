#pragma once

#include <QImage>

#include <cstdint>
#include <string>

#include <opencv2/videoio.hpp>

class UsbCamera {
public:
    UsbCamera() = default;
    ~UsbCamera();

    bool openDevice(const std::string& devicePath,
                    uint32_t width = 640,
                    uint32_t height = 480);
    void closeDevice();
    bool isOpen() const;
    bool readFrame(QImage& frame);

    std::string lastError() const;
    std::string formatName() const;

private:
    bool applyCaptureOptions(uint32_t width, uint32_t height);
    static std::string fourccToString(int fourcc);

    cv::VideoCapture capture_;
    std::string lastError_;
};
