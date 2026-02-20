#include "usb_camera.h"

#include <opencv2/imgproc.hpp>

#include <sstream>

UsbCamera::~UsbCamera() {
    closeDevice();
}

bool UsbCamera::applyCaptureOptions(uint32_t width, uint32_t height) {
    if (!capture_.isOpened()) {
        return false;
    }

    if (width > 0) {
        capture_.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(width));
    }
    if (height > 0) {
        capture_.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(height));
    }

    capture_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    capture_.set(cv::CAP_PROP_BUFFERSIZE, 1.0);
    return true;
}

bool UsbCamera::openDevice(const std::string& devicePath,
                           uint32_t width,
                           uint32_t height) {
    closeDevice();
    lastError_.clear();

    try {
        if (!capture_.open(devicePath, cv::CAP_V4L2)) {
            if (!capture_.open(0, cv::CAP_V4L2)) {
                lastError_ = "cannot open /dev/video0 with OpenCV V4L2";
                return false;
            }
        }

        if (!applyCaptureOptions(width, height)) {
            lastError_ = "failed to apply camera options";
            closeDevice();
            return false;
        }

        cv::Mat warmup;
        for (int i = 0; i < 5; ++i) {
            capture_.read(warmup);
        }

        if (warmup.empty()) {
            lastError_ = "camera opened but warmup frame is empty";
            closeDevice();
            return false;
        }
    } catch (const cv::Exception& e) {
        lastError_ = std::string("OpenCV exception: ") + e.what();
        closeDevice();
        return false;
    } catch (...) {
        lastError_ = "unknown exception while opening camera";
        closeDevice();
        return false;
    }

    return true;
}

void UsbCamera::closeDevice() {
    if (capture_.isOpened()) {
        capture_.release();
    }
}

bool UsbCamera::isOpen() const {
    return capture_.isOpened();
}

bool UsbCamera::readFrame(QImage& frame) {
    if (!capture_.isOpened()) {
        lastError_ = "camera is not opened";
        return false;
    }

    try {
        cv::Mat bgr;
        if (!capture_.read(bgr) || bgr.empty()) {
            lastError_ = "capture.read() returned empty frame";
            return false;
        }

        cv::Mat rgb;
        cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
        QImage image(rgb.data, rgb.cols, rgb.rows, rgb.step, QImage::Format_RGB888);
        frame = image.copy();
        return true;
    } catch (const cv::Exception& e) {
        lastError_ = std::string("OpenCV exception: ") + e.what();
        return false;
    } catch (...) {
        lastError_ = "unknown exception while reading frame";
        return false;
    }
}

std::string UsbCamera::lastError() const {
    return lastError_;
}

std::string UsbCamera::fourccToString(int fourcc) {
    char s[5] = {
        static_cast<char>(fourcc & 0xFF),
        static_cast<char>((fourcc >> 8) & 0xFF),
        static_cast<char>((fourcc >> 16) & 0xFF),
        static_cast<char>((fourcc >> 24) & 0xFF),
        '\0',
    };
    return std::string(s);
}

std::string UsbCamera::formatName() const {
    if (!capture_.isOpened()) {
        return "N/A";
    }
    std::ostringstream oss;
    oss << fourccToString(static_cast<int>(capture_.get(cv::CAP_PROP_FOURCC)));
    return oss.str();
}
