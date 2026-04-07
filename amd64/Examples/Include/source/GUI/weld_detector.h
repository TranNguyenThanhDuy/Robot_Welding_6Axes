#pragma once

#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>

#include <string>
#include <vector>

struct Detection {
    cv::Rect box;
    int classId = -1;
    float confidence = 0.0f;
};

class WeldDetector {
public:
    bool loadModel(const std::string& modelPath,
                   float confThreshold = 0.25f,
                   float nmsThreshold = 0.45f,
                   int inputSize = 640);

    bool isLoaded() const;
    std::string lastError() const;

    bool detect(const cv::Mat& bgrFrame, std::vector<Detection>& detections);
    void drawDetections(cv::Mat& bgrFrame,
                        const std::vector<Detection>& detections) const;

private:
    cv::Mat letterbox(const cv::Mat& src,
                      int targetSize,
                      float& scale,
                      int& padX,
                      int& padY) const;

    cv::dnn::Net net_;
    bool loaded_ = false;
    std::string lastError_;
    float confThreshold_ = 0.25f;
    float nmsThreshold_ = 0.45f;
    int inputSize_ = 640;
};
