#include "weld_detector.h"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <iomanip>
#include <sstream>

namespace {
constexpr float kPadValue = 114.0f;
}

bool WeldDetector::loadModel(const std::string& modelPath,
                             float confThreshold,
                             float nmsThreshold,
                             int inputSize) {
    loaded_ = false;
    lastError_.clear();

    if (inputSize <= 0) {
        lastError_ = "input size must be > 0";
        return false;
    }

    try {
        net_ = cv::dnn::readNet(modelPath);
    } catch (const cv::Exception& e) {
        lastError_ = std::string("failed to load model: ") + e.what();
        return false;
    }

    if (net_.empty()) {
        lastError_ = "loaded model is empty";
        return false;
    }

    confThreshold_ = confThreshold;
    nmsThreshold_ = nmsThreshold;
    inputSize_ = inputSize;

    loaded_ = true;
    return true;
}

bool WeldDetector::isLoaded() const {
    return loaded_;
}

std::string WeldDetector::lastError() const {
    return lastError_;
}

cv::Mat WeldDetector::letterbox(const cv::Mat& src,
                                int targetSize,
                                float& scale,
                                int& padX,
                                int& padY) const {
    const int w = src.cols;
    const int h = src.rows;

    scale = std::min(static_cast<float>(targetSize) / static_cast<float>(w),
                     static_cast<float>(targetSize) / static_cast<float>(h));

    const int nw = static_cast<int>(std::round(w * scale));
    const int nh = static_cast<int>(std::round(h * scale));

    cv::Mat resized;
    cv::resize(src, resized, cv::Size(nw, nh), 0.0, 0.0, cv::INTER_LINEAR);

    padX = (targetSize - nw) / 2;
    padY = (targetSize - nh) / 2;

    cv::Mat boxed(targetSize, targetSize, CV_8UC3, cv::Scalar(kPadValue, kPadValue, kPadValue));
    resized.copyTo(boxed(cv::Rect(padX, padY, nw, nh)));
    return boxed;
}

bool WeldDetector::detect(const cv::Mat& bgrFrame, std::vector<Detection>& detections) {
    detections.clear();

    if (!loaded_) {
        lastError_ = "model is not loaded";
        return false;
    }
    if (bgrFrame.empty()) {
        lastError_ = "input frame is empty";
        return false;
    }

    try {
        float scale = 1.0f;
        int padX = 0;
        int padY = 0;
        cv::Mat input = letterbox(bgrFrame, inputSize_, scale, padX, padY);

        cv::Mat blob = cv::dnn::blobFromImage(input, 1.0 / 255.0, cv::Size(inputSize_, inputSize_), cv::Scalar(), true, false);
        net_.setInput(blob);

        std::vector<cv::Mat> outputs;
        net_.forward(outputs, net_.getUnconnectedOutLayersNames());
        if (outputs.empty()) {
            lastError_ = "model output is empty";
            return false;
        }

        cv::Mat out = outputs[0];

        int rows = 0;
        int dims = 0;
        bool transposed = false;

        if (out.dims == 3) {
            const int d1 = out.size[1];
            const int d2 = out.size[2];

            if (d1 > d2) {
                rows = d2;
                dims = d1;
                transposed = true;
            } else {
                rows = d1;
                dims = d2;
            }

            out = out.reshape(1, {rows, dims});
            if (transposed) {
                cv::transpose(out, out);
            }
        } else if (out.dims == 2) {
            rows = out.rows;
            dims = out.cols;
        } else {
            lastError_ = "unexpected output dimension";
            return false;
        }

        if (dims < 5) {
            lastError_ = "unexpected output channel count";
            return false;
        }

        std::vector<cv::Rect> boxes;
        std::vector<float> scores;
        std::vector<int> classIds;

        for (int i = 0; i < out.rows; ++i) {
            const float* data = out.ptr<float>(i);

            const float obj = (dims > 5) ? data[4] : 1.0f;
            if (obj < 1e-6f) {
                continue;
            }

            int classId = 0;
            float classScore = 1.0f;
            if (dims > 5) {
                classScore = data[5];
                for (int c = 6; c < dims; ++c) {
                    if (data[c] > classScore) {
                        classScore = data[c];
                        classId = c - 5;
                    }
                }
            }

            const float score = obj * classScore;
            if (score < confThreshold_) {
                continue;
            }

            const float cx = data[0];
            const float cy = data[1];
            const float w = data[2];
            const float h = data[3];

            float x0 = cx - w * 0.5f;
            float y0 = cy - h * 0.5f;
            float x1 = cx + w * 0.5f;
            float y1 = cy + h * 0.5f;

            x0 = (x0 - static_cast<float>(padX)) / scale;
            y0 = (y0 - static_cast<float>(padY)) / scale;
            x1 = (x1 - static_cast<float>(padX)) / scale;
            y1 = (y1 - static_cast<float>(padY)) / scale;

            x0 = std::clamp(x0, 0.0f, static_cast<float>(bgrFrame.cols - 1));
            y0 = std::clamp(y0, 0.0f, static_cast<float>(bgrFrame.rows - 1));
            x1 = std::clamp(x1, 0.0f, static_cast<float>(bgrFrame.cols - 1));
            y1 = std::clamp(y1, 0.0f, static_cast<float>(bgrFrame.rows - 1));

            const int left = static_cast<int>(std::round(x0));
            const int top = static_cast<int>(std::round(y0));
            const int right = static_cast<int>(std::round(x1));
            const int bottom = static_cast<int>(std::round(y1));

            const int bw = right - left;
            const int bh = bottom - top;
            if (bw <= 1 || bh <= 1) {
                continue;
            }

            boxes.emplace_back(left, top, bw, bh);
            scores.push_back(score);
            classIds.push_back(classId);
        }

        std::vector<int> keep;
        cv::dnn::NMSBoxes(boxes, scores, confThreshold_, nmsThreshold_, keep);

        detections.reserve(keep.size());
        for (int idx : keep) {
            Detection d;
            d.box = boxes[idx];
            d.classId = classIds[idx];
            d.confidence = scores[idx];
            detections.push_back(d);
        }

        return true;
    } catch (const cv::Exception& e) {
        lastError_ = std::string("OpenCV exception in detect: ") + e.what();
        return false;
    } catch (...) {
        lastError_ = "unknown exception in detect";
        return false;
    }
}

void WeldDetector::drawDetections(cv::Mat& bgrFrame,
                                  const std::vector<Detection>& detections) const {
    for (const Detection& d : detections) {
        cv::rectangle(bgrFrame, d.box, cv::Scalar(60, 220, 60), 2);

        std::ostringstream oss;
        oss << "defect " << std::fixed << std::setprecision(2) << d.confidence;
        const std::string label = oss.str();

        int baseline = 0;
        const cv::Size textSize = cv::getTextSize(label,
                                                  cv::FONT_HERSHEY_SIMPLEX,
                                                  0.5,
                                                  1,
                                                  &baseline);

        int x = d.box.x;
        int y = std::max(0, d.box.y - textSize.height - 6);
        const int width = textSize.width + 8;
        const int height = textSize.height + 6;

        cv::rectangle(bgrFrame,
                      cv::Rect(x, y, width, height),
                      cv::Scalar(60, 220, 60),
                      cv::FILLED);
        cv::putText(bgrFrame,
                    label,
                    cv::Point(x + 4, y + textSize.height + 1),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.5,
                    cv::Scalar(20, 20, 20),
                    1,
                    cv::LINE_AA);
    }
}
