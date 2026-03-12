#include "weld_detector.h"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <iostream>

namespace {
constexpr float kPadValue = 114.0f;
}

bool WeldDetector::loadModel(const std::string& modelPath, float confThreshold, float nmsThreshold, int inputSize) {
    loaded_ = false;
    lastError_.clear();
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

bool WeldDetector::isLoaded() const { return loaded_; }
std::string WeldDetector::lastError() const { return lastError_; }

cv::Mat WeldDetector::letterbox(const cv::Mat& src, int targetSize, float& scale, int& padX, int& padY) const {
    const int w = src.cols;
    const int h = src.rows;
    scale = std::min(static_cast<float>(targetSize) / w, static_cast<float>(targetSize) / h);
    int nw = static_cast<int>(std::round(w * scale));
    int nh = static_cast<int>(std::round(h * scale));
    cv::Mat resized;
    cv::resize(src, resized, cv::Size(nw, nh));
    padX = (targetSize - nw) / 2;
    padY = (targetSize - nh) / 2;
    cv::Mat boxed(targetSize, targetSize, CV_8UC3, cv::Scalar(kPadValue, kPadValue, kPadValue));
    resized.copyTo(boxed(cv::Rect(padX, padY, nw, nh)));
    return boxed;
}

bool WeldDetector::detect(const cv::Mat& bgrFrame, std::vector<Detection>& detections) {
    detections.clear();
    if (!loaded_ || bgrFrame.empty()) return false;

    try {
        float scale = 1.0f;
        int padX = 0, padY = 0;
        cv::Mat input = letterbox(bgrFrame, inputSize_, scale, padX, padY);
        cv::Mat blob = cv::dnn::blobFromImage(input, 1.0 / 255.0, cv::Size(inputSize_, inputSize_), cv::Scalar(), true, false);
        net_.setInput(blob);

        std::vector<cv::Mat> outputs;
        net_.forward(outputs, net_.getUnconnectedOutLayersNames());
        cv::Mat out = outputs[0];

        int rows = 0, dims = 0;
        if (out.dims == 3) {
            dims = out.size[1]; 
            rows = out.size[2]; 
            out = out.reshape(1, dims);
            cv::transpose(out, out);
        } else {
            rows = out.rows;
            dims = out.cols;
        }

        std::vector<cv::Rect> boxes;
        std::vector<float> scores;
        std::vector<int> classIds;

        float global_max = 0.0f; // Để debug

        for (int i = 0; i < out.rows; ++i) {
            const float* data = out.ptr<float>(i);
            float* scores_ptr = (float*)(data + 4);
            int num_classes = dims - 4;
            
            int classId = 0;
            float max_score = scores_ptr[0];
            for (int c = 1; c < num_classes; ++c) {
                if (scores_ptr[c] > max_score) {
                    max_score = scores_ptr[c];
                    classId = c;
                }
            }

            if (max_score < confThreshold_) continue;

            // --- ĐOẠN FIX TỌA ĐỘ ---
            float cx = data[0];
            float cy = data[1];
            float w  = data[2];
            float h  = data[3];

            // Kiểm tra nếu mô hình trả về giá trị chuẩn hóa (0-1) thay vì pixel (0-640)
            // Nếu cx < 1, khả năng cao là giá trị chuẩn hóa
            if (cx < 1.0f && w < 1.0f) {
                cx *= inputSize_;
                cy *= inputSize_;
                w  *= inputSize_;
                h  *= inputSize_;
            }

            // Chuyển về góc trái trên (Top-left)
            float left   = (cx - w * 0.5f - (float)padX) / scale;
            float top    = (cy - h * 0.5f - (float)padY) / scale;
            float width  = w / scale;
            float height = h / scale;

            // Ép kiểu int sau khi đã tính toán xong trên số thực
            int ix = static_cast<int>(std::round(left));
            int iy = static_cast<int>(std::round(top));
            int iw = static_cast<int>(std::round(width));
            int ih = static_cast<int>(std::round(height));

            // Đảm bảo box hợp lệ
            if (iw <= 5 || ih <= 5) continue; 

            boxes.emplace_back(ix, iy, iw, ih);
            scores.push_back(max_score);
            classIds.push_back(classId);
        }

        

        std::vector<int> keep;
        cv::dnn::NMSBoxes(boxes, scores, confThreshold_, nmsThreshold_, keep);
        for (int idx : keep) {
            Detection d;
            d.box = boxes[idx] & cv::Rect(0, 0, bgrFrame.cols, bgrFrame.rows);
            d.classId = classIds[idx];
            d.confidence = scores[idx];
            detections.push_back(d);
        }
        return true;
    } catch (...) { return false; }
}

void WeldDetector::drawDetections(cv::Mat& bgrFrame, const std::vector<Detection>& detections) const {
    for (const auto& d : detections) {
        cv::rectangle(bgrFrame, d.box, cv::Scalar(0, 255, 0), 2);
        std::string label = "Class " + std::to_string(d.classId) + ": " + std::to_string(int(d.confidence * 100)) + "%";
        cv::putText(bgrFrame, label, cv::Point(d.box.x, d.box.y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
    }
}