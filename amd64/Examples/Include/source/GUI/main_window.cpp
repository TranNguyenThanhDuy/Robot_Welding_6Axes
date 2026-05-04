#include "main_window.h"

#include <QCoreApplication>
#include <QDir>
#include <QFont>
#include <QFileInfo>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPlainTextEdit>
#include <QPixmap>
#include <QPushButton>
#include <QSpinBox>
#include <QString>
#include <QStringList>
#include <QVBoxLayout>
#include <QMetaObject> 
#include <QLineEdit>
#include <QFileDialog> // Thư viện mở cửa sổ chọn file

#include <opencv2/core/mat.hpp>

#include <iostream>
#include <vector>
#include <thread> 
#include <chrono>

namespace {
constexpr const char* kUsbCameraDevice = "/dev/video0";
constexpr const char* kDefaultModelPath = "AI_predict/best.onnx";

// Đưa biến xử lý luồng ra biến tĩnh cục bộ (static global) trong namespace ẩn
std::thread g_cameraThread;
std::atomic<bool> g_cameraRunning{false};

QString resolveModelPath() {
    const QString relativePath = QString::fromUtf8(kDefaultModelPath);
    const QStringList candidates = {
        QDir::current().absoluteFilePath(relativePath),
        QDir(QCoreApplication::applicationDirPath()).absoluteFilePath(relativePath),
        QDir(QCoreApplication::applicationDirPath()).absoluteFilePath(QStringLiteral("../") + relativePath)
    };

    for (const QString& candidate : candidates) {
        const QFileInfo modelFile(candidate);
        if (modelFile.exists() && modelFile.isReadable()) {
            return modelFile.absoluteFilePath();
        }
    }

    return QString();
}
}

MainWindow::MainWindow(QWidget* parent) : QWidget(parent) {
    buildUi();

    coutRedirect_ = std::make_unique<StreamRedirect>(log_, std::cout);
    cerrRedirect_ = std::make_unique<StreamRedirect>(log_, std::cerr);

    controller_.initializeSystem();
    connectSignals();
}

MainWindow::~MainWindow() {
    stopUsbCamera(); // Hàm này giờ sẽ an toàn dừng luồng camera
}

void MainWindow::buildUi() {
    setWindowTitle("Robot Welding 6-Axis Controller");

    auto* root = new QVBoxLayout(this);

    constexpr size_t kGuiAxisCount = 6;
    auto* title = new QLabel(QString("Robot Controller (GUI %1 axes, active %2)")
                                 .arg(kGuiAxisCount)
                                 .arg(AXIS_COUNT));
    QFont titleFont = title->font();
    titleFont.setPointSize(titleFont.pointSize() + 4);
    titleFont.setBold(true);
    title->setFont(titleFont);
    root->addWidget(title);

    auto* controlBox = new QGroupBox("Control");
    auto* controlLayout = new QHBoxLayout(controlBox);
    btnOn_ = new QPushButton("Servo ON");
    btnOff_ = new QPushButton("Servo OFF");
    btnHome_ = new QPushButton("Home");
    btnSetPos_ = new QPushButton("SetPos (Zero)");
    btnGetPos_ = new QPushButton("GetPos");
    controlLayout->addWidget(btnOn_);
    controlLayout->addWidget(btnOff_);
    controlLayout->addWidget(btnHome_);
    controlLayout->addWidget(btnSetPos_);
    controlLayout->addWidget(btnGetPos_);
    root->addWidget(controlBox);

    auto* motionBox = new QGroupBox("Motion");
    auto* motionLayout = new QVBoxLayout(motionBox);

    auto* axisRow = new QHBoxLayout();
    axisInputs_.reserve(kGuiAxisCount);
    for (size_t i = 0; i < kGuiAxisCount; ++i) {
        auto* col = new QVBoxLayout();
        QString axisLabel =
            (i < AXIS_COUNT)
                ? QString::fromStdString(controller_.axisName(i))
                : QString("Motor %1").arg(i + 1);
        auto* label = new QLabel(axisLabel);
        auto* spin = new QSpinBox();
        spin->setRange(-1000000, 1000000);
        spin->setSingleStep(100);
        if (i >= AXIS_COUNT) {
            spin->setEnabled(false);
        }
        axisInputs_.push_back(spin);
        col->addWidget(label);
        col->addWidget(spin);
        axisRow->addLayout(col);
    }
    motionLayout->addLayout(axisRow);

    auto* motionButtons = new QHBoxLayout();
    btnMove_ = new QPushButton("MovePos");
    btnGo_ = new QPushButton("Go (Recorded/File)");
    btnModeToggle_ = new QPushButton("Mode: AUTO RECORD");
    btnModeToggle_->setCheckable(true);
    btnRecord_ = new QPushButton("Record");
    btnSavePos_ = new QPushButton("Save Pos");
    btnStop_ = new QPushButton("Stop");
    btnClear_ = new QPushButton("Clear");
    motionButtons->addWidget(btnMove_);
    motionButtons->addWidget(btnGo_);
    motionButtons->addWidget(btnModeToggle_);
    motionButtons->addWidget(btnRecord_);
    motionButtons->addWidget(btnSavePos_);
    motionButtons->addWidget(btnStop_);
    motionButtons->addWidget(btnClear_);
    motionLayout->addLayout(motionButtons);
    root->addWidget(motionBox);

    auto* fileBox = new QGroupBox("File Path");
    auto* fileLayout = new QHBoxLayout(fileBox);

    filePathEdit_ = new QLineEdit();
    filePathEdit_->setPlaceholderText("Enter file path...");
    
    // Giao diện chọn file
    btnBrowseFile_ = new QPushButton("Browse...");

    fileLayout->addWidget(new QLabel("Path:"));
    fileLayout->addWidget(filePathEdit_);
    fileLayout->addWidget(btnBrowseFile_);

    root->addWidget(fileBox);

    auto* cameraBox = new QGroupBox("Camera");
    auto* cameraLayout = new QVBoxLayout(cameraBox);
    cameraPreview_ = new QLabel("Camera Preview");
    cameraPreview_->setMinimumSize(640, 360);
    cameraPreview_->setAlignment(Qt::AlignCenter);
    cameraPreview_->setStyleSheet(
        "QLabel {"
        "background-color: #111;"
        "color: #ddd;"
        "border: 1px solid #444;"
        "border-radius: 6px;"
        "}");
    cameraLayout->addWidget(cameraPreview_);

    auto* cameraButtons = new QHBoxLayout();
    btnCamStart_ = new QPushButton("Start Cam");
    btnCamStop_ = new QPushButton("Stop Cam");
    cameraButtons->addWidget(btnCamStart_);
    cameraButtons->addWidget(btnCamStop_);
    cameraLayout->addLayout(cameraButtons);

    root->addWidget(cameraBox);

    log_ = new QPlainTextEdit();
    log_->setReadOnly(true);
    log_->setMinimumHeight(220);
    root->addWidget(new QLabel("Log"));
    root->addWidget(log_);
}

void MainWindow::connectSignals() {
    QObject::connect(btnOn_, &QPushButton::clicked, [&]() {
        std::thread([this]() { controller_.servoOn(); }).detach();
    });

    QObject::connect(btnOff_, &QPushButton::clicked, [&]() {
        std::thread([this]() { controller_.servoOff(); }).detach();
    });

    QObject::connect(btnHome_, &QPushButton::clicked, [&]() {
        std::thread([this]() { controller_.home(); }).detach();
    });

    QObject::connect(btnSetPos_, &QPushButton::clicked, [&]() {
        std::thread([this]() { controller_.setOriginPos(); }).detach();
    });

    QObject::connect(btnGetPos_, &QPushButton::clicked, [&]() {
        // Đưa đọc vị trí vào thread để không block UI nếu cáp lỏng/lag
        std::thread([this]() {
            AxisPositions pos{};
            if (!controller_.getPos(pos)) {
                QMetaObject::invokeMethod(this, [this]() { logLine("Failed to read current positions."); });
                return;
            }

            // Gọi ngược lại UI thread để cập nhật giao diện
            QMetaObject::invokeMethod(this, [this, pos]() {
                QStringList parts;
                for (size_t i = 0; i < AXIS_COUNT; ++i) {
                    parts << QString::fromStdString(controller_.axisName(i)) + ": " + QString::number(pos[i]);
                    if (i < axisInputs_.size()) {
                        axisInputs_[i]->setValue(pos[i]);
                    }
                }
                logLine("Current positions - " + parts.join(", "));
            });
        }).detach();
    });

    QObject::connect(btnMove_, &QPushButton::clicked, [&]() {
        AxisPositions targets{};
        for (size_t i = 0; i < AXIS_COUNT; ++i) {
            targets[i] = axisInputs_[i]->value();
        }
        std::thread([this, targets]() {
            controller_.movePos(targets);
        }).detach();
    });

    QObject::connect(btnModeToggle_, &QPushButton::clicked, [&]() {
        if (controller_.isSaveMode()) {
            controller_.setModeRecord();
            btnModeToggle_->setText("Mode: AUTO RECORD");
        }
        else if (controller_.isFileMode()) {
            controller_.setModeSave();
            btnModeToggle_->setText("Mode: MANUAL SAVE");
        } 
        else {
            controller_.setModeFile();
            btnModeToggle_->setText("Mode: File");
        }
    });

    QObject::connect(filePathEdit_, &QLineEdit::editingFinished, [&]() {
        QString path = filePathEdit_->text();
        if (path.isEmpty()) return;
        QString linuxPath = convertPathToLinux(path);
        controller_.setFileName(linuxPath.toStdString());
        logLine("File path saved: " + linuxPath);
    });

    // Xử lý khi bấm nút Browse (Chọn File)
    QObject::connect(btnBrowseFile_, &QPushButton::clicked, [this]() {
        // Tự động nhảy vào ổ C của Windows
        QString startPath = QDir::currentPath();
        if (QDir("/mnt/c/Users/").exists()) {
            startPath = "/mnt/c/Users/";
        }

        QString filePath = QFileDialog::getOpenFileName(
            this, 
            "Chọn file toạ độ (Trajectory File)", 
            startPath, 
            "Text Files (*.txt);;All Files (*)"
        );

        if (!filePath.isEmpty()) {
            QString linuxPath = convertPathToLinux(filePath);
            filePathEdit_->setText(linuxPath); // Cập nhật lên ô text
            controller_.setFileName(linuxPath.toStdString()); // Gửi xuống hệ thống
            logLine("Đã tải thành công file: " + linuxPath);
        }
    });

    // Cập nhật tính năng Auto-Update trên nút "Go"
    QObject::connect(btnGo_, &QPushButton::clicked, [this]() { 
        // Lấy lại text mới nhất đề phòng người dùng copy dán nhưng chưa ấn Enter
        QString path = filePathEdit_->text();
        if (!path.isEmpty()) {
            QString linuxPath = convertPathToLinux(path);
            controller_.setFileName(linuxPath.toStdString());
        }
        std::thread([this]() { controller_.go(); }).detach(); 
    });

    QObject::connect(btnRecord_, &QPushButton::clicked, [&]() { 
        controller_.record(); 
    });

    QObject::connect(btnSavePos_, &QPushButton::clicked, [&]() {
        std::thread([this]() { controller_.savePos(); }).detach();
    });

    QObject::connect(btnStop_, &QPushButton::clicked, [&]() {
        btnStop_->setEnabled(false); // Tránh người dùng bấm spam liên tục
        std::thread([this]() {
            controller_.stop();
            // Bật lại nút sau khi stop xong
            QMetaObject::invokeMethod(this, [this]() { btnStop_->setEnabled(true); });
        }).detach();
    });

    QObject::connect(btnClear_, &QPushButton::clicked, [&]() { 
        controller_.clear(); 
    });

    QObject::connect(btnCamStart_, &QPushButton::clicked, [&]() { startUsbCamera(); });
    QObject::connect(btnCamStop_, &QPushButton::clicked, [&]() { stopUsbCamera(); });
}

void MainWindow::logLine(const QString& text) {
    std::cout << text.toStdString() << std::endl;
}

void MainWindow::logDetections(const std::vector<Detection>& detections) {
    for (size_t i = 0; i < detections.size(); ++i) {
        const Detection& d = detections[i];
        logLine(QString("Detection %1: x=%2 y=%3 w=%4 h=%5 conf=%6")
                    .arg(i + 1)
                    .arg(d.box.x)
                    .arg(d.box.y)
                    .arg(d.box.width)
                    .arg(d.box.height)
                    .arg(d.confidence, 0, 'f', 2));
    }
}

void MainWindow::startUsbCamera() {
    if (g_cameraRunning.load()) {
        return; // Camera đã đang chạy
    }

    const QFileInfo camDev(QString::fromUtf8(kUsbCameraDevice));
    if (!camDev.exists() || !camDev.isReadable()) {
        logLine("Camera device not found or unreadable: /dev/video0");
        return;
    }

    g_cameraRunning.store(true);
    btnCamStart_->setEnabled(false);
    cameraPreview_->setText("Initializing Camera & AI...");

    // LUỒNG CHUYÊN DỤNG CHO CAMERA & AI (Tránh đóng băng UI)
    g_cameraThread = std::thread([this]() {
        
        // 1. Mở camera
        if (!camera_.openDevice(kUsbCameraDevice, 640, 480)) {
            QMetaObject::invokeMethod(this, [this]() {
                logLine(QString("Cannot open USB camera: /dev/video0 (%1)").arg(QString::fromStdString(camera_.lastError())));
                stopUsbCamera();
            });
            return;
        }

        // 2. Load Model AI
        if (!detectorReady_) {
            const QString modelPath = resolveModelPath();
            if (!modelPath.isEmpty()) {
                if (detector_.loadModel(modelPath.toStdString())) {
                    detectorReady_ = true;
                    QMetaObject::invokeMethod(this, [this, modelPath]() {
                        logLine(QString("AI model loaded: %1").arg(modelPath));
                    });
                } else {
                    QMetaObject::invokeMethod(this, [this]() {
                        logLine(QString("AI model load failed: %1").arg(QString::fromStdString(detector_.lastError())));
                    });
                }
            } else {
                QMetaObject::invokeMethod(this, [this]() {
                    logLine(QString("AI model not found: %1 (preview-only)").arg(QString::fromUtf8(kDefaultModelPath)));
                });
            }
        }

        QMetaObject::invokeMethod(this, [this]() {
            logLine(QString("USB camera started: /dev/video0 (format %1)").arg(QString::fromStdString(camera_.formatName())));
        });

        int frameFailCount = 0;
        int noDetectionCount = 0;
        int detectFailCount = 0;

        // 3. Vòng lặp lấy khung hình & chạy AI liên tục
        while (g_cameraRunning.load()) {
            cv::Mat frameBgr;
            if (!camera_.readFrameBgr(frameBgr)) {
                ++frameFailCount;
                if (frameFailCount >= 30) {
                    QMetaObject::invokeMethod(this, [this]() {
                        logLine("Camera stopped after repeated frame failures.");
                        stopUsbCamera();
                    });
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            frameFailCount = 0;

            // Xử lý AI
            if (detectorReady_) {
                std::vector<Detection> detections;
                if (detector_.detect(frameBgr, detections)) {
                    if (!detections.empty()) {
                        // Gọi an toàn về UI thread để log
                        QMetaObject::invokeMethod(this, [this, detections]() {
                            logDetections(detections);
                        });
                        noDetectionCount = 0;
                    } else {
                        ++noDetectionCount;
                        if (noDetectionCount == 1 || noDetectionCount % 120 == 0) {
                            QMetaObject::invokeMethod(this, [this]() {
                                logLine("AI inference active: no detections in current frames.");
                            });
                        }
                    }
                    detector_.drawDetections(frameBgr, detections);
                    detectFailCount = 0;
                } else {
                    ++detectFailCount;
                    if (detectFailCount == 1 || detectFailCount % 60 == 0) {
                        QMetaObject::invokeMethod(this, [this]() {
                            logLine(QString("AI detect failed: %1").arg(QString::fromStdString(detector_.lastError())));
                        });
                    }
                }
            }

            // Chuyển đổi sang QImage và sao chép sâu (deep copy) để truyền an toàn qua UI Thread
            QImage frameImg = UsbCamera::bgrToQImage(frameBgr).copy();

            // Gửi hình lên GUI
            QMetaObject::invokeMethod(this, [this, frameImg]() {
                if (cameraPreview_ && g_cameraRunning.load()) {
                    cameraPreview_->setPixmap(
                        QPixmap::fromImage(frameImg).scaled(cameraPreview_->size(),
                                                            Qt::KeepAspectRatio,
                                                            Qt::SmoothTransformation));
                }
            }, Qt::QueuedConnection);

            // Nghỉ 1 chút để không chiếm 100% CPU thread
            std::this_thread::sleep_for(std::chrono::milliseconds(15));
        }

        // 4. Giải phóng camera khi thoát vòng lặp
        camera_.closeDevice();
    });
}

void MainWindow::stopUsbCamera() {
    if (!g_cameraRunning.load()) {
        return;
    }

    g_cameraRunning.store(false); // Báo hiệu cho vòng lặp dừng lại

    // Đợi luồng camera tắt hẳn
    if (g_cameraThread.joinable()) {
        g_cameraThread.join();
    }

    if (cameraPreview_) {
        cameraPreview_->setPixmap(QPixmap());
        cameraPreview_->setText("Camera Preview");
    }
    
    btnCamStart_->setEnabled(true);
    logLine("Camera stream stopped.");
}

QString MainWindow::convertPathToLinux(const QString& path) {
    QString p = path;
    p.replace("\\", "/");
    if (p.length() > 2 && p[1] == ':') {
        QChar drive = p[0].toLower();
        p = "/mnt/" + QString(drive) + p.mid(2);
    }
    return p;
}