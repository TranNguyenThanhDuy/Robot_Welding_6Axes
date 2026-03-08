#include "main_window.h"

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
#include <QTimer>
#include <QVBoxLayout>

#include <opencv2/core/mat.hpp>

#include <iostream>
#include <vector>

namespace {
constexpr const char* kUsbCameraDevice = "/dev/video0";
constexpr const char* kDefaultModelPath = "AI_predict/best.onnx";
}

MainWindow::MainWindow(QWidget* parent) : QWidget(parent) {
    buildUi();

    coutRedirect_ = std::make_unique<StreamRedirect>(log_, std::cout);
    cerrRedirect_ = std::make_unique<StreamRedirect>(log_, std::cerr);

    controller_.initializeSystem();
    connectSignals();
}

MainWindow::~MainWindow() {
    stopUsbCamera();
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
    btnGo_ = new QPushButton("Go (Recorded)");
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

    cameraTimer_ = new QTimer(this);
    cameraTimer_->setInterval(33);
}

void MainWindow::connectSignals() {
    QObject::connect(btnOn_, &QPushButton::clicked,
                     [&]() { controller_.servoOn(); });
    QObject::connect(btnOff_, &QPushButton::clicked,
                     [&]() { controller_.servoOff(); });
    QObject::connect(btnHome_, &QPushButton::clicked,
                     [&]() { controller_.home(); });
    QObject::connect(btnSetPos_, &QPushButton::clicked,
                     [&]() { controller_.setOriginPos(); });
    QObject::connect(btnGetPos_, &QPushButton::clicked, [&]() {
        AxisPositions pos{};
        if (!controller_.getPos(pos)) {
            logLine("Failed to read current positions.");
            return;
        }

        QStringList parts;
        for (size_t i = 0; i < AXIS_COUNT; ++i) {
            parts << QString::fromStdString(controller_.axisName(i)) + ": " +
                         QString::number(pos[i]);
            if (i < axisInputs_.size()) {
                axisInputs_[i]->setValue(pos[i]);
            }
        }
        logLine("Current positions - " + parts.join(", "));
    });
    QObject::connect(btnMove_, &QPushButton::clicked, [&]() {
        AxisPositions targets{};
        for (size_t i = 0; i < AXIS_COUNT; ++i) {
            targets[i] = axisInputs_[i]->value();
        }
        controller_.movePos(targets);
    });
    QObject::connect(btnModeToggle_, &QPushButton::toggled, [&](bool checked) {
        if (checked) {
            controller_.setModeSave();
            btnModeToggle_->setText("Mode: MANUAL SAVE");
        } else {
            controller_.setModeRecord();
            btnModeToggle_->setText("Mode: AUTO RECORD");
        }
    });
    QObject::connect(btnGo_, &QPushButton::clicked, [&]() { controller_.go(); });
    QObject::connect(btnRecord_, &QPushButton::clicked,
                     [&]() { controller_.record(); });
    QObject::connect(btnSavePos_, &QPushButton::clicked,
                     [&]() { controller_.savePos(); });
    QObject::connect(btnStop_, &QPushButton::clicked,
                     [&]() { controller_.stop(); });
    QObject::connect(btnClear_, &QPushButton::clicked,
                     [&]() { controller_.clear(); });
    QObject::connect(btnCamStart_, &QPushButton::clicked,
                     [&]() { startUsbCamera(); });
    QObject::connect(btnCamStop_, &QPushButton::clicked,
                     [&]() { stopUsbCamera(); });
    QObject::connect(cameraTimer_, &QTimer::timeout,
                     [&]() { updateCameraFrame(); });
}

void MainWindow::logLine(const QString& text) {
    std::cout << text.toStdString() << std::endl;
}

void MainWindow::startUsbCamera() {
    if (camera_.isOpen()) {
        return;
    }

    const QFileInfo camDev(QString::fromUtf8(kUsbCameraDevice));
    if (!camDev.exists()) {
        logLine("Camera device not found: /dev/video0");
        return;
    }
    if (!camDev.isReadable()) {
        logLine("Camera device is not readable: /dev/video0");
        return;
    }

    if (!camera_.openDevice(kUsbCameraDevice, 640, 480)) {
        logLine(QString("Cannot open USB camera: /dev/video0 (%1)")
                    .arg(QString::fromStdString(camera_.lastError())));
        return;
    }

    if (!detectorReady_) {
        const QFileInfo modelFile(QString::fromUtf8(kDefaultModelPath));
        if (modelFile.exists() && modelFile.isReadable()) {
            if (detector_.loadModel(kDefaultModelPath)) {
                detectorReady_ = true;
                logLine(QString("AI model loaded: %1").arg(QString::fromUtf8(kDefaultModelPath)));
            } else {
                logLine(QString("AI model load failed: %1")
                            .arg(QString::fromStdString(detector_.lastError())));
            }
        } else {
            logLine(QString("AI model not found: %1 (camera will run preview-only)")
                        .arg(QString::fromUtf8(kDefaultModelPath)));
        }
    }

    cameraPreview_->setText("Starting USB camera...");
    cameraFrameFailCount_ = 0;
    detectorFailCount_ = 0;
    cameraTimer_->start();
    logLine(QString("USB camera started: /dev/video0 (format %1)")
                .arg(QString::fromStdString(camera_.formatName())));
}

void MainWindow::stopUsbCamera() {
    if (cameraTimer_) {
        cameraTimer_->stop();
    }
    camera_.closeDevice();
    cameraFrameFailCount_ = 0;
    detectorFailCount_ = 0;

    if (cameraPreview_) {
        cameraPreview_->setPixmap(QPixmap());
        cameraPreview_->setText("Camera Preview");
    }
}

void MainWindow::updateCameraFrame() {
    cv::Mat frameBgr;
    if (!camera_.readFrameBgr(frameBgr)) {
        ++cameraFrameFailCount_;
        if (cameraFrameFailCount_ == 30) {
            logLine(QString("Camera opened but no frame received from /dev/video0 (%1)")
                        .arg(QString::fromStdString(camera_.lastError())));
            stopUsbCamera();
            logLine("Camera stopped after repeated frame failures.");
        }
        return;
    }
    cameraFrameFailCount_ = 0;

    if (detectorReady_) {
        std::vector<Detection> detections;
        if (detector_.detect(frameBgr, detections)) {
            detector_.drawDetections(frameBgr, detections);
            detectorFailCount_ = 0;
        } else {
            ++detectorFailCount_;
            if (detectorFailCount_ == 1 || detectorFailCount_ % 60 == 0) {
                logLine(QString("AI detect failed: %1")
                            .arg(QString::fromStdString(detector_.lastError())));
            }
        }
    }

    QImage frame = UsbCamera::bgrToQImage(frameBgr);
    cameraPreview_->setPixmap(
        QPixmap::fromImage(frame).scaled(cameraPreview_->size(),
                                         Qt::KeepAspectRatio,
                                         Qt::SmoothTransformation));
}
