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

#include <iostream>

namespace {
constexpr const char* kUsbCameraDevice = "/dev/video0";
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
    btnRecord_ = new QPushButton("Record");
    btnStop_ = new QPushButton("Stop");
    btnClear_ = new QPushButton("Clear");
    motionButtons->addWidget(btnMove_);
    motionButtons->addWidget(btnGo_);
    motionButtons->addWidget(btnRecord_);
    motionButtons->addWidget(btnStop_);
    motionButtons->addWidget(btnClear_);
    motionLayout->addLayout(motionButtons);
    root->addWidget(motionBox);

    auto* tableBox = new QGroupBox("Position Table");
    auto* tableLayout = new QHBoxLayout(tableBox);
    btnPosTable_ = new QPushButton("Save Table");
    btnGoPos_ = new QPushButton("GoPos");
    btnPrintTable_ = new QPushButton("Print Table");
    tableLayout->addWidget(btnPosTable_);
    tableLayout->addWidget(btnGoPos_);
    tableLayout->addWidget(btnPrintTable_);
    root->addWidget(tableBox);

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
    QObject::connect(btnGo_, &QPushButton::clicked, [&]() { controller_.go(); });
    QObject::connect(btnRecord_, &QPushButton::clicked,
                     [&]() { controller_.record(); });
    QObject::connect(btnStop_, &QPushButton::clicked,
                     [&]() { controller_.stop(); });
    QObject::connect(btnClear_, &QPushButton::clicked,
                     [&]() { controller_.clear(); });
    QObject::connect(btnPosTable_, &QPushButton::clicked,
                     [&]() { controller_.posTable(); });
    QObject::connect(btnGoPos_, &QPushButton::clicked,
                     [&]() { controller_.goPos(); });
    QObject::connect(btnPrintTable_, &QPushButton::clicked,
                     [&]() { controller_.printTable(); });
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

    cameraPreview_->setText("Starting USB camera...");
    cameraFrameFailCount_ = 0;
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

    if (cameraPreview_) {
        cameraPreview_->setPixmap(QPixmap());
        cameraPreview_->setText("Camera Preview");
    }
}

void MainWindow::updateCameraFrame() {
    QImage frame;
    if (!camera_.readFrame(frame)) {
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

    cameraPreview_->setPixmap(
        QPixmap::fromImage(frame).scaled(cameraPreview_->size(),
                                         Qt::KeepAspectRatio,
                                         Qt::SmoothTransformation));
}
