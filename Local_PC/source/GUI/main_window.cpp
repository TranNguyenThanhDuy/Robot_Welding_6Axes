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
#include <QProcess>
#include <QPushButton>
#include <QSpinBox>
#include <QString>
#include <QStringList>
#include <QTimer>
#include <QVBoxLayout>

#include <QLineEdit>//để điền link

#include <opencv2/core/mat.hpp>

#include <iostream>
#include <vector>

namespace {
QString resolveModelPath(const AppConfig& config) {
    const QString relativePath = config.modelPath;
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

QString resolveLineMappedPath(const AppConfig& config) {
    const QString outputDir = config.lineMappingOutputDir;
    const QString mappedFile = config.defaultMappedFileName;
    const QStringList dirCandidates = {
        QDir::current().absoluteFilePath(outputDir),
        QDir(QCoreApplication::applicationDirPath()).absoluteFilePath(outputDir),
        QDir(QCoreApplication::applicationDirPath()).absoluteFilePath(QStringLiteral("../") + outputDir)
    };

    for (const QString& dirPath : dirCandidates) {
        QDir dir(dirPath);
        if (!dir.exists()) {
            continue;
        }

        const QFileInfo preferredFile(dir.absoluteFilePath(mappedFile));
        if (preferredFile.exists() && preferredFile.isReadable() && preferredFile.isFile()) {
            return preferredFile.absoluteFilePath();
        }

        QFileInfoList mappedFiles = dir.entryInfoList(
            QStringList() << QStringLiteral("*_mapped.txt"),
            QDir::Files | QDir::Readable,
            QDir::Time);
        if (!mappedFiles.isEmpty()) {
            return mappedFiles.front().absoluteFilePath();
        }
    }

    return QString();
}
}

MainWindow::MainWindow(QWidget* parent) : QWidget(parent), config_(loadAppConfig()) {
    buildUi();

    coutRedirect_ = std::make_unique<StreamRedirect>(log_, std::cout);
    cerrRedirect_ = std::make_unique<StreamRedirect>(log_, std::cerr);

    logLine("Loaded path config: " + config_.configPath);
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
    btnDone_ = new QPushButton("Done");
    motionButtons->addWidget(btnMove_);
    motionButtons->addWidget(btnGo_);
    motionButtons->addWidget(btnModeToggle_);
    motionButtons->addWidget(btnRecord_);
    motionButtons->addWidget(btnSavePos_);
    motionButtons->addWidget(btnStop_);
    motionButtons->addWidget(btnClear_);
    motionButtons->addWidget(btnDone_);
    motionLayout->addLayout(motionButtons);
    root->addWidget(motionBox);

    auto* fileBox = new QGroupBox("File Path");
    auto* fileLayout = new QHBoxLayout(fileBox);

    filePathEdit_ = new QLineEdit();
    filePathEdit_->setPlaceholderText("Auto-select from line_mapping_outputs");
    filePathEdit_->setReadOnly(true);

    fileLayout->addWidget(new QLabel("Path:"));
    fileLayout->addWidget(filePathEdit_);

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
            refreshFileModePath();
        }

    });
    QObject::connect(btnGo_, &QPushButton::clicked, [&]() {
        if (controller_.isFileMode()) {
            refreshFileModePath();
        }
        controller_.go();
    });
    QObject::connect(btnRecord_, &QPushButton::clicked,
                     [&]() { controller_.record(); });
    QObject::connect(btnSavePos_, &QPushButton::clicked,
                     [&]() { controller_.savePos(); });
    QObject::connect(btnStop_, &QPushButton::clicked,
                     [&]() { controller_.stop(); });
    QObject::connect(btnClear_, &QPushButton::clicked,
                     [&]() { controller_.clear(); });
    QObject::connect(btnDone_, &QPushButton::clicked,
                     [&]() { triggerPythonLineMap(); });
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

void MainWindow::refreshFileModePath() {
    const QString mappedPath = resolveLineMappedPath(config_);
    if (mappedPath.isEmpty()) {
        controller_.setFileName("");
        filePathEdit_->clear();
        logLine(QString("FILE mode failed: no mapped file found in %1")
                    .arg(config_.lineMappingOutputDir));
        return;
    }

    filePathEdit_->setText(mappedPath);
    controller_.setFileName(mappedPath.toStdString());
    logLine("FILE mode using: " + mappedPath);
}

void MainWindow::startUsbCamera() {
    if (camera_.isOpen()) {
        return;
    }

    const QFileInfo camDev(config_.usbCameraDevice);
    if (!camDev.exists()) {
        logLine("Camera device not found: " + config_.usbCameraDevice);
        return;
    }
    if (!camDev.isReadable()) {
        logLine("Camera device is not readable: " + config_.usbCameraDevice);
        return;
    }

    if (!camera_.openDevice(config_.usbCameraDevice.toStdString(), 640, 480)) {
        logLine(QString("Cannot open USB camera: %1 (%2)")
                    .arg(config_.usbCameraDevice)
                    .arg(QString::fromStdString(camera_.lastError())));
        return;
    }

    if (!detectorReady_) {
        const QString modelPath = resolveModelPath(config_);
        if (!modelPath.isEmpty()) {
            if (detector_.loadModel(modelPath.toStdString())) {
                detectorReady_ = true;
                logLine(QString("AI model loaded: %1").arg(modelPath));
            } else {
                logLine(QString("AI model load failed: %1")
                            .arg(QString::fromStdString(detector_.lastError())));
            }
        } else {
            logLine(QString("AI model not found: %1 (camera will run preview-only)")
                        .arg(config_.modelPath));
        }
    }

    cameraPreview_->setText("Starting USB camera...");
    cameraFrameFailCount_ = 0;
    detectorFailCount_ = 0;
    detectorNoDetectionCount_ = 0;
    cameraTimer_->start();
    logLine(QString("USB camera started: %1 (format %2)")
                .arg(config_.usbCameraDevice)
                .arg(QString::fromStdString(camera_.formatName())));
}

void MainWindow::stopUsbCamera() {
    if (cameraTimer_) {
        cameraTimer_->stop();
    }
    camera_.closeDevice();
    cameraFrameFailCount_ = 0;
    detectorFailCount_ = 0;
    detectorNoDetectionCount_ = 0;

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
            logLine(QString("Camera opened but no frame received from %1 (%2)")
                        .arg(config_.usbCameraDevice)
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
            if (!detections.empty()) {
                logDetections(detections);
                detectorNoDetectionCount_ = 0;
            } else {
                ++detectorNoDetectionCount_;
                if (detectorNoDetectionCount_ == 1 || detectorNoDetectionCount_ % 120 == 0) {
                    logLine("AI inference active: no detections in current frames.");
                }
            }
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

void MainWindow::triggerPythonLineMap() {
    const QString linuxPath = config_.defaultInputPath;
    const QFileInfo inputFile(linuxPath);
    if (!inputFile.exists() || !inputFile.isFile()) {
        logLine("Done failed: input file does not exist: " + linuxPath);
        return;
    }

    const QFileInfo pythonMain(config_.pythonMainPath);
    if (!pythonMain.exists() || !pythonMain.isFile()) {
        logLine("Done failed: Python main.py not found: " + pythonMain.absoluteFilePath());
        return;
    }

    QProcess process;
    process.setProgram(config_.pythonExecutable);
    process.setArguments({pythonMain.absoluteFilePath(), "line-map", linuxPath});
    process.setProcessChannelMode(QProcess::MergedChannels);

    logLine("Running Python line-map for: " + linuxPath);
    process.start();
    if (!process.waitForStarted()) {
        logLine("Done failed: cannot start Python process.");
        return;
    }

    if (!process.waitForFinished(-1)) {
        logLine("Done failed: Python process did not finish correctly.");
        return;
    }

    const QString output = QString::fromLocal8Bit(process.readAll()).trimmed();
    if (!output.isEmpty()) {
        logLine(output);
    }

    if (process.exitStatus() == QProcess::NormalExit && process.exitCode() == 0) {
        logLine("Done completed: Python line-map finished successfully.");
        refreshFileModePath();
    } else {
        logLine(QString("Done failed: Python exited with code %1.").arg(process.exitCode()));
    }
}

QString MainWindow::convertPathToLinux(const QString& path)
{
    QString p = path;

    p.replace("\\", "/");

    if (p.length() > 2 && p[1] == ':')
    {
        QChar drive = p[0].toLower();
        p = "/mnt/" + QString(drive) + p.mid(2);
    }

    return p;
}
