#include "main_window.h"

#include <QCoreApplication>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFont>
#include <QFileInfo>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPlainTextEdit>
#include <QPixmap>
#include <QProcess>
#include <QPushButton>
#include <QRegularExpression>
#include <QSpinBox>
#include <QString>
#include <QStringList>
#include <QTimer>
#include <QVBoxLayout>
#include <QLineEdit>

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

QString resolveLinePreviewPath(const AppConfig& config, const QString& suffix) {
    const QString outputDir = config.lineMappingOutputDir;
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

        QFileInfoList previewFiles = dir.entryInfoList(
            QStringList() << QStringLiteral("*_line_%1.png").arg(suffix),
            QDir::Files | QDir::Readable,
            QDir::Time);
        if (!previewFiles.isEmpty()) {
            return previewFiles.front().absoluteFilePath();
        }
    }
    return QString();
}

void setupPreviewLabel(QLabel* label, const QString& text) {
    label->setMinimumSize(560, 360);
    label->setAlignment(Qt::AlignCenter);
    label->setText(text);
    label->setStyleSheet(
        "QLabel {"
        "background-color: #f7f8fa;"
        "color: #596273;"
        "border: 1px solid #ccd2dc;"
        "border-radius: 4px;"
        "}");
}
}

MainWindow::MainWindow(QWidget* parent) : QWidget(parent), config_(loadAppConfig()) {
    buildUi();

    // KHÓA 2 DÒNG NÀY LẠI ĐỂ HIỂN THỊ LOG RA TERMINAL ĐEN KHI DEBUG UART
    // coutRedirect_ = std::make_unique<StreamRedirect>(log_, std::cout);
    // cerrRedirect_ = std::make_unique<StreamRedirect>(log_, std::cerr);

    logLine("Loaded path config: " + config_.configPath);
    controller_.initializeSystem();
    connectSignals();
    startUsbSerial();
}

MainWindow::~MainWindow() {
    stopUsbSerial();
    stopUsbCamera();
}

void MainWindow::buildUi() {
    setWindowTitle("Robot Welding 6-Axis Controller");
    resize(1920, 1080);

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

    auto* operationBox = new QGroupBox("Operation");
    auto* operationLayout = new QHBoxLayout(operationBox);
    operationLayout->setSpacing(12);

    auto* axisBox = new QGroupBox("Motor Pulses");
    axisBox->setFixedWidth(230);
    auto* axisColumn = new QVBoxLayout(axisBox);
    auto* axisList = new QVBoxLayout();
    axisList->setSpacing(6);
    axisInputs_.reserve(kGuiAxisCount);
    for (size_t i = 0; i < kGuiAxisCount; ++i) {
        auto* row = new QHBoxLayout();
        QString axisLabel =
            (i < AXIS_COUNT)
                ? QString::fromStdString(controller_.axisName(i))
                : QString("Motor %1").arg(i + 1);
        auto* label = new QLabel(axisLabel);
        label->setFixedWidth(70);
        auto* spin = new QSpinBox();
        spin->setRange(-1000000, 1000000);
        spin->setSingleStep(100);
        spin->setFixedWidth(130);
        if (i >= AXIS_COUNT) {
            spin->setEnabled(false);
        }
        axisInputs_.push_back(spin);
        row->setSpacing(4);
        row->addWidget(label);
        row->addWidget(spin);
        row->addStretch();
        axisList->addLayout(row);
    }
    axisColumn->addLayout(axisList);
    axisColumn->addStretch();

    auto* motionBox = new QGroupBox("Motion");
    motionBox->setFixedWidth(340);
    auto* motionColumn = new QVBoxLayout(motionBox);
    auto* moveTimeRow = new QHBoxLayout();
    auto* moveTimeLabel = new QLabel("Move Time");
    moveTimeLabel->setFixedWidth(70);
    moveTimeInput_ = new QDoubleSpinBox();
    moveTimeInput_->setRange(0.0, 3600.0);
    moveTimeInput_->setDecimals(3);
    moveTimeInput_->setSingleStep(0.1);
    moveTimeInput_->setSuffix(" s");
    moveTimeInput_->setSpecialValueText("Default");
    moveTimeInput_->setFixedWidth(150);
    moveTimeRow->setSpacing(4);
    moveTimeRow->addWidget(moveTimeLabel);
    moveTimeRow->addWidget(moveTimeInput_);
    moveTimeRow->addStretch();
    motionColumn->addLayout(moveTimeRow);

    auto* baseVelocityRow = new QHBoxLayout();
    auto* baseVelocityLabel = new QLabel("Base Vel");
    baseVelocityLabel->setFixedWidth(70);
    baseVelocityInput_ = new QDoubleSpinBox();
    baseVelocityInput_->setRange(0.0, 1000000.0);
    baseVelocityInput_->setDecimals(0);
    baseVelocityInput_->setSingleStep(100.0);
    baseVelocityInput_->setSpecialValueText("Default");
    baseVelocityInput_->setFixedWidth(150);
    baseVelocityRow->setSpacing(4);
    baseVelocityRow->addWidget(baseVelocityLabel);
    baseVelocityRow->addWidget(baseVelocityInput_);
    baseVelocityRow->addStretch();
    motionColumn->addLayout(baseVelocityRow);

    btnMove_ = new QPushButton("MovePos");
    btnGo_ = new QPushButton("Go (Recorded)");
    btnModeToggle_ = new QPushButton("Mode: AUTO RECORD");
    btnModeToggle_->setCheckable(true);
    btnRecord_ = new QPushButton("Record");
    btnSavePos_ = new QPushButton("Save Pos");
    btnStop_ = new QPushButton("Stop");
    btnClear_ = new QPushButton("Clear");
    btnDone_ = new QPushButton("Done");
    const std::vector<QPushButton*> motionButtons = {
        btnMove_, btnGo_, btnModeToggle_, btnRecord_,
        btnSavePos_, btnStop_, btnClear_, btnDone_,
    };
    auto* motionButtonGrid = new QGridLayout();
    motionButtonGrid->setHorizontalSpacing(8);
    motionButtonGrid->setVerticalSpacing(8);
    for (size_t i = 0; i < motionButtons.size(); ++i) {
        QPushButton* button = motionButtons[i];
        button->setFixedSize(150, 30);
        motionButtonGrid->addWidget(button, static_cast<int>(i % 4), static_cast<int>(i / 4));
    }
    motionColumn->addLayout(motionButtonGrid);
    motionColumn->addStretch();

    auto* controlBox = new QGroupBox("Control");
    controlBox->setFixedWidth(175);
    auto* controlColumn = new QVBoxLayout(controlBox);
    btnOn_ = new QPushButton("Servo ON");
    btnOff_ = new QPushButton("Servo OFF");
    btnHome_ = new QPushButton("Home");
    btnSetPos_ = new QPushButton("SetPos (Zero)");
    btnGetPos_ = new QPushButton("GetPos");
    const std::vector<QPushButton*> controlButtons = {
        btnOn_, btnOff_, btnHome_, btnSetPos_, btnGetPos_,
    };
    for (QPushButton* button : controlButtons) {
        button->setFixedSize(150, 30);
        controlColumn->addWidget(button);
    }
    controlColumn->addStretch();

    auto* infoBox = new QGroupBox("Info");
    infoBox->setFixedWidth(290);
    auto* infoColumn = new QVBoxLayout(infoBox);
    const std::vector<QPushButton*> infoButtons = {
        new QPushButton("Lamp Info"),
        new QPushButton("Error Code"),
        new QPushButton("Input Status"),
        new QPushButton("Output Status"),
        new QPushButton("Alarm Reset"),
        new QPushButton("Clear Info"),
    };
    auto* infoGrid = new QGridLayout();
    infoGrid->setHorizontalSpacing(8);
    infoGrid->setVerticalSpacing(8);
    for (size_t i = 0; i < infoButtons.size(); ++i) {
        QPushButton* button = infoButtons[i];
        button->setFixedSize(130, 30);
        infoGrid->addWidget(button, static_cast<int>(i / 2), static_cast<int>(i % 2));
    }
    infoColumn->addLayout(infoGrid);
    infoColumn->addStretch();

    operationLayout->addWidget(controlBox, 0, Qt::AlignTop);
    operationLayout->addWidget(motionBox, 0, Qt::AlignTop);
    operationLayout->addWidget(axisBox, 0, Qt::AlignTop);
    operationLayout->addWidget(infoBox, 0, Qt::AlignTop);
    operationLayout->addStretch();
    root->addWidget(operationBox);

    auto* fileBox = new QGroupBox("File Path");
    auto* fileLayout = new QHBoxLayout(fileBox);

    filePathEdit_ = new QLineEdit();
    filePathEdit_->setPlaceholderText("Auto-select from line_mapping_outputs");
    filePathEdit_->setReadOnly(true);

    fileLayout->addWidget(new QLabel("Path:"));
    fileLayout->addWidget(filePathEdit_);

    root->addWidget(fileBox);

    auto* pointLogBox = new QGroupBox("Point Log");
    auto* pointLogLayout = new QVBoxLayout(pointLogBox);
    pointLog_ = new QPlainTextEdit();
    pointLog_->setReadOnly(true);
    pointLog_->setMinimumHeight(110);
    pointLog_->setMaximumHeight(150);
    QFont pointLogFont = pointLog_->font();
    pointLogFont.setFamily("monospace");
    pointLog_->setFont(pointLogFont);
    pointLogLayout->addWidget(pointLog_);
    root->addWidget(pointLogBox);

    auto* previewBox = new QGroupBox("Line Map Preview");
    auto* previewLayout = new QHBoxLayout(previewBox);

    auto* oxyColumn = new QVBoxLayout();
    oxyColumn->addWidget(new QLabel("OXY"));
    oxyPreview_ = new QLabel();
    setupPreviewLabel(oxyPreview_, "OXY preview will appear after Done");
    oxyColumn->addWidget(oxyPreview_);

    auto* oxzColumn = new QVBoxLayout();
    oxzColumn->addWidget(new QLabel("OXZ"));
    oxzPreview_ = new QLabel();
    setupPreviewLabel(oxzPreview_, "OXZ preview will appear after Done");
    oxzColumn->addWidget(oxzPreview_);

    previewLayout->addLayout(oxyColumn);
    previewLayout->addLayout(oxzColumn);
    root->addWidget(previewBox);

    log_ = new QPlainTextEdit(this);
    log_->setReadOnly(true);
    log_->hide();
}

void MainWindow::connectSignals() {
    QObject::connect(btnOn_, &QPushButton::clicked, [&]() {
        applyBaseVelocity();
        if (controller_.servoOn() && pointLog_) {
            pointLog_->appendPlainText("All Servo ON");
        }
    });
    QObject::connect(btnOff_, &QPushButton::clicked, [&]() {
        applyBaseVelocity();
        if (controller_.servoOff() && pointLog_) {
            pointLog_->appendPlainText("All Servo OFF");
        }
    });
    QObject::connect(btnHome_, &QPushButton::clicked,
                     [&]() { applyBaseVelocity(); controller_.home(); });
    QObject::connect(btnSetPos_, &QPushButton::clicked,
                     [&]() { applyBaseVelocity(); controller_.setOriginPos(); });
    QObject::connect(btnGetPos_, &QPushButton::clicked, [&]() {
        applyBaseVelocity();
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
        applyBaseVelocity();
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
        applyBaseVelocity();
        if (controller_.isFileMode()) {
            refreshFileModePath();
        }
        controller_.go();
    });
    QObject::connect(btnRecord_, &QPushButton::clicked, [&]() {
        const bool wasRecording = controller_.isRecording();
        controller_.record();
        if (!wasRecording && controller_.isRecording()) {
            appendPointLog("Record");
        }
    });
    QObject::connect(btnSavePos_, &QPushButton::clicked, [&]() {
        if (controller_.savePos()) {
            appendPointLog("SavePos");
        }
    });
    QObject::connect(btnStop_, &QPushButton::clicked,
                     [&]() { controller_.stop(); });
    QObject::connect(btnClear_, &QPushButton::clicked,
                     [&]() { controller_.clear(); });
    QObject::connect(btnDone_, &QPushButton::clicked,
                     [&]() { triggerPythonLineMap(); });
}

void MainWindow::startUsbSerial() {
    std::cerr << "[MainWindow] startUsbSerial() called\n" << std::flush;
    logLine("[DEBUG] startUsbSerial() entry point");
    
    if (usbSerial_.isRunning()) {
        logLine("[DEBUG] USB serial already running, skipping");
        return;
    }

    // ÉP CỨNG CỔNG ttyUSB1 VÀ TỐC ĐỘ 115200 ĐỂ BỎ QUA FILE CẤU HÌNH LỖI
    QString targetDevice = "/dev/ttyUSB1"; 
    int targetBaudRate = 115200;

    logLine(QString("[DEBUG] Attempting to open port: %1 @ %2").arg(targetDevice).arg(targetBaudRate));
    
    // Sửa biến truyền vào thành targetDevice và targetBaudRate
    if (!usbSerial_.openPort(targetDevice.toStdString(), targetBaudRate)) {
        QString errMsg = QString("Failed to open USB serial port %1: %2")
                    .arg(targetDevice)
                    .arg(QString::fromStdString(usbSerial_.lastError()));
        std::cerr << "[ERROR] " << errMsg.toStdString() << "\n" << std::flush;
        logLine(errMsg);
        return;
    }

    logLine("[DEBUG] Port opened successfully, starting reader thread");
    usbSerial_.start([this](const std::string& line) {
        onUsbSerialMessage(line);
    });
    logLine(QString("USB serial listener started on %1 @ %2").arg(targetDevice).arg(targetBaudRate));
    std::cerr << "[DEBUG] USB serial listener started\n" << std::flush;
}

void MainWindow::onUsbSerialMessage(const std::string& message) {
    const QString qmsg = QString::fromStdString(message).simplified().toLower();
    logLine("DEBUG: onUsbSerialMessage called with raw: [" + QString::fromStdString(message) + "]");
    logLine("DEBUG: processed message: [" + qmsg + "]");
    
    if (!qmsg.isEmpty()) {
        QMetaObject::invokeMethod(this, [this, qmsg]() {
            processUsbMessage(qmsg);
        }, Qt::QueuedConnection);
    } else {
        logLine("DEBUG: Message is empty after processing, skipping");
    }
}

void MainWindow::stopUsbSerial() {
    logLine("Stopping USB serial listener...");
    if (usbSerial_.isRunning()) {
        usbSerial_.stop();
    }
    if (usbSerial_.isOpen()) {
        usbSerial_.closePort();
    }
    logLine("USB serial listener stopped.");
}

void MainWindow::processUsbMessage(const QString& message) {
    // ÉP LÀM SẠCH CHUỖI TUYỆT ĐỐI (DỌN RÁC UART TỪ PHẦN CỨNG HOẶC DRIVER WSL)
    const QString cmd = message.trimmed().simplified().toLower();
    if (cmd.isEmpty()) {
        return;
    }

    // Bắt buộc in trực tiếp ra std::cout để thấy luồng nhảy vào xử lý GUI
    std::cout << "[MAIN WINDOW] Thuc thi lenh tu UART: " << cmd.toStdString() << std::endl;
    logLine("USB command processing: " + cmd);

    if (cmd == QLatin1String("on") || cmd == QLatin1String("servo on") || cmd == QLatin1String("servoon")) {
        applyBaseVelocity();
        controller_.servoOn();
        if (pointLog_) {
            pointLog_->appendPlainText("USB command executed: SERVO ON");
        }
        return;
    }

    if (cmd == QLatin1String("off") || cmd == QLatin1String("servo off") || cmd == QLatin1String("servooff")) {
        applyBaseVelocity();
        controller_.servoOff();
        if (pointLog_) {
            pointLog_->appendPlainText("USB command executed: SERVO OFF");
        }
        return;
    }

    if (cmd == QLatin1String("home")) {
        applyBaseVelocity();
        controller_.home();
        if (pointLog_) {
            pointLog_->appendPlainText("USB command executed: HOME");
        }
        return;
    }

    if (cmd == QLatin1String("setpos") || cmd == QLatin1String("set pos") || cmd == QLatin1String("zero") || cmd == QLatin1String("setpos zero")) {
        applyBaseVelocity();
        controller_.setOriginPos();
        if (pointLog_) {
            pointLog_->appendPlainText("USB command executed: SETPOS");
        }
        return;
    }

    if (cmd == QLatin1String("getpos") || cmd == QLatin1String("get pos")) {
        btnGetPos_->click();
        if (pointLog_) {
            pointLog_->appendPlainText("USB command executed: GETPOS");
        }
        return;
    }

    if (cmd == QLatin1String("move") || cmd == QLatin1String("move pos") || cmd == QLatin1String("movepos")) {
        applyBaseVelocity();
        AxisPositions targets{};
        for (size_t i = 0; i < AXIS_COUNT; ++i) {
            targets[i] = axisInputs_[i]->value();
        }
        controller_.movePos(targets);
        if (pointLog_) {
            pointLog_->appendPlainText("USB command executed: MOVE POS");
        }
        return;
    }

    if (cmd == QLatin1String("go") || cmd == QLatin1String("go beside") || cmd == QLatin1String("gobeside") || cmd == QLatin1String("goside") || cmd == QLatin1String("go recorded") || cmd == QLatin1String("go (recorded)")) {
        applyBaseVelocity();
        if (controller_.isFileMode()) {
            refreshFileModePath();
        }
        controller_.go();
        if (pointLog_) {
            pointLog_->appendPlainText("USB command executed: GO");
        }
        return;
    }

    if (cmd == QLatin1String("record") || cmd == QLatin1String("start record") || cmd == QLatin1String("record start")) {
        btnRecord_->click();
        if (pointLog_) {
            pointLog_->appendPlainText("USB command executed: RECORD");
        }
        return;
    }

    if (cmd == QLatin1String("savepos") || cmd == QLatin1String("save pos")) {
        btnSavePos_->click();
        if (pointLog_) {
            pointLog_->appendPlainText("USB command executed: SAVE POS");
        }
        return;
    }

    if (cmd == QLatin1String("stop")) {
        controller_.stop();
        if (pointLog_) {
            pointLog_->appendPlainText("USB command executed: STOP");
        }
        return;
    }

    if (cmd == QLatin1String("clear")) {
        controller_.clear();
        if (pointLog_) {
            pointLog_->appendPlainText("USB command executed: CLEAR");
        }
        return;
    }

    if (cmd == QLatin1String("done") || cmd == QLatin1String("finish")) {
        triggerPythonLineMap();
        if (pointLog_) {
            pointLog_->appendPlainText("USB command executed: DONE");
        }
        return;
    }

    if (cmd == QLatin1String("mode") || cmd == QLatin1String("mode toggle") || cmd == QLatin1String("toggle mode")) {
        btnModeToggle_->click();
        if (pointLog_) {
            pointLog_->appendPlainText("USB command executed: MODE TOGGLE");
        }
        return;
    }

    if (cmd == QLatin1String("mode auto") || cmd == QLatin1String("mode auto record") || cmd == QLatin1String("auto mode") || cmd == QLatin1String("auto record")) {
        controller_.setModeRecord();
        btnModeToggle_->setText("Mode: AUTO RECORD");
        if (pointLog_) {
            pointLog_->appendPlainText("USB command executed: MODE AUTO RECORD");
        }
        return;
    }

    if (cmd == QLatin1String("mode save") || cmd == QLatin1String("mode manual") || cmd == QLatin1String("manual save") || cmd == QLatin1String("save mode")) {
        controller_.setModeSave();
        btnModeToggle_->setText("Mode: MANUAL SAVE");
        if (pointLog_) {
            pointLog_->appendPlainText("USB command executed: MODE MANUAL SAVE");
        }
        return;
    }

    if (cmd == QLatin1String("mode file") || cmd == QLatin1String("file mode")) {
        controller_.setModeFile();
        btnModeToggle_->setText("Mode: File");
        refreshFileModePath();
        if (pointLog_) {
            pointLog_->appendPlainText("USB command executed: MODE FILE");
        }
        return;
    }

    logLine("Unknown USB command: " + cmd);
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

void MainWindow::appendPointLog(const QString& action) {
    if (!pointLog_) {
        return;
    }

    AxisPositions pos{};
    if (!controller_.getPos(pos)) {
        return;
    }

    QStringList parts;
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        parts << QString::fromStdString(controller_.axisName(i)) + "=" + QString::number(pos[i]);
    }
    pointLog_->appendPlainText(QString("%1 | %2").arg(action, parts.join("  ")));
}

void MainWindow::applyBaseVelocity() {
    controller_.setBaseVelocityOverride(baseVelocityInput_ ? baseVelocityInput_->value() : 0.0);
}

void MainWindow::loadFirstFilePointToInputs(const QString& path) {
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        logLine("Cannot open mapped file for first point: " + path);
        return;
    }

    while (!file.atEnd()) {
        const QString line = QString::fromUtf8(file.readLine()).trimmed();
        if (line.isEmpty() || line.startsWith('#') || line.startsWith(';')) {
            continue;
        }

        const QStringList parts = line.split(QRegularExpression(QStringLiteral("\\s+")),
                                             Qt::SkipEmptyParts);
        if (parts.size() < static_cast<int>(AXIS_COUNT)) {
            logLine("First mapped point has fewer values than active axes.");
            return;
        }

        for (size_t i = 0; i < AXIS_COUNT && i < axisInputs_.size(); ++i) {
            bool ok = false;
            const int value = parts[static_cast<int>(i)].toInt(&ok);
            if (!ok) {
                logLine("Invalid pulse value in first mapped point: " + parts[static_cast<int>(i)]);
                return;
            }
            axisInputs_[i]->setValue(value);
        }
        logLine("Loaded first FILE point into Motor Pulses.");
        return;
    }

    logLine("Mapped file does not contain any pulse point: " + path);
}

void MainWindow::refreshLineMapPreviews() {
    const QString oxyPath = resolveLinePreviewPath(config_, QStringLiteral("oxy"));
    const QString oxzPath = resolveLinePreviewPath(config_, QStringLiteral("oxz"));

    auto loadPreview = [](QLabel* label, const QString& path, const QString& fallbackText) {
        if (!label) {
            return;
        }
        if (path.isEmpty()) {
            label->setPixmap(QPixmap());
            label->setText(fallbackText);
            return;
        }

        QPixmap pixmap(path);
        if (pixmap.isNull()) {
            label->setPixmap(QPixmap());
            label->setText("Cannot load preview image");
            return;
        }
        label->setPixmap(pixmap.scaled(label->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    };

    loadPreview(oxyPreview_, oxyPath, "OXY preview not found");
    loadPreview(oxzPreview_, oxzPath, "OXZ preview not found");

    if (!oxyPath.isEmpty() || !oxzPath.isEmpty()) {
        logLine(QString("Line map previews updated. OXY: %1 | OXZ: %2")
                    .arg(oxyPath.isEmpty() ? QStringLiteral("missing") : oxyPath)
                    .arg(oxzPath.isEmpty() ? QStringLiteral("missing") : oxzPath));
    }
}

QString MainWindow::filePathWithGuiMoveTime(const QString& path) {
    if (!moveTimeInput_ || moveTimeInput_->value() <= 0.0) {
        return path;
    }

    QFile input(path);
    if (!input.open(QIODevice::ReadOnly | QIODevice::Text)) {
        return path;
    }

    const QByteArray content = input.readAll();
    input.close();

    const QString text = QString::fromUtf8(content);
    const QRegularExpression timingHeader(
        QStringLiteral(R"(^\s*[#;]\s*(segment_period_s|segment_seconds|segment_period_ms|segment_ms|segment_period_us|segment_us|movetimevalue|move_time|move_time_s)\s*=)"),
        QRegularExpression::CaseInsensitiveOption | QRegularExpression::MultilineOption);
    if (timingHeader.match(text).hasMatch()) {
        return path;
    }

    const QFileInfo sourceInfo(path);
    const QString timedPath = sourceInfo.dir().absoluteFilePath(
        sourceInfo.completeBaseName() + QStringLiteral("_gui_time.") + sourceInfo.suffix());

    QFile output(timedPath);
    if (!output.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) {
        logLine("Cannot create GUI moveTimeValue file: " + timedPath);
        return path;
    }

    output.write(QString("# moveTimeValue=%1\n")
                     .arg(moveTimeInput_->value(), 0, 'f', 3)
                     .toUtf8());
    output.write(content);
    output.close();

    logLine(QString("GUI moveTimeValue file created: %1").arg(timedPath));
    return timedPath;
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
    loadFirstFilePointToInputs(mappedPath);
    const QString runPath = filePathWithGuiMoveTime(mappedPath);
    controller_.setFileName(runPath.toStdString());
    logLine("FILE mode using: " + runPath);
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
        refreshLineMapPreviews();
    } else {
        logLine(QString("Done failed: Python exited with code %1.").arg(process.exitCode()));
    }
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