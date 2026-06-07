#include "main_window.h"

#include <QCoreApplication>
#include <QAbstractItemView>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFont>
#include <QFileInfo>
#include <QGridLayout>
#include <QGroupBox>
#include <QHeaderView>
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
#include <QTableWidget>
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

QString xyzFileNameFromMappedFileName(const QString& mappedFile) {
    QFileInfo fileInfo(mappedFile);
    QString baseName = fileInfo.completeBaseName();
    const QString suffix = fileInfo.suffix().isEmpty() ? QStringLiteral("txt") : fileInfo.suffix();

    if (baseName.endsWith(QStringLiteral("_mapped"))) {
        baseName.chop(QStringLiteral("_mapped").size());
        baseName += QStringLiteral("_xyz");
    } else {
        baseName += QStringLiteral("_xyz");
    }

    return baseName + QStringLiteral(".") + suffix;
}

QString resolveLineXyzPath(const AppConfig& config) {
    const QString outputDir = config.lineMappingOutputDir;
    const QString preferredXyzFile = xyzFileNameFromMappedFileName(config.defaultMappedFileName);
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

        const QFileInfo preferredFile(dir.absoluteFilePath(preferredXyzFile));
        if (preferredFile.exists() && preferredFile.isReadable() && preferredFile.isFile()) {
            return preferredFile.absoluteFilePath();
        }

        QFileInfoList xyzFiles = dir.entryInfoList(
            QStringList() << QStringLiteral("*_line_xyz.txt"),
            QDir::Files | QDir::Readable,
            QDir::Time);
        if (!xyzFiles.isEmpty()) {
            return xyzFiles.front().absoluteFilePath();
        }
    }

    return QString();
}

QString mappedSiblingPath(const QString& mappedPath, const QString& replacementSuffix) {
    const QFileInfo mappedInfo(mappedPath);
    QString baseName = mappedInfo.completeBaseName();
    const QString suffix = mappedInfo.suffix().isEmpty() ? QStringLiteral("txt") : mappedInfo.suffix();

    if (baseName.endsWith(QStringLiteral("_mapped"))) {
        baseName.chop(QStringLiteral("_mapped").size());
        baseName += replacementSuffix;
    } else {
        baseName += replacementSuffix;
    }

    return mappedInfo.dir().absoluteFilePath(baseName + QStringLiteral(".") + suffix);
}

QString xyzActualPathFromXyzPath(const QString& xyzPath) {
    const QFileInfo xyzInfo(xyzPath);
    const QString suffix = xyzInfo.suffix().isEmpty() ? QStringLiteral("txt") : xyzInfo.suffix();
    return xyzInfo.dir().absoluteFilePath(
        xyzInfo.completeBaseName() + QStringLiteral("_actual.") + suffix);
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
    btnConvert_ = new QPushButton("Convert");
    const std::vector<QPushButton*> motionButtons = {
        btnMove_,
        btnGo_,
        btnModeToggle_,
        btnRecord_,
        btnSavePos_,
        btnStop_,
        btnClear_,
        btnDone_,
        btnConvert_,
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
        btnOn_,
        btnOff_,
        btnHome_,
        btnSetPos_,
        btnGetPos_,
    };
    for (QPushButton* button : controlButtons) {
        button->setFixedSize(150, 30);
        controlColumn->addWidget(button);
    }
    controlColumn->addStretch();

    auto* infoBox = new QGroupBox("Info");
    infoBox->setFixedWidth(290);
    auto* infoColumn = new QVBoxLayout(infoBox);
    btnLampInfo_ = new QPushButton("Lamp Info");
    btnErrorCode_ = new QPushButton("Error Code");
    btnAlarmReset_ = new QPushButton("Alarm Reset");
    const std::vector<QPushButton*> infoButtons = {
        btnLampInfo_,
        btnErrorCode_,
        new QPushButton("Input Status"),
        new QPushButton("Output Status"),
        btnAlarmReset_,
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
    QObject::connect(btnConvert_, &QPushButton::clicked,
                     [&]() { triggerPythonXyzMap(); });
    QObject::connect(btnAlarmReset_, &QPushButton::clicked,
                     [&]() { controller_.alarmReset(); });
    QObject::connect(btnLampInfo_, &QPushButton::clicked,
                     [&]() { showLampInfoDialog(); });
    QObject::connect(btnErrorCode_, &QPushButton::clicked,
                     [&]() { showErrorCodeDialog(); });
}

void MainWindow::logLine(const QString& text) {
    std::cout << text.toStdString() << std::endl;
}

void MainWindow::showLampInfoDialog() {
    QDialog dialog(this);
    dialog.setWindowTitle("Lamp Information");
    dialog.resize(1100, 330);

    auto* layout = new QVBoxLayout(&dialog);
    auto* table = new QTableWidget(4, 4, &dialog);
    table->setHorizontalHeaderLabels(
        QStringList() << "Indication" << "Color" << "Function" << "ON/OFF Condition");
    table->verticalHeader()->setVisible(false);
    table->setEditTriggers(QAbstractItemView::NoEditTriggers);
    table->setSelectionMode(QAbstractItemView::NoSelection);
    table->setWordWrap(true);

    const QStringList rows[] = {
        {"PWR", "Green", "Power Input Indication",
         "LED is turned ON when power is applied"},
        {"INP", "Yellow", "Complete positioning motion",
         "Lights On when Positioning error reaches within the preset pulse selected by parameter"},
        {"SON", "Orange", "Servo On/Off Indication",
         "Servo On: Lights On, Servo Off: Lights Off"},
        {"ALM", "Red", "Alarm Indication",
         "Flash when protection function is activated (Identifiable which protection mode is activated by counting the blinking times)"},
    };

    for (int row = 0; row < table->rowCount(); ++row) {
        for (int column = 0; column < table->columnCount(); ++column) {
            auto* item = new QTableWidgetItem(rows[row][column]);
            item->setTextAlignment(Qt::AlignLeft | Qt::AlignVCenter);
            table->setItem(row, column, item);
        }
    }

    table->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    table->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
    table->horizontalHeader()->setSectionResizeMode(2, QHeaderView::Stretch);
    table->horizontalHeader()->setSectionResizeMode(3, QHeaderView::Stretch);
    table->resizeRowsToContents();
    layout->addWidget(table);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Close, &dialog);
    QObject::connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
    layout->addWidget(buttons);

    dialog.exec();
}

void MainWindow::showErrorCodeDialog() {
    QDialog dialog(this);
    dialog.setWindowTitle("Error Code Information");
    dialog.resize(1200, 650);

    auto* layout = new QVBoxLayout(&dialog);
    auto* table = new QTableWidget(12, 3, &dialog);
    table->setHorizontalHeaderLabels(QStringList() << "Times" << "Protection" << "Conditions");
    table->verticalHeader()->setVisible(false);
    table->setEditTriggers(QAbstractItemView::NoEditTriggers);
    table->setSelectionMode(QAbstractItemView::NoSelection);
    table->setWordWrap(true);

    const QStringList rows[] = {
        {"1", "Over Current Error",
         "The current through power devices in drive exceeds the limit value*1"},
        {"2", "Over Speed Error",
         "Motor speed exceeds 3,000 [rpm]"},
        {"3", "Position Tracking Error",
         QStringLiteral("Position error value is higher than 180\u00B0 in motor run state*2")},
        {"4", "Over Load Error",
         "The motor is continuously operated more than 5 seconds under a load exceeding the max. torque"},
        {"5", "Over Temperature Error",
         QStringLiteral("Inside temperature of drive exceeds 85\u2103")},
        {"6", "Over Regenerated Voltage Error",
         "Back-EMF is higher than limit value*3"},
        {"7", "Motor Connection Error",
         "The power is ON without connection of the motor cable to drive"},
        {"8", "Encoder Connect Error",
         "Cable connection error in Encoder connection of drive"},
        {"10", "In-Position Error",
         "After operation is finished, position error more than 1 pulse is continued for more than 3 second"},
        {"11", "System Error",
         "Error occurs in drive system"},
        {"12", "ROM Error",
         "Error occurs in parameter storage device (ROM)"},
        {"15", "Position Overflow Error",
         QStringLiteral("Position error value is higher than 180\u00B0 in motor stop state*2")},
    };

    for (int row = 0; row < table->rowCount(); ++row) {
        for (int column = 0; column < table->columnCount(); ++column) {
            auto* item = new QTableWidgetItem(rows[row][column]);
            item->setTextAlignment(Qt::AlignLeft | Qt::AlignVCenter);
            table->setItem(row, column, item);
        }
    }

    table->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    table->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
    table->horizontalHeader()->setSectionResizeMode(2, QHeaderView::Stretch);
    table->resizeRowsToContents();
    layout->addWidget(table);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Close, &dialog);
    QObject::connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
    layout->addWidget(buttons);

    dialog.exec();
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

void MainWindow::triggerPythonXyzMap() {
    const QString xyzPath = resolveLineXyzPath(config_);
    if (xyzPath.isEmpty()) {
        logLine(QString("Convert failed: no XYZ file found in %1")
                    .arg(config_.lineMappingOutputDir));
        return;
    }

    const QFileInfo xyzFile(xyzPath);
    if (!xyzFile.exists() || !xyzFile.isFile()) {
        logLine("Convert failed: XYZ file does not exist: " + xyzPath);
        return;
    }

    const QFileInfo referenceFile(config_.defaultInputPath);
    if (!referenceFile.exists() || !referenceFile.isFile()) {
        logLine("Convert failed: reference input file does not exist: " + config_.defaultInputPath);
        return;
    }

    const QFileInfo pythonMain(config_.pythonMainPath);
    if (!pythonMain.exists() || !pythonMain.isFile()) {
        logLine("Convert failed: Python main.py not found: " + pythonMain.absoluteFilePath());
        return;
    }

    QString mappedPath = resolveLineMappedPath(config_);
    if (mappedPath.isEmpty()) {
        QDir outputDir(QDir::current().absoluteFilePath(config_.lineMappingOutputDir));
        if (!outputDir.exists()) {
            outputDir.mkpath(QStringLiteral("."));
        }
        mappedPath = outputDir.absoluteFilePath(config_.defaultMappedFileName);
    }

    const QString anglesPath = mappedSiblingPath(mappedPath, QStringLiteral("_angles"));
    const QString actualXyzPath = xyzActualPathFromXyzPath(xyzPath);

    QProcess process;
    process.setProgram(config_.pythonExecutable);
    process.setArguments({
        pythonMain.absoluteFilePath(),
        QStringLiteral("xyz-map"),
        xyzPath,
        referenceFile.absoluteFilePath(),
        mappedPath,
        anglesPath,
        actualXyzPath,
    });
    process.setProcessChannelMode(QProcess::MergedChannels);

    logLine("Running Python xyz-map for: " + xyzPath);
    logLine("Convert reference input file: " + referenceFile.absoluteFilePath());
    logLine("Convert output mapped file: " + mappedPath);
    process.start();
    if (!process.waitForStarted()) {
        logLine("Convert failed: cannot start Python process.");
        return;
    }

    if (!process.waitForFinished(-1)) {
        logLine("Convert failed: Python process did not finish correctly.");
        return;
    }

    const QString output = QString::fromLocal8Bit(process.readAll()).trimmed();
    if (!output.isEmpty()) {
        logLine(output);
    }

    if (process.exitStatus() == QProcess::NormalExit && process.exitCode() == 0) {
        logLine("Convert completed: XYZ mapping finished successfully.");
        refreshFileModePath();
        refreshLineMapPreviews();
    } else {
        logLine(QString("Convert failed: Python exited with code %1.").arg(process.exitCode()));
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
