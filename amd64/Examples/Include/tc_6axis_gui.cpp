#include "axis_controller.h"
#include <QApplication>
#include <QFont>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMetaObject>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QString>
#include <QStringList>
#include <QVBoxLayout>
#include <QWidget>
#include <iostream>
#include <mutex>
#include <streambuf>
#include <string>
#include <vector>

class GuiLogStreambuf : public std::streambuf {
public:
    GuiLogStreambuf(QPlainTextEdit* edit, std::streambuf* fallback)
        : edit_(edit), fallback_(fallback) {}

protected:
    int overflow(int ch) override {
        if (ch == traits_type::eof()) {
            return traits_type::not_eof(ch);
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            buffer_.push_back(static_cast<char>(ch));
        }
        if (ch == '\n') {
            flushBuffer();
        }
        if (fallback_) {
            fallback_->sputc(static_cast<char>(ch));
        }
        return ch;
    }

    int sync() override {
        flushBuffer();
        if (fallback_) {
            fallback_->pubsync();
        }
        return 0;
    }

private:
    void flushBuffer() {
        std::string text;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            text.swap(buffer_);
        }
        if (text.empty()) {
            return;
        }
        if (!text.empty() && text.back() == '\n') {
            text.pop_back();
        }
        if (text.empty()) {
            return;
        }
        const QString line = QString::fromStdString(text);
        QMetaObject::invokeMethod(edit_, [edit = edit_, line]() {
            edit->appendPlainText(line);
        }, Qt::QueuedConnection);
    }

    QPlainTextEdit* edit_;
    std::streambuf* fallback_;
    std::mutex mutex_;
    std::string buffer_;
};

class StreamRedirect {
public:
    StreamRedirect(QPlainTextEdit* edit, std::ostream& stream)
        : stream_(stream), oldBuf_(stream.rdbuf()), guiBuf_(edit, oldBuf_) {
        stream_.rdbuf(&guiBuf_);
    }

    ~StreamRedirect() {
        stream_.rdbuf(oldBuf_);
    }

private:
    std::ostream& stream_;
    std::streambuf* oldBuf_;
    GuiLogStreambuf guiBuf_;
};

int main(int argc, char** argv) {
    QApplication app(argc, argv);

    AxisController controller;

    QWidget window;
    window.setWindowTitle("Robot Welding 6-Axis Controller");

    auto* root = new QVBoxLayout(&window);

    constexpr size_t kGuiAxisCount = 6;
    auto* title = new QLabel(QString("Robot Controller (GUI %1 axes, active %2)")
                                 .arg(kGuiAxisCount)
                                 .arg(AXIS_COUNT));
    QFont titleFont = title->font();
    titleFont.setPointSize(titleFont.pointSize() + 4);
    titleFont.setBold(true);
    title->setFont(titleFont);
    root->addWidget(title);

    auto* log = new QPlainTextEdit();
    log->setReadOnly(true);
    log->setMinimumHeight(220);

    StreamRedirect coutRedirect(log, std::cout);
    StreamRedirect cerrRedirect(log, std::cerr);

    auto logLine = [&](const QString& text) {
        std::cout << text.toStdString() << std::endl;
    };

    auto* controlBox = new QGroupBox("Control");
    auto* controlLayout = new QHBoxLayout(controlBox);
    auto* btnOn = new QPushButton("Servo ON");
    auto* btnOff = new QPushButton("Servo OFF");
    auto* btnHome = new QPushButton("Home");
    auto* btnSetPos = new QPushButton("SetPos (Zero)");
    auto* btnGetPos = new QPushButton("GetPos");
    controlLayout->addWidget(btnOn);
    controlLayout->addWidget(btnOff);
    controlLayout->addWidget(btnHome);
    controlLayout->addWidget(btnSetPos);
    controlLayout->addWidget(btnGetPos);
    root->addWidget(controlBox);

    auto* motionBox = new QGroupBox("Motion");
    auto* motionLayout = new QVBoxLayout(motionBox);

    auto* axisRow = new QHBoxLayout();
    std::vector<QSpinBox*> axisInputs;
    axisInputs.reserve(kGuiAxisCount);
    for (size_t i = 0; i < kGuiAxisCount; ++i) {
        auto* col = new QVBoxLayout();
        QString axisLabel =
            (i < AXIS_COUNT)
                ? QString::fromStdString(controller.axisName(i))
                : QString("Motor %1").arg(i + 1);
        auto* label = new QLabel(axisLabel);
        auto* spin = new QSpinBox();
        spin->setRange(-1000000, 1000000);
        spin->setSingleStep(100);
        if (i >= AXIS_COUNT) {
            spin->setEnabled(false);
        }
        axisInputs.push_back(spin);
        col->addWidget(label);
        col->addWidget(spin);
        axisRow->addLayout(col);
    }
    motionLayout->addLayout(axisRow);

    auto* motionButtons = new QHBoxLayout();
    auto* btnMove = new QPushButton("MovePos");
    auto* btnGo = new QPushButton("Go (Recorded)");
    auto* btnRecord = new QPushButton("Record");
    auto* btnStop = new QPushButton("Stop");
    auto* btnClear = new QPushButton("Clear");
    motionButtons->addWidget(btnMove);
    motionButtons->addWidget(btnGo);
    motionButtons->addWidget(btnRecord);
    motionButtons->addWidget(btnStop);
    motionButtons->addWidget(btnClear);
    motionLayout->addLayout(motionButtons);
    root->addWidget(motionBox);

    auto* tableBox = new QGroupBox("Position Table");
    auto* tableLayout = new QHBoxLayout(tableBox);
    auto* btnPosTable = new QPushButton("Save Table");
    auto* btnGoPos = new QPushButton("GoPos");
    auto* btnPrintTable = new QPushButton("Print Table");
    tableLayout->addWidget(btnPosTable);
    tableLayout->addWidget(btnGoPos);
    tableLayout->addWidget(btnPrintTable);
    root->addWidget(tableBox);

    root->addWidget(new QLabel("Log"));
    root->addWidget(log);

    controller.initializeSystem();

    QObject::connect(btnOn, &QPushButton::clicked, [&]() {
        //logLine("Servo ON requested.");
        controller.servoOn();
    });
    QObject::connect(btnOff, &QPushButton::clicked, [&]() {
        //logLine("Servo OFF requested.");
        controller.servoOff();
    });
    QObject::connect(btnHome, &QPushButton::clicked, [&]() {
        //logLine("Home requested.");
        controller.home();
    });
    QObject::connect(btnSetPos, &QPushButton::clicked, [&]() {
        //logLine("SetPos requested.");
        controller.setOriginPos();
    });
    QObject::connect(btnGetPos, &QPushButton::clicked, [&]() {
        AxisPositions pos{};
        if (!controller.getPos(pos)) {
            logLine("Failed to read current positions.");
            return;
        }
        QStringList parts;
        for (size_t i = 0; i < AXIS_COUNT; ++i) {
            parts << QString::fromStdString(controller.axisName(i)) + ": " +
                         QString::number(pos[i]);
            if (i < axisInputs.size()) {
                axisInputs[i]->setValue(pos[i]);
            }
        }
        logLine("Current positions - " + parts.join(", "));
    });
    QObject::connect(btnMove, &QPushButton::clicked, [&]() {
        AxisPositions targets{};
        for (size_t i = 0; i < AXIS_COUNT; ++i) {
            targets[i] = axisInputs[i]->value();
        }
        //logLine("MovePos requested.");
        controller.movePos(targets);
    });
    QObject::connect(btnGo, &QPushButton::clicked, [&]() {
        //logLine("Go (recorded) requested.");
        controller.go();
    });
    QObject::connect(btnRecord, &QPushButton::clicked, [&]() {
        //logLine("Record requested.");
        controller.record();
    });
    QObject::connect(btnStop, &QPushButton::clicked, [&]() {
       //logLine("Stop requested.");
        controller.stop();
    });
    QObject::connect(btnClear, &QPushButton::clicked, [&]() {
        //logLine("Clear requested.");
        controller.clear();
    });
    QObject::connect(btnPosTable, &QPushButton::clicked, [&]() {
        //logLine("Save position table requested.");
        controller.posTable();
    });
    QObject::connect(btnGoPos, &QPushButton::clicked, [&]() {
        //logLine("GoPos requested.");
        controller.goPos();
    });
    QObject::connect(btnPrintTable, &QPushButton::clicked, [&]() {
        //logLine("Print table requested (see console output).");
        controller.printTable();
    });

    window.show();
    return app.exec();
}
