#pragma once

#include "axis_controller.h"
#include "gui_log_redirect.h"
#include "usb_camera.h"

#include <memory>
#include <vector>

#include <QWidget>

class QPlainTextEdit;
class QPushButton;
class QLabel;
class QSpinBox;
class QTimer;

class MainWindow : public QWidget {
public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override;

private:
    void buildUi();
    void connectSignals();
    void logLine(const QString& text);
    void startUsbCamera();
    void stopUsbCamera();
    void updateCameraFrame();

    AxisController controller_;

    QPlainTextEdit* log_ = nullptr;
    QLabel* cameraPreview_ = nullptr;
    std::vector<QSpinBox*> axisInputs_;

    QPushButton* btnOn_ = nullptr;
    QPushButton* btnOff_ = nullptr;
    QPushButton* btnHome_ = nullptr;
    QPushButton* btnSetPos_ = nullptr;
    QPushButton* btnGetPos_ = nullptr;
    QPushButton* btnMove_ = nullptr;
    QPushButton* btnGo_ = nullptr;
    QPushButton* btnRecord_ = nullptr;
    QPushButton* btnStop_ = nullptr;
    QPushButton* btnClear_ = nullptr;
    QPushButton* btnPosTable_ = nullptr;
    QPushButton* btnGoPos_ = nullptr;
    QPushButton* btnPrintTable_ = nullptr;
    QPushButton* btnCamStart_ = nullptr;
    QPushButton* btnCamStop_ = nullptr;

    std::unique_ptr<StreamRedirect> coutRedirect_;
    std::unique_ptr<StreamRedirect> cerrRedirect_;

    UsbCamera camera_;
    QTimer* cameraTimer_ = nullptr;
    int cameraFrameFailCount_ = 0;
};
