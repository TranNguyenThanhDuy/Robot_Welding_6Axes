#pragma once

#include "app_config.h"
#include "axis_controller.h"
#include "gui_log_redirect.h"
#include "usb_camera.h"
#include "weld_detector.h"

#include <memory>
#include <vector>

#include <QWidget>
#include <QLineEdit>

class QPlainTextEdit;
class QPushButton;
class QLabel;
class QSpinBox;
class QDoubleSpinBox;
class QTimer;
class QString;

class MainWindow : public QWidget {
public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override;

private:
    void buildUi();
    void connectSignals();
    void logLine(const QString& text);
    void logDetections(const std::vector<Detection>& detections);
    void appendPointLog(const QString& action);
    void refreshLineMapPreviews();
    void applyBaseVelocity();
    void loadFirstFilePointToInputs(const QString& path);
    QString filePathWithGuiMoveTime(const QString& path);
    void refreshFileModePath();
    void startUsbCamera();
    void stopUsbCamera();
    void updateCameraFrame();
    void showLampInfoDialog();
    void showErrorCodeDialog();
    void triggerPythonLineMap();
    void triggerPythonXyzMap();

    AxisController controller_;

    QPlainTextEdit* log_ = nullptr;
    QLabel* cameraPreview_ = nullptr;
    QLabel* oxyPreview_ = nullptr;
    QLabel* oxzPreview_ = nullptr;
    QPlainTextEdit* pointLog_ = nullptr;
    std::vector<QSpinBox*> axisInputs_;
    QDoubleSpinBox* moveTimeInput_ = nullptr;
    QDoubleSpinBox* baseVelocityInput_ = nullptr;

    QPushButton* btnOn_ = nullptr;
    QPushButton* btnOff_ = nullptr;
    QPushButton* btnHome_ = nullptr;
    QPushButton* btnSetPos_ = nullptr;
    QPushButton* btnGetPos_ = nullptr;
    QPushButton* btnMove_ = nullptr;
    QPushButton* btnGo_ = nullptr;
    QPushButton* btnModeToggle_ = nullptr;
    QPushButton* btnRecord_ = nullptr;
    QPushButton* btnRelayState_ = nullptr;
    QPushButton* btnSavePos_ = nullptr;
    QPushButton* btnStop_ = nullptr;
    QPushButton* btnClear_ = nullptr;
    QPushButton* btnDone_ = nullptr;
    QPushButton* btnConvert_ = nullptr;
    QPushButton* btnLampInfo_ = nullptr;
    QPushButton* btnErrorCode_ = nullptr;
    QPushButton* btnAlarmReset_ = nullptr;
    QPushButton* btnCamStart_ = nullptr;
    QPushButton* btnCamStop_ = nullptr;
    QLineEdit* filePathEdit_;
    QString convertPathToLinux(const QString& path);

    AppConfig config_;
    std::unique_ptr<StreamRedirect> coutRedirect_;
    std::unique_ptr<StreamRedirect> cerrRedirect_;

    UsbCamera camera_;
    QTimer* cameraTimer_ = nullptr;
    int cameraFrameFailCount_ = 0;

    WeldDetector detector_;
    bool detectorReady_ = false;
    int detectorFailCount_ = 0;
    int detectorNoDetectionCount_ = 0;
};
