#pragma once

#include <QString>

struct AppConfig {
    QString configPath;
    QString usbCameraDevice = "/dev/video0";
    QString modelPath = "AI_predict/best.onnx";
    QString pythonExecutable = "python3";
    QString pythonMainPath = "./Kinematic/main.py";
    QString lineMappingOutputDir = "line_mapping_outputs";
    QString defaultMappedFileName = "test_line_mapped.txt";
    QString defaultInputPath = "./file_test/test.txt";
};

AppConfig loadAppConfig(const QString& configPath = QString());
