#include "app_config.h"

#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QSettings>
#include <QStringList>

namespace {
QString resolveConfigPath(const QString& requestedPath) {
    if (!requestedPath.trimmed().isEmpty()) {
        return requestedPath;
    }

    const QString relativePath = QStringLiteral("config/paths.ini");
    const QStringList candidates = {
        QDir::current().absoluteFilePath(relativePath),
        QDir(QCoreApplication::applicationDirPath()).absoluteFilePath(relativePath),
        QDir(QCoreApplication::applicationDirPath()).absoluteFilePath(QStringLiteral("../") + relativePath)
    };

    for (const QString& candidate : candidates) {
        const QFileInfo fileInfo(candidate);
        if (fileInfo.exists() && fileInfo.isFile()) {
            return fileInfo.absoluteFilePath();
        }
    }

    return candidates.front();
}
}

AppConfig loadAppConfig(const QString& configPath) {
    AppConfig config;
    config.configPath = resolveConfigPath(configPath);

    QSettings settings(config.configPath, QSettings::IniFormat);
    settings.beginGroup("paths");

    config.usbCameraDevice =
        settings.value("usb_camera_device", config.usbCameraDevice).toString();
    config.usbSerialDevice =
        settings.value("usb_serial_device", config.usbSerialDevice).toString();
    config.usbSerialBaudRate =
        settings.value("usb_serial_baud_rate", config.usbSerialBaudRate).toInt();
    config.modelPath =
        settings.value("model_path", config.modelPath).toString();
    config.pythonExecutable =
        settings.value("python_executable", config.pythonExecutable).toString();
    config.pythonMainPath =
        settings.value("python_main_path", config.pythonMainPath).toString();
    config.lineMappingOutputDir =
        settings.value("line_mapping_output_dir", config.lineMappingOutputDir).toString();
    config.defaultMappedFileName =
        settings.value("default_mapped_file", config.defaultMappedFileName).toString();
    config.defaultInputPath =
        settings.value("default_input_path", config.defaultInputPath).toString();

    settings.endGroup();
    return config;
}
