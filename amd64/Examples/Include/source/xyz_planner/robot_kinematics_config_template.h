#pragma once

// Fill this file with your real robot data before using XYZ control.
// Units:
// - length: millimeters
// - angle: radians unless noted otherwise
// - encoder scale: radians per encoder count

#include <array>

namespace robot_config_template {

constexpr std::size_t kAxisCount = 6;
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct KinematicsConfig {
    std::array<double, kAxisCount> aMm {};
    std::array<double, kAxisCount> dMm {};
    std::array<double, kAxisCount> alphaRad {};
    std::array<double, kAxisCount> thetaOffsetRad {};
    std::array<double, kAxisCount> encoderToRad {};
    std::array<double, kAxisCount> jointMinRad {};
    std::array<double, kAxisCount> jointMaxRad {};
    std::array<double, kAxisCount> homeJointRad {};
    Vec3 tcpOffsetMm {};
};

// DH/kinematics notes per axis:
// J1: base rotation
// J2: shoulder
// J3: elbow
// J4: wrist roll / pitch depending on your mechanism
// J5: wrist pitch / yaw depending on your mechanism
// J6: tool flange rotation
//
// Replace all placeholder values below.
constexpr KinematicsConfig kRobotConfig {
    {
        0.0,   // J1 a(mm)
        0.0,   // J2 a(mm)
        0.0,   // J3 a(mm)
        0.0,   // J4 a(mm)
        0.0,   // J5 a(mm)
        0.0    // J6 a(mm)
    },
    {
        0.0,   // J1 d(mm)
        0.0,   // J2 d(mm)
        0.0,   // J3 d(mm)
        0.0,   // J4 d(mm)
        0.0,   // J5 d(mm)
        0.0    // J6 d(mm)
    },
    {
        0.0 * kDegToRad, // J1 alpha
        0.0 * kDegToRad, // J2 alpha
        0.0 * kDegToRad, // J3 alpha
        0.0 * kDegToRad, // J4 alpha
        0.0 * kDegToRad, // J5 alpha
        0.0 * kDegToRad  // J6 alpha
    },
    {
        0.0 * kDegToRad, // J1 theta offset
        0.0 * kDegToRad, // J2 theta offset
        0.0 * kDegToRad, // J3 theta offset
        0.0 * kDegToRad, // J4 theta offset
        0.0 * kDegToRad, // J5 theta offset
        0.0 * kDegToRad  // J6 theta offset
    },
    {
        0.0, // J1 encoderToRad
        0.0, // J2 encoderToRad
        0.0, // J3 encoderToRad
        0.0, // J4 encoderToRad
        0.0, // J5 encoderToRad
        0.0  // J6 encoderToRad
    },
    {
        -180.0 * kDegToRad, // J1 min
        -180.0 * kDegToRad, // J2 min
        -180.0 * kDegToRad, // J3 min
        -180.0 * kDegToRad, // J4 min
        -180.0 * kDegToRad, // J5 min
        -360.0 * kDegToRad  // J6 min
    },
    {
        180.0 * kDegToRad, // J1 max
        180.0 * kDegToRad, // J2 max
        180.0 * kDegToRad, // J3 max
        180.0 * kDegToRad, // J4 max
        180.0 * kDegToRad, // J5 max
        360.0 * kDegToRad  // J6 max
    },
    {
        0.0 * kDegToRad, // J1 home
        0.0 * kDegToRad, // J2 home
        0.0 * kDegToRad, // J3 home
        0.0 * kDegToRad, // J4 home
        0.0 * kDegToRad, // J5 home
        0.0 * kDegToRad  // J6 home
    },
    {
        0.0, // TCP X offset(mm)
        0.0, // TCP Y offset(mm)
        0.0  // TCP Z offset(mm)
    }
};

} // namespace robot_config_template
