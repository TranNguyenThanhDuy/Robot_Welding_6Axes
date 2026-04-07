#pragma once

#include <array>
#include <cstddef>

namespace robot_pose {

constexpr std::size_t kAxisCount = 6;
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    Vec3 operator+(const Vec3& rhs) const { return {x + rhs.x, y + rhs.y, z + rhs.z}; }
    Vec3 operator-(const Vec3& rhs) const { return {x - rhs.x, y - rhs.y, z - rhs.z}; }
    Vec3 operator*(double scale) const { return {x * scale, y * scale, z * scale}; }
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

inline constexpr KinematicsConfig kRobotConfig {
    {0.0, 260.0, 220.0, 0.0, 0.0, 0.0},
    {180.0, 0.0, 0.0, 240.0, 0.0, 90.0},
    {kPi / 2.0, 0.0, 0.0, kPi / 2.0, -kPi / 2.0, 0.0},
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {kDegToRad / 100.0, kDegToRad / 100.0, kDegToRad / 100.0,
     kDegToRad / 100.0, kDegToRad / 100.0, kDegToRad / 100.0},
    {-170.0 * kDegToRad, -120.0 * kDegToRad, -170.0 * kDegToRad,
     -190.0 * kDegToRad, -120.0 * kDegToRad, -360.0 * kDegToRad},
    {170.0 * kDegToRad, 120.0 * kDegToRad, 170.0 * kDegToRad,
     190.0 * kDegToRad, 120.0 * kDegToRad, 360.0 * kDegToRad},
    {0.0, -30.0 * kDegToRad, 60.0 * kDegToRad, 0.0, 30.0 * kDegToRad, 0.0},
    {0.0, 0.0, 180.0}
};

} // namespace robot_pose
