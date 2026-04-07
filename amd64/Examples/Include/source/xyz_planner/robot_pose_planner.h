#pragma once

#include "robot_kinematics_config.h"

#include <array>
#include <string>
#include <vector>

namespace robot_pose {

constexpr double kRadToDeg = 180.0 / kPi;

using JointArray = std::array<double, kAxisCount>;
using EncoderArray = std::array<int, kAxisCount>;

struct Pose {
    Vec3 position {};
    std::array<double, 9> rotation {
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0
    };
};

struct Rpy {
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
};

struct PlannedPoint {
    JointArray jointsRad {};
    EncoderArray encoders {};
    Pose pose {};
};

struct PlannedPath {
    std::vector<PlannedPoint> points;
};

double length(const Vec3& v);
std::array<double, 9> rpyToRotation(const Rpy& rpy);
Rpy rotationToRpy(const std::array<double, 9>& rotation);

class RobotKinematics {
public:
    explicit RobotKinematics(const KinematicsConfig& config);

    JointArray encodersToJointRad(const EncoderArray& encoders) const;
    EncoderArray jointRadToEncoders(const JointArray& joints) const;
    Pose forward(const JointArray& joints) const;
    bool inversePosition(const Vec3& targetMm,
                         const JointArray& seedRad,
                         JointArray& solutionRad,
                         std::string& error) const;
    bool inversePose(const Pose& targetPose,
                     const JointArray& seedRad,
                     JointArray& solutionRad,
                     std::string& error) const;
    const JointArray& homeJointRad() const;

private:
    KinematicsConfig config_;
};

class CartesianPlanner {
public:
    explicit CartesianPlanner(const RobotKinematics& kinematics);

    bool planMoveXYZ(const JointArray& startJoint,
                     const Vec3& target,
                     JointArray& solvedJoint,
                     Pose& solvedPose,
                     std::string& error) const;

    bool planMovePose(const JointArray& startJoint,
                      const Pose& target,
                      JointArray& solvedJoint,
                      Pose& solvedPose,
                      std::string& error) const;

    bool interpolateXYZ(const JointArray& startJoint,
                        const Vec3& target,
                        double stepMm,
                        PlannedPath& path,
                        std::string& error) const;

    bool interpolatePose(const JointArray& startJoint,
                         const Pose& target,
                         double stepMm,
                         PlannedPath& path,
                         std::string& error) const;

private:
    const RobotKinematics& kinematics_;
};

} // namespace robot_pose
