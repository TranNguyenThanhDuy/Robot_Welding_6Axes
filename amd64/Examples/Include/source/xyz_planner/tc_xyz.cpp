#include "DriverConnection.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace {

constexpr std::size_t kAxisCount = 6;
constexpr unsigned int kBaseVelocity = 20000;
constexpr unsigned int kMinVelocity = 50;
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;

using JointArray = std::array<double, kAxisCount>;
using EncoderArray = std::array<int, kAxisCount>;
using BoolArray = std::array<bool, kAxisCount>;
using VelocityArray = std::array<unsigned int, kAxisCount>;
using AxisStatusArray = std::array<EZISERVO_AXISSTATUS, kAxisCount>;

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    Vec3 operator+(const Vec3& rhs) const { return {x + rhs.x, y + rhs.y, z + rhs.z}; }
    Vec3 operator-(const Vec3& rhs) const { return {x - rhs.x, y - rhs.y, z - rhs.z}; }
    Vec3 operator*(double scale) const { return {x * scale, y * scale, z * scale}; }
};

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

struct Mat4 {
    std::array<double, 16> m {
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0
    };
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

// Replace these values with your actual robot dimensions and encoder scale.
constexpr KinematicsConfig kRobotConfig {
    {0.0, 260.0, 220.0, 0.0, 0.0, 0.0},                       // a(mm)
    {180.0, 0.0, 0.0, 240.0, 0.0, 90.0},                      // d(mm)
    {kPi / 2.0, 0.0, 0.0, kPi / 2.0, -kPi / 2.0, 0.0},       // alpha(rad)
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},                           // theta offsets(rad)
    {kDegToRad / 100.0, kDegToRad / 100.0, kDegToRad / 100.0, // rad per encoder count
     kDegToRad / 100.0, kDegToRad / 100.0, kDegToRad / 100.0},
    {-170.0 * kDegToRad, -120.0 * kDegToRad, -170.0 * kDegToRad,
     -190.0 * kDegToRad, -120.0 * kDegToRad, -360.0 * kDegToRad},
    {170.0 * kDegToRad, 120.0 * kDegToRad, 170.0 * kDegToRad,
     190.0 * kDegToRad, 120.0 * kDegToRad, 360.0 * kDegToRad},
    {0.0, -30.0 * kDegToRad, 60.0 * kDegToRad, 0.0, 30.0 * kDegToRad, 0.0},
    {0.0, 0.0, 180.0}
};

Mat4 multiply(const Mat4& lhs, const Mat4& rhs) {
    Mat4 out {};
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            double sum = 0.0;
            for (int k = 0; k < 4; ++k) {
                sum += lhs.m[row * 4 + k] * rhs.m[k * 4 + col];
            }
            out.m[row * 4 + col] = sum;
        }
    }
    return out;
}

Mat4 dhTransform(double a, double d, double alpha, double theta) {
    const double ct = std::cos(theta);
    const double st = std::sin(theta);
    const double ca = std::cos(alpha);
    const double sa = std::sin(alpha);

    return Mat4 {{
        ct, -st * ca, st * sa, a * ct,
        st, ct * ca, -ct * sa, a * st,
        0.0, sa, ca, d,
        0.0, 0.0, 0.0, 1.0
    }};
}

Mat4 translation(double x, double y, double z) {
    Mat4 t {};
    t.m[3] = x;
    t.m[7] = y;
    t.m[11] = z;
    return t;
}

Pose toPose(const Mat4& tf) {
    Pose pose {};
    pose.position = {tf.m[3], tf.m[7], tf.m[11]};
    pose.rotation = {
        tf.m[0], tf.m[1], tf.m[2],
        tf.m[4], tf.m[5], tf.m[6],
        tf.m[8], tf.m[9], tf.m[10]
    };
    return pose;
}

std::array<double, 9> multiply3x3(const std::array<double, 9>& lhs,
                                  const std::array<double, 9>& rhs) {
    std::array<double, 9> out {};
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            double sum = 0.0;
            for (int k = 0; k < 3; ++k) {
                sum += lhs[row * 3 + k] * rhs[k * 3 + col];
            }
            out[row * 3 + col] = sum;
        }
    }
    return out;
}

std::array<double, 9> transpose3x3(const std::array<double, 9>& m) {
    return {
        m[0], m[3], m[6],
        m[1], m[4], m[7],
        m[2], m[5], m[8]
    };
}

std::array<double, 9> rpyToRotation(const Rpy& rpy) {
    const double cr = std::cos(rpy.roll);
    const double sr = std::sin(rpy.roll);
    const double cp = std::cos(rpy.pitch);
    const double sp = std::sin(rpy.pitch);
    const double cy = std::cos(rpy.yaw);
    const double sy = std::sin(rpy.yaw);

    const std::array<double, 9> rx {
        1.0, 0.0, 0.0,
        0.0, cr, -sr,
        0.0, sr, cr
    };
    const std::array<double, 9> ry {
        cp, 0.0, sp,
        0.0, 1.0, 0.0,
        -sp, 0.0, cp
    };
    const std::array<double, 9> rz {
        cy, -sy, 0.0,
        sy, cy, 0.0,
        0.0, 0.0, 1.0
    };
    return multiply3x3(multiply3x3(rz, ry), rx);
}

Rpy rotationToRpy(const std::array<double, 9>& r) {
    Rpy out {};
    out.pitch = std::atan2(-r[6], std::sqrt(r[0] * r[0] + r[3] * r[3]));
    if (std::abs(std::cos(out.pitch)) > 1e-6) {
        out.roll = std::atan2(r[7], r[8]);
        out.yaw = std::atan2(r[3], r[0]);
    } else {
        out.roll = 0.0;
        out.yaw = std::atan2(-r[1], r[4]);
    }
    return out;
}

Vec3 orientationError(const std::array<double, 9>& current,
                      const std::array<double, 9>& target) {
    const auto rErr = multiply3x3(target, transpose3x3(current));
    return {
        0.5 * (rErr[7] - rErr[5]),
        0.5 * (rErr[2] - rErr[6]),
        0.5 * (rErr[3] - rErr[1])
    };
}

double length(const Vec3& v) {
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

double clamp(double value, double minValue, double maxValue) {
    if (value < minValue) return minValue;
    if (value > maxValue) return maxValue;
    return value;
}

class RobotKinematics {
public:
    explicit RobotKinematics(const KinematicsConfig& config) : config_(config) {}

    JointArray encodersToJointRad(const EncoderArray& encoders) const {
        JointArray joints {};
        for (std::size_t i = 0; i < kAxisCount; ++i) {
            joints[i] = static_cast<double>(encoders[i]) * config_.encoderToRad[i];
        }
        return joints;
    }

    EncoderArray jointRadToEncoders(const JointArray& joints) const {
        EncoderArray encoders {};
        for (std::size_t i = 0; i < kAxisCount; ++i) {
            encoders[i] = static_cast<int>(std::llround(joints[i] / config_.encoderToRad[i]));
        }
        return encoders;
    }

    Pose forward(const JointArray& joints) const {
        Mat4 current {};
        for (std::size_t i = 0; i < kAxisCount; ++i) {
            current = multiply(current,
                               dhTransform(config_.aMm[i],
                                           config_.dMm[i],
                                           config_.alphaRad[i],
                                           joints[i] + config_.thetaOffsetRad[i]));
        }
        current = multiply(current,
                           translation(config_.tcpOffsetMm.x,
                                       config_.tcpOffsetMm.y,
                                       config_.tcpOffsetMm.z));
        return toPose(current);
    }

    bool inversePosition(const Vec3& targetMm,
                         const JointArray& seedRad,
                         JointArray& solutionRad,
                         std::string& error) const {
        return inversePose({targetMm, forward(seedRad).rotation}, seedRad, solutionRad, error);
    }

    bool inversePose(const Pose& targetPose,
                     const JointArray& seedRad,
                     JointArray& solutionRad,
                     std::string& error) const {
        JointArray q = seedRad;
        constexpr int maxIterations = 200;
        constexpr double epsilon = 1e-5;
        constexpr double toleranceMm = 0.5;
        constexpr double toleranceRad = 0.02;
        constexpr double lambda = 0.25;
        constexpr double stepScale = 0.7;

        for (int iter = 0; iter < maxIterations; ++iter) {
            const Pose pose = forward(q);
            const Vec3 posErr = targetPose.position - pose.position;
            const Vec3 rotErr = orientationError(pose.rotation, targetPose.rotation);
            if (length(posErr) < toleranceMm && length(rotErr) < toleranceRad) {
                solutionRad = q;
                return true;
            }

            std::array<std::array<double, 6>, kAxisCount> jacobian {};
            for (std::size_t joint = 0; joint < kAxisCount; ++joint) {
                JointArray perturbed = q;
                perturbed[joint] += epsilon;
                const Pose p = forward(perturbed);
                const Vec3 rotDelta = orientationError(pose.rotation, p.rotation);
                jacobian[joint] = {{
                    (p.position.x - pose.position.x) / epsilon,
                    (p.position.y - pose.position.y) / epsilon,
                    (p.position.z - pose.position.z) / epsilon,
                    rotDelta.x / epsilon,
                    rotDelta.y / epsilon,
                    rotDelta.z / epsilon
                }};
            }

            std::array<std::array<double, 6>, 6> jjt {};
            for (std::size_t k = 0; k < kAxisCount; ++k) {
                for (int row = 0; row < 6; ++row) {
                    for (int col = 0; col < 6; ++col) {
                        jjt[row][col] += jacobian[k][row] * jacobian[k][col];
                    }
                }
            }
            for (int i = 0; i < 6; ++i) {
                jjt[i][i] += lambda * lambda;
            }

            std::array<std::array<double, 12>, 6> aug {};
            for (int row = 0; row < 6; ++row) {
                for (int col = 0; col < 6; ++col) {
                    aug[row][col] = jjt[row][col];
                }
                aug[row][row + 6] = 1.0;
            }

            for (int pivot = 0; pivot < 6; ++pivot) {
                int bestRow = pivot;
                for (int row = pivot + 1; row < 6; ++row) {
                    if (std::abs(aug[row][pivot]) > std::abs(aug[bestRow][pivot])) {
                        bestRow = row;
                    }
                }
                if (std::abs(aug[bestRow][pivot]) < 1e-12) {
                    error = "IK failed: singular or ill-conditioned Jacobian.";
                    return false;
                }
                if (bestRow != pivot) {
                    std::swap(aug[bestRow], aug[pivot]);
                }

                const double div = aug[pivot][pivot];
                for (double& value : aug[pivot]) {
                    value /= div;
                }
                for (int row = 0; row < 6; ++row) {
                    if (row == pivot) continue;
                    const double factor = aug[row][pivot];
                    for (int col = 0; col < 12; ++col) {
                        aug[row][col] -= factor * aug[pivot][col];
                    }
                }
            }

            std::array<std::array<double, 6>, 6> inv {};
            for (int row = 0; row < 6; ++row) {
                for (int col = 0; col < 6; ++col) {
                    inv[row][col] = aug[row][col + 6];
                }
            }

            const std::array<double, 6> e {
                posErr.x, posErr.y, posErr.z,
                rotErr.x, rotErr.y, rotErr.z
            };
            std::array<double, 6> tmp {};
            for (int row = 0; row < 6; ++row) {
                for (int col = 0; col < 6; ++col) {
                    tmp[row] += inv[row][col] * e[col];
                }
            }

            for (std::size_t joint = 0; joint < kAxisCount; ++joint) {
                double delta = 0.0;
                for (int i = 0; i < 6; ++i) {
                    delta += jacobian[joint][i] * tmp[i];
                }
                q[joint] += stepScale * delta;
                q[joint] = clamp(q[joint], config_.jointMinRad[joint], config_.jointMaxRad[joint]);
            }
        }

        error = "IK failed: target not reached within iteration limit.";
        return false;
    }

    const JointArray& homeJointRad() const { return config_.homeJointRad; }

private:
    KinematicsConfig config_;
};

class XYZRobotController {
public:
    XYZRobotController() {
        portIds_.fill(0);
        for (std::size_t i = 0; i < kAxisCount; ++i) {
            slaveNos_[i] = static_cast<unsigned char>(i);
        }
    }

    ~XYZRobotController() {
        for (std::size_t i = 0; i < kAxisCount; ++i) {
            FAS_Close(portIds_[i]);
        }
    }

    bool connectAll(const wchar_t* portName = L"ttyUSB0", unsigned int baudRate = 115200) {
        bool allOk = true;
        for (std::size_t i = 0; i < kAxisCount; ++i) {
            if (!Driver_Connection(portName, baudRate, portIds_[i], slaveNos_[i])) {
                std::cout << "Axis " << (i + 1) << " connection failed." << std::endl;
                allOk = false;
            }
        }
        return allOk;
    }

    bool servoOnAll() { return setServoState(true); }
    bool servoOffAll() { return setServoState(false); }

    bool readEncoders(EncoderArray& encoders) const {
        for (std::size_t i = 0; i < kAxisCount; ++i) {
            if (FAS_GetActualPos(portIds_[i], slaveNos_[i], &encoders[i]) != FMM_OK) {
                return false;
            }
        }
        return true;
    }

    bool moveJoints(const EncoderArray& targetEncoders) const {
        EncoderArray current {};
        if (!readEncoders(current)) {
            std::cout << "Failed to read current encoder positions." << std::endl;
            return false;
        }

        AxisStatusArray statuses {};
        if (!readStatuses(statuses)) {
            std::cout << "Failed to read axis status." << std::endl;
            return false;
        }
        for (const auto& st : statuses) {
            if (!st.FFLAG_SERVOON) {
                std::cout << "Some servos are OFF. Turn them ON before moving." << std::endl;
                return false;
            }
        }

        BoolArray hasCommand {};
        int maxDistance = 0;
        for (std::size_t i = 0; i < kAxisCount; ++i) {
            hasCommand[i] = (targetEncoders[i] != current[i]);
            const int distance = std::abs(targetEncoders[i] - current[i]);
            if (distance > maxDistance) {
                maxDistance = distance;
            }
        }

        VelocityArray velocities {};
        velocities.fill(kBaseVelocity);
        if (maxDistance > 0) {
            for (std::size_t i = 0; i < kAxisCount; ++i) {
                if (!hasCommand[i]) continue;
                const int distance = std::abs(targetEncoders[i] - current[i]);
                const unsigned int scaled = static_cast<unsigned int>(
                    (static_cast<long long>(distance) * kBaseVelocity) / maxDistance);
                velocities[i] = std::max(kMinVelocity, scaled);
            }
        }

        for (std::size_t i = 0; i < kAxisCount; ++i) {
            if (!hasCommand[i]) continue;
            if (FAS_MoveSingleAxisAbsPos(portIds_[i], slaveNos_[i], targetEncoders[i],
                                         velocities[i]) != FMM_OK) {
                std::cout << "Move command failed on axis " << (i + 1) << std::endl;
                return false;
            }
        }

        BoolArray done {};
        while (true) {
            bool allDone = true;
            for (std::size_t i = 0; i < kAxisCount; ++i) {
                if (!hasCommand[i] || done[i]) continue;
                EZISERVO_AXISSTATUS status {};
                if (FAS_GetAxisStatus(portIds_[i], slaveNos_[i], &status.dwValue) != FMM_OK) {
                    std::cout << "Status read failed on axis " << (i + 1) << std::endl;
                    return false;
                }
                done[i] = !status.FFLAG_MOTIONING;
                allDone = allDone && done[i];
            }
            if (allDone) break;
        }

        return true;
    }

private:
    bool setServoState(bool on) const {
        bool allOk = true;
        for (std::size_t i = 0; i < kAxisCount; ++i) {
            const bool ok = on ? ServoOn(portIds_[i], slaveNos_[i])
                               : ServoOff(portIds_[i], slaveNos_[i]);
            if (!ok) {
                allOk = false;
            }
        }
        return allOk;
    }

    bool readStatuses(AxisStatusArray& statuses) const {
        for (std::size_t i = 0; i < kAxisCount; ++i) {
            if (FAS_GetAxisStatus(portIds_[i], slaveNos_[i], &statuses[i].dwValue) != FMM_OK) {
                return false;
            }
        }
        return true;
    }

    std::array<int, kAxisCount> portIds_ {};
    std::array<unsigned char, kAxisCount> slaveNos_ {};
};

void printHelp() {
    std::cout << "Commands:\n"
              << "  on                 : servo on all axes\n"
              << "  off                : servo off all axes\n"
              << "  joint              : print current joint values (deg)\n"
              << "  xyz                : print current TCP position (mm)\n"
              << "  pose               : print current TCP XYZ + RPY(deg)\n"
              << "  movexyz X Y Z      : move TCP to target XYZ (mm)\n"
              << "  movelxyz X Y Z [S] : linear XYZ interpolation, optional step S(mm)\n"
              << "  movepose X Y Z R P Y      : move TCP to target pose, RPY in deg\n"
              << "  movelpose X Y Z R P Y [S] : linear pose interpolation, step S(mm)\n"
              << "  homej              : move to configured joint home pose\n"
              << "  q                  : quit\n";
}

void printJoints(const JointArray& jointsRad) {
    std::cout << "Joint(deg): ";
    for (std::size_t i = 0; i < kAxisCount; ++i) {
        std::cout << "J" << (i + 1) << "=" << std::fixed << std::setprecision(2)
                  << jointsRad[i] * kRadToDeg;
        if (i + 1 < kAxisCount) std::cout << ", ";
    }
    std::cout << std::endl;
}

void printPose(const Pose& pose) {
    const Rpy rpy = rotationToRpy(pose.rotation);
    std::cout << "TCP(mm): X=" << std::fixed << std::setprecision(2) << pose.position.x
              << ", Y=" << pose.position.y
              << ", Z=" << pose.position.z
              << " | RPY(deg): R=" << rpy.roll * kRadToDeg
              << ", P=" << rpy.pitch * kRadToDeg
              << ", Y=" << rpy.yaw * kRadToDeg
              << std::endl;
}

} // namespace

int main() {
    std::cout << "XYZ robot controller example" << std::endl;
    std::cout << "Note: update kRobotConfig in source/tc_xyz.cpp to match your robot geometry."
              << std::endl;

    const RobotKinematics kinematics(kRobotConfig);
    XYZRobotController robot;
    robot.connectAll();
    printHelp();

    std::string line;
    while (true) {
        std::cout << "> ";
        if (!std::getline(std::cin, line)) break;
        if (line.empty()) continue;
        if (line == "q") break;

        if (line == "on") {
            robot.servoOnAll();
            continue;
        }
        if (line == "off") {
            robot.servoOffAll();
            continue;
        }

        if (line == "joint" || line == "xyz" || line == "pose") {
            EncoderArray encoders {};
            if (!robot.readEncoders(encoders)) {
                std::cout << "Failed to read encoder positions." << std::endl;
                continue;
            }
            const JointArray joints = kinematics.encodersToJointRad(encoders);
            if (line == "joint") {
                printJoints(joints);
            } else {
                printPose(kinematics.forward(joints));
            }
            continue;
        }

        if (line == "homej") {
            const EncoderArray target = kinematics.jointRadToEncoders(kinematics.homeJointRad());
            robot.moveJoints(target);
            continue;
        }

        if (line.rfind("movexyz", 0) == 0) {
            std::istringstream iss(line);
            std::string cmd;
            Vec3 target {};
            if (!(iss >> cmd >> target.x >> target.y >> target.z)) {
                std::cout << "Usage: movexyz X Y Z" << std::endl;
                continue;
            }

            EncoderArray encoders {};
            if (!robot.readEncoders(encoders)) {
                std::cout << "Failed to read encoder positions." << std::endl;
                continue;
            }

            const JointArray seed = kinematics.encodersToJointRad(encoders);
            JointArray solved {};
            std::string ikError;
            if (!kinematics.inversePosition(target, seed, solved, ikError)) {
                std::cout << ikError << std::endl;
                continue;
            }

            std::cout << "IK solution found." << std::endl;
            printJoints(solved);
            printPose(kinematics.forward(solved));

            const EncoderArray targetEncoders = kinematics.jointRadToEncoders(solved);
            robot.moveJoints(targetEncoders);
            continue;
        }

        if (line.rfind("movelxyz", 0) == 0) {
            std::istringstream iss(line);
            std::string cmd;
            Vec3 target {};
            double stepMm = 10.0;
            if (!(iss >> cmd >> target.x >> target.y >> target.z)) {
                std::cout << "Usage: movelxyz X Y Z [step_mm]" << std::endl;
                continue;
            }
            if (iss >> stepMm) {
                if (stepMm <= 0.0) {
                    std::cout << "step_mm must be > 0" << std::endl;
                    continue;
                }
            }

            EncoderArray currentEncoders {};
            if (!robot.readEncoders(currentEncoders)) {
                std::cout << "Failed to read encoder positions." << std::endl;
                continue;
            }

            JointArray currentJoint = kinematics.encodersToJointRad(currentEncoders);
            const Pose startPose = kinematics.forward(currentJoint);
            const Vec3 delta = target - startPose.position;
            const double distanceMm = length(delta);
            const int segments =
                std::max(1, static_cast<int>(std::ceil(distanceMm / stepMm)));

            std::cout << "MoveL from ";
            printPose(startPose);
            std::cout << "Target(mm): X=" << std::fixed << std::setprecision(2) << target.x
                      << ", Y=" << target.y
                      << ", Z=" << target.z
                      << ", step=" << stepMm
                      << ", segments=" << segments << std::endl;

            bool ok = true;
            for (int segment = 1; segment <= segments; ++segment) {
                const double t = static_cast<double>(segment) / static_cast<double>(segments);
                const Vec3 waypoint = startPose.position + delta * t;

                JointArray solved {};
                std::string ikError;
                if (!kinematics.inversePosition(waypoint, currentJoint, solved, ikError)) {
                    std::cout << "MoveL aborted at segment " << segment << "/" << segments
                              << ": " << ikError << std::endl;
                    ok = false;
                    break;
                }

                const EncoderArray targetEncoders = kinematics.jointRadToEncoders(solved);
                if (!robot.moveJoints(targetEncoders)) {
                    std::cout << "MoveL aborted at segment " << segment << "/" << segments
                              << ": axis move failed." << std::endl;
                    ok = false;
                    break;
                }

                currentJoint = solved;
            }

            if (ok) {
                std::cout << "MoveL complete." << std::endl;
            }
            continue;
        }

        if (line.rfind("movepose", 0) == 0) {
            std::istringstream iss(line);
            std::string cmd;
            Pose targetPose {};
            Rpy targetRpyDeg {};
            if (!(iss >> cmd
                  >> targetPose.position.x >> targetPose.position.y >> targetPose.position.z
                  >> targetRpyDeg.roll >> targetRpyDeg.pitch >> targetRpyDeg.yaw)) {
                std::cout << "Usage: movepose X Y Z Roll Pitch Yaw" << std::endl;
                continue;
            }
            const Rpy targetRpyRad {
                targetRpyDeg.roll * kDegToRad,
                targetRpyDeg.pitch * kDegToRad,
                targetRpyDeg.yaw * kDegToRad
            };
            targetPose.rotation = rpyToRotation(targetRpyRad);

            EncoderArray encoders {};
            if (!robot.readEncoders(encoders)) {
                std::cout << "Failed to read encoder positions." << std::endl;
                continue;
            }

            const JointArray seed = kinematics.encodersToJointRad(encoders);
            JointArray solved {};
            std::string ikError;
            if (!kinematics.inversePose(targetPose, seed, solved, ikError)) {
                std::cout << ikError << std::endl;
                continue;
            }

            std::cout << "IK pose solution found." << std::endl;
            printJoints(solved);
            printPose(kinematics.forward(solved));
            robot.moveJoints(kinematics.jointRadToEncoders(solved));
            continue;
        }

        if (line.rfind("movelpose", 0) == 0) {
            std::istringstream iss(line);
            std::string cmd;
            Pose targetPose {};
            Rpy targetRpyDeg {};
            double stepMm = 10.0;
            if (!(iss >> cmd
                  >> targetPose.position.x >> targetPose.position.y >> targetPose.position.z
                  >> targetRpyDeg.roll >> targetRpyDeg.pitch >> targetRpyDeg.yaw)) {
                std::cout << "Usage: movelpose X Y Z Roll Pitch Yaw [step_mm]" << std::endl;
                continue;
            }
            if (iss >> stepMm) {
                if (stepMm <= 0.0) {
                    std::cout << "step_mm must be > 0" << std::endl;
                    continue;
                }
            }
            const Rpy targetRpyRad {
                targetRpyDeg.roll * kDegToRad,
                targetRpyDeg.pitch * kDegToRad,
                targetRpyDeg.yaw * kDegToRad
            };
            targetPose.rotation = rpyToRotation(targetRpyRad);

            EncoderArray currentEncoders {};
            if (!robot.readEncoders(currentEncoders)) {
                std::cout << "Failed to read encoder positions." << std::endl;
                continue;
            }

            JointArray currentJoint = kinematics.encodersToJointRad(currentEncoders);
            const Pose startPose = kinematics.forward(currentJoint);
            const Vec3 delta = targetPose.position - startPose.position;
            const double distanceMm = length(delta);
            const int segments =
                std::max(1, static_cast<int>(std::ceil(distanceMm / stepMm)));
            const Rpy startRpy = rotationToRpy(startPose.rotation);

            bool ok = true;
            for (int segment = 1; segment <= segments; ++segment) {
                const double t = static_cast<double>(segment) / static_cast<double>(segments);
                const Pose waypoint {
                    startPose.position + delta * t,
                    rpyToRotation({
                        startRpy.roll + (targetRpyRad.roll - startRpy.roll) * t,
                        startRpy.pitch + (targetRpyRad.pitch - startRpy.pitch) * t,
                        startRpy.yaw + (targetRpyRad.yaw - startRpy.yaw) * t
                    })
                };

                JointArray solved {};
                std::string ikError;
                if (!kinematics.inversePose(waypoint, currentJoint, solved, ikError)) {
                    std::cout << "MoveLPose aborted at segment " << segment << "/" << segments
                              << ": " << ikError << std::endl;
                    ok = false;
                    break;
                }
                if (!robot.moveJoints(kinematics.jointRadToEncoders(solved))) {
                    std::cout << "MoveLPose aborted at segment " << segment << "/" << segments
                              << ": axis move failed." << std::endl;
                    ok = false;
                    break;
                }
                currentJoint = solved;
            }

            if (ok) {
                std::cout << "MoveLPose complete." << std::endl;
            }
            continue;
        }

        std::cout << "Unknown command." << std::endl;
        printHelp();
    }

    return 0;
}
