#include "robot_pose_planner.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <utility>

namespace robot_pose {

namespace {

struct Mat4 {
    std::array<double, 16> m {
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0
    };
};

double clamp(double value, double minValue, double maxValue) {
    if (value < minValue) return minValue;
    if (value > maxValue) return maxValue;
    return value;
}

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

Vec3 orientationError(const std::array<double, 9>& current,
                      const std::array<double, 9>& target) {
    const auto rErr = multiply3x3(target, transpose3x3(current));
    return {
        0.5 * (rErr[7] - rErr[5]),
        0.5 * (rErr[2] - rErr[6]),
        0.5 * (rErr[3] - rErr[1])
    };
}

Pose interpolatePoseInternal(const Pose& startPose, const Pose& targetPose, double t) {
    const Rpy startRpy = rotationToRpy(startPose.rotation);
    const Rpy endRpy = rotationToRpy(targetPose.rotation);

    return Pose {
        startPose.position + (targetPose.position - startPose.position) * t,
        rpyToRotation({
            startRpy.roll + (endRpy.roll - startRpy.roll) * t,
            startRpy.pitch + (endRpy.pitch - startRpy.pitch) * t,
            startRpy.yaw + (endRpy.yaw - startRpy.yaw) * t
        })
    };
}

} // namespace

double length(const Vec3& v) {
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
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

RobotKinematics::RobotKinematics(const KinematicsConfig& config) : config_(config) {}

JointArray RobotKinematics::encodersToJointRad(const EncoderArray& encoders) const {
    JointArray joints {};
    for (std::size_t i = 0; i < kAxisCount; ++i) {
        joints[i] = static_cast<double>(encoders[i]) * config_.encoderToRad[i];
    }
    return joints;
}

EncoderArray RobotKinematics::jointRadToEncoders(const JointArray& joints) const {
    EncoderArray encoders {};
    for (std::size_t i = 0; i < kAxisCount; ++i) {
        encoders[i] = static_cast<int>(std::llround(joints[i] / config_.encoderToRad[i]));
    }
    return encoders;
}

Pose RobotKinematics::forward(const JointArray& joints) const {
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

bool RobotKinematics::inversePosition(const Vec3& targetMm,
                                      const JointArray& seedRad,
                                      JointArray& solutionRad,
                                      std::string& error) const {
    return inversePose({targetMm, forward(seedRad).rotation}, seedRad, solutionRad, error);
}

bool RobotKinematics::inversePose(const Pose& targetPose,
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

const JointArray& RobotKinematics::homeJointRad() const {
    return config_.homeJointRad;
}

CartesianPlanner::CartesianPlanner(const RobotKinematics& kinematics)
    : kinematics_(kinematics) {}

bool CartesianPlanner::planMoveXYZ(const JointArray& startJoint,
                                   const Vec3& target,
                                   JointArray& solvedJoint,
                                   Pose& solvedPose,
                                   std::string& error) const {
    if (!kinematics_.inversePosition(target, startJoint, solvedJoint, error)) {
        return false;
    }
    solvedPose = kinematics_.forward(solvedJoint);
    return true;
}

bool CartesianPlanner::planMovePose(const JointArray& startJoint,
                                    const Pose& target,
                                    JointArray& solvedJoint,
                                    Pose& solvedPose,
                                    std::string& error) const {
    if (!kinematics_.inversePose(target, startJoint, solvedJoint, error)) {
        return false;
    }
    solvedPose = kinematics_.forward(solvedJoint);
    return true;
}

bool CartesianPlanner::interpolateXYZ(const JointArray& startJoint,
                                      const Vec3& target,
                                      double stepMm,
                                      PlannedPath& path,
                                      std::string& error) const {
    const Pose startPose = kinematics_.forward(startJoint);
    const Vec3 delta = target - startPose.position;
    const double distanceMm = length(delta);
    const int segments = std::max(1, static_cast<int>(std::ceil(distanceMm / stepMm)));

    path.points.clear();
    path.points.reserve(segments);

    JointArray currentJoint = startJoint;
    for (int segment = 1; segment <= segments; ++segment) {
        const double t = static_cast<double>(segment) / static_cast<double>(segments);
        const Vec3 waypoint = startPose.position + delta * t;

        JointArray solved {};
        if (!kinematics_.inversePosition(waypoint, currentJoint, solved, error)) {
            path.points.clear();
            return false;
        }

        path.points.push_back(
            {solved, kinematics_.jointRadToEncoders(solved), kinematics_.forward(solved)});
        currentJoint = solved;
    }

    return true;
}

bool CartesianPlanner::interpolatePose(const JointArray& startJoint,
                                       const Pose& target,
                                       double stepMm,
                                       PlannedPath& path,
                                       std::string& error) const {
    const Pose startPose = kinematics_.forward(startJoint);
    const Vec3 delta = target.position - startPose.position;
    const double distanceMm = length(delta);
    const int segments = std::max(1, static_cast<int>(std::ceil(distanceMm / stepMm)));

    path.points.clear();
    path.points.reserve(segments);

    JointArray currentJoint = startJoint;
    for (int segment = 1; segment <= segments; ++segment) {
        const double t = static_cast<double>(segment) / static_cast<double>(segments);
        const Pose waypoint = interpolatePoseInternal(startPose, target, t);

        JointArray solved {};
        if (!kinematics_.inversePose(waypoint, currentJoint, solved, error)) {
            path.points.clear();
            return false;
        }

        path.points.push_back(
            {solved, kinematics_.jointRadToEncoders(solved), kinematics_.forward(solved)});
        currentJoint = solved;
    }

    return true;
}

} // namespace robot_pose
