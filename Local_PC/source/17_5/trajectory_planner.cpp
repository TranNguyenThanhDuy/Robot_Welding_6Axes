#include "trajectory_planner.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace {
using AxisDoubles = std::array<double, AXIS_COUNT>;

double clampDouble(double value, double low, double high) {
    return std::max(low, std::min(high, value));
}

AxisPositions pathPointAt(const AxisVectors& paths, size_t idx) {
    AxisPositions point{};
    for (size_t axis = 0; axis < AXIS_COUNT; ++axis) {
        point[axis] = paths[axis][idx];
    }
    return point;
}

double vectorLength(const AxisPositions& from, const AxisPositions& to) {
    double sum = 0.0;
    for (size_t axis = 0; axis < AXIS_COUNT; ++axis) {
        const double d = static_cast<double>(to[axis]) - static_cast<double>(from[axis]);
        sum += d * d;
    }
    return std::sqrt(sum);
}

double cornerSpeedScale(const AxisPositions& before,
                        const AxisPositions& corner,
                        const AxisPositions& after,
                        const TrajectoryPlannerConfig& config) {
    const double lenA = vectorLength(before, corner);
    const double lenB = vectorLength(corner, after);
    if (lenA <= 0.0 || lenB <= 0.0) {
        return 1.0;
    }

    double dot = 0.0;
    for (size_t axis = 0; axis < AXIS_COUNT; ++axis) {
        const double a = static_cast<double>(corner[axis]) - static_cast<double>(before[axis]);
        const double b = static_cast<double>(after[axis]) - static_cast<double>(corner[axis]);
        dot += a * b;
    }

    const double cosTheta = clampDouble(dot / (lenA * lenB), -1.0, 1.0);
    if (cosTheta >= config.straight_cos_threshold) {
        return 1.0;
    }
    return std::max(config.min_corner_speed_scale, (1.0 + cosTheta) * 0.5);
}

double lookaheadSpeedScale(const AxisVectors& paths,
                           size_t currentIndex,
                           size_t steps,
                           const AxisPositions& previous,
                           const TrajectoryPlannerConfig& config) {
    double scale = 1.0;
    if (steps < 2 || currentIndex >= steps - 1) {
        return scale;
    }

    const size_t lastCorner =
        std::min(steps - 2, currentIndex + config.lookahead_window - 1);
    for (size_t cornerIdx = currentIndex; cornerIdx <= lastCorner; ++cornerIdx) {
        AxisPositions before{};
        if (cornerIdx == currentIndex) {
            before = previous;
        } else {
            before = pathPointAt(paths, cornerIdx - 1);
        }
        const AxisPositions corner = pathPointAt(paths, cornerIdx);
        const AxisPositions after = pathPointAt(paths, cornerIdx + 1);
        scale = std::min(scale, cornerSpeedScale(before, corner, after, config));
    }
    return scale;
}

double clampScaleStep(double previousScale,
                      double desiredScale,
                      const TrajectoryPlannerConfig& config) {
    const double safePrevious =
        clampDouble(previousScale, config.min_corner_speed_scale, 1.0);
    const double safeDesired =
        clampDouble(desiredScale, config.min_corner_speed_scale, 1.0);

    if (safeDesired < safePrevious - config.max_scale_drop_per_segment) {
        return safePrevious - config.max_scale_drop_per_segment;
    }
    if (safeDesired > safePrevious + config.max_scale_rise_per_segment) {
        return safePrevious + config.max_scale_rise_per_segment;
    }
    return safeDesired;
}

AxisDoubles desiredSignedSpeeds(const AxisPositions& current,
                                const AxisPositions& target,
                                double dt,
                                double speedScale,
                                const TrajectoryPlannerConfig& config) {
    AxisDoubles speeds{};
    const double safeDt = (dt > 0.0) ? dt : config.segment_seconds;
    for (size_t axis = 0; axis < AXIS_COUNT; ++axis) {
        const double delta =
            static_cast<double>(target[axis]) - static_cast<double>(current[axis]);
        speeds[axis] = clampDouble((delta / safeDt) * speedScale,
                                   -config.max_velocity_pps,
                                   config.max_velocity_pps);
    }
    return speeds;
}

void applyJerkLimit(const AxisDoubles& desiredSpeeds,
                    double dt,
                    const TrajectoryPlannerConfig& config,
                    AxisDoubles& commandSpeeds,
                    AxisDoubles& commandAccelerations) {
    const double safeDt = (dt > 0.0) ? dt : config.segment_seconds;
    for (size_t axis = 0; axis < AXIS_COUNT; ++axis) {
        const double oldSpeed = commandSpeeds[axis];
        const double targetAccel =
            clampDouble((desiredSpeeds[axis] - commandSpeeds[axis]) / safeDt,
                        -config.max_acceleration_pps2,
                        config.max_acceleration_pps2);
        const double maxAccelDelta = config.max_jerk_pps3 * safeDt;
        commandAccelerations[axis] +=
            clampDouble(targetAccel - commandAccelerations[axis],
                        -maxAccelDelta,
                        maxAccelDelta);
        commandAccelerations[axis] =
            clampDouble(commandAccelerations[axis],
                        -config.max_acceleration_pps2,
                        config.max_acceleration_pps2);
        commandSpeeds[axis] += commandAccelerations[axis] * safeDt;
        commandSpeeds[axis] =
            clampDouble(commandSpeeds[axis],
                        -config.max_velocity_pps,
                        config.max_velocity_pps);

        const double oldError = desiredSpeeds[axis] - oldSpeed;
        const double newError = desiredSpeeds[axis] - commandSpeeds[axis];
        if ((oldError > 0.0 && newError < 0.0) ||
            (oldError < 0.0 && newError > 0.0)) {
            commandSpeeds[axis] = desiredSpeeds[axis];
            commandAccelerations[axis] = 0.0;
        }
    }
}

AxisVelocities toDriverVelocities(const AxisDoubles& speeds) {
    AxisVelocities velocities{};
    velocities.fill(1);
    for (size_t axis = 0; axis < AXIS_COUNT; ++axis) {
        const double speed = std::ceil(std::abs(speeds[axis]));
        if (speed > static_cast<double>(std::numeric_limits<int>::max())) {
            velocities[axis] = std::numeric_limits<int>::max();
        } else {
            velocities[axis] = std::max(1, static_cast<int>(speed));
        }
    }
    return velocities;
}
}

Trajectory planTrajectory(const AxisVectors& positions,
                          const TrajectoryPlannerConfig& config) {
    Trajectory trajectory;
    const size_t steps = positions[0].size();
    if (steps == 0) {
        return trajectory;
    }

    AxisPositions previous = pathPointAt(positions, 0);
    AxisDoubles commandSpeeds{};
    AxisDoubles commandAccelerations{};
    commandSpeeds.fill(0.0);
    commandAccelerations.fill(0.0);

    double currentScale = 1.0;
    int timeMs = 0;

    for (size_t idx = 0; idx < steps; ++idx) {
        const AxisPositions target = pathPointAt(positions, idx);
        const double desiredScale =
            (idx + 1 < steps) ? lookaheadSpeedScale(positions, idx, steps, previous, config)
                              : 1.0;
        currentScale = clampScaleStep(currentScale, desiredScale, config);

        const AxisDoubles desired =
            desiredSignedSpeeds(previous, target, config.segment_seconds, currentScale, config);
        applyJerkLimit(desired,
                       config.segment_seconds,
                       config,
                       commandSpeeds,
                       commandAccelerations);

        TrajectoryPoint point{};
        point.positions = target;
        point.velocities = toDriverVelocities(commandSpeeds);
        point.time_ms = timeMs;
        trajectory.push_back(point);

        timeMs += static_cast<int>(std::llround(config.segment_seconds * 1000.0));
        previous = target;
    }

    return trajectory;
}
