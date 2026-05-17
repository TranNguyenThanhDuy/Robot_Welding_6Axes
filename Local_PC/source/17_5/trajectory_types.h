#pragma once

#include "../axis_controller.h"
#include <vector>

struct TrajectoryPoint {
    AxisPositions positions{};
    AxisVelocities velocities{};
    int time_ms = 0;
};

struct TrajectoryPlannerConfig {
    double segment_seconds = 0.05;
    size_t lookahead_window = 5;
    double min_corner_speed_scale = 0.25;
    double straight_cos_threshold = 0.98;
    double max_scale_drop_per_segment = 0.18;
    double max_scale_rise_per_segment = 0.10;
    double max_velocity_pps = static_cast<double>(base_velocity);
    double max_acceleration_pps2 = 120000.0;
    double max_jerk_pps3 = 2400000.0;
};

using Trajectory = std::vector<TrajectoryPoint>;
