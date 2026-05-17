#pragma once

#include "trajectory_types.h"

Trajectory planTrajectory(const AxisVectors& positions,
                          const TrajectoryPlannerConfig& config);
