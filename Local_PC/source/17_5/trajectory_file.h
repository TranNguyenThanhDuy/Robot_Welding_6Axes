#pragma once

#include "trajectory_types.h"
#include <string>

bool readMappedPositionFile(const std::string& filename,
                            AxisVectors& positions,
                            double& segmentSeconds,
                            double& moveTimeSeconds);

bool writeTrajectoryFile(const std::string& filename,
                         const Trajectory& trajectory);

bool readTrajectoryFile(const std::string& filename,
                        Trajectory& trajectory);
