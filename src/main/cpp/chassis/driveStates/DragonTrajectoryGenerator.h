//====================================================================================================================================================
// Copyright 2024 Lake Orion Robotics FIRST Team 302
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================

#pragma once

// C++ Includes
#include <map>

// FRC Includes
#include <frc/geometry/Pose2d.h>
#include "units/velocity.h"
#include <units/acceleration.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryConfig.h>

// Team 302 Includes
#include <utils/FMSData.h>

class DragonTrajectoryGenerator
{
public:
    enum WAYPOINTS
    {
        GRID_WALL_COL_ONE,
        GRID_WALL_COL_TWO,
        GRID_WALL_COL_THREE,
        GRID_COOP_COL_ONE,
        GRID_COOP_COL_TWO,
        GRID_COOP_COL_THREE,
        GRID_HP_COL_ONE,
        GRID_HP_COL_TWO,
        GRID_HP_COL_THREE,
        GRID_WALL_INTERMEDIATE,
        GRID_COOP_INTERMEDIATE,
        GRID_HP_INTERMEDIATE
    };

    enum TARGET_POSITION
    {
        COLUMN_ONE,
        COLUMN_TWO,
        COLUMN_THREE,
        HUMAN_PLAYER_SUBSTATION
    };

    DragonTrajectoryGenerator(units::meters_per_second_t maxVelocity,
                              units::meters_per_second_squared_t maxAcceleration);
    ~DragonTrajectoryGenerator() = default;

    /// @brief Generate a trajectory to be fed into TrajectoryDrive
    /// @param currentPose current robot position
    /// @param endPoint ending/goal point
    /// @return frc::Trajectory - the calculated trajectory based on given points
    frc::Trajectory GenerateTrajectory(frc::Pose2d currentPose, TARGET_POSITION endPoint);

private:
    frc::TrajectoryConfig m_config;

    std::unordered_map<WAYPOINTS, frc::Pose2d> m_redWaypoints;
    std::unordered_map<WAYPOINTS, frc::Pose2d> m_blueWaypoints;
    FMSData *m_fmsData;
};