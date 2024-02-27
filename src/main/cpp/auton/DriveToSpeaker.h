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

// Team302 Includes
#include <chassis/SwerveChassis.h>
#include "DragonVision/DragonVision.h"
#include "chassis/DragonDriveTargetFinder.h"
#include "chassis/driveStates/TrajectoryDrivePathPlanner.h"
#include "chassis/driveStates/RobotDrive.h"

// third party includes
#include "pathplanner/lib/path/PathPlannerTrajectory.h"
#include "pathplanner/lib/path/PathPlannerPath.h"
#include "chassis/ChassisMovement.h"

class DriveToSpeaker : public TrajectoryDrivePathPlanner
{
public:
    DriveToSpeaker(RobotDrive *robotDrive, TrajectoryDrivePathPlanner *trajectoryDrivePathPlanner);

    std::array<frc::SwerveModuleState, 4> UpdateSwerveModuleStates(
        ChassisMovement &chassisMovement) override;

    void Init(ChassisMovement &chassisMovement) override;

private:
    pathplanner::PathPlannerTrajectory CreateDriveToSpeakerTrajectory(frc::Pose2d currentPose2d, frc::Pose2d targetPose2d);

    SwerveChassis *m_chassis;
    DragonVision *m_dragonVision;
    DragonDriveTargetFinder *m_dragonDriveTargetFinder;
    TrajectoryDrivePathPlanner *m_trajectoryDrivePathPlanner;

    const units::meters_per_second_t m_maxVel = 3.0_mps;
    const units::meters_per_second_squared_t m_maxAccel = 3.0_mps_sq;
    const units::radians_per_second_t m_maxAngularVel = 360_deg_per_s;
    const units::radians_per_second_squared_t m_maxAngularAccel = 720_deg_per_s_sq;

    pathplanner::PathPlannerTrajectory m_trajectory;
    frc::Pose2d m_targetPose2d;
    frc::Pose2d m_oldTargetPose2d = frc::Pose2d();

    bool m_makeTrajectory;
};