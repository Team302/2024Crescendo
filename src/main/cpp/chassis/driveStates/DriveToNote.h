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
#include <vector>

// FRC Includes
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Pose2d.h>

// Team302 Includes
#include "chassis/driveStates/RobotDrive.h"
#include "DragonVision/DragonVision.h"
#include "chassis/DragonDriveTargetFinder.h"
#include "pathplanner/lib/path/PathPlannerTrajectory.h"
#include "chassis/driveStates/TrajectoryDrivePathPlanner.h"
#include "utils/FMSData.h"
#include "chassis/SwerveChassis.h"

class DriveToNote : public TrajectoryDrivePathPlanner
{
public:
    DriveToNote(RobotDrive *robotDrive, TrajectoryDrivePathPlanner *trajectoryDrivePathPlanner);
    std::string GetDriveStateName() const override;

    pathplanner::PathPlannerTrajectory CreateDriveToNote();
    pathplanner::PathPlannerTrajectory CreateDriveToNoteTrajectory(frc::Pose2d currentPose, frc::Pose2d notePose);
    void Init(ChassisMovement &chassisMovement) override;
    void InitFromTrajectory(ChassisMovement &chassisMovement, pathplanner::PathPlannerTrajectory trajectory);
    pathplanner::PathPlannerTrajectory GetTrajectory() const { return m_trajectory; }

    bool IsDone();

private:
    // bool keepIntaking = false;

    int m_intakeNoteTimer = 0;
    int m_finishTime = 25; // 0.50 seconds

    void ResetIntakeNoteTimer() { m_intakeNoteTimer = 0; };
    void IntakeNoteTimerIncrement() { m_intakeNoteTimer++; };

    DragonDriveTargetFinder *m_dragonDriveTargetFinder;
    TrajectoryDrivePathPlanner *m_trajectoryDrivePathPlanner;
    // DragonVision *m_visiondata;

    frc::Pose2d m_targetPose;
    frc::Pose2d m_oldTargetPose;

    pathplanner::PathPlannerTrajectory m_trajectory;
};