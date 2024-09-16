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

#include <string>

// FRC Includes
#include "frc/Timer.h"

// Team302 Includes
#include "chassis/driveStates/RobotDrive.h"
#include "chassis/SwerveChassis.h"

// Third party includes
#include "pathplanner/lib/path/PathPlannerPath.h"
#include "pathplanner/lib/path/PathPlannerTrajectory.h"
#include "pathplanner/lib/controllers/PPHolonomicDriveController.h"

class TrajectoryDrivePathPlanner : public RobotDrive
{
public:
    TrajectoryDrivePathPlanner(RobotDrive *robotDrive);
    std::string GetDriveStateName() const override;

    std::array<frc::SwerveModuleState, 4> UpdateSwerveModuleStates(ChassisMovement &chassisMovement) override;

    void Init(ChassisMovement &chassisMovement) override;

    std::string WhyDone() const { return m_whyDone; };
    bool IsDone();
    units::angular_velocity::degrees_per_second_t CalcHeadingCorrection(units::angle::degree_t targetAngle, double kPFine, double kPCoarse);

protected:
    const units::meters_per_second_t m_maxVel = 4.65_mps;
    const units::meters_per_second_squared_t m_maxAccel = 5.0_mps_sq;
    const units::radians_per_second_t m_maxAngularVel = 540_deg_per_s;
    const units::radians_per_second_squared_t m_maxAngularAccel = 720_deg_per_s_sq;

private:
    bool IsSamePose(frc::Pose2d currentPose, frc::Pose2d previousPose, frc::ChassisSpeeds velocity, double xyTolerance, double rotTolerance, double speedTolerance);

    void LogPose(frc::Pose2d pose) const;
    void LogState(pathplanner::PathPlannerTrajectory::State state) const;
    pathplanner::PathPlannerTrajectory m_trajectory;
    RobotDrive *m_robotDrive;
    pathplanner::PPHolonomicDriveController m_longpathHolonomicController;
    pathplanner::PPHolonomicDriveController m_shortpathHolonomicController;
    std::vector<pathplanner::PathPlannerTrajectory::State> m_trajectoryStates;
    pathplanner::PathPlannerTrajectory::State m_finalState = pathplanner::PathPlannerTrajectory::State();
    frc::Pose2d m_prevPose;
    bool m_wasMoving;
    frc::Transform2d m_delta;
    std::unique_ptr<frc::Timer> m_timer;

    std::string m_whyDone;
    units::time::second_t m_totalTrajectoryTime;

    double m_kPCoarse = 5.0;
    double m_kPFine = 9.0;
    const double m_percentageCompleteThreshold = 0.90;
};