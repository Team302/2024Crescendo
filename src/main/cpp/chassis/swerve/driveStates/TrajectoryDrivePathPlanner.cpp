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

#include "frc/geometry/Pose2d.h"
#include "frc/controller/PIDController.h"
#include "frc/controller/ProfiledPIDController.h"

// Team302 Includes
#include "chassis/swerve/driveStates/TrajectoryDrivePathPlanner.h"
#include "chassis/ChassisMovement.h"
#include "configs/RobotConfigMgr.h"
#include "configs/RobotConfig.h"
#include "configs/RobotElementNames.h"
#include "utils/logging/Logger.h"
#include "chassis/swerve/headingStates/SpecifiedHeading.h"

using frc::Pose2d;

TrajectoryDrivePathPlanner::TrajectoryDrivePathPlanner(RobotDrive *robotDrive) : RobotDrive(),
                                                                                 m_trajectory(),
                                                                                 m_robotDrive(robotDrive),
                                                                                 m_holonomicController(pathplanner::PIDConstants(0.75, 0.0, 0.0),
                                                                                                       pathplanner::PIDConstants(0.75, 0.0, 0.0),
                                                                                                       units::velocity::feet_per_second_t(15.0),
                                                                                                       units::length::meter_t(0.5),
                                                                                                       units::time::second_t(0.02)),
                                                                                 m_desiredState(),
                                                                                 m_trajectoryStates(),
                                                                                 m_prevPose(),
                                                                                 m_wasMoving(false),
                                                                                 m_timer(std::make_unique<frc::Timer>()),
                                                                                 m_chassis(nullptr),
                                                                                 m_whyDone("Trajectory isn't finished/Error")

{
    auto config = RobotConfigMgr::GetInstance()->GetCurrentConfig();
    m_chassis = config != nullptr ? config->GetSwerveChassis() : nullptr;
    if (m_chassis != nullptr)
    {
        m_prevPose = m_chassis->GetPose();
    }
}

void TrajectoryDrivePathPlanner::Init(ChassisMovement &chassisMovement)
{
    // m_holonomicController.setTolerance(frc::Pose2d{units::length::meter_t(0.1), units::length::meter_t(0.1), frc::Rotation2d(units::angle::degree_t(2.0))});
    // Clear m_trajectoryStates in case it holds onto a previous trajectory
    m_trajectoryStates.clear();

    m_trajectory = chassisMovement.pathplannerTrajectory;
    m_trajectoryStates = m_trajectory.getStates();

    if (!m_trajectoryStates.empty()) // only go if path name found
    {
        // Desired state is first state in trajectory
        m_desiredState = m_trajectoryStates.front(); // m_desiredState is the first state, or starting position

        m_finalState = m_trajectoryStates.back();

        m_timer.get()->Reset(); // Restarts and starts timer
        m_timer.get()->Start();
    }

    m_delta = m_finalState.getTargetHolonomicPose() - m_chassis->GetPose();
}

std::array<frc::SwerveModuleState, 4> TrajectoryDrivePathPlanner::UpdateSwerveModuleStates(ChassisMovement &chassisMovement)
{
    if (!m_trajectoryStates.empty()) // If we have a path parsed / have states to run
    {
        if (m_trajectory.getInitialTargetHolonomicPose() != chassisMovement.pathplannerTrajectory.getInitialTargetHolonomicPose())
        {
            Init(chassisMovement);
        }

        // calculate where we are and where we want to be
        CalcCurrentAndDesiredStates();

        // Use the controller to calculate the chassis speeds for getting there
        frc::ChassisSpeeds refChassisSpeeds;

        // trying to use the last rotation of the path as the target
        refChassisSpeeds = m_holonomicController.calculateRobotRelativeSpeeds(m_chassis->GetPose(), m_desiredState);

        chassisMovement.chassisSpeeds = refChassisSpeeds;

        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "Trajectory Drive Path Planner", "HolonomicRotation (Degs)", m_desiredState.targetHolonomicRotation.Degrees().to<double>());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "Trajectory Drive Path Planner", "Omega (Rads Per Sec)", refChassisSpeeds.omega.to<double>());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "Trajectory Drive Path Planner", "Yaw Odometry (Degs)", m_chassis->GetPose().Rotation().Degrees().to<double>());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "Trajectory Drive Path Planner", "Yaw Pigeon (Degs)", RobotConfigMgr::GetInstance()->GetCurrentConfig()->GetPigeon(RobotElementNames::PIGEON_USAGE::PIGEON_ROBOT_CENTER)->GetYaw().to<double>());

        // Set chassisMovement speeds that will be used by RobotDrive
        return m_robotDrive->UpdateSwerveModuleStates(chassisMovement);
    }
    else // If we don't have states to run, don't move the robot
    {
        // Create 0 speed frc::ChassisSpeeds
        frc::ChassisSpeeds speeds;
        speeds.vx = 0_mps;
        speeds.vy = 0_mps;
        speeds.omega = units::angular_velocity::radians_per_second_t(0);

        // Set chassisMovement speeds that will be used by RobotDrive
        chassisMovement.chassisSpeeds = speeds;

        return m_robotDrive->UpdateSwerveModuleStates(chassisMovement);
    }
}

void TrajectoryDrivePathPlanner::CalcCurrentAndDesiredStates()
{
    // Get current time
    auto sampleTime = units::time::second_t(m_timer.get()->Get());
    // Set desired state to the state at current time
    m_desiredState = m_trajectory.sample(sampleTime);
}

bool TrajectoryDrivePathPlanner::IsDone()
{

    bool isDone = false;

    if (!m_trajectoryStates.empty()) // If we have states...
    {
        // isDone = m_holonomicController.atReference();
        isDone = IsSamePose(m_chassis->GetPose(), m_finalState.getTargetHolonomicPose(), 10.0, 1.0);
    }
    else
    {
        m_whyDone = "No states in trajectory";
        isDone = true;
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "trajectory drive path planner", "why done", m_whyDone);
    }

    return isDone;
}

bool TrajectoryDrivePathPlanner::IsSamePose(frc::Pose2d currentPose, frc::Pose2d previousPose, double xyTolerance, double rotTolerance)
{
    // Detect if the two poses are the same within a tolerance
    double dCurPosX = currentPose.X().to<double>() * 100; // cm
    double dCurPosY = currentPose.Y().to<double>() * 100;
    double dPrevPosX = previousPose.X().to<double>() * 100;
    double dPrevPosY = previousPose.Y().to<double>() * 100;

    double dCurPosRot = currentPose.Rotation().Degrees().to<double>();
    double dPrevPosRot = previousPose.Rotation().Degrees().to<double>();

    double dDeltaX = abs(dPrevPosX - dCurPosX);
    double dDeltaY = abs(dPrevPosY - dCurPosY);
    double dDeltaRot = abs(dPrevPosRot - dCurPosRot);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "trajectory drive path planner", "dCurPosX", dCurPosX);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "trajectory drive path planner", "dCurPosY", dCurPosY);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "trajectory drive path planner", "dCurPosRot", dCurPosRot);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "trajectory drive path planner", "dPrevPosX", dPrevPosX);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "trajectory drive path planner", "dPrevPosY", dPrevPosY);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "trajectory drive path planner", "dPrevPosRot", dPrevPosRot);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "trajectory drive path planner", "dDeltaX", dDeltaX);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "trajectory drive path planner", "dDeltaY", dDeltaY);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "trajectory drive path planner", "dDeltaRot", dDeltaRot);

    //  If Position of X or Y has moved since last scan..  Using Delta X/Y
    return ((dDeltaX <= xyTolerance) && (dDeltaY <= xyTolerance) && (dDeltaRot <= rotTolerance));
}