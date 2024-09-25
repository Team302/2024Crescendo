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
#include <math.h>
#include <string>

#include "frc/geometry/Pose2d.h"
#include "frc/controller/PIDController.h"
#include "frc/controller/ProfiledPIDController.h"
#include "utils/AngleUtils.h"
#include "utils/FMSData.h"

// Team302 Includes
#include "chassis/driveStates/TrajectoryDrivePathPlanner.h"
#include "chassis/ChassisMovement.h"
#include "utils/logging/Logger.h"
#include "chassis/headingStates/SpecifiedHeading.h"
#include "chassis/DragonDriveTargetFinder.h"
#include "chassis/driveStates/DriveToNote.h"

using frc::Pose2d;
using std::string;

TrajectoryDrivePathPlanner::TrajectoryDrivePathPlanner(RobotDrive *robotDrive) : RobotDrive(robotDrive->GetChassis()),
                                                                                 m_trajectory(),
                                                                                 m_robotDrive(robotDrive),
                                                                                 // TODO need to tune this also update radius as it is probably wrong
                                                                                 m_longpathHolonomicController(pathplanner::PIDConstants(1.5, 0.65, 0.0), //(1.95, 0.95, 0.0)
                                                                                                               pathplanner::PIDConstants(2.5, 2.0, 0.0),
                                                                                                               robotDrive->GetChassis()->GetMaxSpeed(),
                                                                                                               units::length::inch_t(sqrt(((robotDrive->GetChassis()->GetWheelBase().to<double>() / 2.0) * (robotDrive->GetChassis()->GetWheelBase().to<double>() / 2.0) + (robotDrive->GetChassis()->GetTrack().to<double>() / 2.0) * (robotDrive->GetChassis()->GetTrack().to<double>() / 2.0)))),
                                                                                                               units::time::second_t(0.02)),
                                                                                 m_shortpathHolonomicController(pathplanner::PIDConstants(1.5, 0.65, 0.0),
                                                                                                                pathplanner::PIDConstants(2.5, 2.0, 0.0),
                                                                                                                robotDrive->GetChassis()->GetMaxSpeed(),
                                                                                                                units::length::inch_t(sqrt(((robotDrive->GetChassis()->GetWheelBase().to<double>() / 2.0) * (robotDrive->GetChassis()->GetWheelBase().to<double>() / 2.0) + (robotDrive->GetChassis()->GetTrack().to<double>() / 2.0) * (robotDrive->GetChassis()->GetTrack().to<double>() / 2.0)))),
                                                                                                                units::time::second_t(0.02)),
                                                                                 m_trajectoryStates(),
                                                                                 m_prevPose(),
                                                                                 m_wasMoving(false),
                                                                                 m_timer(std::make_unique<frc::Timer>()),
                                                                                 m_whyDone("Trajectory isn't finished/Error"),
                                                                                 m_totalTrajectoryTime(units::time::second_t(0.0))

{
    m_prevPose = m_chassis != nullptr ? m_chassis->GetPose() : Pose2d();
}

std::string TrajectoryDrivePathPlanner::GetDriveStateName() const
{
    return std::string("TrajectoryDrivePathPlanner");
}

void TrajectoryDrivePathPlanner::Init(ChassisMovement &chassisMovement)
{
    m_trajectoryStates.clear();
    m_trajectory = chassisMovement.pathplannerTrajectory;
    m_trajectoryStates = m_trajectory.getStates();
    if (!m_trajectoryStates.empty())
    {
        m_totalTrajectoryTime = m_trajectory.getTotalTime();

        m_finalState = m_trajectoryStates.back();

        m_timer.get()->Reset(); // Restarts and starts timer
        m_timer.get()->Start();
    }
}

std::array<frc::SwerveModuleState, 4> TrajectoryDrivePathPlanner::UpdateSwerveModuleStates(ChassisMovement &chassisMovement)
{

    auto states = chassisMovement.pathplannerTrajectory.getStates();
    if (states.size() != m_trajectoryStates.size())
    {
        Init(chassisMovement);
    }
    if (!m_trajectoryStates.empty()) // If we have a path parsed / have states to run
    {
        if (m_trajectory.getInitialTargetHolonomicPose() != chassisMovement.pathplannerTrajectory.getInitialTargetHolonomicPose())
        {
            Init(chassisMovement);
        }
        /// TO DO If a the target position changes by 0.3m then re generate the path
        auto info = DragonDriveTargetFinder::GetInstance()->GetPose(DragonVision::VISION_ELEMENT::NOTE);
        auto type = get<0>(info);
        auto newNotePos = get<1>(info);

        if (type == DragonDriveTargetFinder::TARGET_INFO::VISION_BASED && chassisMovement.driveOption == ChassisOptionEnums::DriveStateType::DRIVE_TO_NOTE)
        {
            frc::Pose2d currentTargetPos = m_trajectory.getEndState().getTargetHolonomicPose();
            units::length::meter_t distance = currentTargetPos.Translation().Distance(newNotePos.Translation());
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "trajectory drive", "New Note Distance", distance.to<double>());
        }

        auto desiredState = m_trajectory.sample(m_timer.get()->Get() + units::time::second_t(0.02));
        LogState(desiredState);

        frc::ChassisSpeeds refChassisSpeeds;
        if (chassisMovement.pathnamegains == ChassisOptionEnums::PathGainsType::LONG)
        {
            refChassisSpeeds = m_longpathHolonomicController.calculateRobotRelativeSpeeds(m_chassis->GetPose(), desiredState);
        }
        else
        {
            refChassisSpeeds = m_shortpathHolonomicController.calculateRobotRelativeSpeeds(m_chassis->GetPose(), desiredState);
        }

        chassisMovement.chassisSpeeds = refChassisSpeeds;

        m_chassis->SetStoredHeading(m_chassis->GetPose().Rotation().Degrees());
    }
    else // If we don't have states to run, don't move the robot
    {
        // Create 0 speed frc::ChassisSpeeds
        frc::ChassisSpeeds speeds;
        speeds.vx = 0_mps;
        speeds.vy = 0_mps;
        speeds.omega = units::angular_velocity::radians_per_second_t(0);
        chassisMovement.chassisSpeeds = speeds;
    }

    return m_robotDrive->UpdateSwerveModuleStates(chassisMovement);
}

bool TrajectoryDrivePathPlanner::IsDone()
{

    bool isDone = false;

    auto currentPose = m_chassis != nullptr ? m_chassis->GetPose() : Pose2d();
    if (!m_trajectoryStates.empty()) // If we have states...
    {
        auto currentTime = m_timer.get()->Get();

        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "TrajectoryDrive", "current time", currentTime.value());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "TrajectoryDrive", "total time", m_totalTrajectoryTime.value());

        if ((currentTime) / m_totalTrajectoryTime > 0.9)
        {

            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "TrajectoryDrive", "current pose X", currentPose.X().value());
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "TrajectoryDrive", "current pose Y", currentPose.Y().value());
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "TrajectoryDrive", "current pose Rotation", currentPose.Rotation().Degrees().value());

            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "TrajectoryDrive", "target pose X", m_finalState.getTargetHolonomicPose().X().value());
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "TrajectoryDrive", "target pose Y", m_finalState.getTargetHolonomicPose().Y().value());
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "TrajectoryDrive", "target pose Rotation", m_finalState.getTargetHolonomicPose().Rotation().Degrees().value());

            isDone = IsSamePose(currentPose, m_finalState.getTargetHolonomicPose(), m_chassis->GetChassisSpeeds(), 10.0, 3.0, 1.5);
        }
    }
    else
    {
        m_whyDone = "No states in trajectory";
        isDone = true;
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "", "why done", m_whyDone);
    }

    return isDone;
}

bool TrajectoryDrivePathPlanner::IsSamePose(frc::Pose2d currentPose, frc::Pose2d previousPose, frc::ChassisSpeeds velocity, double xyTolerance, double rotTolerance, double speedTolerance)
{
    // Detect if the two poses are the same within a tolerance
    double dCurPosX = currentPose.X().to<double>() * 100; // cm
    double dCurPosY = currentPose.Y().to<double>() * 100;
    double dPrevPosX = previousPose.X().to<double>() * 100;
    double dPrevPosY = previousPose.Y().to<double>() * 100;

    double dCurPosRot = currentPose.Rotation().Degrees().to<double>();
    double dPrevPosRot = previousPose.Rotation().Degrees().to<double>();

    dCurPosRot = dCurPosRot < 0 ? 360 + dCurPosRot : dCurPosRot;
    dPrevPosRot = dPrevPosRot < 0 ? 360 + dPrevPosRot : dPrevPosRot;

    double dDeltaX = abs(dPrevPosX - dCurPosX);
    double dDeltaY = abs(dPrevPosY - dCurPosY);
    double dDeltaRot = abs(dCurPosRot - dPrevPosRot);

    units::velocity::meters_per_second_t chassisSpeed = units::math::sqrt((velocity.vx * velocity.vx) + (velocity.vy * velocity.vy));

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "trajectory drive", "deltaX", dPrevPosX - dCurPosX);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "trajectory drive", "deltaY", dPrevPosY - dCurPosY);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "trajectory drive", "deltaRotation", dDeltaRot);

    //  If Position of X or Y has moved since last scan..  Using Delta X/Y
    return ((dDeltaX <= xyTolerance) && (dDeltaY <= xyTolerance) && (dDeltaRot <= rotTolerance) && (chassisSpeed.to<double>() <= speedTolerance));
}

units::angular_velocity::degrees_per_second_t TrajectoryDrivePathPlanner::CalcHeadingCorrection(units::angle::degree_t targetAngle, double kPFine, double kPCoarse)
{
    units::angle::degree_t currentAngle = m_chassis->GetPose().Rotation().Degrees();
    units::angular_velocity::degrees_per_second_t correction = units::angular_velocity::degrees_per_second_t(0);

    auto errorAngle = AngleUtils::GetEquivAngle(AngleUtils::GetDeltaAngle(currentAngle, targetAngle));

    if (errorAngle < units::angle::degree_t(5.0))
        correction = units::angular_velocity::degrees_per_second_t(errorAngle.to<double>() * kPFine);
    else
        correction = units::angular_velocity::degrees_per_second_t(errorAngle.to<double>() * kPCoarse);

    return correction;
}

void TrajectoryDrivePathPlanner::LogPose(Pose2d pose) const
{
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "trajectory drive", "Target X", pose.X().to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "trajectory drive", "Target Y", pose.Y().to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "trajectory drive", "Target Angle", pose.Rotation().Degrees().to<double>());
}
void TrajectoryDrivePathPlanner::LogState(pathplanner::PathPlannerTrajectory::State state) const
{
    LogPose(state.getTargetHolonomicPose());
}