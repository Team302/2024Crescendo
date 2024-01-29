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
#include "chassis/driveStates/TrajectoryDrive.h"
#include "chassis/ChassisMovement.h"
#include "chassis/ChassisConfig.h"
#include "chassis/ChassisConfigMgr.h"
#include "utils/logging/Logger.h"
#include "chassis/headingStates/SpecifiedHeading.h"

using frc::Pose2d;

TrajectoryDrive::TrajectoryDrive(RobotDrive *robotDrive) : RobotDrive(),
                                                           m_trajectory(),
                                                           m_robotDrive(robotDrive),
                                                           m_holonomicController(frc::PIDController{1.1, 0.00035, 0},
                                                                                 frc::PIDController{1.1, 0.00035, 0},
                                                                                 frc::ProfiledPIDController<units::radian>{0.0, 0.0, 0,
                                                                                                                           frc::TrapezoidProfile<units::radian>::Constraints{0_rad_per_s, 0_rad_per_s / 1_s}}),
                                                           m_desiredState(),
                                                           m_trajectoryStates(),
                                                           m_prevPose(),
                                                           m_wasMoving(false),
                                                           m_timer(std::make_unique<frc::Timer>()),
                                                           m_chassis(nullptr),
                                                           m_whyDone("Trajectory isn't finished/Error")

{
    auto config = ChassisConfigMgr::GetInstance()->GetCurrentConfig();
    m_chassis = config != nullptr ? config->GetSwerveChassis() : nullptr;
    if (m_chassis != nullptr)
    {
        m_prevPose = m_chassis->GetPose();
    }
}

void TrajectoryDrive::Init(ChassisMovement &chassisMovement)
{
    // Clear m_trajectoryStates in case it holds onto a previous trajectory
    m_trajectoryStates.clear();

    m_trajectory = chassisMovement.trajectory;
    m_trajectoryStates = m_trajectory.States();

    if (!m_trajectoryStates.empty()) // only go if path name found
    {
        // Desired state is first state in trajectory
        m_desiredState = m_trajectoryStates.front(); // m_desiredState is the first state, or starting position

        m_finalState = m_trajectoryStates.back();

        m_timer.get()->Reset(); // Restarts and starts timer
        m_timer.get()->Start();
    }

    m_delta = m_finalState.pose - m_chassis->GetPose();
}

std::array<frc::SwerveModuleState, 4> TrajectoryDrive::UpdateSwerveModuleStates(ChassisMovement &chassisMovement)
{
    if (!m_trajectoryStates.empty()) // If we have a path parsed / have states to run
    {
        if (m_trajectory.InitialPose() != chassisMovement.trajectory.InitialPose())
        {
            Init(chassisMovement);
        }

        // calculate where we are and where we want to be
        CalcCurrentAndDesiredStates();

        // Use the controller to calculate the chassis speeds for getting there
        frc::ChassisSpeeds refChassisSpeeds;

        refChassisSpeeds = m_holonomicController.Calculate(m_chassis->GetPose(),
                                                           m_desiredState,
                                                           frc::Rotation2d(chassisMovement.yawAngle));
        chassisMovement.chassisSpeeds = refChassisSpeeds;

        if (chassisMovement.headingOption == ChassisOptionEnums::HeadingOption::SPECIFIED_ANGLE)
        {
            auto specifedHeading = dynamic_cast<SpecifiedHeading *>(m_chassis->GetHeadingState(chassisMovement));
            chassisMovement.chassisSpeeds.omega = units::angular_velocity::radians_per_second_t(0);
            specifedHeading->UpdateChassisSpeeds(chassisMovement);
        }

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

void TrajectoryDrive::CalcCurrentAndDesiredStates()
{
    // Get current time
    auto sampleTime = units::time::second_t(m_timer.get()->Get());
    // Set desired state to the state at current time
    m_desiredState = m_trajectory.Sample(sampleTime);
}

bool TrajectoryDrive::IsDone()
{

    bool isDone = false;

    if (!m_trajectoryStates.empty()) // If we have states...
    {
        auto curPos = m_chassis->GetPose();

        // Check if the current pose and the trajectory's final pose are the same

        if (IsSamePose(curPos, m_finalState.pose, 7.5))
        {
            isDone = true;
            m_whyDone = "Current Pose = Trajectory final pose";
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "trajectory drive", "why done", m_whyDone);
        }
    }
    else
    {
        m_whyDone = "No states in trajectory";
        isDone = true;
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "trajectory drive", "why done", m_whyDone);
    }

    return isDone;
}

bool TrajectoryDrive::IsSamePose(frc::Pose2d currentPose, frc::Pose2d previousPose, double tolerance)
{
    // Detect if the two poses are the same within a tolerance
    double dCurPosX = currentPose.X().to<double>() * 100; // cm
    double dCurPosY = currentPose.Y().to<double>() * 100;
    double dPrevPosX = previousPose.X().to<double>() * 100;
    double dPrevPosY = previousPose.Y().to<double>() * 100;

    double dDeltaX = abs(dPrevPosX - dCurPosX);
    double dDeltaY = abs(dPrevPosY - dCurPosY);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "trajectory drive", "dCurPosX", dCurPosX);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "trajectory drive", "dCurPosY", dCurPosY);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "trajectory drive", "dPrevPosX", dPrevPosX);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "trajectory drive", "dPrevPosY", dPrevPosY);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "trajectory drive", "dDeltaX", dDeltaX);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "trajectory drive", "dDeltaY", dDeltaY);

    //  If Position of X or Y has moved since last scan..  Using Delta X/Y
    return (dDeltaX <= tolerance && dDeltaY <= tolerance);
}