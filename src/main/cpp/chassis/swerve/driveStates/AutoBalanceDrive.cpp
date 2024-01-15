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

#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Pose2d.h>

// Team302 Includes
#include <chassis/swerve/driveStates/AutoBalanceDrive.h>
#include "chassis/swerve/SwerveChassis.h"
#include "configs/RobotConfig.h"
#include "configs/RobotConfigMgr.h"

/// DEBUGGING
#include "utils/logging/Logger.h"

AutoBalanceDrive::AutoBalanceDrive(RobotDrive *robotDrive) : RobotDrive(),
                                                             m_robotDrive(robotDrive),
                                                             m_chassis(nullptr)
{
    auto config = RobotConfigMgr::GetInstance()->GetCurrentConfig();
    m_chassis = config != nullptr ? config->GetSwerveChassis() : nullptr;
}

std::array<frc::SwerveModuleState, 4> AutoBalanceDrive::UpdateSwerveModuleStates(ChassisMovement &chassisMovement)
{
    // pitch is positive when back of robot is lifted and negative when front of robot is lifted
    // roll is positive when the left side of the robot is lifted and negative when the right side of the robot is lifted

    auto pitch = m_chassis != nullptr ? m_chassis->GetPitch().to<double>() : 0.0;
    auto roll = m_chassis != nullptr ? m_chassis->GetRoll().to<double>() : 0.0;

    // need to drive toware the lifted side
    chassisMovement.chassisSpeeds.vx = -1.0 * pitch * m_pitchConstant;
    chassisMovement.chassisSpeeds.vy = -1.0 * roll * m_rollConstant;
    chassisMovement.chassisSpeeds.omega = units::radians_per_second_t(0.0);

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "Auto Balance Drive", "pitch", pitch);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "Auto Balance Drive", "roll", roll);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "Auto Balance Drive", "vx", chassisMovement.chassisSpeeds.vx.to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "Auto Balance Drive", "vy", chassisMovement.chassisSpeeds.vy.to<double>());

    return m_robotDrive->UpdateSwerveModuleStates(chassisMovement);
}

void AutoBalanceDrive::Init(ChassisMovement &chassisMovement)
{
}
