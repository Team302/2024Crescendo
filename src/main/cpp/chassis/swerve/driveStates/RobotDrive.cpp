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

#include <string>

// FRC Includes
#include <frc/filter/SlewRateLimiter.h>
#include "units/velocity.h"
#include "units/angle.h"

// Team302 Includes

#include <chassis/swerve/driveStates/RobotDrive.h>
#include <chassis/ChassisMovement.h>
#include "configs/RobotConfig.h"
#include "configs/RobotConfigMgr.h"
#include "utils/logging/Logger.h"
#include <utils/FMSData.h>
#include <chassis/swerve/driveStates/AntiTip.h>

using std::string;

RobotDrive::RobotDrive() : ISwerveDriveState::ISwerveDriveState(),
                           m_flState(),
                           m_frState(),
                           m_blState(),
                           m_brState(),
                           m_wheelbase(units::length::inch_t(20.0)),
                           m_wheeltrack(units::length::inch_t(20.0)),
                           m_maxspeed(units::velocity::feet_per_second_t(1.0))
{
    auto config = RobotConfigMgr::GetInstance()->GetCurrentConfig();
    chassis = config != nullptr ? config->GetSwerveChassis() : nullptr;
    if (chassis != nullptr)
    {
        m_wheelbase = chassis->GetWheelBase();
        m_wheeltrack = chassis->GetTrack();
        m_maxspeed = chassis->GetMaxSpeed();
    }
    else
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("RobotDrive"), string("Chassis"), string("nullptr"));
    }
}

std::array<frc::SwerveModuleState, 4> RobotDrive::UpdateSwerveModuleStates(ChassisMovement &chassisMovement)
{
    if (chassisMovement.checkTipping)
    {
        AntiTip::DecideTipCorrection(chassisMovement, m_maxspeed);
    }

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "RobotDrive", "Vx", chassisMovement.chassisSpeeds.vx.to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "RobotDrive", "Vy", chassisMovement.chassisSpeeds.vy.to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "RobotDrive", "Omega", chassisMovement.chassisSpeeds.omega.to<double>());

    wpi::array<frc::SwerveModuleState, 4> states = chassis->GetKinematics().ToSwerveModuleStates(chassisMovement.chassisSpeeds, chassisMovement.centerOfRotationOffset + m_centerOfRotation);
    return {states[0], states[1], states[2], states[3]};
}

void RobotDrive::Init(ChassisMovement &chassisMovement)
{
}