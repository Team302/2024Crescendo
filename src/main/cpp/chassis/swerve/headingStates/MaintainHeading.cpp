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

// Team302 Includes
#include <chassis/swerve/headingStates/MaintainHeading.h>
#include <chassis/ChassisOptionEnums.h>
#include "configs/RobotConfig.h"
#include "configs/RobotConfigMgr.h"

/// DEBUGGING
#include "utils/logging/Logger.h"

MaintainHeading::MaintainHeading() : ISwerveDriveOrientation(ChassisOptionEnums::HeadingOption::MAINTAIN)
{
}

void MaintainHeading::UpdateChassisSpeeds(ChassisMovement &chassisMovement)
{
    units::angular_velocity::degrees_per_second_t correction = units::angular_velocity::degrees_per_second_t(0.0);

    units::radians_per_second_t rot = chassisMovement.chassisSpeeds.omega;

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "Maintain", "VxBEFORE", chassisMovement.chassisSpeeds.vx.to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "Maintain", "VyBEFORE", chassisMovement.chassisSpeeds.vy.to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "Maintain", "OmegaBEFORE", chassisMovement.chassisSpeeds.omega.to<double>());

    auto config = RobotConfigMgr::GetInstance()->GetCurrentConfig();
    auto chassis = config != nullptr ? config->GetSwerveChassis() : nullptr;

    if (std::abs(rot.to<double>()) < 0.1)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "Maintain", "HitIf", true);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "Maintain", "HitElse", false);
        chassisMovement.chassisSpeeds.omega = units::radians_per_second_t(0.0);
        if (abs(chassisMovement.chassisSpeeds.vx.to<double>()) > 0.0 || abs(chassisMovement.chassisSpeeds.vy.to<double>() > 0.0))
        {
            correction = CalcHeadingCorrection(chassis->GetStoredHeading(), m_kPMaintainHeadingControl);

            chassisMovement.chassisSpeeds.omega += correction;
        }
    }
    chassis->SetStoredHeading(chassis->GetPose().Rotation().Degrees());

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "Maintain", "VxAFTER", chassisMovement.chassisSpeeds.vx.to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "Maintain", "VyAFTER", chassisMovement.chassisSpeeds.vy.to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "Maintain", "OmegaAFTER", chassisMovement.chassisSpeeds.omega.to<double>());
}