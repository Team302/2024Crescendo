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
#include "chassis/headingStates/MaintainHeading.h"
#include "chassis/ChassisOptionEnums.h"
#include "chassis/ChassisConfig.h"
#include "chassis/ChassisConfigMgr.h"

/// DEBUGGING
#include "utils/logging/Logger.h"

MaintainHeading::MaintainHeading() : ISwerveDriveOrientation(ChassisOptionEnums::HeadingOption::MAINTAIN)
{
}

void MaintainHeading::UpdateChassisSpeeds(ChassisMovement &chassisMovement)
{
    units::angular_velocity::degrees_per_second_t correction = units::angular_velocity::degrees_per_second_t(0.0);

    units::radians_per_second_t rot = chassisMovement.chassisSpeeds.omega;
    auto config = ChassisConfigMgr::GetInstance()->GetCurrentConfig();
    auto chassis = config != nullptr ? config->GetSwerveChassis() : nullptr;
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "MaintainDebugging", "Current Rotation (deg)", chassis->GetPose().Rotation().Degrees().to<double>());

    if (units::math::abs(rot).to<double>() < 0.1)
    {
        chassisMovement.chassisSpeeds.omega = units::radians_per_second_t(0.0);
        if ((abs(chassisMovement.chassisSpeeds.vx.to<double>()) > 0.0) || (abs(chassisMovement.chassisSpeeds.vy.to<double>()) > 0.0))
        {
            correction = CalcHeadingCorrection(chassis->GetStoredHeading(), m_kPMaintainHeadingControl);
        }
    }
    else
        chassis->SetStoredHeading(chassis->GetPose().Rotation().Degrees());

    chassisMovement.chassisSpeeds.omega += correction;
}