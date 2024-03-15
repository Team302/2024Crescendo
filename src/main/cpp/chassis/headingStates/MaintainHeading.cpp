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

// Team302 Includes
#include "chassis/ChassisConfig.h"
#include "chassis/ChassisConfigMgr.h"
#include "chassis/ChassisOptionEnums.h"
#include "chassis/headingStates/MaintainHeading.h"
#include "utils/AngleUtils.h"

/// DEBUGGING
#include "utils/logging/Logger.h"

using std::string;

MaintainHeading::MaintainHeading() : ISwerveDriveOrientation(ChassisOptionEnums::HeadingOption::MAINTAIN)
{
    m_controller.EnableContinuousInput(-180.0, 180.0);
}

void MaintainHeading::UpdateChassisSpeeds(ChassisMovement &chassisMovement)
{
    auto config = ChassisConfigMgr::GetInstance()->GetCurrentConfig();
    auto chassis = config != nullptr ? config->GetSwerveChassis() : nullptr;
    if (chassis != nullptr)
    {
        auto correction = units::angular_velocity::degrees_per_second_t(0.0);

        auto isRotating = chassis->IsRotating();
        if (!isRotating)
        {
            auto currentAngle = units::angle::radian_t(AngleUtils::GetEquivAngle(chassis->GetYaw()));
            auto targetAngle = units::angle::radian_t(AngleUtils::GetEquivAngle(chassis->GetStoredHeading()));

            auto radianCorrection = m_controller.Calculate(currentAngle.value(), targetAngle.value());

            correction = units::angular_velocity::radians_per_second_t(radianCorrection);
            chassisMovement.chassisSpeeds.omega += correction;

            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("maintain"), string("currentAngle"), currentAngle.value());
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("maintain"), string("targetAngle"), targetAngle.value());
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("maintain"), string("correction"), radianCorrection);
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("maintain"), string("omega"), chassisMovement.chassisSpeeds.omega.value());
        }
    }
}