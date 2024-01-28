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
#include "chassis/headingStates/ISwerveDriveOrientation.h"
#include "configs/RobotConfig.h"
#include "configs/RobotConfigMgr.h"
#include "utils/AngleUtils.h"

ISwerveDriveOrientation::ISwerveDriveOrientation(ChassisOptionEnums::HeadingOption headingOption) : m_headingOption(headingOption)
{
}

units::angular_velocity::degrees_per_second_t ISwerveDriveOrientation::CalcHeadingCorrection(units::angle::degree_t targetAngle, double kP)
{
    units::angle::degree_t currentAngle = units::angle::degree_t(0.0);
    auto config = RobotConfigMgr::GetInstance()->GetCurrentConfig();
    auto chassis = config != nullptr ? config->GetSwerveChassis() : nullptr;
    if (chassis != nullptr)
    {
        currentAngle = chassis->GetPose().Rotation().Degrees();
    }
    auto errorAngle = AngleUtils::GetEquivAngle(AngleUtils::GetDeltaAngle(currentAngle, targetAngle));

    auto correction = units::angular_velocity::degrees_per_second_t(errorAngle.to<double>() * kP);

    return correction;
}

void ISwerveDriveOrientation::SetStoredHeading(units::angle::degree_t heading)
{
    m_storedYaw = heading;
}