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
#include "chassis/ChassisConfig.h"
#include "chassis/ChassisConfigMgr.h"
#include "utils/AngleUtils.h"

#include "utils/logging/Logger.h"

ISwerveDriveOrientation::ISwerveDriveOrientation(ChassisOptionEnums::HeadingOption headingOption) : m_headingOption(headingOption)
{
    m_pid.EnableContinuousInput(-180.0, 180.0);
    m_pid.SetIZone(30.0);
    m_pid.SetIntegratorRange(-1.0, 1.0);
}

units::angular_velocity::degrees_per_second_t ISwerveDriveOrientation::CalcHeadingCorrection(units::angle::degree_t targetAngle, double kP)
{
    Logger::GetLogger()->LogDataDirectlyOverNT(std::string("Specified Heading"), std::string("Wrong Meathod"), "True");

    return CalcHeadingCorrection(targetAngle, {kP, 0.0});
}

units::angular_velocity::degrees_per_second_t ISwerveDriveOrientation::CalcHeadingCorrection(units::angle::degree_t targetAngle, std::pair<double, double> gains)
{
    units::angle::degree_t currentAngle = units::angle::degree_t(0.0);
    auto config = ChassisConfigMgr::GetInstance()->GetCurrentConfig();
    auto chassis = config != nullptr ? config->GetSwerveChassis() : nullptr;
    if (chassis != nullptr)
    {
        currentAngle = chassis->GetPose().Rotation().Degrees();
    }
    auto errorAngle = AngleUtils::GetEquivAngle(AngleUtils::GetDeltaAngle(targetAngle, currentAngle));

    m_pid.SetP(gains.first);
    m_pid.SetI(gains.second);

    auto correction = units::angular_velocity::degrees_per_second_t(m_pid->Calculate(currentAngle.value(), targetAngle.value()));

    return correction;
}
