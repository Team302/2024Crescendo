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
#include "chassis/configs/ChassisConfig.h"
#include "chassis/configs/ChassisConfigMgr.h"
#include "utils/AngleUtils.h"

#include "utils/logging/Logger.h"
frc::PIDController *ISwerveDriveOrientation::m_pid = new frc::PIDController(6.0, 5.0, 0.0);

ISwerveDriveOrientation::ISwerveDriveOrientation(ChassisOptionEnums::HeadingOption headingOption) : m_headingOption(headingOption)
{
    m_pid->EnableContinuousInput(-180.0, 180.0);
    m_pid->SetIZone(20.0);
    m_pid->SetIntegratorRange(-360.0, 360.0);
    m_pid->Reset();
}

units::angular_velocity::degrees_per_second_t ISwerveDriveOrientation::CalcHeadingCorrection(units::angle::degree_t targetAngle, double kP)
{
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

    m_pid->SetP(gains.first);
    if (units::math::abs(currentAngle - targetAngle) > units::angle::degree_t(1.0))
        m_pid->SetI(gains.second);
    else
        m_pid->SetI(0);

    auto correction = units::angular_velocity::degrees_per_second_t(m_pid->Calculate(currentAngle.value(), targetAngle.value()));

    return correction;
}
