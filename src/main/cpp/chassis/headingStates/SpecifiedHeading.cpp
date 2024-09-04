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
#include "chassis/ChassisOptionEnums.h"
#include "chassis/headingStates/SpecifiedHeading.h"
#include "chassis/configs/ChassisConfig.h"
#include "chassis/configs/ChassisConfigMgr.h"

#include "utils/logging/Logger.h"

SpecifiedHeading::SpecifiedHeading() : ISwerveDriveOrientation(ChassisOptionEnums::HeadingOption::SPECIFIED_ANGLE),
                                       m_targetAngle(units::angle::degree_t(0.0))
{
}

SpecifiedHeading::SpecifiedHeading(ChassisOptionEnums::HeadingOption option) : ISwerveDriveOrientation(option),
                                                                               m_targetAngle(units::angle::degree_t(0.0))
{
}

std::string SpecifiedHeading::GetHeadingStateName() const
{
    return std::string("SpecifiedHeading");
}

void SpecifiedHeading::UpdateChassisSpeeds(ChassisMovement &chassisMovement)
{
    m_targetAngle = GetTargetAngle(chassisMovement);

    auto config = ChassisConfigMgr::GetInstance()->GetCurrentConfig();
    auto chassis = config != nullptr ? config->GetSwerveChassis() : nullptr;
    if (chassis != nullptr)
    {
        auto correction = CalcHeadingCorrection(m_targetAngle, kPSpecifiedHeading);
        chassisMovement.chassisSpeeds.omega += correction;
        chassis->SetStoredHeading(m_targetAngle);
    }
}

units::angle::degree_t SpecifiedHeading::GetTargetAngle(ChassisMovement &chassisMovement) const
{
    return chassisMovement.yawAngle;
}