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
#include "configs/RobotConfig.h"
#include "configs/RobotConfigMgr.h"

// Standish Quick Fix
#include <frc/DriverStation.h>

SpecifiedHeading::SpecifiedHeading() : ISwerveDriveOrientation(ChassisOptionEnums::HeadingOption::SPECIFIED_ANGLE),
                                       m_targetAngle(units::angle::degree_t(0.0))
{
}

void SpecifiedHeading::UpdateChassisSpeeds(ChassisMovement &chassisMovement)
{
    m_targetAngle = chassisMovement.yawAngle;
    if (frc::DriverStation::IsAutonomous())
    {
        chassisMovement.chassisSpeeds.omega += CalcHeadingCorrection(m_targetAngle, m_kPGoalHeadingControl);
    }
    else
    {
        chassisMovement.chassisSpeeds.omega += CalcHeadingCorrection(m_targetAngle, m_kPGoalHeadingControl_STANDISH);
    }

    auto config = RobotConfigMgr::GetInstance()->GetCurrentConfig();
    auto chassis = config != nullptr ? config->GetSwerveChassis() : nullptr;
    if (chassis != nullptr)
    {
        chassis->SetStoredHeading(chassis->GetPose().Rotation().Degrees());
    }
}