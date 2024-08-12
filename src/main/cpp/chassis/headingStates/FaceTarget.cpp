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

#include <tuple>

// Team302 Includes
#include "chassis/DragonDriveTargetFinder.h"
#include "chassis/configs/ChassisConfigMgr.h"
#include "chassis/headingStates/FaceTarget.h"
#include "frc/geometry/Pose2d.h"
#include "utils/FMSData.h"

#include "utils/logging/Logger.h"

FaceTarget::FaceTarget(ChassisOptionEnums::HeadingOption headingOption) : SpecifiedHeading(headingOption)
{
    frc::DriverStation::Alliance m_allianceColor = FMSData::GetInstance()->GetAllianceColor();
}

units::angle::degree_t FaceTarget::GetTargetAngle(ChassisMovement &chassisMovement) const
{
    auto finder = DragonDriveTargetFinder::GetInstance();
    if (finder != nullptr)
    {
        auto info = finder->GetPose(GetVisionElement());
        if (get<0>(info) != DragonDriveTargetFinder::TARGET_INFO::NOT_FOUND)
        {
            auto config = ChassisConfigMgr::GetInstance()->GetCurrentConfig();
            auto chassis = config != nullptr ? config->GetSwerveChassis() : nullptr;

            if (chassis != nullptr)
            {
                auto targetPose = get<1>(info);

                units::angle::degree_t fieldRelativeAngle = (m_allianceColor == frc::DriverStation::Alliance::kBlue && GetVisionElement() == DragonVision::VISION_ELEMENT::SPEAKER) ? (units::angle::degree_t(180) + targetPose.Rotation().Degrees()) : targetPose.Rotation().Degrees();

                chassisMovement.yawAngle = fieldRelativeAngle;
                *m_targetPose = targetPose;
                return fieldRelativeAngle;
            }
        }
    }

    auto config = ChassisConfigMgr::GetInstance()->GetCurrentConfig();
    auto chassis = config != nullptr ? config->GetSwerveChassis() : nullptr;
    if (chassis != nullptr)
    {
        return chassis->GetStoredHeading();
    }

    return chassisMovement.yawAngle;
}

bool FaceTarget::AtTarget()
{
    auto config = ChassisConfigMgr::GetInstance()->GetCurrentConfig();
    auto chassis = config != nullptr ? config->GetSwerveChassis() : nullptr;
    if (chassis != nullptr)
    {
        auto currentPose = chassis->GetPose();
        units::angle::degree_t error = units::math::abs(chassis->GetStoredHeading() - chassis->GetYaw());
        if (GetVisionElement() == DragonVision::VISION_ELEMENT::SPEAKER)
        {

            units::length::meter_t targetMin = (m_allianceColor == frc::DriverStation::Alliance::kBlue) ? m_targetPose->Translation().X() - 1.051_m : m_targetPose->Translation().X() + 1.051_m;
            units::length::meter_t targetMax = (m_allianceColor == frc::DriverStation::Alliance::kBlue) ? m_targetPose->Translation().X() + 1.051_m : m_targetPose->Translation().X() - 1.051_m;

            units::angle::degree_t minError = units::math::atan2(targetMin - currentPose.X(), m_targetPose->Y() - currentPose.Y());
            units::angle::degree_t maxError = units::math::atan2(targetMax - currentPose.X(), m_targetPose->Y() - currentPose.Y());

            return (currentPose.Rotation().Degrees() >= (currentPose.Rotation().Degrees() - minError) && error <= (maxError + currentPose.Rotation().Degrees()));
        }

        else
        {
            return error <= units::angle::degree_t(2.5);
        }
    }

    return false;
}