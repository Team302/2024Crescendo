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
#include "chassis/ChassisConfigMgr.h"
#include "chassis/headingStates/FaceTarget.h"
#include "frc/geometry/Pose2d.h"

#include "utils/logging/Logger.h"

FaceTarget::FaceTarget(ChassisOptionEnums::HeadingOption headingOption) : SpecifiedHeading(headingOption)
{
}

units::angle::degree_t FaceTarget::GetTargetAngle(ChassisMovement &chassisMovement) const
{
    auto finder = DragonDriveTargetFinder::GetInstance();
    if (finder != nullptr)
    {
        auto info = finder->GetPose(GetVisionElement());
        if (get<0>(info) == DragonDriveTargetFinder::TARGET_INFO::VISION_BASED)
        {
            auto targetPose = get<1>(info);
            chassisMovement.yawAngle = targetPose.Rotation().Degrees();

            return targetPose.Rotation().Degrees();
        }
        else
        {
            auto config = ChassisConfigMgr::GetInstance()->GetCurrentConfig();
            auto chassis = config != nullptr ? config->GetSwerveChassis() : nullptr;
            auto currentPose = chassis->GetPose();
            if (chassis != nullptr)
            {
                auto targetPose = get<1>(info);
                units::angle::degree_t rawCorrection = units::angle::radian_t(atan(targetPose.Y().to<double>() / targetPose.X().to<double>()));
                units::angle::degree_t fieldRelativeAngle = currentPose.Rotation().Degrees() + rawCorrection;
                chassisMovement.yawAngle = fieldRelativeAngle;
                Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "AlignDebugging", "Raw Correction", rawCorrection.value());
                Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "AlignDebugging", "fieldRelativeAngle", fieldRelativeAngle.value());
                Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "AlignDebugging", "Target X", targetPose.X().value());
                Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "AlignDebugging", "Target Y", targetPose.Y().value());

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
