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

#include "frc/apriltag/AprilTagFieldLayout.h"
#include "frc/apriltag/AprilTagFields.h"
#include "frc/DriverStation.h"
#include "utils/FMSData.h"

// Team302 Includes
#include "chassis/ChassisOptionEnums.h"
#include "chassis/headingStates/FaceCenterStage.h"
#include "utils/FMSData.h"
#include "DragonVision/DragonVision.h"
#include "DragonVision/DragonAprilTagInfo.h"

FaceCenterStage::FaceCenterStage() : FaceTarget(ChassisOptionEnums::HeadingOption::FACE_CENTER_STAGE)
{
}

std::optional<frc::Pose3d> FaceCenterStage::GetAprilTagPose()
{
    int aprilTag = (FMSData::GetInstance()->GetAllianceColor() == frc::DriverStation::kBlue ? FaceTarget::RED_STAGE_CENTER : FaceTarget::BLUE_STAGE_CENTER);
    return GetLayout().GetTagPose(aprilTag);

    std::optional<frc::Transform3d> GetVisionTargetTransform() override;

    auto vision = DragonVision::GetDragonVision();
    if (vision != nullptr)
    {
        return vision->GetVisionDataFromElement();
    }
    return std::nullopt;
}

std::optional<frc::Transform3d> FaceCenterStage::GetVisionTargetTransform()
{
    auto vision = DragonVision::GetDragonVision();
    if (vision != nullptr)
    {
        auto data = vision->GetVisionData(DragonVision::VISION_ELEMENT::SPEAKER);
        if (data)
        {
            return std::optional<frc::Transform3d>(data.value().deltaToTarget);
        }
    }
    return std::nullopt;
}
