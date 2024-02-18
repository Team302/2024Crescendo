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
#include "chassis/headingStates/FaceAmp.h"
#include "DragonVision/DragonVision.h"
#include "utils/FMSData.h"

FaceAmp::FaceAmp() : FaceTarget(ChassisOptionEnums::HeadingOption::FACE_AMP)
{
}

std::optional<frc::Pose3d> FaceAmp::GetAprilTagPose()
{
    // change the aprilTag variable to use the AprilTagIDs enum
    int aprilTag = (FMSData::GetInstance()->GetAllianceColor() == frc::DriverStation::kBlue ? FaceTarget::BLUE_AMP : FaceTarget::RED_AMP);
    return DragonVision::GetAprilTagLayout().GetTagPose(aprilTag);
}

std::optional<frc::Transform3d> FaceAmp::GetVisionTargetTransform()
{
    auto vision = DragonVision::GetDragonVision();
    if (vision != nullptr)
    {
        auto data = vision->GetVisionData(DragonVision::VISION_ELEMENT::AMP);
        if (data)
        {
            return std::optional<frc::Transform3d>(data.value().transformToTarget);
        }
    }
    return std::nullopt;
}
