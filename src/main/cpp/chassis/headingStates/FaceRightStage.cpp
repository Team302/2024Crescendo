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

#include <optional>

// Team302 Includes
#include "chassis/ChassisOptionEnums.h"
#include "chassis/headingStates/FaceRightStage.h"
#include "chassis/headingStates/FaceTarget.h"
#include "utils/FMSData.h"
#include "frc/apriltag/AprilTagFields.h"

FaceRightStage::FaceRightStage() : FaceTarget(ChassisOptionEnums::HeadingOption::FACE_RIGHT_STAGE)
{
}

std::optional<frc::Pose3d> FaceRightStage::GetAprilTagPose()
{
    int aprilTag = (FMSData::GetInstance()->GetAllianceColor() == frc::DriverStation::kBlue ? FaceTarget::BLUE_STAGE_RIGHT : FaceTarget::RED_STAGE_RIGHT);
    return GetLayout().GetTagPose(aprilTag);
}

std::optional<frc::Transform3d> FaceRightStage::GetVisionTargetTransform()
{
    return std::nullopt;
}
