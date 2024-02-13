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
#include "chassis/headingStates/FaceAmp.h"
#include <chassis/ChassisConfigMgr.h>
#include "chassis/headingStates/FaceTarget.h"
#include "frc/geometry/Pose3d.h"

FaceTarget::FaceTarget(ChassisOptionEnums::HeadingOption headingOption) : ISwerveDriveOrientation(headingOption)

{
}

void FaceTarget::UpdateChassisSpeeds(ChassisMovement &chassisMovement)
{
    auto config = ChassisConfigMgr::GetInstance()->GetCurrentConfig();
    auto chassis = config != nullptr ? config->GetSwerveChassis() : nullptr;
    if (chassis != nullptr)
    {
        auto target = GetVisionTargetTransform();
        if (target)
        {
            auto rotation = target.value().Rotation().Angle();
            chassis->SetStoredHeading(rotation);
        }
        else
        {
            auto currentPose = chassis->GetPose();
            auto pose3d = GetAprilTagPose();
            if (pose3d)
            {
                auto targetPose = pose3d.value().ToPose2d();

                auto trans = currentPose - targetPose;
                chassis->SetStoredHeading(trans.Rotation().Degrees());
            }
        }
    }
}