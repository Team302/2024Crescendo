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

#pragma once
#include <optional>

#include "frc/apriltag/AprilTagFieldLayout.h"
#include "frc/apriltag/AprilTagFields.h"
#include "frc/geometry/Pose3d.h"
#include "frc/geometry/Transform3d.h"

// Team302 Includes
#include "chassis/ChassisOptionEnums.h"
#include "chassis/headingStates/ISwerveDriveOrientation.h"

class FaceTarget : public ISwerveDriveOrientation
{
public:
    enum AprilTagIDs
    {
        NO_APRIL_TAG = 0,
        BLUE_SOURCE_ONE = 1,
        BLUE_SOURCE_TWO = 2,
        RED_SOURCE_ONE = 10,
        RED_SOURCE_TWO = 9,
        BLUE_AMP = 6,
        BLUE_STAGE_LEFT = 15,
        BLUE_STAGE_CENTER = 14,
        BLUE_STAGE_RIGHT = 16,
        RED_AMP = 5,
        RED_STAGE_LEFT = 11,
        RED_STAGE_CENTER = 13,
        RED_STAGE_RIGHT = 12,
        BLUE_SUBWOOFER = 8,
        BLUE_SPEAKER = 7,
        RED_SUBWOOFER = 3,
        RED_SPEAKER = 4,
        MAX_APRIL_TAGS

    };

    FaceTarget() = delete;

    FaceTarget(ChassisOptionEnums::HeadingOption headingOption);
    ~FaceTarget() = default;
    void UpdateChassisSpeeds(ChassisMovement &chassisMovement) override;

protected:
    virtual std::optional<frc::Pose3d> GetAprilTagPose() = 0;
    virtual std::optional<frc::Transform3d> GetVisionTargetTransform() = 0;
    frc::AprilTagFieldLayout GetLayout() const { return m_layout; }

private:
    frc::AprilTagFieldLayout m_layout;
};