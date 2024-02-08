//====================================================================================================================================================
/// Copyright 2024 Lake Orion Robotics FIRST Team 302
///
/// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
/// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
/// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
/// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
/// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================
#pragma once

#include <string>

#include "DragonVision/DragonCamera.h"

class DragonPhotonCam : public DragonCamera
{
public:
    DragonPhotonCam(std::string name,                       /// <I> - camera name
                    DragonCamera::PIPELINE initialPipeline, /// <I> - pipeline the camera starts in
                    units::length::inch_t mountingXOffset,  /// <I> - mounting forward (relative to robot) offset of the limelight
                    units::length::inch_t mountingYOffset,  /// <I> - mounting horizontal (left/right of robot) from the middle of the robot
                    units::length::inch_t mountingZOffset,  /// <I> mounting offset up/down
                    units::angle::degree_t pitch,           /// <I> - Pitch of limelight
                    units::angle::degree_t yaw,             /// <I> - Yaw of limelight
                    units::angle::degree_t roll);           /// <I> - Roll of limelight

    DragonPhotonCam() = delete;

    bool HasTarget();

    units::angle::degree_t GetTargetYaw();
    units::angle::degree_t GetTargetYawRobotFrame();

    double GetPoseAmbiguity();

    units::angle::degree_t GetTargetPitchRobotFrame();
    units::angle::degree_t GetTargetPitch();

    units::time::millisecond_t GetPipelineLatency();
    int GetAprilTagID();
    units::angle::degree_t GetTargetSkew();
    double GetTargetArea();

    std::optional<VisionPose> GetFieldPosition();
    std::optional<VisionPose> GetFieldPosition(frc::DriverStation::Alliance alliance);

    units::length::inch_t EstimateTargetXDistance();
    units::length::inch_t EstimateTargetYDistance();
    units::length::inch_t EstimateTargetZDistance();
    units::length::inch_t EstimateTargetXDistance_RelToRobotCoords();
    units::length::inch_t EstimateTargetYDistance_RelToRobotCoords();
    units::length::inch_t EstimateTargetZDistance_RelToRobotCoords();
    bool UpdatePipeline();

    std::optional<VisionData> GetDataToNearestAprilTag();

private:
    std::string m_name = ""; // camera name
};
