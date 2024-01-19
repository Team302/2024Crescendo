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

#include "units/angle.h"
#include "units/length.h"
#include "units/time.h"

#include <string>
#include <vector>
#include <frc/geometry/Pose2d.h>

class DragonCamera
{
public:
    enum PIPELINE
    {
        MACHINE_LEARNING,
        APIRL_TAG,
        COLOR_THRESHOLD
    };

    bool HasTarget() const;

    // Getters

    units::angle::degree_t GetTargetHorizontalOffset() const;
    units::angle::degree_t GetTargetHorizontalOffsetRobotFrame(units::length::inch_t *targetDistOffset_RF, units::length::inch_t *targetDistfromRobot_RF) const;
    units::angle::degree_t GetTargetVerticalOffset() const;
    units::time::microsecond_t GetPipelineLatency() const;
    std::vector<double> Get3DSolve() const;
    PIPELINE getPipeline() const;
    int getAprilTagID() const;

    frc::Pose2d GetRedFieldPosition() const;
    frc::Pose2d GetBlueFieldPosition() const;

    //  Estimating targets

    units::length::inch_t EstimateTargetXdistance() const;
    units::length::inch_t EstimateTargetYdistance() const;

    units::length::inch_t EstimateTargetXdistance_RelToRobotCoords() const;
    units::length::inch_t EstimateTargetYdistance_RelToRobotCoords() const;

    // Setters

    bool SetPipeline(int pipeline);

    // Limelight

    units::angle::degree_t GetLimelightPitch() const { return m_pitch; }
    units::angle::degree_t GetLimelightYaw() const { return m_yaw; }
    units::angle::degree_t GetLimelightRoll() const { return m_roll; }
    units::length::inch_t GetLimelightMountingHeight() const { return m_mountHeight; }
    std::optional<units::length::inch_t> GetTargetHeight() const;

    void SetLimelightPosition(units::length::inch_t mountHeight,
                              units::length::inch_t mountHorizontalOffset,
                              units::length::inch_t mountForwardOffset,
                              units::angle::degree_t pitch,
                              units::angle::degree_t yaw,
                              units::angle::degree_t roll);

    DragonCamera(
        std::string cameraName,                         /// <I> camera name/type
        PIPELINE pipeline,                              /// <I> enum for pipeline
        units::length::inch_t mountingHeight,           /// <I> - mounting height of the limelight
        units::length::inch_t mountingHorizontalOffset, /// <I> - mounting horizontal offset from the middle of the robot
        units::length::inch_t forwardOffset,            /// <I> mounting offset forward/back
        units::angle::degree_t pitch,                   /// <I> - Pitch of limelight
        units::angle::degree_t yaw,                     /// <I> - Yaw of limelight
        units::angle::degree_t roll                     /// <I> - Roll of limelight
    );

protected:
    units::angle::degree_t GetTx() const;
    units::angle::degree_t GetTy() const;

    units::length::inch_t m_mountHeight;
    units::length::inch_t m_mountingHorizontalOffset;
    units::length::inch_t m_mountingForwardOffset;
    units::angle::degree_t m_yaw;
    units::angle::degree_t m_pitch;
    units::angle::degree_t m_roll;
};