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
#include <frc/geometry/Pose3d.h>
#include <frc/DriverStation.h>
class DragonCamera
{
public:
    enum PIPELINE
    {
        MACHINE_LEARNING,
        APIRL_TAG,
        COLOR_THRESHOLD
    };

    virtual bool HasTarget() const;

    // Getters

    virtual units::angle::degree_t GetTargetHorizontalOffset() const = 0;
    virtual units::angle::degree_t GetTargetHorizontalOffsetRobotFrame(units::length::inch_t *targetDistOffset_RF, units::length::inch_t *targetDistfromRobot_RF) const = 0;
    virtual units::angle::degree_t GetTargetVerticalOffsetRobotFrame(units::length::inch_t *targetDistOffset_RF, units::length::inch_t *targetDistfromRobot_RF) const = 0;
    virtual units::angle::degree_t GetTargetVerticalOffset() const = 0;
    virtual units::time::microsecond_t GetPipelineLatency() const = 0;
    virtual PIPELINE getPipeline() const = 0;
    virtual int getAprilTagID() const = 0;

    virtual frc::Pose3d GetFieldPosition() const = 0;
    virtual frc::Pose3d GetFieldPosisition(frc::DriverStation::Alliance alliance) = 0;
    //  Estimating targets

    virtual units::length::inch_t EstimateTargetXdistance() const = 0;
    virtual units::length::inch_t EstimateTargetYdistance() const = 0;

    virtual units::length::inch_t EstimateTargetXdistance_RelToRobotCoords() const = 0;
    virtual units::length::inch_t EstimateTargetYdistance_RelToRobotCoords() const = 0;

    // Setters
    virtual double GetTargetArea() const = 0;
    virtual bool SetPipeline(int pipeline) = 0;
    virtual units::angle::degree_t GetTargetSkew() const = 0;
    units::angle::degree_t GetCameraPitch() const { return m_pitch; }
    units::angle::degree_t GetCameraYaw() const { return m_yaw; }
    units::angle::degree_t GetCameraRoll() const { return m_roll; }
    units::length::inch_t GetMountingYOffset() const { return m_mountingYOffset; }
    units::length::inch_t GetMountingXOffset() const { return m_mountingXOffset; }
    units::length::inch_t GetMountingZOffset() const { return m_mountingZOffset; }

    virtual void SetCameraPosition(
        units::length::inch_t mountingXOffset,
        units::length::inch_t mountingYOffset,
        units::length::inch_t mountingZOffset,
        units::angle::degree_t pitch,
        units::angle::degree_t yaw,
        units::angle::degree_t roll);

    DragonCamera(
        std::string cameraName, /// <I> camera name/type
        PIPELINE pipeline,      /// <I> enum for pipeline
        units::length::inch_t mountingXOffset,
        units::length::inch_t mountingYOffset,
        units::length::inch_t mountingZOffset,
        units::angle::degree_t pitch, /// <I> - Pitch of limelight
        units::angle::degree_t yaw,   /// <I> - Yaw of limelight
        units::angle::degree_t roll   /// <I> - Roll of limelight
    );
    DragonCamera() = delete;

protected:
    units::length::inch_t m_mountingXOffset;
    units::length::inch_t m_mountingYOffset;
    units::length::inch_t m_mountingZOffset;
    units::angle::degree_t m_yaw;
    units::angle::degree_t m_pitch;
    units::angle::degree_t m_roll;
};

/*
TODO:
      one large comment block in FaceAprilTage.cpp

      one large comment block in VisionDrive.cppp

      one comment block, one comment line in FaceGamePiece.cpp

      four commented lines in HolonomicDrive.cpp

      one commented line in VisionDrivePrimitive.cpp
TODO:
*/