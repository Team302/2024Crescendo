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
#include "frc/geometry/Pose3d.h"
#include "frc/DriverStation.h"
#include "DragonVision/DragonVisonStructs.h"

class DragonCamera
{
public:
    enum PIPELINE
    {
        OFF,
        UNKNOWN,
        MACHINE_LEARNING,
        APRIL_TAG,
        COLOR_THRESHOLD
    };

    DragonCamera(
        std::string cameraName,                /// <I> camera name/type
        PIPELINE pipeline,                     /// <I> enum for pipeline
        units::length::inch_t mountingXOffset, /// <I> x offset of cam from robot center (forward relative to robot is positive)
        units::length::inch_t mountingYOffset, /// <I> y offset of cam from robot center (left relative to robot is positive)
        units::length::inch_t mountingZOffset, /// <I> z offset of cam from robot center (up relative to robot is positive)
        units::angle::degree_t pitch,          /// <I> - Pitch of camera
        units::angle::degree_t yaw,            /// <I> - Yaw of camera
        units::angle::degree_t roll            /// <I> - Roll of camera
    );
    DragonCamera() = delete;

    virtual bool HasTarget() const = 0;

    // Getters
    virtual units::angle::degree_t GetTargetYaw() const = 0;
    virtual units::angle::degree_t GetTargetYawRobotFrame() const = 0;
    virtual units::angle::degree_t GetTargetPitchRobotFrame() const = 0;
    virtual units::angle::degree_t GetTargetPitchAngle() const = 0;
    virtual units::time::millisecond_t GetPipelineLatency() const = 0;
    virtual units::angle::degree_t GetTargetSkew() const = 0;
    virtual double GetTargetArea() const = 0;
    virtual int GetAprilTagID() const = 0;

    virtual VisionPose GetFieldPosition() const = 0;
    virtual VisionPose GetFieldPosition(frc::DriverStation::Alliance alliance) const = 0;

    //  Estimating distance
    virtual units::length::inch_t GetEstimatedTargetXDistance() const = 0;
    virtual units::length::inch_t GetEstimatedTargetYDistance() const = 0;
    virtual units::length::inch_t GetEstimatedTargetZDistance() const = 0;

    virtual units::length::inch_t GetEstimatedTargetXDistance_RelToRobotCoords() const = 0;
    virtual units::length::inch_t GetEstimatedTargetYDistance_RelToRobotCoords() const = 0;
    virtual units::length::inch_t GetEstimatedTargetZDistance_RelToRobotCoords() const = 0;

    virtual VisionData GetDataToNearestApriltag() const = 0;

    // Getters
    PIPELINE GetPipeline() const { return m_pipeline; }
    units::angle::degree_t GetCameraPitch() const { return m_robotCenterToCam.Rotation().Y(); }
    units::angle::degree_t GetCameraYaw() const { return m_robotCenterToCam.Rotation().Z(); }
    units::angle::degree_t GetCameraRoll() const { return m_robotCenterToCam.Rotation().X(); } // rotates around x-axis
    units::length::inch_t GetMountingXOffset() const { return m_robotCenterToCam.X(); }
    units::length::inch_t GetMountingYOffset() const { return m_robotCenterToCam.Y(); }
    units::length::inch_t GetMountingZOffset() const { return m_robotCenterToCam.Z(); }

    // Setters
    void SetPipeline(PIPELINE pipeline)
    {
        m_pipeline = pipeline;
    }

    void SetCameraPosition(
        units::length::inch_t mountingXOffset,
        units::length::inch_t mountingYOffset,
        units::length::inch_t mountingZOffset,
        units::angle::degree_t pitch,
        units::angle::degree_t yaw,
        units::angle::degree_t roll);  /// TODO: implement
    virtual bool UpdatePipeline() = 0; // children will handle updating the co-processor to current m_pipeline value

protected:
    frc::Pose3d m_cameraPose;
    frc::Transform3d m_robotCenterToCam;
    PIPELINE m_pipeline;

    const units::length::inch_t m_noteVerticalOffset = units::length::inch_t(0.0); // This represents the note being at the same level as center of robot
};

/*
TODO:
      one large comment block in FaceAprilTage.cpp

      one large comment block in VisionDrive.cpp

      one comment block, one comment line in FaceGamePiece.cpp

       VisionDrivePrimitive.cpp(removed commented lines, still need code)

       FaceGoalHeading.cpp (33-34 change to add actual code)
TODO:
*/
