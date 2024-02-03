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
#include "DragonVision/DragonVisionStructs.h"

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

    ///@brief Gets the yaw error to the target relative to the camera
    ///@return units::angle::degree_t
    virtual units::angle::degree_t GetTargetYaw() const = 0;
    /// @brief Gets the yaw error to target relative to the robot frame
    /// @return units::angle::degree_t
    virtual units::angle::degree_t GetTargetYawRobotFrame() const = 0;
    /// @brief Gets the pitch error relative to the robot frame
    /// @return units::angle::degree_t
    virtual units::angle::degree_t GetTargetPitchRobotFrame() const = 0;
    /// @brief Gets the pitch of the current target
    /// @return units::angle::degree_t
    virtual units::angle::degree_t GetTargetPitch() const = 0;
    /// @brief returns the curent latancy of the vision pipeline
    /// @return units::time::millisecond_t
    virtual units::time::millisecond_t GetPipelineLatency() const = 0;
    /// @brief rturns the skew of the target
    /// @return units::angle::degree_t
    virtual units::angle::degree_t GetTargetSkew() const = 0;
    /// @brief returns the area of the "detection box"
    /// @return double
    virtual double GetTargetArea() const = 0;
    /// @brief returns the current apriltag id
    /// @return int
    virtual int GetAprilTagID() const = 0;
    /// @brief returns the position of the robot
    /// @return std::optional<VisionPose>
    virtual std::optional<VisionPose> GetFieldPosition() const = 0;

    /// @brief gets the robot position relative to the feild depending on wich alliance is specified
    /// @param frc::DriverStation::Alliance
    /// @return std::optional<visionData
    virtual std::optional<VisionPose> GetFieldPosition(frc::DriverStation::Alliance alliance) const = 0;

    //  Estimating distance
    virtual units::length::inch_t EstimateTargetXDistance() const = 0;
    virtual units::length::inch_t EstimateTargetYDistance() const = 0;
    virtual units::length::inch_t EstimateTargetZDistance() const = 0;

    virtual units::length::inch_t EstimateTargetXDistance_RelToRobotCoords() const = 0;
    virtual units::length::inch_t EstimateTargetYDistance_RelToRobotCoords() const = 0;
    virtual units::length::inch_t EstimateTargetZDistance_RelToRobotCoords() const = 0;

    virtual std::optional<VisionData> GetDataToNearestAprilTag() const = 0;

    // Getters
    PIPELINE GetPipeline() const { return m_pipeline; }
    units::angle::degree_t GetCameraPitch() const { return m_robotCenterToCam.Rotation().Y(); }
    units::angle::degree_t GetCameraYaw() const { return m_robotCenterToCam.Rotation().Z(); }
    units::angle::degree_t GetCameraRoll() const { return m_robotCenterToCam.Rotation().X(); } // rotates around x-axis
    units::length::inch_t GetMountingXOffset() const { return m_robotCenterToCam.X(); }
    units::length::inch_t GetMountingYOffset() const { return m_robotCenterToCam.Y(); }
    units::length::inch_t GetMountingZOffset() const { return m_robotCenterToCam.Z(); }

    frc::Transform3d GetTransformFromRobotCenter() const { return m_robotCenterToCam; }

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