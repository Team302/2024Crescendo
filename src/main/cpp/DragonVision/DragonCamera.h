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
        UNKNOWN = -1,
        OFF,
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

    virtual bool HasTarget() = 0;
    virtual bool HealthCheck() = 0;

    // Getters

    ///@brief Gets the yaw error to the target relative to the camera
    ///@return units::angle::degree_t
    virtual std::optional<units::angle::degree_t> GetTargetYaw() = 0;
    /// @brief Gets the yaw error to target relative to the robot frame
    /// @return units::angle::degree_t
    virtual std::optional<units::angle::degree_t> GetTargetYawRobotFrame() = 0;
    /// @brief Gets the pitch error relative to the robot frame
    /// @return units::angle::degree_t
    virtual std::optional<units::angle::degree_t> GetTargetPitchRobotFrame() = 0;
    /// @brief Gets the pitch of the current target
    /// @return units::angle::degree_t
    virtual std::optional<units::angle::degree_t> GetTargetPitch() = 0;
    /// @brief returns the curent latancy of the vision pipeline
    /// @return units::time::millisecond_t
    virtual std::optional<units::time::millisecond_t> GetPipelineLatency() = 0;
    /// @brief rturns the skew of the target
    /// @return units::angle::degree_t
    virtual std::optional<units::angle::degree_t> GetTargetSkew() = 0;
    /// @brief returns the area of the "detection box"
    /// @return double
    virtual std::optional<double> GetTargetArea() = 0;
    /// @brief returns the current apriltag id
    /// @return int
    virtual std::optional<int> GetAprilTagID() = 0;
    /// @brief returns the position of the robot
    /// @return std::optional<VisionPose>
    virtual std::optional<VisionPose> GetFieldPosition() = 0;

    /// @brief gets the robot position relative to the feild depending on wich alliance is specified
    /// @param frc::DriverStation::Alliance
    /// @return std::optional<visionData
    virtual std::optional<VisionPose> GetFieldPosition(frc::DriverStation::Alliance alliance) = 0;

    //  Estimating distance
    virtual std::optional<units::length::inch_t> EstimateTargetXDistance() = 0;
    virtual std::optional<units::length::inch_t> EstimateTargetYDistance() = 0;
    virtual std::optional<units::length::inch_t> EstimateTargetZDistance() = 0;

    virtual std::optional<units::length::inch_t> EstimateTargetXDistance_RelToRobotCoords() = 0;
    virtual std::optional<units::length::inch_t> EstimateTargetYDistance_RelToRobotCoords() = 0;
    virtual std::optional<units::length::inch_t> EstimateTargetZDistance_RelToRobotCoords() = 0;

    virtual std::optional<VisionData> GetDataToNearestAprilTag() = 0;
    virtual std::optional<VisionData> GetDataToSpecifiedTag(int id) = 0;

    // Getters
    PIPELINE GetPipeline() const { return m_pipeline; }
    units::angle::degree_t GetCameraPitch() const { return m_cameraPose.Rotation().Y(); }
    units::angle::degree_t GetCameraYaw() const { return m_cameraPose.Rotation().Z(); }
    units::angle::degree_t GetCameraRoll() const { return m_cameraPose.Rotation().X(); } // rotates around x-axis
    units::length::inch_t GetMountingXOffset() const { return m_cameraPose.X(); }
    units::length::inch_t GetMountingYOffset() const { return m_cameraPose.Y(); }
    units::length::inch_t GetMountingZOffset() const { return m_cameraPose.Z(); }
    std::string GetCameraName() const { return m_cameraName; }

    frc::Transform3d GetTransformFromRobotCenter() const
    {
        return m_robotCenterToCam;
    }

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
        units::angle::degree_t roll);
    virtual bool UpdatePipeline() = 0; // children will handle updating the co-processor to current m_pipeline value

protected:
    frc::Pose3d m_cameraPose;
    frc::Transform3d m_robotCenterToCam;
    PIPELINE m_pipeline;
    std::string m_cameraName;

    const units::length::inch_t m_noteVerticalOffset = units::length::inch_t(0.0); // This represents the note being at the same level as center of robot
};