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

// C++ Includes
#include <string>
#include <vector>

// FRC includes
#include "networktables/NetworkTable.h"
#include "units/angle.h"
#include "units/length.h"
#include "units/time.h"
#include "frc/geometry/Pose2d.h"

// Team 302 includes
#include "DragonVision/DragonCamera.h"

// Third Party Includes

// DragonLimelight needs to be a child of DragonCamera
class DragonLimelight : public DragonCamera
{
public:
    enum LED_MODE
    {
        LED_UNKNOWN = -1,
        LED_DEFAULT,
        LED_OFF,
        LED_BLINK,
        LED_ON
    };

    enum CAM_MODE
    {
        CAM_UNKNOWN = -1,
        CAM_VISION,
        CAM_DRIVER
    };

    enum STREAM_MODE
    {
        STREAM_UNKNOWN = -1,
        STREAM_DEFAULT,         // side by side if two cams
        STREAM_MAIN_AND_SECOND, // Second Cam bottom right of Main Cam
        STREAM_SECOND_AND_MAIN  // Main Cam bottom right of Second Cam
    };

    enum SNAPSHOT_MODE
    {
        SNAPSHOT_MODE_UNKNOWN = -1,
        SNAP_OFF,
        SNAP_ON
    };

    ///-----------------------------------------------------------------------------------
    /// Method:         DragonLimelight (constructor)
    /// Description:    Create the object
    ///-----------------------------------------------------------------------------------
    DragonLimelight() = delete;
    DragonLimelight(
        std::string name,                      /// <I> - network table name
        PIPELINE initialPipeline,              /// <I> enum for starting pipeline
        units::length::inch_t mountingXOffset, /// <I> x offset of cam from robot center (forward relative to robot)
        units::length::inch_t mountingYOffset, /// <I> y offset of cam from robot center (left relative to robot)
        units::length::inch_t mountingZOffset, /// <I> z offset of cam from robot center (up relative to robot)
        units::angle::degree_t pitch,          /// <I> - Pitch of camera
        units::angle::degree_t yaw,            /// <I> - Yaw of camera
        units::angle::degree_t roll,           /// <I> - Roll of camera
        LED_MODE ledMode,
        CAM_MODE camMode,
        STREAM_MODE streamMode,
        SNAPSHOT_MODE snapMode);

    ///-----------------------------------------------------------------------------------
    /// Method:         ~DragonLimelight (destructor)
    /// Description:    Delete the object
    ///-----------------------------------------------------------------------------------
    ~DragonLimelight() = default;

    bool HasTarget();

    units::angle::degree_t GetTargetYaw();
    units::angle::degree_t GetTargetYawRobotFrame();
    units::angle::degree_t GetTargetPitch();
    units::angle::degree_t GetTargetPitchRobotFrame();
    double GetTargetArea();
    units::angle::degree_t GetTargetSkew();
    units::time::millisecond_t GetPipelineLatency();
    std::vector<double> Get3DSolve();
    int GetAprilTagID();

    std::optional<VisionPose> GetFieldPosition();
    std::optional<VisionPose> GetFieldPosition(frc::DriverStation::Alliance alliance);

    std::optional<VisionPose> GetRedFieldPosition();
    std::optional<VisionPose> GetBlueFieldPosition();
    std::optional<VisionPose> GetOriginFieldPosition();

    std::optional<VisionData> GetDataToNearestAprilTag();

    units::length::inch_t EstimateTargetXDistance();
    units::length::inch_t EstimateTargetYDistance();
    units::length::inch_t EstimateTargetZDistance();

    units::length::inch_t EstimateTargetXDistance_RelToRobotCoords();
    units::length::inch_t EstimateTargetYDistance_RelToRobotCoords();
    units::length::inch_t EstimateTargetZDistance_RelToRobotCoords();

    // Setters
    void SetLEDMode(DragonLimelight::LED_MODE mode);
    void SetCamMode(DragonLimelight::CAM_MODE mode);
    void SetStreamMode(DragonLimelight::STREAM_MODE mode);
    void ToggleSnapshot(DragonLimelight::SNAPSHOT_MODE toggle);
    void SetCrosshairPos(double crosshairPosX, double crosshairPosY);
    void SetSecondaryCrosshairPos(double crosshairPosX, double crosshairPosY);

    bool UpdatePipeline();

    void PrintValues(); // Prints out all values to ensure everything is working and connected

protected:
    units::angle::degree_t GetTx() const;
    units::angle::degree_t GetTy() const;

    std::shared_ptr<nt::NetworkTable> m_networktable;

private:
};
