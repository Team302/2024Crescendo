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
#include "DragonVision/DragonAprilTagInfo.h"
#include <DragonVision/DragonCamera.h>

// Third Party Includes

// DragonLimelight needs to be a child of DragonCamera
class DragonLimelight : public DragonCamera
{
public:
    // set the values according to the pipeline indices in the limelights
    // this assumes that each of the limelights has the same pipelines and they are in the same order
    enum PIPELINE_MODE
    {
        UNKNOWN = -1,
        OFF = 0,
        APRIL_TAG = 1,
        // These can be removed, add NOTE_COLOR and NOTE_ML
        CONE = 3,
        CUBE = 4,
        CONE_SUBSTATION = 5,
        CUBE_SUBSTATION = 6,
        MAX_PIPELINE_MODE
    };

    enum LED_MODE
    {
        LED_DEFAULT,
        LED_OFF,
        LED_BLINK,
        LED_ON
    };

    enum CAM_MODE
    {
        CAM_VISION,
        CAM_DRIVER
    };

    enum STREAM_MODE
    {
        STREAM_DEFAULT,         // side by side if two cams
        STREAM_MAIN_AND_SECOND, // Second Cam bottom right of Main Cam
        STREAM_SECOND_AND_MAIN  // Main Cam bottom right of Second Cam
    };

    enum SNAPSHOT_MODE
    {
        SNAP_OFF,
        SNAP_ON
    };

    ///-----------------------------------------------------------------------------------
    /// Method:         DragonLimelight (constructor)
    /// Description:    Create the object
    ///-----------------------------------------------------------------------------------
    DragonLimelight() = delete;
    DragonLimelight(
        std::string tableName,                          /// <I> - network table name
        units::length::inch_t mountingHeight,           /// <I> - mounting height of the limelight
        units::length::inch_t mountingHorizontalOffset, /// <I> - mounting horizontal offset from the middle of the robot
        units::length::inch_t forwardOffset,            /// <I> mounting offset forward/back
        units::angle::degree_t pitch,                   /// <I> - Pitch of limelight
        units::angle::degree_t yaw,                     /// <I> - Yaw of limelight
        units::angle::degree_t roll,                    /// <I> - Roll of limelight
        units::length::inch_t targetHeight,             /// <I> - height the target
        units::length::inch_t targetHeight2,            /// <I> - height of second target
        LED_MODE ledMode,
        CAM_MODE camMode,
        STREAM_MODE streamMode,
        SNAPSHOT_MODE snapMode);

    ///-----------------------------------------------------------------------------------
    /// Method:         ~DragonLimelight (destructor)
    /// Description:    Delete the object
    ///-----------------------------------------------------------------------------------
    ~DragonLimelight() = default;

    /// These methods need to override the methods of the same name from DragonCamera

    bool HasTarget() const;

    units::angle::degree_t GetTargetHorizontalOffset() const;
    units::angle::degree_t GetTargetHorizontalOffsetRobotFrame(units::length::inch_t *targetDistOffset_RF, units::length::inch_t *targetDistfromRobot_RF) const;
    units::angle::degree_t GetTargetVerticalOffset() const;
    units::angle::degree_t GetTargetVerticalOffsetRobotFrame(units::length::inch_t *targetDistOffset_RF, units::length::inch_t *targetDistfromRobot_RF) const;
    double GetTargetArea() const;
    units::angle::degree_t GetTargetSkew() const;
    units::time::microsecond_t GetPipelineLatency() const;
    std::vector<double> Get3DSolve() const;
    DragonCamera::PIPELINE GetPipeline() const;
    int GetAprilTagID() const;

    frc::Pose3d GetFieldPosition() const;
    frc::Pose3d GetFieldPosition(frc::DriverStation::Alliance alliance) const;

    frc::Pose3d GetRedFieldPosition() const;
    frc::Pose3d GetBlueFieldPosition() const;
    frc::Pose3d GetOriginFieldPosition() const;

    units::length::inch_t EstimateTargetXDistance() const;
    units::length::inch_t EstimateTargetYDistance() const;
    units::length::inch_t EstimateTargetZDistance() const;

    units::length::inch_t EstimateTargetXDistance_RelToRobotCoords() const;
    units::length::inch_t EstimateTargetYDistance_RelToRobotCoords() const;
    units::length::inch_t EstimateTargetZDistance_RelToRobotCoords() const;

    // Setters
    void SetTargetHeight(units::length::inch_t targetHeight); // don't need targetheight
    void SetLEDMode(DragonLimelight::LED_MODE mode);
    void SetCamMode(DragonLimelight::CAM_MODE mode);
    bool SetPipeline(DragonCamera::PIPELINE pipeline);
    void SetStreamMode(DragonLimelight::STREAM_MODE mode);
    void ToggleSnapshot(DragonLimelight::SNAPSHOT_MODE toggle);
    void SetCrosshairPos(double crosshairPosX, double crosshairPosY);
    void SetSecondaryCrosshairPos(double crosshairPosX, double crosshairPosY);

    void PrintValues(); // Prints out all values to ensure everything is working and connected

protected:
    units::angle::degree_t GetTx() const;
    units::angle::degree_t GetTy() const;

    std::shared_ptr<nt::NetworkTable> m_networktable;

private:
    DragonAprilTagInfo m_aprilTagInfo;
};
