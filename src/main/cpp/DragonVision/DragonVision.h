
//====================================================================================================================================================
// Copyright 2022 Lake Orion Robotics FIRST Team 302
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
#include <map>
#include <string>

// FRC Includes
#include "frc/apriltag/AprilTagFieldLayout.h"
#include "frc/apriltag/AprilTagFields.h"

// PhotonVision Includes
#include "photon/PhotonPoseEstimator.h"

#include "DragonVision/DragonVisionStructs.h"
#include "DragonVision/DragonCamera.h"

class DragonCamera;
class DragonVision
{
public:
    static DragonVision *GetDragonVision();

    static frc::AprilTagFieldLayout GetAprilTagLayout();

    enum CAMERA_POSITION
    {
        LAUNCHER,
        PLACER,
        PLACER_INTAKE,
        LAUNCHER_INTAKE
    };

    enum VISION_ELEMENT
    {
        LAUNCHER_NOTE,
        PLACER_NOTE,
        NOTE,
        SPEAKER,
        AMP,
        STAGE,
        SOURCE,
        NEAREST_APRILTAG
    };

    /// @brief sets the pipeline of the camera at the chosen position
    /// @param mode the pipeline to set the camera to
    /// @param position the physical position of the camera
    /// @return if successful or not (not currently implemented)
    bool SetPipeline(DragonCamera::PIPELINE mode, CAMERA_POSITION position);

    /// @brief gets the pipeline of the camera at the chosen position
    /// @param position the physical position of the camera
    /// @return DragonCamera::PIPELINE - the currently selected pipeline
    DragonCamera::PIPELINE GetPipeline(CAMERA_POSITION position);

    /// @brief gets the field position of the robot (right blue driverstation origin)
    /// @return std::optional<VisionPose> - the estimated position, timestamp of estimation, and confidence as array of std devs
    std::optional<VisionPose> GetRobotPosition();

    /// @brief gets the distances and angles to the specified field element based on AprilTag readings or detections
    /// @param element the specified game element to get data to
    /// @return std::optional<VisionData> - a transform containg x, y, z distances and yaw, pitch, roll to target, and AprilTag Id
    std::optional<VisionData> GetVisionData(VISION_ELEMENT element);

    /// @brief detects and returns transformation to the closest AprilTag using a specified camera
    /// @param position the physical position of the camera
    /// @return /// @return std::optional<VisionData> - a transform containg x, y, z distances and yaw, pitch, roll to target, and AprilTag Id
    std::optional<VisionData> GetDataToNearestAprilTag(CAMERA_POSITION position);

    /// @brief adds a camera at the specified position to DragonVision
    /// @param camera pointer to the camera object that should be added
    /// @param position the physical position of the camera
    void AddCamera(DragonCamera *camera, CAMERA_POSITION position);

    static frc::AprilTagFieldLayout m_aprilTagLayout;

private:
    DragonVision();
    ~DragonVision() = default;

    std::optional<VisionData> GetVisionDataFromNote(VISION_ELEMENT element);
    std::optional<VisionData> GetVisionDataFromElement(VISION_ELEMENT element);
    std::optional<VisionData> GetVisionDataToNearestTag();
    std::optional<VisionData> GetVisionDataToNearestStageTag();

    static DragonVision *m_dragonVision;

    DragonCamera *m_dragonCamera;

    std::map<CAMERA_POSITION, DragonCamera *> m_dragonCameraMap;

    std::vector<photon::PhotonPoseEstimator *> m_poseEstimators;
};
