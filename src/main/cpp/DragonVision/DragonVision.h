
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

#include "DragonVision/DragonVisonStructs.h"
#include "DragonVision/DragonCamera.h"

class DragonCamera;
class DragonVision
{
public:
    static DragonVision *GetDragonVision();

    static frc::AprilTagFieldLayout GetAprilTagLayout();

    enum CAMERA_POSITION
    {
        FRONT,
        BACK,
        BACK_INTAKE,
        FRONT_INTAKE
    };

    enum VISION_ELEMENT
    {
        LAUNCHER_NOTE,
        PLACER_NOTE,
        NOTE,
        SPEAKER,
        SUBWOOFER,
        AMP,
        STAGE,
        NEAREST_APRILTAG
    };

    // bool setPipeline(DragonLimelight::PIPELINE_MODE mode, LIMELIGHT_POSITION position);
    bool SetPipeline(DragonCamera::PIPELINE mode, CAMERA_POSITION position);
    // bool setPipeline(DragonLimelight::PIPELINE_MODE mode);
    DragonCamera::PIPELINE GetPipeline(CAMERA_POSITION position);
    // std::shared_ptr<DragonVisionTarget> getTargetInfo(LIMELIGHT_POSITION position) const;
    // std::shared_ptr<DragonVisionTarget> getTargetInfo() const;

    std::optional<VisionPose> GetRobotPosition();
    std::optional<VisionData> GetVisionData(VISION_ELEMENT element);
    std::optional<VisionData> GetDataToNearestAprilTag(CAMERA_POSITION position);

    void AddCamera(DragonCamera *camera, CAMERA_POSITION position);

    static frc::AprilTagFieldLayout m_aprilTagLayout;

private:
    DragonVision();
    ~DragonVision() = default;

    std::optional<VisionData> GetVisionDataFromNote(VISION_ELEMENT element);
    std::optional<VisionData> GetVisionDataFromElement(VISION_ELEMENT element);

    // may not be needed, if so can be changed to inline and return from map
    //  DragonLimelight *getLimelight(LIMELIGHT_POSITION position) const;

    static DragonVision *m_dragonVision;

    DragonCamera *m_dragonCamera;

    std::map<CAMERA_POSITION, DragonCamera *> m_DragonCameraMap;
};
