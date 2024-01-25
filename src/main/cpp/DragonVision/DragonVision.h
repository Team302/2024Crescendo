
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

#include "State.h"
#include "DragonVision/DragonCamera.h"

using std::map;

class DragonLimelight;
class DragonVision
{
public:
    static DragonVision *GetDragonVision();

    enum CAMERA_POSITION
    {
        FRONT,
        BACK,
        BACK_INTAKE
    };

    enum VISION_ELEMENT
    {
        NOTE,
        RED_SPEAKER,
        BLUE_SPEAKER,
        RED_SUBWOOFER,
        BLUE_SUBWOOFER,
        RED_AMP,
        BLUE_AMP,
        RED_CENTER_STAGE,
        RED_LEFT_STAGE,
        RED_RIGHT_STAGE,
        BLUE_CENTER_STAGE,
        BLUE_LEFT_STAGE,
        BLUE_RIGHT_STAGE
    };

    // bool setPipeline(DragonLimelight::PIPELINE_MODE mode, LIMELIGHT_POSITION position);
    // bool setPipeline(DragonLimelight::PIPELINE_MODE mode);
    // DragonLimelight::PIPELINE_MODE getPipeline(LIMELIGHT_POSITION position);
    // std::shared_ptr<DragonVisionTarget> getTargetInfo(LIMELIGHT_POSITION position) const;
    // std::shared_ptr<DragonVisionTarget> getTargetInfo() const;

    std::optional<VisionPose> GetRobotPosition() const;
    std::optional<VisionData> GetVisionData(VISION_ELEMENT element) const;

    void AddCamera(DragonCamera *camera, CAMERA_POSITION position);

private:
    DragonVision();
    ~DragonVision() = default;

    // may not be needed, if so can be changed to inline and return from map
    //  DragonLimelight *getLimelight(LIMELIGHT_POSITION position) const;

    static DragonVision *m_dragonVision;

    std::map<CAMERA_POSITION, DragonCamera *> m_DragonCameraMap;
};
