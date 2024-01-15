
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
#include <DragonVision/DragonLimelight.h>
#include <DragonVision/DragonVisionTarget.h>

using std::map;

class DragonLimelight;
class DragonVision
{
public:
    static DragonVision *GetDragonVision();

    enum LIMELIGHT_POSITION
    {
        FRONT,
        BACK
    };

    bool setPipeline(DragonLimelight::PIPELINE_MODE mode, LIMELIGHT_POSITION position);
    bool setPipeline(DragonLimelight::PIPELINE_MODE mode);
    DragonLimelight::PIPELINE_MODE getPipeline(LIMELIGHT_POSITION position);
    std::shared_ptr<DragonVisionTarget> getTargetInfo(LIMELIGHT_POSITION position) const;
    std::shared_ptr<DragonVisionTarget> getTargetInfo() const;

    frc::Pose2d GetRobotPosition() const;
    frc::Pose2d GetRobotPosition(LIMELIGHT_POSITION position) const;

private:
    DragonVision();
    ~DragonVision() = default;

    DragonLimelight *getLimelight(LIMELIGHT_POSITION position) const;

    static DragonVision *m_dragonVision;

    std::map<LIMELIGHT_POSITION, DragonLimelight *> m_DragonLimelightMap;
};
