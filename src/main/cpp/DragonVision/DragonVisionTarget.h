
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

#include "units/angle.h"
#include "units/length.h"
#include "units/time.h"
#include <DragonVision/DragonLimelight.h>
class DragonVisionTarget
{
public:
    DragonVisionTarget() = default;

    DragonVisionTarget(DragonLimelight::PIPELINE_MODE targetType,
                       units::length::inch_t distanceFromTarget,
                       units::angle::degree_t horizontalAngleFromTarget,
                       units::angle::degree_t verticalAngleFromTarget,
                       units::length::inch_t xDistanceToTargetRobotFrame,
                       units::length::inch_t yDistanceToTargetRobotFrame,
                       int apriltagID,
                       units::time::millisecond_t latency);

    ~DragonVisionTarget() = default;

    units::length::inch_t getDistanceToTarget();
    units::angle::degree_t getHorizontalAngleToTarget();
    units::length::inch_t getYdistanceToTargetRobotFrame();
    units::length::inch_t getXdistanceToTargetRobotFrame();
    units::angle::degree_t getVerticalAngleToTarget();
    DragonLimelight::PIPELINE_MODE getTargetType();
    units::time::millisecond_t getLatency();
    int getApriltagID();

private:
    units::length::inch_t m_distanceFromTarget;
    units::angle::degree_t m_horizontalAngleToTarget;
    units::length::inch_t m_yDistanceToTargetRobotFrame;
    units::length::inch_t m_xDistanceToTargetRobotFrame;
    units::angle::degree_t m_verticalAngleToTarget;
    DragonLimelight::PIPELINE_MODE m_targetType;
    int m_tagID;
    units::time::millisecond_t m_latency;
};
