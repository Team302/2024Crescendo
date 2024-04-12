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

#include "frc/geometry/Pose3d.h"
#include "frc/geometry/Translation3d.h"
#include "wpi/array.h"
#include "units/time.h"

enum PoseEstimationStrategy
{
    MULTI_TAG,
    SINGLE_TAG,
    MEGA_TAG,
    MEGA_TAG_2,
    NONE
};

struct VisionPose
{
    frc::Pose3d estimatedPose = frc::Pose3d{};                                     // empty pose3d if we don't give one out
    units::time::millisecond_t timeStamp = units::time::millisecond_t(-1.0);       // negative timestamp for no timestamp
    wpi::array<double, 3> visionMeasurementStdDevs = {0.1, 0.1, 0.1};              // default std devs from WPI docs
    PoseEstimationStrategy estimationStrategy = PoseEstimationStrategy::MULTI_TAG; // default estimation strategy, what should be used
};

struct VisionData
{
    frc::Transform3d transformToTarget = frc::Transform3d{}; // from robot center
    frc::Translation3d translationToTarget = frc::Translation3d{};
    frc::Rotation3d rotationToTarget = frc::Rotation3d{};
    int tagId = -1; // if we don't have april tag data, use null id
};