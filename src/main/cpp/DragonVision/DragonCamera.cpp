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

#include "DragonVision/DragonCamera.h"

#include "units/angle.h"
#include "units/length.h"
#include "units/time.h"

#include <string>
#include <vector>
#include <frc/geometry/Pose3d.h>

DragonCamera::DragonCamera(std::string cameraName, /// <I> camera name/type
                           PIPELINE pipeline,      /// <I> enum for pipeline
                           units::length::inch_t mountingXDistance,
                           units::length::inch_t mountingYDistance,
                           units::length::inch_t mountingZDistance,
                           units::angle::degree_t pitch, /// <I> - Pitch of limelight
                           units::angle::degree_t yaw,   /// <I> - Yaw of limelight
                           units::angle::degree_t roll) : m_cameraPose(mountingXDistance,
                                                                       mountingYDistance,
                                                                       mountingZDistance,
                                                                       frc::Rotation3d(roll, pitch, yaw)),
                                                          m_robotCenterToCam(frc::Pose3d{}, m_cameraPose), // transform from center of robot to camera
                                                          m_cameraName(cameraName)
{
}

void DragonCamera::SetCameraPosition(units::length::inch_t mountingXOffset,
                                     units::length::inch_t mountingYOffset,
                                     units::length::inch_t mountingZOffset,
                                     units::angle::degree_t pitch,
                                     units::angle::degree_t yaw,
                                     units::angle::degree_t roll)
{
    m_cameraPose = frc::Pose3d{mountingXOffset, mountingYOffset, mountingZOffset, frc::Rotation3d(roll, pitch, yaw)};
    m_robotCenterToCam = frc::Transform3d(frc::Pose3d{}, m_cameraPose);
}
