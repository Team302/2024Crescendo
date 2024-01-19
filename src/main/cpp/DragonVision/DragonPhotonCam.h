//====================================================================================================================================================
/// Copyright 2024 Lake Orion Robotics FIRST Team 302
///
/// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
/// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
/// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
/// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
/// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================

#include <photon/PhotonCamera.h>

#include "DragonVision/DragonCamera.h"

class DragonPhotonCam : public DragonCamera
{
public:
    DragonPhotonCam(std::string name,                               /// <I> - camera name
                    units::length::inch_t mountingHeight,           /// <I> - mounting height of the limelight
                    units::length::inch_t mountingHorizontalOffset, /// <I> - mounting horizontal offset from the middle of the robot
                    units::length::inch_t forwardOffset,            /// <I> mounting offset forward/back
                    units::angle::degree_t pitch,                   /// <I> - Pitch of limelight
                    units::angle::degree_t yaw,                     /// <I> - Yaw of limelight
                    units::angle::degree_t roll);

private:
    photon::PhotonCamera m_camera;
    std::string m_name;
    units::length::inch_t m_mountHeight;
    units::length::inch_t m_mountingHorizontalOffset;
    units::length::inch_t m_mountingForwardOffset;
    units::angle::degree_t m_yaw;
    units::angle::degree_t m_pitch;
    units::angle::degree_t m_roll;
};