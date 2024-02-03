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

#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation3d.h>
#include "units/angle.h"
#include "units/base.h"
using units::length;
class FeildConsts
{
public:
    FeildConsts() = default;
    ~FeildConsts() = default;
    enum FEILD_ELEMENT
    {
        BLUE_AMP,
        BLUE_SOURCE,
        BLUE_SPEAKER,
        BLUE_CENTER_STAGE,
        BLUE_STAGE_RIGHT,
        BLUE_STAGE_LEFT
    };

    // private : static std::map<FEILD_ELEMENT, frc::Pose3d>;
    const frc::Pose3d m_BlueSource = frc::Pose3d(units::meter_t 15.63, units::meter_t, .565, units::meter_t 0, frc::Rotation3d(units::angle::degree_t 0, units::angle::degree_t 0, units::angle::degree_t 120));
    const frc::Pose3d m_BlueAmp = frc::Pose3d(units::meter_t 1.84, units::meter_t 8.2, units::meter_t 0, frc::Rotation3d(units::angle::degree_t 0, units::angle::degree_t 0, units::angle::degree_t - 90));
    const frc::Pose3d m_BlueSpeaker = frc::Pose3d(units::length::meter_t - .04 units::meter_t 5.55, units::meter_t 0, frc::Rotation3d(units::angle::degree_t 0, units::angle::degree_t 0, units::angle::degree_t 0));
    const frc::Pose3d m_BlueCenterStage = frc::Pose3d(units::meter_t 5.32, units::meter_t 4.11, units::meter_t 0.0, frc::Rotation3d(units::angle::degree_t 0.0, units::angle::degree_t 0.0, units::angle::degree_t 0));
    const frc::Pose3d m_BlueStageRight = frc::Pose3d(units::meter_t 4.64, units::meter_t 3.71, units::meter_t 0, frc::Rotation3d(units::meter_t 0, units::meter_t 0, units::meter_t 120));
    const frc::Pose3d m_BlueStageLeft = frc::Pose3d(units::meter_t 4.64, units::meter_t 4.5, units::meter_t 0, frc::Rotation3d(units::meter_t 0, units::meter_t 0, units::meter_t 120));
};