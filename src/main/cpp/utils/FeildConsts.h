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
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation3d.h>
#include "units/angle.h"
#include "units/base.h"
class FieldConsts
{
public:
    FieldConsts();
    ~FieldConsts() = default;
    enum FIELD_ELEMENT
    {
        BLUE_SOURCE,
        BLUE_AMP,
        BLUE_SPEAKER,
        BLUE_CENTER_STAGE,
        BLUE_STAGE_RIGHT,
        BLUE_STAGE_LEFT,
        RED_SOURCE,
        RED_AMP,
        RED_SPEAKER,
        RED_CENTER_STAGE,
        RED_STAGE_RIGHT,
        RED_STAGE_LEFT
    };
    frc::Pose3d GetFeildElement(FIELD_ELEMENT element);

private:
    static std::map<FIELD_ELEMENT, frc::Pose3d> FieldMap;
    // blue
    const frc::Pose3d m_BlueSource = frc::Pose3d(units::length::meter_t(15.63), units::length::meter_t(0.565), units::length::meter_t(0.0), frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(120)));
    const frc::Pose3d m_BlueAmp = frc::Pose3d(units::length::meter_t(1.84), units::length::meter_t(8.2), units::length::meter_t(0.0), frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(-90.0)));
    const frc::Pose3d m_BlueSpeaker = frc::Pose3d(units::length::meter_t(-.04), units::length::meter_t(5.55), units::length::meter_t(0.0), frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(0.0)));
    const frc::Pose3d m_BlueCenterStage = frc::Pose3d(units::length::meter_t(5.32), units::length::meter_t(4.11), units::length::meter_t(0.0), frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(0.0)));
    const frc::Pose3d m_BlueStageRight = frc::Pose3d(units::length::meter_t(4.64), units::length::meter_t(3.71), units::length::meter_t(0.0), frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(120.0)));
    const frc::Pose3d m_BlueStageLeft = frc::Pose3d(units::length::meter_t(4.64), units::length::meter_t(4.5), units::length::meter_t(0.0), frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(120.0)));
    // red
    const frc::Pose3d m_RedSource = frc::Pose3d(units::length::meter_t(.91), units::length::meter_t(.565), units::length::meter_t(0, 0), frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(60.0)));
    const frc::Pose3d m_RedAmp = frc::Pose3d(units::length::meter_t(14.7), units::length::meter_t(8.2), units::length::meter_t(0, 0), frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(-90.0)));
    const frc::Pose3d m_RedSpeaker = frc::Pose3d(units::length::meter_t(16.58), units::length::meter_t(5.55), units::length::meter_t(0.0), frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(180.0)));
    const frc::Pose3d m_RedCenterStage = frc::Pose3d(units::length::meter_t(11.22), units::length::meter_t(4.11), units::length::meter_t(0.0), frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(180.0)));
    const frc::Pose3d m_RedStageRight = frc::Pose3d(units::length::meter_t(11.9), units::length::meter_t(3.71), units::length::meter_t(0.0), frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(0.0)));
    const frc::Pose3d m_RedStageLeft = frc::Pose3d(units::length::meter_t(11.9), units::length::meter_t(4.5), units::length::meter_t(0.0), frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(60.0)));
    std::vector<frc::Pose3d> vector{
        m_BlueSource,
        m_BlueAmp,
        m_BlueSpeaker,
        m_BlueCenterStage,
        m_BlueStageRight,
        m_BlueStageLeft,
        m_RedSource,
        m_RedAmp,
        m_RedSpeaker,
        m_RedCenterStage,
        m_RedStageRight,
        m_RedStageLeft};
};