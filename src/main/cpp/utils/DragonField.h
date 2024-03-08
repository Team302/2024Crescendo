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

// FRC Include
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/FieldObject2d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/trajectory/Trajectory.h>
#include <DragonVision/DragonVisionStructs.h>

class DragonField
{
public:
    DragonField();
    ~DragonField() = default;

    void UpdateRobotPosition(frc::Pose2d robotPose);

    void AddPose(std::string name, frc::Pose2d pose);
    void AddTrajectory(std::string name, frc::Trajectory trajectory);
    void UpdateObject(std::string name, frc::Pose2d pose);
    void UpdateObjectVisionPose(std::string name, std::optional<VisionPose> visionPose);

    /// @brief get the singeleton of FMSData
    static DragonField *GetInstance();

    void ResetField();

private:
    frc::Field2d m_field;
    std::vector<frc::FieldObject2d *> m_objects;

    static DragonField *m_instance;
};