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

// C++ Includes
#include <vector>

// FRC Includes
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Rotation2d.h>

// Team302 Includes
#include "chassis/driveStates/RobotDrive.h"
#include "chassis/SwerveChassis.h"
#include "DragonVision/DragonVision.h"
#include "utils/FMSData.h"

class DriveToRightStage
{
public:
    DriveToRightStage();
    ~DriveToRightStage() = default;

    static DriveToRightStage *GetInstance();
    pathplanner::PathPlannerTrajectory CreateDriveToRightStage();

private:
    SwerveChassis *m_chassis;
    static DriveToRightStage *m_instance;

    const units::meters_per_second_t m_maxVel = 3.0_mps;
    const units::meters_per_second_squared_t m_maxAccel = 3.0_mps_sq;
    const units::radians_per_second_t m_maxAngularVel = 360_deg_per_s;
    const units::radians_per_second_squared_t m_maxAngularAccel = 720_deg_per_s_sq;
};