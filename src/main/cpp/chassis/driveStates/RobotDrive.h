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
#include <string>

#include "frc/kinematics/SwerveModuleState.h"
#include "units/length.h"
#include "units/time.h"
#include "units/velocity.h"

// Team302 Includes
#include "chassis/SwerveChassis.h"
#include "chassis/driveStates/ISwerveDriveState.h"
#include "chassis/ChassisMovement.h"
#include "teleopcontrol/TeleopControl.h"

class RobotDrive : public ISwerveDriveState
{
public:
    RobotDrive() = delete;
    RobotDrive(SwerveChassis *chassis);
    ~RobotDrive() = default;
    std::string GetDriveStateName() const override;

    std::array<frc::SwerveModuleState, 4> UpdateSwerveModuleStates(ChassisMovement &chassisMovement) override;

    void Init(ChassisMovement &chassisMovement) override;
    SwerveChassis *GetChassis() const { return m_chassis; }

protected:
    SwerveChassis *m_chassis;

    frc::SwerveModuleState m_flState;
    frc::SwerveModuleState m_frState;
    frc::SwerveModuleState m_blState;
    frc::SwerveModuleState m_brState;
    frc::Translation2d m_centerOfRotation = frc::Translation2d(units::meter_t(0.0), units::meter_t(0.0));

    units::length::inch_t m_wheelbase;
    units::length::inch_t m_wheeltrack;
    units::velocity::feet_per_second_t m_maxspeed;
};