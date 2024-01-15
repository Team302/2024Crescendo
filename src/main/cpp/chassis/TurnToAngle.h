
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
#include "State.h"
#include "units/angle.h"
#include "units/angular_velocity.h"
#include <units/angular_acceleration.h>

#include <frc/controller/PIDController.h>
// #include <frc/controller/ProfiledPIDController.h>
// #include <frc/trajectory/TrapezoidProfile.h>

#include "chassis/swerve/SwerveChassis.h"

///	 @brief     this state will allow the robot to rotate to a specified angle
class TurnToAngle : public State
{
public:
    TurnToAngle() = delete;
    TurnToAngle(
        units::angle::degree_t targetAngle);
    ~TurnToAngle() = default;

    void Init() override;
    void Run() override;
    bool AtTarget() override;

private:
    units::angle::degree_t m_targetAngle;

    // Need to tune these
    const double kP = 0.004;
    const double kI = 0.0001;
    const double kD = 0.0;
    const double kF = 0.0;
    const units::angle::degree_t m_angleTolerance = units::angle::degree_t(2.0);
    frc::PIDController m_pid{kP, kI, kD};
    SwerveChassis *m_chassis;
    bool m_atTarget;
};