
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

#include <string>

#include "hw/DragonServo.h"
#include "configs/RobotElementNames.h"
#include "utils/logging/Logger.h"

#include <frc/Servo.h>

using namespace frc;
using namespace std;

DragonServo::DragonServo(
    RobotElementNames::SERVO_USAGE deviceUsage, // <I> - Usage of the servo
    int deviceID,                        // <I> - PWM ID
    units::angle::degree_t minAngle,     // <I> - Minimun desired angle
    units::angle::degree_t maxAngle      // <I> - Maximum desired angle

    ) : m_usage(deviceUsage),
        m_servo(new frc::Servo(deviceID)),
        m_minAngle(minAngle),
        m_maxAngle(maxAngle)
{
    m_servo->SetAngle(0.0);
}

void DragonServo::Set(double value)
{
    if (m_servo != nullptr)
    {
        m_servo->Set(value);
    }
}
void DragonServo::SetOffline()
{
    if (m_servo != nullptr)
    {
        m_servo->SetOffline();
    }
}
double DragonServo::Get() const
{
    double value = 0.0;
    if (m_servo != nullptr)
    {
        value = m_servo->Get();
    }
    return value;
}
void DragonServo::SetAngle(units::angle::degree_t angle)
{
    if (m_servo != nullptr)
    {
        m_servo->SetAngle(angle.to<double>());
    }
}
units::angle::degree_t DragonServo::GetAngle() const
{
    units::angle::degree_t angle = units::angle::degree_t(0.0);
    if (m_servo != nullptr)
    {
        angle = units::angle::degree_t(m_servo->GetAngle());
    }
    return angle;
}

void DragonServo::MoveToMaxAngle()
{
    SetAngle(m_maxAngle);
}

void DragonServo::MoveToMinAngle()
{
    SetAngle(m_minAngle);
}

RobotElementNames::SERVO_USAGE DragonServo::GetUsage() const
{
    return m_usage;
}