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

#include <frc/Timer.h>

#include "mechanisms/controllers/ControlData.h"
#include <mechanisms/controllers/DragonPID.h>

DragonPID::DragonPID(
    ControlData *controlData) : m_kP(controlData->GetP()),
                                m_kI(controlData->GetI()),
                                m_kD(controlData->GetD()),
                                m_kF(controlData->GetF()),
                                m_accumError(0.0),
                                m_prevError(0.0),
                                m_timer(new frc::Timer())
{
    m_timer->Start();
}

void DragonPID::UpdateKP(double kP)
{
    m_kP = kP;
}
void DragonPID::UpdateKI(double kI)
{
    m_kI = kI;
}
void DragonPID::UpdateKD(double kD)
{
    m_kD = kD;
}
void DragonPID::UpdateKF(double kF)
{
    m_kF = kF;
}

double DragonPID::Calculate(
    double motorOutput,
    double currentVal,
    double targetVal)
{
    auto error = targetVal - currentVal;
    m_accumError += error;
    auto deltaT = m_timer->Get().to<double>();
    m_timer->Reset();
    auto deltaErr = error - m_prevError;
    m_prevError = error;

    return motorOutput + m_kP * error + m_kI * m_accumError + m_kD * deltaErr / deltaT + m_kF;
}