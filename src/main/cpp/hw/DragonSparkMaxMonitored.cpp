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

#include "hw/DragonSparkMaxMonitored.h"

DragonSparkMaxMonitored::DragonSparkMaxMonitored(int id,
                                                 RobotElementNames::MOTOR_CONTROLLER_USAGE deviceType,
                                                 CANSparkMax::MotorType motorType,
                                                 rev::SparkRelativeEncoder::Type feedbackType,
                                                 rev::SparkLimitSwitch::Type forwardType,
                                                 rev::SparkLimitSwitch::Type reverseType,
                                                 const DistanceAngleCalcStruc &calcStruc) : DragonSparkMax(id,
                                                                                                           deviceType,
                                                                                                           motorType,
                                                                                                           feedbackType,
                                                                                                           forwardType,
                                                                                                           reverseType,
                                                                                                           calcStruc)

{
}

void DragonSparkMaxMonitored::ConfigureCurrentFiltering(int filterLength)
{
    // set the filter length and initialze all values to zero
    m_currentHistoryValues.resize(filterLength, 0);
}

void DragonSparkMaxMonitored::ConfigureCurrentShutoff(double currentThreshold, int loopCountThreshold)
{
    m_currentThreshold = currentThreshold;
    m_loopCountThreshold = loopCountThreshold;
}

void DragonSparkMaxMonitored::MonitorCurrent()
{
    FilterCurrentValue();
}

double DragonSparkMaxMonitored::GetCurrent()
{
    return m_currentAverage;
}

void DragonSparkMaxMonitored::FilterCurrentValue()
{
    double latestCurrent = DragonSparkMax::GetCurrent();

    double total = m_currentAverage * m_currentHistoryValues.size();

    total -= m_currentHistoryValues.back();
    m_currentHistoryValues.pop_back();

    m_currentHistoryValues.push_front(latestCurrent);

    total += latestCurrent;

    m_currentAverage = total / m_currentHistoryValues.size();
}