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

#include <deque>

#include "hw/DragonSparkMax.h"

// namespaces
using namespace rev;

class DragonSparkMaxMonitored : public DragonSparkMax
{
public:
    DragonSparkMaxMonitored() = delete;
    DragonSparkMaxMonitored(int id,
                            RobotElementNames::MOTOR_CONTROLLER_USAGE deviceType,
                            rev::CANSparkMax::MotorType motorType,
                            rev::SparkRelativeEncoder::Type feedbackType,
                            rev::SparkLimitSwitch::Type forwardType,
                            rev::SparkLimitSwitch::Type reverseType,
                            const DistanceAngleCalcStruc &calcStruc);
    virtual ~DragonSparkMaxMonitored() = default;

    void ConfigureCurrentFiltering(int filterLength);
    void ConfigureCurrentShutoff(double currentThreshold, int loopCountThreshold);

    void MonitorCurrent() override;
    double GetCurrent() override;

    inline void EnableOverCurrentShutoff(bool enable) { m_overCurrentShutoffEnabled = enable; }

private:
    double m_currentThreshold;
    int m_loopCountThreshold;

    std::deque<double> m_currentHistoryValues;
    double m_currentAverage;

    bool m_overCurrentShutoffEnabled;

    void FilterCurrentValue();
};
