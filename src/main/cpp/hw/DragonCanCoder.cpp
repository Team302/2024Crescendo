
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

#include "hw/DragonCanCoder.h"
#include "utils/logging/Logger.h"

using ctre::phoenix6::configs::CANcoderConfiguration;
using ctre::phoenix6::configs::CANcoderConfigurator;
using ctre::phoenix6::hardware::CANcoder;
using ctre::phoenix6::signals::AbsoluteSensorRangeValue;
using ctre::phoenix6::signals::SensorDirectionValue;
using std::string;

DragonCanCoder::DragonCanCoder(string networkTableName,
                               RobotElementNames::CANCODER_USAGE usage,
                               int canID,
                               string canBusName,
                               double offset,
                               bool reverse) : m_networkTableName(networkTableName),
                                               m_usage(usage),
                                               m_cancoder(CANcoder(canID, canBusName))
{

    CANcoderConfiguration configs{};
    m_cancoder.GetConfigurator().Refresh(configs);
    configs.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue::Unsigned_0To1;
    configs.MagnetSensor.SensorDirection = reverse ? SensorDirectionValue::Clockwise_Positive : SensorDirectionValue::CounterClockwise_Positive;
    configs.MagnetSensor.MagnetOffset = offset;

    m_cancoder.GetConfigurator().Apply(configs);
}

void DragonCanCoder::SetRange(AbsoluteSensorRangeValue range)
{
    CANcoderConfiguration configs{};
    m_cancoder.GetConfigurator().Refresh(configs);
    configs.MagnetSensor.AbsoluteSensorRange = range;
    m_cancoder.GetConfigurator().Apply(configs);
}

void DragonCanCoder::SetMagnetOffset(double offset)
{
    CANcoderConfiguration configs{};
    m_cancoder.GetConfigurator().Refresh(configs);
    configs.MagnetSensor.MagnetOffset = offset;
    m_cancoder.GetConfigurator().Apply(configs);
}

void DragonCanCoder::SetDirection(SensorDirectionValue direction)
{
    CANcoderConfiguration configs{};
    m_cancoder.GetConfigurator().Refresh(configs);
    configs.MagnetSensor.SensorDirection = direction;
    m_cancoder.GetConfigurator().Apply(configs);
}

units::angle::radian_t DragonCanCoder::GetAbsolutePosition()
{
    auto pos = m_cancoder.GetAbsolutePosition();
    units::angle::radian_t val = pos.GetValue();
    return val;
}

units::angular_velocity::revolutions_per_minute_t DragonCanCoder::GetVelocity()
{
    auto vel = m_cancoder.GetVelocity();
    units::angular_velocity::revolutions_per_minute_t val = vel.GetValue();
    return val;
}

int DragonCanCoder::GetCanId()
{
    return m_cancoder.GetDeviceID();
}
