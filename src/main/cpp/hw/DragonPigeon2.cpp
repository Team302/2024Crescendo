
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

#include "hw/DragonPigeon2.h"
#include <memory>

using ctre::phoenix6::configs::Pigeon2Configuration;
using ctre::phoenix6::hardware::Pigeon2;
using std::string;

DragonPigeon2::DragonPigeon2(int canID,
                             string canBusName,
                             RobotElementNames::PIGEON_USAGE usage,
                             units::angle::degree_t initialYaw,
                             units::angle::degree_t initialPitch,
                             units::angle::degree_t initialRoll) : m_pigeon(Pigeon2(canID, canBusName)),
                                                                   m_usage(usage)
{
    m_pigeon.Reset();
    Pigeon2Configuration configs{};
    m_pigeon.GetConfigurator().Refresh(configs);
    configs.MountPose.MountPoseYaw = initialYaw.to<double>();
    configs.MountPose.MountPosePitch = initialPitch.to<double>();
    configs.MountPose.MountPoseRoll = initialRoll.to<double>();
    m_pigeon.GetConfigurator().Apply(configs);
}

units::angle::degree_t DragonPigeon2::GetPitch()
{
    auto sig = m_pigeon.GetPitch();
    return sig.GetValue();
}

units::angle::degree_t DragonPigeon2::GetRoll()
{
    auto sig = m_pigeon.GetRoll();
    return sig.GetValue();
}

units::angle::degree_t DragonPigeon2::GetYaw()
{
    auto sig = m_pigeon.GetYaw();
    return sig.GetValue();
}

void DragonPigeon2::ReZeroPigeon(units::angle::degree_t angle, int timeoutMs)
{
    m_pigeon.SetYaw(angle);
}
