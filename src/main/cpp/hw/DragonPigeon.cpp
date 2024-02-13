
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

#include "wpi/deprecated.h"
WPI_IGNORE_DEPRECATED
#include "ctre/phoenix/sensors/PigeonIMU.h"
WPI_UNIGNORE_DEPRECATED

#include "hw/DragonPigeon.h"
#include <memory>

using namespace std;
using namespace ctre::phoenix::sensors;

DragonPigeon::DragonPigeon(int canID,
                           string canBusName,
                           RobotElementNames::PIGEON_USAGE usage,
                           double rotation) : m_pigeon(WPI_PigeonIMU(canID)),
                                              m_usage(usage),
                                              m_initialYaw(rotation),
                                              m_initialPitch(0.0),
                                              m_initialRoll(0.0)
{
    m_pigeon.ConfigFactoryDefault();
    m_pigeon.SetYaw(rotation, 0);
    m_pigeon.SetFusedHeading(rotation, 0);

    m_pigeon.SetStatusFramePeriod(PigeonIMU_StatusFrame::PigeonIMU_BiasedStatus_4_Mag, 120, 0);
    m_pigeon.SetStatusFramePeriod(PigeonIMU_StatusFrame::PigeonIMU_CondStatus_11_GyroAccum, 120, 0);
    m_pigeon.SetStatusFramePeriod(PigeonIMU_StatusFrame::PigeonIMU_BiasedStatus_6_Accel, 120, 0); // using fused heading not yaw
}

units::angle::degree_t DragonPigeon::GetPitch()
{
    return units::angle::degree_t(-(GetRawPitch() - m_initialPitch)); // TODO: add inversions into code
}

units::angle::degree_t DragonPigeon::GetRoll()
{
    return units::degree_t(GetRawRoll() - m_initialRoll);
}

units::angle::degree_t DragonPigeon::GetYaw()
{
    return units::angle::degree_t(GetRawYaw()); // reset should have taken care of this
}

void DragonPigeon::ReZeroPigeon(units::angle::degree_t angle, int timeoutMs)
{
    m_pigeon.SetYaw(angle.to<double>(), timeoutMs);
}

double DragonPigeon::GetRawPitch()
{
    auto pitch = -m_pigeon.GetPitch();
    return remainder(pitch, 360.0);
}

double DragonPigeon::GetRawRoll()
{
    auto roll = -m_pigeon.GetRoll();
    return remainder(roll, 360.0);
}

double DragonPigeon::GetRawYaw()
{
    double yaw = m_pigeon.GetYaw();

    yaw = remainder(yaw, 360.0);

    // normalize it to be between -180 and + 180
    if (yaw > 180)
    {
        yaw -= 360.0;
    }
    else if (yaw < -180)
    {
        yaw += 360.0;
    }
    return yaw;
}
