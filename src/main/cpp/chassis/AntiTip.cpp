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
//=====================================================================================================================================================

// FRC Includes
#include "units/velocity.h"
#include "units/angle.h"

// Team302 Includes
#include "chassis/ChassisMovement.h"
#include "chassis/AntiTip.h"
#include "chassis/configs/ChassisConfig.h"
#include "chassis/configs/ChassisConfigMgr.h"
#include "utils/FMSData.h"

void AntiTip::CorrectForTipping(ChassisMovement &chassisMovement, units::velocity::feet_per_second_t m_maxspeed)
{
    auto config = ChassisConfigMgr::GetInstance()->GetCurrentConfig();
    auto chassis = config != nullptr ? config->GetSwerveChassis() : nullptr;
    if (chassis != nullptr)
    {
        // pitch is positive when back of robot is lifted and negative when front of robot is lifted
        // vx is positive driving forward, so if pitch is +, need to slow down
        auto pitch = chassis->GetPitch();
        if (std::abs(pitch.to<double>()) > chassisMovement.tippingTolerance.to<double>())
        {
            auto adjust = m_maxspeed * chassisMovement.tippingCorrection * pitch.to<double>();
            chassisMovement.chassisSpeeds.vx -= adjust;
        }

        // roll is positive when the left side of the robot is lifted and negative when the right side of the robot is lifted
        // vy is positive strafing left, so if roll is +, need to strafe slower
        auto roll = chassis->GetRoll();
        if (std::abs(roll.to<double>()) > chassisMovement.tippingTolerance.to<double>())
        {
            auto adjust = m_maxspeed * chassisMovement.tippingCorrection * roll.to<double>();
            chassisMovement.chassisSpeeds.vy -= adjust;
        }
    }
}
void AntiTip::DecideTipCorrection(ChassisMovement &chassisMovement, units::velocity::feet_per_second_t m_maxspeed)
{
    if (frc::DriverStation::IsFMSAttached())
    {
        if (frc::DriverStation::GetMatchTime() > units::time::second_t(20.0))
        {
            AntiTip::CorrectForTipping(chassisMovement, m_maxspeed);
        }
    }
    else
    {
        CorrectForTipping(chassisMovement, m_maxspeed);
    }
}
