
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

#include <cmath>

#include "units/angle.h"
#include <units/angular_acceleration.h>
#include "units/angular_velocity.h"
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/controller/ProfiledPIDController.h>

#include "configs/RobotConfig.h"
#include "configs/RobotConfigMgr.h"
#include <chassis/ChassisMovement.h>
#include "chassis/swerve/SwerveChassis.h"
#include <chassis/TurnToAngle.h>
#include "State.h"
#include <utils/AngleUtils.h>

using namespace frc;
using namespace std;

TurnToAngle::TurnToAngle(
    units::angle::degree_t targetAngle) : State(string("TurnToAAngle"), -1),
                                          m_targetAngle(targetAngle),
                                          m_chassis(nullptr),
                                          m_atTarget(false)
{
    auto config = RobotConfigMgr::GetInstance()->GetCurrentConfig();
    m_chassis = config != nullptr ? config->GetSwerveChassis() : nullptr;
}

void TurnToAngle::Init()
{
}

// Worst case scenario
//
//
//                current orientation (0)
//                 +---------------+
//                 | @           @ |
//                 |       ^       |
//                 |       |       |
//                 |               |
//                 |               |
//                 | @           @ |
//                 +---------------+
//
//                desired orientation (180 / -180) worse case scenario
//                 +---------------+
//                 | @           @ |
//                 |       |       |
//                 |      \/       |
//                 |               |
//                 |               |
//                 | @           @ |
//                 +---------------+
//
//                desired orientation (+90)
//                 +---------------+
//                 | @           @ |
//                 |               |
//                 |     <--       |
//                 |               |
//                 |               |
//                 | @           @ |
//                 +---------------+
//
//                desired orientation (-90)
//                 +---------------+
//                 | @           @ |
//                 |               |
//                 |     -->       |
//                 |               |
//                 |               |
//                 | @           @ |
//                 +---------------+
//
//
void TurnToAngle::Run()
{
    if (m_chassis != nullptr)
    {
        auto currentAngle = m_chassis->GetYaw();
        auto delta = AngleUtils::GetDeltaAngle(currentAngle, m_targetAngle);
        ChassisMovement moveInfo;
        moveInfo.chassisSpeeds.vx = 0_mps;
        moveInfo.chassisSpeeds.vy = 0_mps;
        moveInfo.driveOption = ChassisOptionEnums::DriveStateType::ROBOT_DRIVE;
        moveInfo.headingOption = ChassisOptionEnums::HeadingOption::TOWARD_GOAL;

        if (std::abs(delta.to<double>()) > m_angleTolerance.to<double>())
        {
            m_pid.SetSetpoint(m_targetAngle.to<double>());
            auto rotatePercent = m_pid.Calculate(currentAngle.to<double>());
            rotatePercent = clamp(rotatePercent, -1.0, 1.0);
            moveInfo.chassisSpeeds.omega = rotatePercent * m_chassis->GetMaxAngularSpeed();
        }
        else
        {
            moveInfo.chassisSpeeds.omega = units::angular_velocity::degrees_per_second_t(0.0);
            m_atTarget = true;
        }
        m_chassis->Drive(moveInfo);
    }
}

bool TurnToAngle::AtTarget()
{

    return m_atTarget;
}