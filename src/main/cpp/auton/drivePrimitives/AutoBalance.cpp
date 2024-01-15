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

// C++
#include <string>

// FRC Includes

// 302 Includes
#include <auton/drivePrimitives/AutoBalance.h>
#include <chassis/ChassisMovement.h>
#include <chassis/ChassisOptionEnums.h>
#include "configs/RobotConfig.h"
#include "configs/RobotConfigMgr.h"

/// DEBUGGING
#include "utils/logging/Logger.h"

using namespace std;
using namespace frc;

using namespace wpi::math;

AutoBalance::AutoBalance() : m_chassis(nullptr),
                             // max velocity of 1 rotation per second and a max acceleration of 180 degrees per second squared.
                             m_headingOption(ChassisOptionEnums::HeadingOption::MAINTAIN),
                             m_heading(0.0),
                             m_ntName("AutoBalance"),
                             m_timer(new frc::Timer()),
                             m_maxTime(units::time::second_t(-1.0)),
                             m_maxTimeTimer(new frc::Timer())

{
    auto config = RobotConfigMgr::GetInstance()->GetCurrentConfig();
    m_chassis = config != nullptr ? config->GetSwerveChassis() : nullptr;
}
void AutoBalance::Init(PrimitiveParams *params)
{
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_ntName, "ArrivedAtInit", true);
    m_maxTime = params->GetTime();
    m_maxTimeTimer->Reset();
    m_maxTimeTimer->Start();
    m_headingOption = params->GetHeadingOption();
    m_heading = params->GetHeading();
}
void AutoBalance::Run()
{
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_ntName, "ArrivedAtRun", true);

    if (m_chassis != nullptr)
    {
        ChassisMovement moveInfo;
        moveInfo.driveOption = ChassisOptionEnums::DriveStateType::AUTO_BALANCE;
        moveInfo.headingOption = m_headingOption;
        moveInfo.yawAngle = units::angle::degree_t(m_heading);

        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_ntName, "DriveState", moveInfo.driveOption);
        m_chassis->Drive(moveInfo);
    }
}

bool AutoBalance::IsDone()
{
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_ntName, "ArrivedAtDone", true);
    if (m_chassis != nullptr)
    {
        if (m_maxTimeTimer->Get() > m_maxTime && m_maxTime > units::time::second_t(0.0))
        {
            return true;
        }
        else
        {
            auto pitch = m_chassis->GetPitch().to<double>();
            auto roll = m_chassis->GetRoll().to<double>();
            if (abs(pitch) < m_balanceTolerance && abs(roll) < m_balanceTolerance)
            {
                m_timer->Start();
            }
            else
            {
                m_timer->Reset();
            }

            return m_timer->Get().to<double>() > m_balanceTimeout;
        }
    }
    return true;
}