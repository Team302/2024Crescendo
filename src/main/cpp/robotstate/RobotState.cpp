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
//======================================================\==============================================================================================

#include "robotstate/RobotState.h"

#include <string>
#include <vector>

#include "chassis/configs/ChassisConfig.h"
#include "chassis/configs/ChassisConfigMgr.h"
#include "chassis/SwerveChassis.h"
#include "robotstate/RobotStateChangeBroker.h"
#include "teleopcontrol/TeleopControl.h"
#include "utils/DragonField.h"
#include "hw/factories/CompressorFactory.h"

using frc::DriverStation;

RobotState *RobotState::m_instance = nullptr;

RobotState *RobotState::GetInstance()
{
    if (RobotState::m_instance == nullptr)
    {
        RobotState::m_instance = new RobotState();
    }
    return RobotState::m_instance;
}

RobotState::RobotState() : m_chassis(nullptr),
                           m_brokers(),
                           m_scoringMode(RobotStateChanges::ScoringMode::Launcher),
                           m_climbMode(RobotStateChanges::ClimbMode::ClimbModeOff),
                           m_gamePhase(RobotStateChanges::Disabled),
                           m_scoringModeButtonReleased(true),
                           m_climbModeButtonReleased(true)
{
    m_brokers.reserve(RobotStateChanges::LoopCounter);
    auto start = static_cast<int>(RobotStateChanges::DesiredScoringMode);
    auto end = static_cast<int>(RobotStateChanges::LoopCounter);
    for (auto i = start; i < end; ++i)
    {
        m_brokers.emplace_back(new RobotStateChangeBroker(static_cast<RobotStateChanges::StateChange>(i)));
    }
}

RobotState::~RobotState()
{
    for (auto broker : m_brokers)
    {
        delete broker;
    }
    m_brokers.clear();
}

void RobotState::Init()
{
    auto chassisConfig = ChassisConfigMgr::GetInstance()->GetCurrentConfig();
    m_chassis = chassisConfig != nullptr ? chassisConfig->GetSwerveChassis() : nullptr;
}

void RobotState::Run()
{
    PublishGameStateChanges();
    if (DriverStation::IsTeleopEnabled())
    {
        auto controller = TeleopControl::GetInstance();
        if (controller != nullptr)
        {
            PublishScoringMode(controller);
            PublishClimbMode(controller);
        }
    }
}

void RobotState::RegisterForStateChanges(IRobotStateChangeSubscriber *subscriber, RobotStateChanges::StateChange change)
{
    auto slot = static_cast<unsigned int>(change);
    if (slot < m_brokers.size())
    {
        m_brokers[slot]->AddSubscriber(subscriber);
    }
}

void RobotState::PublishStateChange(RobotStateChanges::StateChange change, int newValue)
{
    auto slot = static_cast<unsigned int>(change);
    if (slot < m_brokers.size())
    {
        m_brokers[slot]->Notify(newValue);
    }
}

void RobotState::PublishGameStateChanges()
{
    auto gameState = m_gamePhase;
    if (frc::DriverStation::IsEnabled())
    {
        if (DriverStation::IsAutonomousEnabled())
        {
            gameState = RobotStateChanges::Auton;
        }
        else if (DriverStation::IsTeleopEnabled())
        {
            gameState = RobotStateChanges::Teleop;
        }
    }
    else
    {
        gameState = RobotStateChanges::Disabled;
    }

    if (gameState != m_gamePhase)
    {
        m_gamePhase = gameState;
        PublishStateChange(RobotStateChanges::GameState, gameState);
    }
}
void RobotState::PublishScoringMode(TeleopControl *controller)
{
    if (controller->IsButtonPressed(TeleopControlFunctions::SCORING_MODE))
    {
        if (m_scoringModeButtonReleased)
        {
            m_scoringMode = (m_scoringMode == RobotStateChanges::Launcher) ? RobotStateChanges::Placer : RobotStateChanges::Launcher;
            PublishStateChange(RobotStateChanges::DesiredScoringMode, m_scoringMode);
        }
    }
    m_scoringModeButtonReleased = !controller->IsButtonPressed(TeleopControlFunctions::SCORING_MODE);
}

void RobotState::PublishClimbMode(TeleopControl *controller)
{
    if (controller->IsButtonPressed(TeleopControlFunctions::CLIMB_MODE))
    {
        if (m_climbModeButtonReleased)
        {
            m_climbMode = (m_climbMode == RobotStateChanges::ClimbModeOff) ? RobotStateChanges::ClimbModeOn : RobotStateChanges::ClimbModeOff;
            if (m_climbMode == RobotStateChanges::ClimbMode::ClimbModeOn)
            {
                m_scoringMode = RobotStateChanges::ScoringMode::Placer;
                PublishStateChange(RobotStateChanges::DesiredScoringMode, m_scoringMode);
            }

            PublishStateChange(RobotStateChanges::ClimbModeStatus, m_climbMode);
        }
    }
    m_climbModeButtonReleased = !controller->IsButtonPressed(TeleopControlFunctions::CLIMB_MODE);
}
