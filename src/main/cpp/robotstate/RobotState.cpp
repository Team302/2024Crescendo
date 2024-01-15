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

#include "configs/RobotConfig.h"
#include "configs/RobotConfigMgr.h"
#include "chassis/IChassis.h"
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
                           m_gamePiece(RobotStateChanges::GamePiece::Cone),
                           m_gamePhase(RobotStateChanges::Disabled),
                           m_wasGamePieceButtonReleased(true),
                           m_wasCompressorButtonReleased(true)
{
    m_brokers.reserve(RobotStateChanges::LoopCounter);
    auto start = static_cast<int>(RobotStateChanges::DesiredGamePiece);
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
    auto config = RobotConfigMgr::GetInstance()->GetCurrentConfig();
    m_chassis = config != nullptr ? config->GetIChassis() : nullptr;
}

void RobotState::Run()
{
    PublishGameStateChanges();
    if (m_chassis != nullptr)
    {
        m_chassis->UpdateOdometry();
    }
    PublishCompressorInfo();
}

void RobotState::PublishCompressorInfo()
{
    if (DriverStation::IsTeleopEnabled())
    {
        auto controller = TeleopControl::GetInstance();
        if (controller != nullptr)
        {
            if (controller->IsButtonPressed(TeleopControlFunctions::TOGGLE_COMPRESSER))
            {
                if (m_wasCompressorButtonReleased)
                {
                    CompressorFactory::GetFactory()->ToggleEnableCompressor();
                }
            }
            m_wasCompressorButtonReleased = !controller->IsButtonPressed(TeleopControlFunctions::TOGGLE_COMPRESSER);
            CheckGamePieceMode(controller);
        }
    }
}

void RobotState::CheckGamePieceMode(TeleopControl *controller)
{
    if (controller->IsButtonPressed(TeleopControlFunctions::CYCLE_CYCLE_GAMEPIECE))
    {
        if (m_wasGamePieceButtonReleased)
        {
            m_gamePiece = (m_gamePiece == RobotStateChanges::Cube) ? RobotStateChanges::Cone : RobotStateChanges::Cube;
            PublishStateChange(RobotStateChanges::DesiredGamePiece, m_gamePiece);
        }
    }
    m_wasGamePieceButtonReleased = !controller->IsButtonPressed(TeleopControlFunctions::CYCLE_CYCLE_GAMEPIECE);
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
