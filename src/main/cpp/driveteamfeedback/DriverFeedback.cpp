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

#include "frc/DriverStation.h"
#include "driveteamfeedback/DriverFeedback.h"
#include "hw/factories/CompressorFactory.h"
#include "robotstate/RobotState.h"
#include "robotstate/RobotStateChanges.h"
#include "robotstate/IRobotStateChangeSubscriber.h"
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <driveteamfeedback/LEDStates.h>

#include "teleopcontrol/TeleopControl.h"
#include "configs/RobotConfigMgr.h"
#include "mechanisms/noteManager/decoratormods/noteManager.h"

using frc::DriverStation;

DriverFeedback *DriverFeedback::m_instance = nullptr;

DriverFeedback *DriverFeedback::GetInstance()
{
    if (DriverFeedback::m_instance == nullptr)
    {
        DriverFeedback::m_instance = new DriverFeedback();
    }
    return DriverFeedback::m_instance;
}

void DriverFeedback::UpdateFeedback()
{
    UpdateLEDStates();
    CheckControllers();
}
void DriverFeedback::UpdateLEDStates()
{
    // reset controller rumble
    // TeleopControl::GetInstance()->SetRumble(0, false, false);

    oldState = currentState;
    if (m_climbMode == RobotStateChanges::ClimbMode::ClimbModeOn)
    {
        currentState = DragonLeds::RED;
        if (oldState != currentState)
            m_LEDStates->ResetVariables();

        m_LEDStates->SolidColorPattern(currentState);
    }
    else
    {
        if (m_scoringMode == RobotStateChanges::ScoringMode::Launcher)
            currentState = DragonLeds::GREEN;
        else if (m_scoringMode == RobotStateChanges::ScoringMode::Placer)
            currentState = DragonLeds::WHITE;

        if (oldState != currentState)
            m_LEDStates->ResetVariables();

        StateMgr *noteStateManager = RobotConfigMgr::GetInstance()->GetCurrentConfig()->GetMechanism(MechanismTypes::NOTE_MANAGER);
        if (noteStateManager != nullptr)
        {
            if (noteStateManager->GetCurrentState() == noteManager::STATE_NAMES::STATE_READY)
                m_LEDStates->BlinkingPattern(currentState);
            else
                m_LEDStates->SolidColorPattern(currentState);
        }
        else
            m_LEDStates->SolidColorPattern(currentState);
    }
}

void DriverFeedback::ResetRequests(void)
{
}

DriverFeedback::DriverFeedback() : IRobotStateChangeSubscriber()
{

    RobotState *RobotStates = RobotState::GetInstance();

    RobotStates->RegisterForStateChanges(this, RobotStateChanges::StateChange::DesiredScoringMode);
    RobotStates->RegisterForStateChanges(this, RobotStateChanges::StateChange::ClimbModeStatus);
}
void DriverFeedback::Update(RobotStateChanges::StateChange change, int value)
{
    if (RobotStateChanges::StateChange::ClimbModeStatus == change)
        m_climbMode = static_cast<RobotStateChanges::ClimbMode>(value);

    else if (RobotStateChanges::StateChange::DesiredScoringMode == change)
        m_scoringMode = static_cast<RobotStateChanges::ScoringMode>(value);
}

void DriverFeedback::CheckControllers()
{
    if (m_controllerCounter == 0)
    {
        auto table = nt::NetworkTableInstance::GetDefault().GetTable("XBOX Controller");
        for (auto i = 0; i < DriverStation::kJoystickPorts; ++i)
        {
            table.get()->PutBoolean(std::string("Controller") + std::to_string(i), DriverStation::GetJoystickIsXbox(i));
        }
    }
    m_controllerCounter++;
    if (m_controllerCounter > 25)
    {
        m_controllerCounter = 0;
    }
}
