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
#include <frc/DriverStation.h>

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
    UpdateRumble();
    UpdateDiagnosticLEDs();
    UpdateLEDStates();
    CheckControllers();
}

void DriverFeedback::UpdateRumble()
{
    auto controller = TeleopControl::GetInstance();
    if (!frc::DriverStation::IsTeleop())
    {
        controller->SetRumble(0, false, false);
        controller->SetRumble(1, false, false);
    }
    else
    {
        StateMgr *noteStateManager = RobotConfigMgr::GetInstance()->GetCurrentConfig()->GetMechanism(MechanismTypes::NOTE_MANAGER);
        auto noteMgr = noteStateManager != nullptr ? dynamic_cast<noteManagerGen *>(noteStateManager) : nullptr;
        if (noteMgr != nullptr)
        {
            if (noteStateManager->GetCurrentState() == noteManager::STATE_NAMES::STATE_FEEDER_INTAKE || noteStateManager->GetCurrentState() == noteManager::STATE_NAMES::STATE_PLACER_INTAKE)
            {
                if (noteMgr->getbackIntakeSensor()->Get() || noteMgr->getfrontIntakeSensor()->Get())
                {
                    if (!m_rumbleIntake)
                    {
                        controller->SetRumble(0, true, true);
                        controller->SetRumble(1, true, true);
                        m_rumbleIntake = true;
                    }
                }
                else
                {
                    if (m_rumbleIntake == true)
                    {
                        m_rumbleIntake = false;
                        controller->SetRumble(0, false, false);
                        controller->SetRumble(1, false, false);
                    }
                }
            }

            if (m_scoringMode == RobotStateChanges::ScoringMode::Launcher && !m_rumbleLauncher)
            {
                if (m_rumbleLoopCounter <= 20)
                {
                    if (m_firstloop == false)
                    {
                        controller->SetRumble(0, true, true);
                        controller->SetRumble(1, true, true);
                    }
                    m_rumbleLoopCounter++;
                }
                else
                {
                    m_rumbleLoopCounter = 0;
                    m_rumbleLauncher = true;
                    m_rumblePlacer = false;
                    m_firstloop = false;
                    controller->SetRumble(0, false, false);
                    controller->SetRumble(1, false, false);
                }
            }
            else if (m_scoringMode == RobotStateChanges::ScoringMode::Placer && !m_rumblePlacer)
            {
                if (m_climbMode == RobotStateChanges::ClimbMode::ClimbModeOn)
                {
                    controller->SetRumble(0, false, false);
                    controller->SetRumble(1, false, false);
                }
                else
                {
                    if (m_rumbleLoopCounter <= 20)
                    {
                        controller->SetRumble(0, true, true);
                        controller->SetRumble(1, true, true);
                        m_rumbleLoopCounter++;
                    }
                    else if (m_rumbleLoopCounter <= 30 && m_rumbleLoopCounter > 20)
                    {
                        controller->SetRumble(0, false, false);
                        controller->SetRumble(1, false, false);
                        m_rumbleLoopCounter++;
                    }
                    else if (m_rumbleLoopCounter <= 50 && m_rumbleLoopCounter > 30)
                    {
                        controller->SetRumble(0, true, true);
                        controller->SetRumble(1, true, true);
                        m_rumbleLoopCounter++;
                    }
                    else
                    {
                        m_rumbleLoopCounter = 0;
                        m_rumblePlacer = true;
                        m_rumbleLauncher = false;
                        controller->SetRumble(0, false, false);
                        controller->SetRumble(1, false, false);
                    }
                }
            }
        }
    }
}

void DriverFeedback::UpdateLEDStates()
{
    oldState = currentState;

    StateMgr *noteStateManager = RobotConfigMgr::GetInstance()->GetCurrentConfig()->GetMechanism(MechanismTypes::NOTE_MANAGER);
    auto noteMgr = noteStateManager != nullptr ? dynamic_cast<noteManagerGen *>(noteStateManager) : nullptr;

    if (noteMgr != nullptr)
    {
        if (frc::DriverStation::IsDisabled())
        {
            m_LEDStates->RainbowPattern();
        }
        else
        {
            if (m_climbMode == RobotStateChanges::ClimbMode::ClimbModeOn)
            {
                currentState = DragonLeds::RED;
                if (oldState != currentState)
                    m_LEDStates->ResetVariables();

                m_LEDStates->SolidColorPattern(currentState);
            }
            else
            {

                if (oldState != currentState)
                    m_LEDStates->ResetVariables();
                if (m_scoringMode == RobotStateChanges::ScoringMode::Launcher)
                {
                    currentState = DragonLeds::GREEN;
                }
                else if (m_scoringMode == RobotStateChanges::ScoringMode::Placer)
                {
                    currentState = DragonLeds::WHITE;
                }
                if (noteMgr != nullptr)
                {
                    if (noteStateManager->GetCurrentState() == noteManager::STATE_NAMES::STATE_READY)
                    {
                        m_LEDStates->SolidColorPattern(currentState);
                    }
                    else if (noteStateManager->GetCurrentState() == noteManager::STATE_NAMES::STATE_HOLD_PLACER)
                    {
                        currentState = DragonLeds::YELLOW;
                        m_LEDStates->SolidColorPattern(currentState);
                    }
                    else if (noteStateManager->GetCurrentState() == noteManager::STATE_NAMES::STATE_HOLD_FEEDER)
                    {
                        currentState = DragonLeds::PURPLE;
                        m_LEDStates->SolidColorPattern(currentState);
                    }
                    else if (noteStateManager->GetCurrentState() == noteManager::STATE_NAMES::STATE_READY_ODOMETRY_LAUNCH)
                    {
                        currentState = DragonLeds::AZUL;
                        m_LEDStates->SolidColorPattern(currentState);
                    }
                    else if (noteStateManager->GetCurrentState() == noteManager::STATE_NAMES::STATE_READY_AUTO_LAUNCH || noteStateManager->GetCurrentState() == noteManager::STATE_NAMES::STATE_AUTO_LAUNCH)
                    {
                        if (noteStateManager->GetCurrentStatePtr()->AtTarget())
                        {
                            m_LEDStates->AlternatingColorBlinkingPattern(currentState, DragonLeds::YELLOW);
                        }
                    }
                    else if (noteStateManager->GetCurrentState() == noteManager::STATE_NAMES::STATE_PLACE_TRAP || noteStateManager->GetCurrentState() == noteManager::STATE_NAMES::STATE_PLACE_AMP)
                    {
                        if (noteStateManager->GetCurrentStatePtr()->AtTarget())
                        {
                            m_LEDStates->AlternatingColorBlinkingPattern(currentState, DragonLeds::PURPLE);
                        }
                    }
                    if (noteStateManager->GetCurrentState() == noteManager::STATE_NAMES::STATE_FEEDER_INTAKE || noteStateManager->GetCurrentState() == noteManager::STATE_NAMES::STATE_PLACER_INTAKE)
                    {
                        if (noteMgr->getbackIntakeSensor()->Get() || noteMgr->getfrontIntakeSensor()->Get())
                        {
                            if (m_scoringMode == RobotStateChanges::ScoringMode::Launcher)
                                currentState = DragonLeds::PURPLE;
                            else
                                currentState = DragonLeds::YELLOW;
                        }
                        else
                        {
                        }
                        m_LEDStates->BlinkingPattern(currentState);
                    }
                }
            }
        }
    }
}

void DriverFeedback::UpdateDiagnosticLEDs()
{
    StateMgr *noteStateManager = RobotConfigMgr::GetInstance()->GetCurrentConfig()->GetMechanism(MechanismTypes::NOTE_MANAGER);
    auto noteMgr = noteStateManager != nullptr ? dynamic_cast<noteManagerGen *>(noteStateManager) : nullptr;
    if (noteMgr != nullptr)
    {
        bool backintake = noteMgr->getbackIntakeSensor()->Get();
        bool frontintake = noteMgr->getfrontIntakeSensor()->Get();
        bool feeder = noteMgr->getfeederSensor()->Get();
        bool launcher = noteMgr->getlauncherSensor()->Get();
        bool placerin = noteMgr->getplacerInSensor()->Get();
        bool placermid = noteMgr->getplacerMidSensor()->Get();
        bool placerout = noteMgr->getplacerOutSensor()->Get();
        m_LEDStates->DiagnosticPattern(FMSData::GetInstance()->GetAllianceColor(), backintake, frontintake, feeder, launcher, placerin, placermid, placerout);
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
