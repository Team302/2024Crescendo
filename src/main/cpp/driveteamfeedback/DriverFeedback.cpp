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

#include "teleopcontrol/TeleopControl.h"

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
    UpdateCompressorState();
    CheckControllers();
    DisplayPressure();
    DisplayDesiredGamePiece();
}

void DriverFeedback::DisplayDesiredGamePiece()
{
    auto table = nt::NetworkTableInstance::GetDefault().GetTable("Game Piece");
    table.get()->PutBoolean(std::string("Desired Piece"), m_wantCone); // true if want cone, false if want cube
}

void DriverFeedback::UpdateCompressorState()
{
    if (m_controllerCounter == 0)
    {
        auto table = nt::NetworkTableInstance::GetDefault().GetTable("Compressor");
        table.get()->PutBoolean(std::string("Compressor on"), m_compressorOn);
    }
}
void DriverFeedback::DisplayPressure() const
{
    auto table = nt::NetworkTableInstance::GetDefault().GetTable("Compressor");
    table.get()->PutNumber(std::string("Pressure"), CompressorFactory::GetFactory()->GetCurrentPressure().to<double>());
}
void DriverFeedback::UpdateLEDStates()
{
    // reset controller rumble
    // TeleopControl::GetInstance()->SetRumble(0, false, false);

    if (DriverFeedback::m_alignedWithConeNode)
    {
        if (m_gamePieceState != DriverFeedbackStates::ALIGNED_WITH_CONE_NODE)
        {
            m_LEDStates->ResetVariables();
        }
        m_LEDStates->ClosingInChaserPattern(DragonLeds::YELLOW);
        m_gamePieceState = DriverFeedbackStates::ALIGNED_WITH_CONE_NODE;
    }
    else if (DriverFeedback::m_alignedWithCubeNode)
    {
        if (m_gamePieceState != DriverFeedbackStates::ALIGNED_WITH_CUBE_NODE)
        {
            m_LEDStates->ResetVariables();
        }
        m_LEDStates->ClosingInChaserPattern(DragonLeds::PURPLE);
        m_gamePieceState = DriverFeedbackStates::ALIGNED_WITH_CUBE_NODE;
    }
    else if (DriverFeedback::m_findingCube)
    {
        m_LEDStates->ResetVariables();
        m_LEDStates->SolidColorPattern(DragonLeds::BLACK);
    }
    else if (DriverFeedback::m_wantCube)
    {
        if (m_gamePieceState != DriverFeedbackStates::WANT_CUBE)
        {
            m_LEDStates->ResetVariables();
            m_gamePieceState = DriverFeedbackStates::WANT_CUBE;
        }
        if (m_intakeStateChanged)
        {
            if (m_intakeIntaking)
                m_LEDStates->BlinkingPattern(DragonLeds::PURPLE);
            else
                m_LEDStates->SolidColorPattern(DragonLeds::PURPLE);
        }
    }
    else if (DriverFeedback::m_wantCone)
    {
        if (m_gamePieceState != DriverFeedbackStates::WANT_CONE)
        {
            m_LEDStates->ResetVariables();
            m_gamePieceState = DriverFeedbackStates::WANT_CONE;
        }
        if (m_intakeStateChanged)
        {
            if (m_intakeIntaking)
                m_LEDStates->BlinkingPattern(DragonLeds::YELLOW);
            else
                m_LEDStates->SolidColorPattern(DragonLeds::YELLOW);
        }
        else
        {
            m_LEDStates->SolidColorPattern(DragonLeds::YELLOW);
        }
        m_gamePieceState = DriverFeedbackStates::WANT_CONE;
    }
    else if (DriverFeedback::m_gamePieceReadyToPickUp)
    {
        if (m_gamePieceState != DriverFeedbackStates::GAME_PIECE_READY_TO_PICK_UP)
        {
            m_LEDStates->ResetVariables();
            m_LEDStates->SolidColorPattern(DragonLeds::GREEN);
            m_gamePieceState = DriverFeedbackStates::GAME_PIECE_READY_TO_PICK_UP;
        }
    }
    else
    {
        if (m_gamePieceState != DriverFeedbackStates::NONE)
        {
            m_LEDStates->ResetVariables();
            m_LEDStates->SolidColorPattern(DragonLeds::GREEN);
            m_gamePieceState = DriverFeedbackStates::NONE;
        }
    }

    if (DriverFeedback::m_gamePieceInIntake)
    {
        if (m_gamePieceState != DriverFeedbackStates::GAME_PIECE_IN_INTAKE)
        {
            m_LEDStates->ResetVariables();
            m_gamePieceState = DriverFeedbackStates::GAME_PIECE_IN_INTAKE;
        }
        if (DriverFeedback::m_wantCone)
        {
            m_LEDStates->BlinkingPattern(DragonLeds::YELLOW);
        }
        else
        {
            m_LEDStates->BlinkingPattern(DragonLeds::PURPLE);
        }
    }
}

void DriverFeedback::ResetRequests(void)
{
    m_intakeIntaking = false;
    m_wantCube = false;
    m_wantCone = false;
    m_gamePieceReadyToPickUp = false;
    m_gamePieceInIntake = false;
    m_alignedWithConeNode = false;
    m_alignedWithCubeNode = false;
    m_findingCube = false;

    m_intakeStateChanged = true;
}

DriverFeedback::DriverFeedback() : IRobotStateChangeSubscriber()
{
    RobotState::GetInstance()->RegisterForStateChanges(this, RobotStateChanges::StateChange::IntakeState);
    RobotState::GetInstance()->RegisterForStateChanges(this, RobotStateChanges::StateChange::DesiredGamePiece);
    RobotState::GetInstance()->RegisterForStateChanges(this, RobotStateChanges::StateChange::HoldingGamePiece);
    RobotState::GetInstance()->RegisterForStateChanges(this, RobotStateChanges::StateChange::GameState);
    RobotState::GetInstance()->RegisterForStateChanges(this, RobotStateChanges::StateChange::CompressorChange);
    RobotState::GetInstance()->RegisterForStateChanges(this, RobotStateChanges::StateChange::FindingCube);
}
void DriverFeedback::Update(RobotStateChanges::StateChange change, int value)
{
    if (change == RobotStateChanges::DesiredGamePiece)
    {
        auto gamepiece = static_cast<RobotStateChanges::GamePiece>(value);
        m_wantCube = gamepiece == RobotStateChanges::Cube;
        m_wantCone = gamepiece == RobotStateChanges::Cone;
    }
    else if (change == RobotStateChanges::IntakeState)
    {
        /**
        auto state = static_cast<IntakeStateMgr::INTAKE_STATE>(value);
        bool newState = state == IntakeStateMgr::INTAKE_STATE::INTAKE;

        if (m_intakeIntaking != newState)
        {
            m_intakeStateChanged = true;
            m_intakeIntaking = newState;
        }
        **/
    }
    else if (change == RobotStateChanges::HoldingGamePiece)
    {
        m_gamePieceInIntake = static_cast<RobotStateChanges::GamePiece>(value) != RobotStateChanges::None;
    }
    else if (change == RobotStateChanges::GameState)
    {
        auto state = static_cast<RobotStateChanges::GamePeriod>(value);
        m_AutonomousEnabled = state == RobotStateChanges::Auton;
        m_TeleopEnabled = state == RobotStateChanges::Teleop;

        ResetRequests();
    }
    else if (change == RobotStateChanges::StateChange::CompressorChange)
    {
        auto compressor = static_cast<RobotStateChanges::CompressorState>(value);
        m_compressorOn = compressor == RobotStateChanges::CompressorOn;
    }
    else if (change == RobotStateChanges::StateChange::FindingCube)
    {
        m_findingCube = (value == 1);
    }
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
