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
// This file was automatically generated by the Team 302 code generator version 1.2.3.6
// Generated on Sunday, February 18, 2024 12:51:47 PM

// C++ Includes
#include <string>

// FRC includes

// Team 302 includes
#include "mechanisms/noteManager/decoratormods/feederIntakeState.h"
#include "teleopcontrol/TeleopControl.h"
#include "teleopcontrol/TeleopControlFunctions.h"
#include "utils/logging/Logger.h"

// Third Party Includes

using namespace std;
using namespace noteManagerStates;

/// @class ExampleForwardState
/// @brief information about the control (open loop, closed loop position, closed loop velocity, etc.) for a mechanism state
feederIntakeState::feederIntakeState(std::string stateName,
									 int stateId,
									 noteManagerAllStatesStateGen *generatedState,
									 noteManager *mech) : State(stateName, stateId), m_genState(generatedState), m_mechanism(mech)
{
}

void feederIntakeState::Init()
{
	Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("feederIntakeState"), string("init"));

	m_genState->Init();
}

void feederIntakeState::Run()
{
	// Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("feederIntakeState"), string("run"));
	m_genState->Run();
}

void feederIntakeState::Exit()
{
	m_genState->Exit();
}

bool feederIntakeState::AtTarget()
{
	auto attarget = m_genState->AtTarget();
	return attarget;
}

bool feederIntakeState::IsTransitionCondition(bool considerGamepadTransitions)
{
	// To get the current state use m_mechanism->GetCurrentState()

	bool buttonsPressed = TeleopControl::GetInstance()->IsButtonPressed(TeleopControlFunctions::INTAKE) || TeleopControl::GetInstance()->IsButtonPressed(TeleopControlFunctions::DRIVE_TO_NOTE);

	return ((considerGamepadTransitions && buttonsPressed && m_mechanism->IsLauncherMode()));
}

std::string feederIntakeState::GetNoteStateName() const
{
	return std::string("FeederIntakeState");
}