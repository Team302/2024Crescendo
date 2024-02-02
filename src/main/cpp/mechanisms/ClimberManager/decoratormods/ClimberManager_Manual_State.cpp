// clang-format off
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
// This file was automatically generated by the Team 302 code generator version 1.2.1.0
// Generated on Monday, January 29, 2024 6:19:01 AM

// C++ Includes
#include <string>

// FRC includes

// Team 302 includes
#include "mechanisms/ClimberManager/generated/ClimberManager_Manual_StateGen.h"
#include "mechanisms/ClimberManager/decoratormods/ClimberManager_Manual_State.h"
#include "teleopcontrol/TeleopControl.h"
#include "teleopcontrol/TeleopControlFunctions.h"
#include "utils/logging/Logger.h"

// Third Party Includes

using namespace std;

/// @class ExampleForwardState
/// @brief information about the control (open loop, closed loop position, closed loop velocity, etc.) for a mechanism state
ClimberManagerManualState::ClimberManagerManualState ( std::string stateName,
        int stateId,
        ClimberManagerManualStateGen *generatedState,
        ClimberManager *mech ) : State ( stateName, stateId ), m_genState ( generatedState ), m_mechanism ( mech )
{
}

void ClimberManagerManualState::Init()
{
	Logger::GetLogger()->LogData ( LOGGER_LEVEL::PRINT, string ( "ArrivedAt" ), string ( "ClimberManagerManualState" ), string ( "init" ) );

	m_genState->Init();
}

void ClimberManagerManualState::Run()
{
	// Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("ClimberManagerManualState"), string("run"));
	m_genState->Run();
}

void ClimberManagerManualState::Exit()
{
	m_genState->Exit();
}

bool ClimberManagerManualState::AtTarget()
{
	auto attarget = m_genState->AtTarget();
	return attarget;
}

bool ClimberManagerManualState::IsTransitionCondition ( bool considerGamepadTransitions )
{
	// To get the current state use m_mechanism->GetCurrentState()
	auto currentState = m_mechanism->GetCurrentState();
	return (m_mechanism->isClimbMode() || (TeleopControl::GetInstance()->IsButtonPressed(TeleopControlFunctions::AUTO_CLIMB) && currentState == m_mechanism->STATE_AUTO_CLIMB));
}
