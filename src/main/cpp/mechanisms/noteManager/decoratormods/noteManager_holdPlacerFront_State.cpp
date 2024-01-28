// clang-format off
//====================================================================================================================================================
// Copyright 2023 Lake Orion Robotics FIRST Team 302
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
// Generated on Sunday, January 28, 2024 1:31:24 PM

// C++ Includes
#include <string>

// FRC includes

// Team 302 includes
#include "State.h"
#include "mechanisms/noteManager/generated/noteManager_holdPlacerFront_StateGen.h"
#include "mechanisms/noteManager/decoratormods/noteManager_holdPlacerFront_State.h"
#include "teleopcontrol/TeleopControl.h"
#include "teleopcontrol/TeleopControlFunctions.h"

#include <utils/logging/LoggableItemMgr.h>
#include "utils/logging/Logger.h"
#include <utils/logging/LoggerData.h>
#include <utils/logging/LoggerEnums.h>

// Third Party Includes

using namespace std;

/// @class ExampleForwardState
/// @brief information about the control (open loop, closed loop position, closed loop velocity, etc.) for a mechanism state
noteManagerholdPlacerFrontState::noteManagerholdPlacerFrontState ( std::string stateName,
        int stateId,
        noteManagerholdPlacerFrontStateGen *generatedState,
        noteManager *mech ) : State ( stateName, stateId ), m_genState ( generatedState ), m_mechanism ( mech )
{
}

void noteManagerholdPlacerFrontState::Init()
{
	Logger::GetLogger()->LogData ( LOGGER_LEVEL::PRINT, string ( "ArrivedAt" ), string ( "noteManagerholdPlacerFrontState" ), string ( "init" ) );

	m_genState->Init();
}

void noteManagerholdPlacerFrontState::Run()
{
	// Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("noteManagerholdPlacerFrontState"), string("run"));
	m_genState->Run();
}

void noteManagerholdPlacerFrontState::Exit()
{
	m_genState->Exit();
}

bool noteManagerholdPlacerFrontState::AtTarget()
{
	auto attarget = m_genState->AtTarget();
	return attarget;
}

bool noteManagerholdPlacerFrontState::IsTransitionCondition ( bool considerGamepadTransitions )
{
	bool placersensor = m_genState->GetnoteManager()->placerInSensor->Get();
	bool frontintakesensor = m_genState->GetnoteManager()->frontIntakeSensor->Get();
	// To get the current state use m_genState->GetMECHANISM()->GetCurrentState()
	// where MECHANISM is the name of the generated mechanism object

	// auto transition = m_genState->IsTransitionCondition(considerGamepadTransitions);
	// return transition;
	return ( placersensor && frontintakesensor );
}
