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
// Generated on Sunday, February 18, 2024 12:44:30 PM

// C++ Includes
#include <string>

// FRC includes

// Team 302 includes
#include "mechanisms/ClimberManager/decoratormods/ManualState.h"
#include "teleopcontrol/TeleopControl.h"
#include "teleopcontrol/TeleopControlFunctions.h"
#include "utils/logging/Logger.h"

// Third Party Includes

using namespace std;
using namespace ClimberManagerStates;

/// @class ExampleFwardState
/// @brief information about the control (open loop, closed loop position, closed loop velocity, etc.) for a mechanism state
ManualState::ManualState(std::string stateName,
						 int stateId,
						 ClimberManagerAllStatesStateGen *generatedState,
						 ClimberManager *mech) : State(stateName, stateId), m_genState(generatedState), m_mechanism(mech)
{
}

void ManualState::Init()
{
	Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("ManualState"), string("init"));

	m_genState->Init();
}

void ManualState::Run()
{
	// Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("ManualState"), string("run"));

	if (abs(TeleopControl::GetInstance()->GetAxisValue(TeleopControlFunctions::MANUAL_CLIMB)) > 0.05)
	{
		double delta = 8.0 * 0.05 * (TeleopControl::GetInstance()->GetAxisValue(TeleopControlFunctions::MANUAL_CLIMB)); // changing by 6 in/s * 0.05 for 20 ms loop time * controller input
		m_target += delta;
		if (m_target < 8.5)
			m_target = 8.5;
		m_mechanism->UpdateTarget(RobotElementNames::MOTOR_CONTROLLER_USAGE::CLIMBER_MANAGER_RIGHT_CLIMBER, m_target);
		m_mechanism->UpdateTarget(RobotElementNames::MOTOR_CONTROLLER_USAGE::CLIMBER_MANAGER_LEFT_CLIMBER, m_target);
	}

	Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Climber"), string("Left"), m_mechanism->getleftClimber()->GetCounts());
	Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Climber"), string("Target"), m_target);

	m_genState->Run();
}

void ManualState::Exit()
{
	m_genState->Exit();
}

bool ManualState::AtTarget()
{
	auto attarget = m_genState->AtTarget();
	return attarget;
}

bool ManualState::IsTransitionCondition(bool considerGamepadTransitions)
{
	// To get the current state use m_mechanism->GetCurrentState()
	auto currentState = m_mechanism->GetCurrentState();
	return (considerGamepadTransitions && (m_mechanism->IsClimbMode()) && (currentState != m_mechanism->STATE_AUTO_CLIMB));
}
