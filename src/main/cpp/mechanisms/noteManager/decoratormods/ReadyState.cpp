
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
// Generated on Friday, February 2, 2024 7:00:50 PM

// C++ Includes
#include <string>

// FRC includes

// Team 302 includes
#include "mechanisms/noteManager/decoratormods/ReadyState.h"
#include "teleopcontrol/TeleopControl.h"
#include "teleopcontrol/TeleopControlFunctions.h"
#include "utils/logging/Logger.h"

// Third Party Includes

using namespace std;
using namespace noteManagerStates;

/// @class ExampleForwardState
/// @brief information about the control (open loop, closed loop position, closed loop velocity, etc.) for a mechanism state
ReadyState::ReadyState(std::string stateName,
					   int stateId,
					   noteManagerAllStatesStateGen *generatedState,
					   noteManager *mech) : State(stateName, stateId), m_genState(generatedState), m_mechanism(mech)
{
}

void ReadyState::Init()
{
	Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("ReadyState"), string("init"));

	m_genState->Init();
}

void ReadyState::Run()
{
	Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("ReadyState"), string("run"));
	m_genState->Run();
	Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Ready"), string("Front Intake Sensor"), m_mechanism->getfrontIntakeSensor()->Get());
	Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Ready"), string("Back Intake Sensor"), m_mechanism->getbackIntakeSensor()->Get());
	Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Ready"), string("Feeder Sensor"), m_mechanism->getfeederSensor()->Get());
	Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Ready"), string("Launcher Sensor"), m_mechanism->getlauncherSensor()->Get());
	Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Ready"), string("Placer In"), m_mechanism->getplacerInSensor()->Get());
	Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Ready"), string("Placer Mid"), m_mechanism->getplacerMidSensor()->Get());
	Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Ready"), string("Placer Out"), m_mechanism->getplacerOutSensor()->Get());
}

void ReadyState::Exit()
{
	m_genState->Exit();
}

bool ReadyState::AtTarget()
{
	auto attarget = m_genState->AtTarget();
	return attarget;
}

bool ReadyState::IsTransitionCondition(bool considerGamepadTransitions)
{
	// To get the current state use m_mechanism->GetCurrentState()
	bool transition = false;
	auto currentState = m_mechanism->GetCurrentState();
	bool placerInSensor = m_mechanism->getplacerInSensor()->Get();
	bool placerMidSensor = m_mechanism->getplacerMidSensor()->Get();
	bool placerOutSensor = m_mechanism->getplacerOutSensor()->Get();
	bool launcherSensor = m_mechanism->getlauncherSensor()->Get();
	bool feederSensor = m_mechanism->getfeederSensor()->Get();
	bool frontIntakeSensor = m_mechanism->getfrontIntakeSensor()->Get();
	bool backIntakeSensor = m_mechanism->getbackIntakeSensor()->Get();

	if (m_mechanism->IsEnabled() && (currentState == static_cast<int>(m_mechanism->STATE_OFF)))
	{
		transition = true;
	}
	else if (TeleopControl::GetInstance()->IsButtonPressed(TeleopControlFunctions::READY))
	{
		transition = true;
	}
	else if (TeleopControl::GetInstance()->IsButtonPressed(TeleopControlFunctions::MANUAL_MODE) &&
			 ((currentState == static_cast<int>(m_mechanism->STATE_BACKUP_MANUAL_LAUNCH)) || (currentState == static_cast<int>(m_mechanism->STATE_BACKUP_MANUAL_PLACE))))
	{
		transition = true;
	}
	else if ((launcherSensor == false) &&
			 (feederSensor == false) &&
			 ((currentState == static_cast<int>(m_mechanism->STATE_MANUAL_LAUNCH)) || (currentState == static_cast<int>(m_mechanism->STATE_AUTO_LAUNCH)) || (currentState == static_cast<int>(m_mechanism->STATE_PASS)) || (currentState == static_cast<int>(m_mechanism->STATE_AUTO_LAUNCH_ODOMETRY))))
	{
		transition = true;
	}
	else if ((placerInSensor == false) &&
			 (placerMidSensor == false) &&
			 (placerOutSensor == false) &&
			 ((currentState == static_cast<int>(m_mechanism->STATE_PLACE_AMP)) || (currentState == static_cast<int>(m_mechanism->STATE_PLACE_TRAP))))
	{
		transition = true;
	}
	else if ((TeleopControl::GetInstance()->IsButtonPressed(TeleopControlFunctions::INTAKE) == false) &&
			 ((frontIntakeSensor == false) || (backIntakeSensor == false)) &&
			 ((currentState == static_cast<int>(m_mechanism->STATE_PLACER_INTAKE)) || (currentState == static_cast<int>(m_mechanism->STATE_FEEDER_INTAKE))))
	{
		transition = true;
	}
	else if ((TeleopControl::GetInstance()->IsButtonPressed(TeleopControlFunctions::EXPEL) == false) &&
			 (currentState == static_cast<int>(m_mechanism->STATE_EXPEL)))
	{
		transition = true;
	}
	return (transition);
}
