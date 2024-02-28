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
					   noteManager *mech) : State(stateName, stateId), m_genState(generatedState), m_mechanism(mech), m_launchTimer(new frc::Timer())
{
}

void ReadyState::Init()
{
	Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("ReadyState"), string("init"));
	m_genState->Init();
}

void ReadyState::Run()
{
	// Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("ReadyState"), string("run"));
	if (abs(TeleopControl::GetInstance()->GetAxisValue(TeleopControlFunctions::ELEVATOR)) > 0.05) // Allows manual cotrol of the elevator if you need to adujst
	{
		double delta = 6.0 * 0.1 * (TeleopControl::GetInstance()->GetAxisValue(TeleopControlFunctions::ELEVATOR)); // changing by 6 in/s * 0.05 for 20 ms loop time * controller input
		m_target += delta;
		m_mechanism->UpdateTarget(RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_ELEVATOR, m_target);
		if (m_target > 16.5) // limiting the travel to 0 through 16.5
			m_target = 16.5;
		else if (m_target < 0)
			m_target = 0;
	}
	m_mechanism->UpdateTarget(RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_PLACER, TeleopControl::GetInstance()->GetAxisValue(TeleopControlFunctions::MANUAL_PLACE));
	m_mechanism->UpdateTarget(RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_FEEDER, TeleopControl::GetInstance()->GetAxisValue(TeleopControlFunctions::MANUAL_FEED));

	if (TeleopControl::GetInstance()->GetAxisValue(TeleopControlFunctions::MANUAL_FEED) > 0) // Transfer turns on if the placer and feeder are ran manually
		m_mechanism->UpdateTarget(RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_TRANSFER, 1.0);
	else if (TeleopControl::GetInstance()->GetAxisValue(TeleopControlFunctions::MANUAL_PLACE) > 0)
		m_mechanism->UpdateTarget(RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_TRANSFER, -1.0);
	else
		m_mechanism->UpdateTarget(RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_TRANSFER, 0.0);

	m_genState->Run();
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

	int reason = 0;

	if (m_mechanism->IsEnabled() && (currentState == static_cast<int>(m_mechanism->STATE_OFF)))
	{
		transition = true;
		reason = 1;
	}
	else if (TeleopControl::GetInstance()->IsButtonPressed(TeleopControlFunctions::READY) && considerGamepadTransitions)
	{
		transition = true;
		reason = 2;
	}
	else if (considerGamepadTransitions && TeleopControl::GetInstance()->IsButtonPressed(TeleopControlFunctions::MANUAL_MODE) &&
			 ((currentState == static_cast<int>(m_mechanism->STATE_BACKUP_MANUAL_LAUNCH)) || (currentState == static_cast<int>(m_mechanism->STATE_BACKUP_MANUAL_PLACE))))
	{
		transition = true;
		reason = 3;
	}
	else if ((launcherSensor == false) &&
			 (feederSensor == false) &&
			 ((currentState == static_cast<int>(m_mechanism->STATE_MANUAL_LAUNCH)) || (currentState == static_cast<int>(m_mechanism->STATE_AUTO_LAUNCH)) || (currentState == static_cast<int>(m_mechanism->STATE_PASS)) || (currentState == static_cast<int>(m_mechanism->STATE_AUTO_LAUNCH_ODOMETRY))))
	{
		m_launchTimer->Start();
		if (m_launchTimer->Get().to<double>() > 0.25)
		{
			transition = true;
			m_launchTimer->Stop();
			m_launchTimer->Reset();
		}
		reason = 4;
	}
	else if ((placerInSensor == false) &&
			 (placerMidSensor == false) &&
			 (placerOutSensor == false) &&
			 ((currentState == static_cast<int>(m_mechanism->STATE_PLACE_AMP)) || (currentState == static_cast<int>(m_mechanism->STATE_PLACE_TRAP))))
	{
		transition = true;
		reason = 5;
	}
	else if (considerGamepadTransitions && (TeleopControl::GetInstance()->IsButtonPressed(TeleopControlFunctions::INTAKE) == false) &&
			 ((frontIntakeSensor == false) && (backIntakeSensor == false)) &&
			 (((currentState == static_cast<int>(m_mechanism->STATE_PLACER_INTAKE)) && ((placerInSensor == false) && (placerMidSensor == false))) ||
			  ((currentState == static_cast<int>(m_mechanism->STATE_FEEDER_INTAKE)) && ((launcherSensor == false) && (feederSensor == false)))))
	{
		transition = true;
		reason = 6;
	}
	else if ((considerGamepadTransitions && TeleopControl::GetInstance()->IsButtonPressed(TeleopControlFunctions::EXPEL) == false) &&
			 (currentState == static_cast<int>(m_mechanism->STATE_EXPEL)))
	{
		transition = true;
		reason = 7;
	}

	Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Ready"), string("Transition Reason"), reason); // Remove logging after Note management is all verifed
	return (transition);
}
