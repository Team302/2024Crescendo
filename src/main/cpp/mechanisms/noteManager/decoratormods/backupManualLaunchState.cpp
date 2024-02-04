
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
// Generated on Friday, February 2, 2024 7:00:51 PM

// C++ Includes
#include <string>

// FRC includes

// Team 302 includes
#include "mechanisms/noteManager/decoratormods/backupManualLaunchState.h"
#include "teleopcontrol/TeleopControl.h"
#include "teleopcontrol/TeleopControlFunctions.h"
#include "utils/logging/Logger.h"

// Third Party Includes

using namespace std;
using namespace noteManagerStates;

/// @class ExampleForwardState
/// @brief information about the control (open loop, closed loop position, closed loop velocity, etc.) for a mechanism state
backupManualLaunchState::backupManualLaunchState(std::string stateName,
												 int stateId,
												 noteManagerAllStatesStateGen *generatedState,
												 noteManager *mech) : State(stateName, stateId), m_genState(generatedState), m_mechanism(mech)
{
}

void backupManualLaunchState::Init()
{
	Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("backupManualLaunchState"), string("init"));

	m_genState->Init();
}

void backupManualLaunchState::Run()
{
	Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("backupManualLaunchState"), string("run"));
	m_genState->Run();
	/* Have to use SetTargetControl until fix for updateTarget skipping a layer of code
	m_genState->GetnoteManager()->UpdateTarget(RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_FRONT_INTAKE, static_cast<double>(TeleopControl::GetInstance()->IsButtonPressed(TeleopControlFunctions::BACKUP_FRONT_INTAKE)));
	m_genState->GetnoteManager()->UpdateTarget(RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_BACK_INTAKE, static_cast<double>(TeleopControl::GetInstance()->IsButtonPressed(TeleopControlFunctions::BACKUP_BACK_INTAKE)));
	m_genState->GetnoteManager()->UpdateTarget(RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_PLACER, TeleopControl::GetInstance()->GetAxisValue(TeleopControlFunctions::MANUAL_PLACE));
	m_genState->GetnoteManager()->UpdateTarget(RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_FEEDER, TeleopControl::GetInstance()->GetAxisValue(TeleopControlFunctions::MANUAL_FEED));
	m_genState->GetnoteManager()->UpdateTarget(RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_ELEVATOR, TeleopControl::GetInstance()->GetAxisValue(TeleopControlFunctions::ELEVATOR) * 0.5);
	m_genState->GetnoteManager()->UpdateTarget(RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_ANGLE, TeleopControl::GetInstance()->GetAxisValue(TeleopControlFunctions::LAUNCH_ANGLE) * 0.5);
	*/
	m_genState->SetTargetControl(RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_FRONT_INTAKE, static_cast<double>(TeleopControl::GetInstance()->IsButtonPressed(TeleopControlFunctions::BACKUP_FRONT_INTAKE)));
	m_genState->SetTargetControl(RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_BACK_INTAKE, static_cast<double>(TeleopControl::GetInstance()->IsButtonPressed(TeleopControlFunctions::BACKUP_BACK_INTAKE)));
	m_genState->SetTargetControl(RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_PLACER, TeleopControl::GetInstance()->GetAxisValue(TeleopControlFunctions::MANUAL_PLACE));
	m_genState->SetTargetControl(RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_FEEDER, TeleopControl::GetInstance()->GetAxisValue(TeleopControlFunctions::MANUAL_FEED));
	m_genState->SetTargetControl(RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_ELEVATOR, TeleopControl::GetInstance()->GetAxisValue(TeleopControlFunctions::ELEVATOR) * 0.5);
	m_genState->SetTargetControl(RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_ANGLE, TeleopControl::GetInstance()->GetAxisValue(TeleopControlFunctions::LAUNCH_ANGLE) * 0.5);
}

void backupManualLaunchState::Exit()
{
	m_genState->Exit();
}

bool backupManualLaunchState::AtTarget()
{
	auto attarget = m_genState->AtTarget();
	return attarget;
}

bool backupManualLaunchState::IsTransitionCondition(bool considerGamepadTransitions)
{
	int currentState = m_mechanism->GetCurrentState();

	return ((considerGamepadTransitions && TeleopControl::GetInstance()->IsButtonPressed(TeleopControlFunctions::MANUAL_MODE) && m_mechanism->isLauncherMode()) ||
			(currentState == static_cast<int>(m_mechanism->STATE_BACKUP_MANUAL_PLACE) && m_mechanism->isLauncherMode()));
}
