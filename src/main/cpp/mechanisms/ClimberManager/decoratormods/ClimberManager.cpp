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

// FRC Includes

// Team 302 includes
#include "PeriodicLooper.h"
#include "mechanisms/ClimberManager/generated/ClimberManager_gen.h"
#include "mechanisms/ClimberManager/decoratormods/ClimberManager.h"

#include "hw/DragonSparkMax.h"
#include "hw/DragonSparkMax.h"

#include "mechanisms/ClimberManager/decoratormods/ClimberManager_Off_State.h"
#include "mechanisms/ClimberManager/decoratormods/ClimberManager_Initialize_State.h"
#include "mechanisms/ClimberManager/decoratormods/ClimberManager_Manual_State.h"
#include "mechanisms/ClimberManager/decoratormods/ClimberManager_autoClimb_State.h"

#include "robotstate/RobotState.h"

using std::string;

/// @brief  This method constructs the mechanism using composition with its various actuators and sensors.
/// @param controlFileName The control file with the PID constants and Targets for each state
/// @param networkTableName Location for logging information
/// @param motor  Motor in the mechanims - code generator should probably use the usage for the variable name
/// @param otherMotor Same as previous
/// @param solenoid Solenoid in the mechanism - code generator should probably use the usage for the variable name
/// Additional actuators and sensors are also in this list.
ClimberManager::ClimberManager(ClimberManager_gen *base) : ClimberManager_gen(), IRobotStateChangeSubscriber(),
														   m_ClimberManager(base)
{
	m_climbMode = RobotStateChanges::ClimbMode::ClimbModeOff;
	m_gamePeriod = RobotStateChanges::GamePeriod::Disabled;
	RobotState::GetInstance()->RegisterForStateChanges(this, RobotStateChanges::StateChange::ClimbModeStatus);
	RobotState::GetInstance()->RegisterForStateChanges(this, RobotStateChanges::StateChange::GameState);
}

void ClimberManager::createAndRegisterStates()
{
	ClimberManagerOffState *OffState = new ClimberManagerOffState(string("Off"), 0, new ClimberManagerOffStateGen(string("Off"), 0, this), this);
	AddToStateVector(OffState);

	ClimberManagerInitializeState *InitializeState = new ClimberManagerInitializeState(string("Initialize"), 1, new ClimberManagerInitializeStateGen(string("Initialize"), 1, this), this);
	AddToStateVector(InitializeState);

	ClimberManagerManualState *ManualState = new ClimberManagerManualState(string("Manual"), 2, new ClimberManagerManualStateGen(string("Manual"), 2, this), this);
	AddToStateVector(ManualState);

	ClimberManagerautoClimbState *autoClimbState = new ClimberManagerautoClimbState(string("autoClimb"), 3, new ClimberManagerautoClimbStateGen(string("autoClimb"), 3, this), this);
	AddToStateVector(autoClimbState);

	OffState->RegisterTransitionState(InitializeState);
	InitializeState->RegisterTransitionState(OffState);
	InitializeState->RegisterTransitionState(ManualState);
	ManualState->RegisterTransitionState(OffState);
	ManualState->RegisterTransitionState(autoClimbState);
	autoClimbState->RegisterTransitionState(OffState);
	autoClimbState->RegisterTransitionState(ManualState);
}

void ClimberManager::Update(RobotStateChanges::StateChange change, int value)
{
	if (change == RobotStateChanges::ClimbModeStatus)
		m_climbMode = static_cast<RobotStateChanges::ClimbMode>(value);
	if (change == RobotStateChanges::GameState)
		m_gamePeriod = static_cast<RobotStateChanges::GamePeriod>(value);
}

bool ClimberManager::isClimbMode()
{
	return m_climbMode == RobotStateChanges::ClimbMode::ClimbModeOn;
}
bool noteManager::IsEnabled()
{
	return m_gamePeriod != RobotStateChanges::GamePeriod::Disabled;
}
// todo not sure what to do with this
/*
bool ClimberManager::IsAtMinPosition(RobotElementNames::ROBOT_ELEMENT_NAMES identifier) const
{
	return m_ClimberManager->IsAtMinPosition(identifier);
}
bool ClimberManager::IsAtMinPosition(RobotElementNames::ROBOT_ELEMENT_NAMES identifier) const
{
	return m_ClimberManager->IsAtMinPosition(identifier);
}
bool ClimberManager::IsAtMaxPosition(RobotElementNames::ROBOT_ELEMENT_NAMES identifier) const
{
	return m_ClimberManager->IsAtMaxPosition(identifier);
}
bool ClimberManager::IsAtMaxPosition(RobotElementNames::ROBOT_ELEMENT_NAMES identifier) const
{
	return m_ClimberManager->IsAtMaxPosition(identifier);
}
*/
