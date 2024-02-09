
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
// This file was automatically generated by the Team 302 code generator version 1.2.3.1
// Generated on Monday, February 5, 2024 10:04:45 PM

// C++ Includes

// FRC Includes

// Team 302 includes
#include "PeriodicLooper.h"
#include "mechanisms/ClimberManager/decoratormods/ClimberManager.h"

#include "hw/DragonSparkMax.h"
#include "mechanisms/ClimberManager/decoratormods/OffState.h"
#include "mechanisms/ClimberManager/decoratormods/InitializeState.h"
#include "mechanisms/ClimberManager/decoratormods/ManualState.h"
#include "mechanisms/ClimberManager/decoratormods/autoClimbState.h"
#include "mechanisms/ClimberManager/decoratormods/HoldState.h"

#include "robotstate/RobotState.h"
#include "utils/logging/Logger.h"

#include "utils/logging/DataTrace.h"

using std::string;
using namespace ClimberManagerStates;

/// @brief  This method constructs the mechanism using composition with its various actuators and sensors.
/// @param controlFileName The control file with the PID constants and Targets for each state
/// @param networkTableName Location for logging information
/// @param motor  Motor in the mechanims - code generator should probably use the usage for the variable name
/// @param otherMotor Same as previous
/// @param solenoid Solenoid in the mechanism - code generator should probably use the usage for the variable name
/// Additional actuators and sensors are also in this list.
ClimberManager::ClimberManager(ClimberManagerGen *base) : ClimberManagerGen(), IRobotStateChangeSubscriber(),
														  m_ClimberManager(base)
{
	PeriodicLooper::GetInstance()->RegisterAll(this);

	m_climbMode = RobotStateChanges::ClimbMode::ClimbModeOff;
	m_gamePeriod = RobotStateChanges::GamePeriod::Disabled;

	RobotState *RobotStates = RobotState::GetInstance();

	RobotStates->RegisterForStateChanges(this, RobotStateChanges::StateChange::ClimbModeStatus);
	RobotStates->RegisterForStateChanges(this, RobotStateChanges::StateChange::GameState);
}

void ClimberManager::RunCommonTasks()
{
	// This function is called once per loop before the current state Run()
	Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("State Transition"), string("Current State"), GetCurrentStatePtr()->GetStateName());
	Cyclic();
#ifdef INCLUDE_DATA_TRACE
	double left = getleftClimber()->GetCounts();
	double right = getleftClimber()->GetCounts();
	DataTrace::GetInstance()->sendArmData(left, right);
#endif
}

void ClimberManager::SetCurrentState(int state, bool run)
{
	ClimberManagerGen::SetCurrentState(state, run);
}

void ClimberManager::CreateAndRegisterStates()
{
	OffState *OffStateInst = new OffState(string("Off"), 0, new ClimberManagerAllStatesStateGen(string("Off"), 0, this), this);
	AddToStateVector(OffStateInst);

	InitializeState *InitializeStateInst = new InitializeState(string("Initialize"), 1, new ClimberManagerAllStatesStateGen(string("Initialize"), 1, this), this);
	AddToStateVector(InitializeStateInst);

	ManualState *ManualStateInst = new ManualState(string("Manual"), 2, new ClimberManagerAllStatesStateGen(string("Manual"), 2, this), this);
	AddToStateVector(ManualStateInst);

	autoClimbState *autoClimbStateInst = new autoClimbState(string("autoClimb"), 3, new ClimberManagerAllStatesStateGen(string("autoClimb"), 3, this), this);
	AddToStateVector(autoClimbStateInst);

	HoldState *HoldStateInst = new HoldState(string("Hold"), 4, new ClimberManagerAllStatesStateGen(string("Hold"), 4, this), this);
	AddToStateVector(HoldStateInst);

	OffStateInst->RegisterTransitionState(InitializeStateInst);
	InitializeStateInst->RegisterTransitionState(HoldStateInst);
	ManualStateInst->RegisterTransitionState(HoldStateInst);
	ManualStateInst->RegisterTransitionState(autoClimbStateInst);
	autoClimbStateInst->RegisterTransitionState(HoldStateInst);
	autoClimbStateInst->RegisterTransitionState(ManualStateInst);
	HoldStateInst->RegisterTransitionState(ManualStateInst);
}

void ClimberManager::Update(RobotStateChanges::StateChange change, int value)
{
	if (change == RobotStateChanges::ClimbModeStatus)
		m_climbMode = static_cast<RobotStateChanges::ClimbMode>(value);
	else if (change == RobotStateChanges::GameState)
		m_gamePeriod = static_cast<RobotStateChanges::GamePeriod>(value);
}
