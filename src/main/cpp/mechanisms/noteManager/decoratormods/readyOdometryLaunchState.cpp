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
#include "chassis/DragonDriveTargetFinder.h"
#include "mechanisms/noteManager/decoratormods/readyOdometryLaunchState.h"
#include "teleopcontrol/TeleopControl.h"
#include "teleopcontrol/TeleopControlFunctions.h"
#include "utils/logging/Logger.h"

// Third Party Includes

using namespace std;
using namespace noteManagerStates;

/// @class ExampleForwardState
/// @brief information about the control (open loop, closed loop position, closed loop velocity, etc.) for a mechanism state
readyOdometryLaunchState::readyOdometryLaunchState(std::string stateName,
												   int stateId,
												   noteManagerAllStatesStateGen *generatedState,
												   noteManager *mech) : State(stateName, stateId), m_genState(generatedState), m_mechanism(mech)
{
}

void readyOdometryLaunchState::Init()
{
	Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("readyOdometryLaunchState"), string("init"));

	m_genState->Init();
}

void readyOdometryLaunchState::Run()
{
	m_genState->Run();

	m_mechanism->SetLauncherTargetsForAutoLaunch(DragonDriveTargetFinder::FINDER_OPTION::FUSE_IF_POSSIBLE);
}

void readyOdometryLaunchState::Exit()
{
	m_genState->Exit();
}

bool readyOdometryLaunchState::AtTarget()
{
	auto attarget = m_genState->AtTarget();
	return attarget;
}

bool readyOdometryLaunchState::IsTransitionCondition(bool considerGamepadTransitions)
{
	// To get the current state use m_mechanism->GetCurrentState()
	auto currentState = static_cast<noteManagerGen::STATE_NAMES>(m_mechanism->GetCurrentState());

	units::length::meter_t distanceFromSpeaker = m_mechanism->GetDistanceFromSpeaker(DragonDriveTargetFinder::FINDER_OPTION::ODOMETRY_ONLY);
	return ((distanceFromSpeaker < m_odometryLaunchThreshold) && (currentState == m_mechanism->STATE_HOLD_FEEDER));
}

std::string readyOdometryLaunchState::GetNoteStateName() const
{
	return std::string("ReadyOdometryLaunchState");
}
