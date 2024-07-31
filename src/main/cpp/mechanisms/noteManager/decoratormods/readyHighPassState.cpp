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
// This file was automatically generated by the Team 302 code generator version 1.3.0.15
// Generated on Tuesday, July 30, 2024 4:43:04 PM

// C++ Includes
#include <string>

// FRC includes

// Team 302 includes
#include "mechanisms/noteManager/decoratormods/readyHighPassState.h"
#include "teleopcontrol/TeleopControl.h"
#include "teleopcontrol/TeleopControlFunctions.h"
#include "utils/logging/Logger.h"

// Third Party Includes

using namespace std;
using namespace noteManagerStates;

/// @class ExampleForwardState
/// @brief information about the control (open loop, closed loop position, closed loop velocity, etc.) for a mechanism state
readyHighPassState::readyHighPassState(std::string stateName,
									   int stateId,
									   noteManagerAllStatesStateGen *generatedState,
									   noteManager *mech) : State(stateName, stateId), m_genState(generatedState), m_mechanism(mech)
{
}

void readyHighPassState::Init()
{
	Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("readyHighPassState"), string("init"));
	m_genState->Init();
}

void readyHighPassState::Run()
{
	m_mechanism->SetLauncherAngleTarget(units::angle::degree_t(m_targetAngle));
	units::angular_velocity::radians_per_second_t targetSpeed = m_mechanism->getlauncherTargetSpeed();
	m_mechanism->UpdateTarget(RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_TOP, units::angular_velocity::revolutions_per_minute_t(targetSpeed));
	m_mechanism->UpdateTarget(RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_BOTTOM, units::angular_velocity::revolutions_per_minute_t(targetSpeed));

	m_genState->Run();
}

void readyHighPassState::Exit()
{
	m_genState->Exit();
}

bool readyHighPassState::AtTarget()
{
	auto attarget = m_genState->AtTarget();
	return attarget;
}

bool readyHighPassState::IsTransitionCondition(bool considerGamepadTransitions)
{
	// To get the current state use m_mechanism->GetCurrentState()
	units::length::meter_t distanceFromSpeaker = m_mechanism->GetDistanceFromSpeaker(DragonDriveTargetFinder::FINDER_OPTION::FUSE_IF_POSSIBLE);

	return (considerGamepadTransitions && distanceFromSpeaker >= m_passLaunchThreshold);
}
