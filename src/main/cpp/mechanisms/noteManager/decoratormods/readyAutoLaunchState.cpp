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
#include "mechanisms/noteManager/decoratormods/readyAutoLaunchState.h"
#include "teleopcontrol/TeleopControl.h"
#include "teleopcontrol/TeleopControlFunctions.h"
#include "utils/logging/Logger.h"

// Third Party Includes

using namespace std;
using namespace noteManagerStates;

/// @class ExampleForwardState
/// @brief information about the control (open loop, closed loop position, closed loop velocity, etc.) for a mechanism state
readyAutoLaunchState::readyAutoLaunchState(std::string stateName,
										   int stateId,
										   noteManagerAllStatesStateGen *generatedState,
										   noteManager *mech) : State(stateName, stateId), m_genState(generatedState), m_mechanism(mech)
{
}

void readyAutoLaunchState::Init()
{
	Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("readyAutoLaunchState"), string("init"));
	m_genState->Init();
}

void readyAutoLaunchState::Run()
{
	// Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("readyAutoLaunchState"), string("run"));
	// Adding ability to control launcher Angel temporarily for tuning
	if (abs(TeleopControl::GetInstance()->GetAxisValue(TeleopControlFunctions::LAUNCH_ANGLE)) > 0.05) // Allows manual cotrol of the elevator if you need to adujst
	{
		double delta = 3.0 * 0.05 * (TeleopControl::GetInstance()->GetAxisValue(TeleopControlFunctions::LAUNCH_ANGLE)); // changing by 6 deg/s * 0.05 for 20 ms loop time * controller input
		m_target += delta;
	}
	// m_target = m_mechanism->GetRequiredLaunchAngle();
	m_mechanism->UpdateTarget(RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_ANGLE, m_target);
	Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Launcher"), string("Angle"), m_mechanism->getlauncherAngle()->GetCounts());
	Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Launcher"), string("Target"), m_target);
	Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Launcher"), string("Distance"), m_mechanism->GetVisionDistance().to<double>());

	m_genState->Run();
}

void readyAutoLaunchState::Exit()
{
	m_genState->Exit();
}

bool readyAutoLaunchState::AtTarget()
{
	auto attarget = m_genState->AtTarget();
	return attarget;
}

bool readyAutoLaunchState::IsTransitionCondition(bool considerGamepadTransitions)
{
	// To get the current state use m_mechanism->GetCurrentState()

	bool visionTargetAcquired = m_mechanism->HasVisionTarget(); // todo this will be set with std::optional<VisionData> optionalvisionData = m_vision->GetVisionData(DragonVision::VISION_ELEMENT::SPEAKER);
	return (visionTargetAcquired);
}
