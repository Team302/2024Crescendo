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
#include "mechanisms/noteManager/decoratormods/manualLaunchState.h"
#include "teleopcontrol/TeleopControl.h"
#include "teleopcontrol/TeleopControlFunctions.h"
#include "utils/logging/Logger.h"

// Third Party Includes

using namespace std;
using namespace noteManagerStates;

/// @class ExampleForwardState
/// @brief information about the control (open loop, closed loop position, closed loop velocity, etc.) for a mechanism state
manualLaunchState::manualLaunchState(std::string stateName,
									 int stateId,
									 noteManagerAllStatesStateGen *generatedState,
									 noteManager *mech) : State(stateName, stateId), m_genState(generatedState), m_mechanism(mech)
{
}

void manualLaunchState::Init()
{
	Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("manualLaunchState"), string("init"));

	m_genState->Init();
	m_mechanism->SetLauncherAngleTarget(m_mechanism->GetManualLaunchTarget());
}

void manualLaunchState::Run()
{
	// Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("manualLaunchState"), string("run"));
	m_genState->Run();
}

void manualLaunchState::Exit()
{
	m_genState->Exit();
}

bool manualLaunchState::AtTarget()
{
	bool attarget = false;
	bool angleIsWithinTolerance = units::math::abs(m_mechanism->GetLauncherAngleFromEncoder() - m_mechanism->GetManualLaunchTarget()) <= units::angle::degree_t(0.5);

	double topSpeed = units::angular_velocity::radians_per_second_t(units::angular_velocity::revolutions_per_minute_t(m_mechanism->getlauncherTop()->GetRPS() * 60)).to<double>();
	double botSpeed = units::angular_velocity::radians_per_second_t(units::angular_velocity::revolutions_per_minute_t(m_mechanism->getlauncherBottom()->GetRPS() * 60)).to<double>();

	bool topSpeedIsWithinTolerance = topSpeed > (m_targetSpeed * 0.95);
	bool bottomSpeedIsWithinTolerance = botSpeed > (m_targetSpeed * 0.95);

	if (m_mechanism->getActiveRobotId() == RobotConfigMgr::RobotIdentifier::PRACTICE_BOT_9999)
	{
		// in the practice bot do not check the launcher speed because speed control is not implemented
		topSpeedIsWithinTolerance = true;
		bottomSpeedIsWithinTolerance = true;
		angleIsWithinTolerance = true;
	}

	attarget = angleIsWithinTolerance && topSpeedIsWithinTolerance && bottomSpeedIsWithinTolerance;

	return (attarget);
}

bool manualLaunchState::IsTransitionCondition(bool considerGamepadTransitions)
{
	return (AtTarget());
}
