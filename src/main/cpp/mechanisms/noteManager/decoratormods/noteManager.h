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
// This file was automatically generated by the Team 302 code generator version 1.3.0.7
// Generated on Tuesday, February 20, 2024 9:06:50 PM

#pragma once

// C++ Includes
#include <string>

// FRC Includes

// Team 302 includes
#include "mechanisms/noteManager/generated/noteManagerGen.h"
#include "mechanisms/base/StateMgr.h"
#include "robotstate/IRobotStateChangeSubscriber.h"
#include "robotstate/RobotStateChanges.h"

// forward declares

class noteManager : public noteManagerGen, public IRobotStateChangeSubscriber
{
public:
	/// @brief  This method constructs the mechanism using composition with its various actuators and sensors.
	/// @param controlFileName The control file with the PID constants and Targets for each state
	/// @param networkTableName Location for logging information
	/// @param motor  Motor in the mechanims - code generator should probably use the usage for the variable name
	/// @param otherMotor Same as previous
	/// @param solenoid Solenoid in the mechanism - code generator should probably use the usage for the variable name
	/// Additional actuators and sensors are also in this list.
	noteManager(noteManagerGen *generatedMech, RobotConfigMgr::RobotIdentifier activeRobotId);
	noteManager() = delete;
	~noteManager() = default;

	void RunCommonTasks() override;
	void SetCurrentState(int state, bool run) override;
	void CreateAndRegisterStates();

	void ResetElevator();
	void ResetLauncherAngle();
	units::length::meter_t GetVisionDistance();
	bool HasVisionTarget();

	bool IsLauncherMode() const { return m_scoringMode == RobotStateChanges::ScoringMode::Launcher; }
	bool IsPlacerMode() const { return m_scoringMode == RobotStateChanges::ScoringMode::Placer; }
	bool IsClimbMode() const { return m_climbMode == RobotStateChanges::ClimbMode::ClimbModeOn; }
	bool IsEnabled() const { return m_gamePeriod != RobotStateChanges::GamePeriod::Disabled; }

	void Update(RobotStateChanges::StateChange change, int value) override;
	double GetRequiredLaunchAngle();
	bool autoLaunchReady();

private:
	noteManagerGen *m_noteManager;
	RobotStateChanges::ScoringMode m_scoringMode;
	RobotStateChanges::ClimbMode m_climbMode;
	RobotStateChanges::GamePeriod m_gamePeriod;
};
