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
// This file was automatically generated by the Team 302 code generator version 1.3.0.10
// Generated on Tuesday, February 27, 2024 6:47:29 PM

#pragma once

// C++ Includes
#include <string>

// FRC Includes

// Team 302 includes
#include "mechanisms/Thing1Mech/generated/Thing1MechGen.h"
#include "mechanisms/base/StateMgr.h"
#include "robotstate/IRobotStateChangeSubscriber.h"
#include "robotstate/RobotStateChanges.h"

class Thing1Mech : public Thing1MechGen,
				   public IRobotStateChangeSubscriber
{
public:
	/// @brief  This method constructs the mechanism using composition with its various actuators and sensors.
	/// @param controlFileName The control file with the PID constants and Targets for each state
	/// @param networkTableName Location for logging information
	/// @param motor  Motor in the mechanims - code generator should probably use the usage for the variable name
	/// @param otherMotor Same as previous
	/// @param solenoid Solenoid in the mechanism - code generator should probably use the usage for the variable name
	/// Additional actuators and sensors are also in this list.
	Thing1Mech(Thing1MechGen *generatedMech, RobotConfigMgr::RobotIdentifier activeRobotId);
	Thing1Mech() = delete;
	~Thing1Mech() = default;

	void RunCommonTasks() override;
	void SetCurrentState(int state, bool run) override;
	void CreateAndRegisterStates();

	RobotConfigMgr::RobotIdentifier getActiveRobotId() { return m_activeRobotId; }
	RobotStateChanges::ScoringMode m_scoringMode;

	void Update(RobotStateChanges::StateChange change, int value) override;

private:
	Thing1MechGen *m_Thing1Mech;
};
