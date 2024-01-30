// clang-format off
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
// Generated on Sunday, January 28, 2024 10:39:19 AM

#pragma once

// C++ Includes

// FRC Includes

// Team 302 includes
#include "mechanisms/noteManager/generated/noteManager_gen.h"
#include "mechanisms/base/StateMgr.h"

// forward declares

class noteManager : public noteManager_gen
{
public:
	/// @brief  This method constructs the mechanism using composition with its various actuators and sensors.
	/// @param controlFileName The control file with the PID constants and Targets for each state
	/// @param networkTableName Location for logging information
	/// @param motor  Motor in the mechanims - code generator should probably use the usage for the variable name
	/// @param otherMotor Same as previous
	/// @param solenoid Solenoid in the mechanism - code generator should probably use the usage for the variable name
	/// Additional actuators and sensors are also in this list.
	noteManager ( noteManager_gen *generatedMech );
	noteManager() = delete;
	~noteManager() = default;

	void createAndRegisterStates();

	// todo not sure what to do with these
	/*
	bool IsAtMinPosition(RobotElementNames::ROBOT_ELEMENT_NAMES identifier) const override;
	bool IsAtMinPosition(RobotElementNames::ROBOT_ELEMENT_NAMES identifier) const override;
	bool IsAtMaxPosition(RobotElementNames::ROBOT_ELEMENT_NAMES identifier) const override;
	bool IsAtMaxPosition(RobotElementNames::ROBOT_ELEMENT_NAMES identifier) const override;
	*/

private:
	noteManager_gen *m_noteManager;
};
