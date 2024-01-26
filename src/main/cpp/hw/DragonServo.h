
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

#pragma once

#include <string>
#include <vector>

#include "units/angle.h"
#include <configs/RobotElementNames.h>

#include <frc/Servo.h>

class DragonServo
{
public:
	DragonServo() = delete;

	//------------------------------------------------------------------------------
	// Method:		<<constructor>>
	// Description:	Create Servos for use in robot mechanisms
	//------------------------------------------------------------------------------
	DragonServo(
		RobotElementNames::SERVO_USAGE deviceUsage, // <I> - Usage of the servo
		int deviceID,								// <I> - PWM ID
		units::angle::degree_t minAngle,			// <I> - Minimun desired angle
		units::angle::degree_t maxAngle				// <I> - Maximum desired angle
	);

	virtual ~DragonServo() = default;

	void Set(double value);
	void SetOffline();
	double Get() const;
	void SetAngle(units::angle::degree_t angle);
	units::angle::degree_t GetAngle() const;

	RobotElementNames::SERVO_USAGE GetUsage() const;
	void MoveToMaxAngle();
	void MoveToMinAngle();

private:
	RobotElementNames::SERVO_USAGE m_usage;
	frc::Servo *m_servo;
	units::angle::degree_t m_minAngle;
	units::angle::degree_t m_maxAngle;
};
