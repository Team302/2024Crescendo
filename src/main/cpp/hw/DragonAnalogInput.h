
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
#include <frc/AnalogInput.h>

class DragonAnalogInput : frc::AnalogInput
{
public:
	enum ANALOG_SENSOR_TYPE
	{
		UNKNOWN_ANALOG_TYPE = -1,
		ANALOG_GENERAL,
		ANALOG_GYRO,
		POTENTIOMETER,
		PRESSURE_GAUGE,
		ELEVATOR_HEIGHT,
		MAX_ANALOG_TYPES
	};

	DragonAnalogInput(
		std::string networkTableName,
		ANALOG_SENSOR_TYPE type,
		int analogID,
		float voltageMin,
		float voltageMax,
		float outputMin,
		float outputMax);
	virtual ~DragonAnalogInput();
	float GetInterpolatedValue() const;
	ANALOG_SENSOR_TYPE GetType() const { return m_type; }

private:
	std::string m_networkTableName;
	ANALOG_SENSOR_TYPE m_type;
	float m_voltMin;
	float m_voltMax;
	float m_outMin;
	float m_outMax;
};
