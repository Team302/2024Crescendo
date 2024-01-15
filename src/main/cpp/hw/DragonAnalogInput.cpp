
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

#include <string>

#include "hw/DragonAnalogInput.h"

using namespace std;

DragonAnalogInput::DragonAnalogInput(string networkTableName,
									 ANALOG_SENSOR_TYPE type,
									 int analogID,
									 float voltageMin,
									 float voltageMax,
									 float outputMin,
									 float outputMax) : AnalogInput(analogID),
														m_networkTableName(networkTableName),
														m_type(type),
														m_voltMin(voltageMin),
														m_voltMax(voltageMax),
														m_outMin(outputMin),
														m_outMax(outputMax)
{
}

DragonAnalogInput::~DragonAnalogInput()
{
}

float DragonAnalogInput::GetInterpolatedValue() const
{
	float output = 0.0;
	float volts = GetVoltage();
	output = (volts / (m_voltMax - m_voltMin)) * (m_outMax - m_outMin) + m_outMin;
	return output;
}