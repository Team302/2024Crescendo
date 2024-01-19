
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

#include <frc/filter/Debouncer.h>
#include <frc/DigitalInput.h>

#include "units/time.h"
#include "hw/DragonDigitalInput.h"
#include <configs/RobotElementNames.h>
#include "utils/logging/Logger.h"

using namespace frc;
using namespace std;

DragonDigitalInput::DragonDigitalInput(
	string networkTableName,
	RobotElementNames::DIGITAL_INPUT_USAGE usage, // <I> - Usage of the digital input
	int deviceID,								  // <I> - digial io ID
	bool reversed,								  // <I>
	units::time::second_t debounceTime) : m_networkTableName(networkTableName),
										  m_digital(new DigitalInput(deviceID)),
										  m_debouncer(new Debouncer(debounceTime, Debouncer::DebounceType::kBoth)),
										  m_reversed(reversed),
										  m_type(usage)
{
}

DragonDigitalInput::~DragonDigitalInput()
{
	delete m_digital;
}

RobotElementNames::DIGITAL_INPUT_USAGE DragonDigitalInput::GetType() const
{
	return m_type;
}

bool DragonDigitalInput::Get() const
{
	if (m_digital != nullptr)
	{
		auto digitReading = m_digital->Get();
		if (m_debouncer != nullptr)
		{
			return (m_reversed) ? m_debouncer->Calculate(!digitReading) : m_debouncer->Calculate(digitReading);
		}
		else
		{
			return (m_reversed) ? !digitReading : digitReading;
		}
	}
	else
	{
		Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, m_networkTableName, string("DigitalInput"), string("Not created"));
	}
	return false;
}
int DragonDigitalInput::GetChannel() const
{
	int channel = 0;
	if (m_digital != nullptr)
	{
		channel = m_digital->GetChannel();
	}
	return channel;
}
