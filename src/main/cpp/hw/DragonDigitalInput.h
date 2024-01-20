
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

#include "configs/RobotElementNames.h"
#include "units/time.h"
namespace frc
{
	class Debouncer;
	class DigitalInput;
}

class DragonDigitalInput
{
public:
	//------------------------------------------------------------------------------
	// Method:		<<constructor>>
	// Description:
	//------------------------------------------------------------------------------
	DragonDigitalInput(
		std::string networkTableName,
		RobotElementNames::DIGITAL_INPUT_USAGE type,
		int deviceID,  // <I> - digial io ID
		bool reversed, // <I>
		units::time::second_t debounceTime);

	DragonDigitalInput() = delete;
	virtual ~DragonDigitalInput();

	bool Get() const;
	int GetChannel() const;
	RobotElementNames::DIGITAL_INPUT_USAGE GetType() const;

private:
	std::string m_networkTableName;
	frc::DigitalInput *m_digital;
	frc::Debouncer *m_debouncer;
	bool m_reversed;
	RobotElementNames::DIGITAL_INPUT_USAGE m_type;
};
