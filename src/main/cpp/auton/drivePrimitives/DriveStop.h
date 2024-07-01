
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

// C++ Includes
#include <memory>

// FRC includes
#include "units/time.h"

// Team 302 includes
#include "auton/drivePrimitives/IPrimitive.h"
#include "chassis/SwerveChassis.h"
#include "mechanisms/noteManager/decoratormods/noteManager.h"

// Third Party Includes

// forward declares
class PrimitiveParams;

namespace frc
{
	class Timer;
}

//========================================================================================================
/// @class  DriveStop
/// @brief  This is an auton primitive that causes the chassis to not drive
//========================================================================================================

class DriveStop : public IPrimitive
{
public:
	/// @brief constructor that creates/initializes the object
	DriveStop();

	/// @brief destructor, clean  up the memory from this object
	virtual ~DriveStop() = default;

	/// @brief initialize this usage of the primitive
	/// @param PrimitiveParms* params the drive parameters
	/// @return void
	void Init(PrimitiveParams *params) override;

	/// @brief run the primitive (periodic routine)
	/// @return void
	void Run() override;

	/// @brief check if the end condition has been met
	/// @return bool true means the end condition was reached, false means it hasn't
	bool IsDone() override;

private:
	units::time::second_t m_maxTime; // Target time
	float m_currentTime;			 // Time since init
	SwerveChassis *m_chassis;
	std::unique_ptr<frc::Timer> m_timer;
	double m_heading;
	noteManager *m_noteManager;
	ChassisOptionEnums::HeadingOption m_headingOption;
};
