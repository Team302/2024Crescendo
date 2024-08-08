
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

// C++ Includes
#include <string>

// FRC includes
#include "units/time.h"

// Team 302 includes
#include "auton/PrimitiveFactory.h"
#include "auton/PrimitiveParams.h"
#include "auton/drivePrimitives/DriveHoldPosition.h"
#include "auton/drivePrimitives/IPrimitive.h"
#include "chassis/configs/ChassisConfigMgr.h"
#include "chassis/configs/ChassisConfig.h"
#include "mechanisms/controllers/ControlModes.h"

// Third Party Includes

using namespace std;
using namespace frc;

DriveHoldPosition::DriveHoldPosition() : IPrimitive(),
										 m_chassis(nullptr),
										 m_timeRemaining(units::time::second_t(0.0)) // Value will be changed in init
{
	auto config = ChassisConfigMgr::GetInstance()->GetCurrentConfig();
	m_chassis = config != nullptr ? config->GetSwerveChassis() : nullptr;
}

void DriveHoldPosition::Init(PrimitiveParams *params)
{

	// Get timeRemaining from m_params
	m_timeRemaining = params->GetTime();
	auto cd = make_shared<ControlData>(ControlModes::CONTROL_TYPE::POSITION_INCH,
									   ControlModes::CONTROL_RUN_LOCS::MOTOR_CONTROLLER,
									   string("DriveHoldPosition"),
									   10.0,
									   0.0,
									   0.0,
									   0.0,
									   ControlData::FEEDFORWARD_TYPE::DUTY_CYCLE,
									   0.0,
									   0.0,
									   0.0,
									   1.0,
									   0.0,
									   false);
	// m_chassis->SetControlConstants( cd.get() );
	// auto left = m_chassis->GetCurrentLeftPosition();
	// auto right = m_chassis->GetCurrentRightPosition();

	// m_chassis->SetOutput( ControlModes::CONTROL_TYPE::POSITION_INCH, left, right );
}

void DriveHoldPosition::Run()
{
	// Decrement time remaining
	m_timeRemaining -= IPrimitive::LOOP_LENGTH;
}

bool DriveHoldPosition::IsDone()
{
	// Return true when the time runs out
	bool holdDone = ((m_timeRemaining <= (IPrimitive::LOOP_LENGTH / 2.0)));
	return holdDone;
}
