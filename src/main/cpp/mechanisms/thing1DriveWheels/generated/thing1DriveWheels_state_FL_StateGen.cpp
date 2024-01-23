// clang-format off
//====================================================================================================================================================
// Copyright 2023 Lake Orion Robotics FIRST Team 302
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
// This file was automatically generated by the Team 302 code generator version 1.1.0.0
// Generated on Monday, January 22, 2024 7:49:51 PM

// C++ Includes
#include <string>

// FRC includes

// Team 302 includes
#include "mechanisms/thing1DriveWheels/generated/thing1DriveWheels_gen.h"
#include "mechanisms/thing1DriveWheels/generated/thing1DriveWheels_state_FL_StateGen.h"
#include "mechanisms/base/BaseMech.h"

#include <utils/logging/LoggableItemMgr.h>
#include "utils/logging/Logger.h"
#include <utils/logging/LoggerData.h>
#include <utils/logging/LoggerEnums.h>

// Third Party Includes

using std::string;

/// @class thing1DriveWheelsstate_FLStateGen
/// @brief information about the control (open loop, closed loop position, closed loop velocity, etc.) for a mechanism state
thing1DriveWheelsstate_FLStateGen::thing1DriveWheelsstate_FLStateGen ( string stateName,
        int stateId,
        thing1DriveWheels_gen *mech ) : thing1DriveWheelsBaseStateGen ( stateName, stateId, mech )
{
}

void thing1DriveWheelsstate_FLStateGen::Init()
{
	SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::THING1DRIVE_WHEELS_TALONSRX_FRONTLEFT, 0.5 );
	SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::THING1DRIVE_WHEELS_TALONSRX_FRONTRIGHT, 0 );
	SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::THING1DRIVE_WHEELS_TALONSRX_BACKLEFT, 0 );
	SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::THING1DRIVE_WHEELS_TALONSRX_BACKRIGHT, 0 );
	SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::THING1DRIVE_WHEELS_TALONFX_5, * ( Getthing1DriveWheels()->talonMotorControlData ), units::angular_velocity::revolutions_per_minute_t ( units::angular_velocity::radians_per_second_t ( 0 ) ) );
	SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::THING1DRIVE_WHEELS_SPARK_MAX, 0 );

	Logger::GetLogger()->LogData ( LOGGER_LEVEL::PRINT, string ( "ArrivedAt" ), string ( "thing1DriveWheelsstate_FLStateGen" ), string ( "init" ) );

	thing1DriveWheelsBaseStateGen::Init();
}

void thing1DriveWheelsstate_FLStateGen::Run()
{
	// Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("thing1DriveWheelsstate_FLStateGen"), string("run"));
	thing1DriveWheelsBaseStateGen::Run();
}

void thing1DriveWheelsstate_FLStateGen::Exit()
{
	thing1DriveWheelsBaseStateGen::Exit();
}

bool thing1DriveWheelsstate_FLStateGen::AtTarget()
{
	return thing1DriveWheelsBaseStateGen::AtTarget();
}
