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
// This file was automatically generated by the Team 302 code generator version 1.2.3.0
// Generated on Saturday, February 3, 2024 3:32:07 PM

// C++ Includes
#include <string>

// FRC includes

// Team 302 includes
#include "mechanisms/Thing1Mech/generated/Thing1MechGen.h"
#include "mechanisms/Thing1Mech/generated/Thing1MechAllStatesStateGen.h"
#include "mechanisms/base/BaseMech.h"
#include "utils/logging/Logger.h"

// Third Party Includes

using std::string;
using namespace Thing1MechStates;

/// @class Thing1MechAllStatesStateGen
/// @brief information about the control (open loop, closed loop position, closed loop velocity, etc.) for a mechanism state
Thing1MechAllStatesStateGen::Thing1MechAllStatesStateGen ( string stateName,
        int stateId,
        Thing1MechGen *mech ) : Thing1MechBaseStateGen ( stateName, stateId, mech )
{
}

void Thing1MechAllStatesStateGen::Init()
{
	if ( false ) {}
	else if ( GetThing1Mech()->GetCurrentState() == Thing1MechGen::STATE_NAMES::STATE_LEFT_FRONT_CW )
	{
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::THING1MECH_BACK_LEFT_MOTOR, 0 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::THING1MECH_RIGHT_BACK_MOTOR, 0 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::THING1MECH_FLACON, 0 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::THING1MECH_NEO550, 0 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::THING1MECH_VORTEX, 0 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::THING1MECH_LEFT_FRONT_MOTOR, 0.5 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::THING1MECH_RIGHT_FRONT_MOTOR, 0 );

	}
	else if ( GetThing1Mech()->GetCurrentState() == Thing1MechGen::STATE_NAMES::STATE_RIGHT_FRONT_CW )
	{
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::THING1MECH_BACK_LEFT_MOTOR, 0 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::THING1MECH_RIGHT_BACK_MOTOR, 0 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::THING1MECH_FLACON, 0 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::THING1MECH_NEO550, 0 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::THING1MECH_VORTEX, 0 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::THING1MECH_LEFT_FRONT_MOTOR, 0 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::THING1MECH_RIGHT_FRONT_MOTOR, 0.5 );

	}
	else if ( GetThing1Mech()->GetCurrentState() == Thing1MechGen::STATE_NAMES::STATE_RIGHT_BACK_CW )
	{
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::THING1MECH_BACK_LEFT_MOTOR, 0 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::THING1MECH_RIGHT_BACK_MOTOR, 0.5 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::THING1MECH_FLACON, 0 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::THING1MECH_NEO550, 0 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::THING1MECH_VORTEX, 0 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::THING1MECH_LEFT_FRONT_MOTOR, 0 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::THING1MECH_RIGHT_FRONT_MOTOR, 0 );

	}
	else if ( GetThing1Mech()->GetCurrentState() == Thing1MechGen::STATE_NAMES::STATE_LEFT_BACK_CW )
	{
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::THING1MECH_BACK_LEFT_MOTOR, 0.5 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::THING1MECH_RIGHT_BACK_MOTOR, 0 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::THING1MECH_FLACON, 0 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::THING1MECH_NEO550, 0 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::THING1MECH_VORTEX, 0 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::THING1MECH_LEFT_FRONT_MOTOR, 0 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::THING1MECH_RIGHT_FRONT_MOTOR, 0 );

	}
	else if ( GetThing1Mech()->GetCurrentState() == Thing1MechGen::STATE_NAMES::STATE_SPARKY_ON )
	{
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::THING1MECH_BACK_LEFT_MOTOR, 0 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::THING1MECH_RIGHT_BACK_MOTOR, 0 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::THING1MECH_FLACON, 0 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::THING1MECH_NEO550, 0.25 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::THING1MECH_VORTEX, 0.25 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::THING1MECH_LEFT_FRONT_MOTOR, 0 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::THING1MECH_RIGHT_FRONT_MOTOR, 0 );

	}
	else if ( GetThing1Mech()->GetCurrentState() == Thing1MechGen::STATE_NAMES::STATE_THING1TALON )
	{
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::THING1MECH_BACK_LEFT_MOTOR, 0 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::THING1MECH_RIGHT_BACK_MOTOR, 0 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::THING1MECH_FLACON, 0.25 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::THING1MECH_NEO550, 0 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::THING1MECH_VORTEX, 0 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::THING1MECH_LEFT_FRONT_MOTOR, 0 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::THING1MECH_RIGHT_FRONT_MOTOR, 0 );

	}


	Logger::GetLogger()->LogData ( LOGGER_LEVEL::PRINT, string ( "ArrivedAt" ), string ( "Thing1MechAllStatesStateGen" ), string ( "init" ) );

	Thing1MechBaseStateGen::Init();
}

void Thing1MechAllStatesStateGen::Run()
{
	// Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("Thing1MechAllStatesStateGen"), string("run"));
	Thing1MechBaseStateGen::Run();
}

void Thing1MechAllStatesStateGen::Exit()
{
	Thing1MechBaseStateGen::Exit();
}

bool Thing1MechAllStatesStateGen::AtTarget()
{
	return Thing1MechBaseStateGen::AtTarget();
}
