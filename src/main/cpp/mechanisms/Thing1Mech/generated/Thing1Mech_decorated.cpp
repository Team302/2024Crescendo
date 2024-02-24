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
// This file was automatically generated by the Team 302 code generator version 1.3.0.9
// Generated on Friday, February 23, 2024 8:58:57 PM

// C++ Includes

// FRC Includes

// Team 302 includes
#include "mechanisms/Thing1Mech/decoratormods/Thing1Mech.h"

#include "hw/DragonTalonSRX.h"
#include "hw/DragonTalonFX.h"
#include "hw/DragonSparkMax.h"
#include "hw/DragonSparkFlex.h"
#include "mechanisms/Thing1Mech/decoratormods/leftFrontCWState.h"
#include "mechanisms/Thing1Mech/decoratormods/rightFrontCWState.h"
#include "mechanisms/Thing1Mech/decoratormods/rightBackCWState.h"
#include "mechanisms/Thing1Mech/decoratormods/leftBackCWState.h"
#include "mechanisms/Thing1Mech/decoratormods/sparkyOnState.h"
#include "mechanisms/Thing1Mech/decoratormods/thing1TalonState.h"

using std::string;
using namespace Thing1MechStates;

void Thing1Mech::CreateAndRegisterStates()
{
	leftFrontCWState* leftFrontCWStateInst = new leftFrontCWState ( string ( "leftFrontCW" ), 0, new Thing1MechAllStatesStateGen ( m_activeRobotId, string ( "leftFrontCW" ), 0, this ), this );
	AddToStateVector ( leftFrontCWStateInst );

	rightFrontCWState* rightFrontCWStateInst = new rightFrontCWState ( string ( "rightFrontCW" ), 1, new Thing1MechAllStatesStateGen ( m_activeRobotId, string ( "rightFrontCW" ), 1, this ), this );
	AddToStateVector ( rightFrontCWStateInst );

	rightBackCWState* rightBackCWStateInst = new rightBackCWState ( string ( "rightBackCW" ), 2, new Thing1MechAllStatesStateGen ( m_activeRobotId, string ( "rightBackCW" ), 2, this ), this );
	AddToStateVector ( rightBackCWStateInst );

	leftBackCWState* leftBackCWStateInst = new leftBackCWState ( string ( "leftBackCW" ), 3, new Thing1MechAllStatesStateGen ( m_activeRobotId, string ( "leftBackCW" ), 3, this ), this );
	AddToStateVector ( leftBackCWStateInst );

	sparkyOnState* sparkyOnStateInst = new sparkyOnState ( string ( "sparkyOn" ), 4, new Thing1MechAllStatesStateGen ( m_activeRobotId, string ( "sparkyOn" ), 4, this ), this );
	AddToStateVector ( sparkyOnStateInst );

	thing1TalonState* thing1TalonStateInst = new thing1TalonState ( string ( "thing1Talon" ), 5, new Thing1MechAllStatesStateGen ( m_activeRobotId, string ( "thing1Talon" ), 5, this ), this );
	AddToStateVector ( thing1TalonStateInst );

	leftFrontCWStateInst->RegisterTransitionState ( rightFrontCWStateInst );
	rightFrontCWStateInst->RegisterTransitionState ( leftBackCWStateInst );
	rightFrontCWStateInst->RegisterTransitionState ( rightBackCWStateInst );
	rightBackCWStateInst->RegisterTransitionState ( leftBackCWStateInst );
	rightBackCWStateInst->RegisterTransitionState ( thing1TalonStateInst );
	leftBackCWStateInst->RegisterTransitionState ( leftFrontCWStateInst );
	leftBackCWStateInst->RegisterTransitionState ( sparkyOnStateInst );
	sparkyOnStateInst->RegisterTransitionState ( leftBackCWStateInst );
	thing1TalonStateInst->RegisterTransitionState ( rightBackCWStateInst );
}
