
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

#include "auton/AutonGrid.h"
#include "auton/ZoneParams.h"
#include "chassis/ChassisOptionEnums.h"
#include "mechanisms/noteManager/generated/noteManager_gen.h"

// @ADDMECH include for your mechanism state mgr

// @ADDMECH mechanism state for mech as parameter
ZoneParams::ZoneParams(int xgrid1,
					   int ygrid1,
					   int xgrid2,
					   int ygrid2,
					   noteManager_gen::STATE_NAMES,
					   ChassisOptionEnums::AutonChassisOptions,
					   ChassisOptionEnums::AutonAvoidOptions)
// @ADDMECH initilize state mgr attribute
{
}
