
//====================================================================================================================================================
/// Copyright 2024 Lake Orion Robotics FIRST Team 302
///
/// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
/// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
/// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
/// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
/// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================

// C++ Includes
#include <memory>
#include <string>
#include <vector>

// FRC includes

// Team 302 includes
#include "State.h"
#include "mechanisms/base/BaseMechMotorState.h"
#include "mechanisms/base/BaseMechServoState.h"
#include "mechanisms/base/BaseMechSolenoidState.h"
#include "mechanisms/example/generated/ExampleGen.h"
#include "mechanisms/example/generated/ExampleMinTravelStateGen.h"
#include "mechanisms/controllers/ControlData.h"
#include "mechanisms/controllers/MechanismTargetData.h"
#include "mechanisms/base/BaseMech.h"
#include "utils/logging/Logger.h"

#include "teleopcontrol/TeleopControl.h"

// Third Party Includes

using namespace std;

/// @class ExampleMinTravelStateGen
/// @brief information about the control (open loop, closed loop position, closed loop velocity, etc.) for a mechanism state
ExampleMinTravelStateGen::ExampleMinTravelStateGen(string stateName,
                                                   int stateId,
                                                   ExampleGen &mech) : ExampleBaseStateGen(stateName, stateId, mech)
{
}

void ExampleMinTravelStateGen::Init()
{
    ExampleBaseStateGen::Init();
}

void ExampleMinTravelStateGen::Run()
{
    ExampleBaseStateGen::Run();
}

void ExampleMinTravelStateGen::Exit()
{
    ExampleBaseStateGen::Exit();
}

bool ExampleMinTravelStateGen::AtTarget()
{
    return ExampleBaseStateGen::AtTarget();
}
