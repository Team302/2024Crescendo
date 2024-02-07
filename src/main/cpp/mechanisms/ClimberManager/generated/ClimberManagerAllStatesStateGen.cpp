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
// This file was automatically generated by the Team 302 code generator version 1.2.3.1
// Generated on Tuesday, February 6, 2024 6:30:38 PM

// C++ Includes
#include <string>

// FRC includes

// Team 302 includes
#include "mechanisms/ClimberManager/generated/ClimberManagerGen.h"
#include "mechanisms/ClimberManager/generated/ClimberManagerAllStatesStateGen.h"
#include "mechanisms/base/BaseMech.h"
#include "utils/logging/Logger.h"

// Third Party Includes

using std::string;
using namespace ClimberManagerStates;

/// @class ClimberManagerAllStatesStateGen
/// @brief information about the control (open loop, closed loop position, closed loop velocity, etc.) for a mechanism state
ClimberManagerAllStatesStateGen::ClimberManagerAllStatesStateGen(string stateName,
                                                                                               int stateId,
                                                                                               ClimberManagerGen *mech) : ClimberManagerBaseStateGen(stateName, stateId, mech)
{
}

void ClimberManagerAllStatesStateGen::Init()
{
    if(false) {}
else if( GetClimberManager()->GetCurrentState() == ClimberManagerGen::STATE_NAMES::STATE_OFF)
{
SetTargetControl(RobotElementNames::MOTOR_CONTROLLER_USAGE::CLIMBER_MANAGER_LEFT_CLIMBER, 0);
SetTargetControl(RobotElementNames::MOTOR_CONTROLLER_USAGE::CLIMBER_MANAGER_RIGHT_CLIMBER, 0);
}
else if( GetClimberManager()->GetCurrentState() == ClimberManagerGen::STATE_NAMES::STATE_INITIALIZE)
{
SetTargetControl(RobotElementNames::MOTOR_CONTROLLER_USAGE::CLIMBER_MANAGER_LEFT_CLIMBER, GetClimberManager()->getclimberPosInch(), units::length::inch_t(units::length::inch_t(20.5)));
SetTargetControl(RobotElementNames::MOTOR_CONTROLLER_USAGE::CLIMBER_MANAGER_RIGHT_CLIMBER, GetClimberManager()->getclimberPosInch(), units::length::inch_t(units::length::inch_t(20.5)));
}
else if( GetClimberManager()->GetCurrentState() == ClimberManagerGen::STATE_NAMES::STATE_MANUAL)
{
SetTargetControl(RobotElementNames::MOTOR_CONTROLLER_USAGE::CLIMBER_MANAGER_LEFT_CLIMBER, GetClimberManager()->getclimberPosInch(), units::length::inch_t(units::length::inch_t(20.5)));
SetTargetControl(RobotElementNames::MOTOR_CONTROLLER_USAGE::CLIMBER_MANAGER_RIGHT_CLIMBER, GetClimberManager()->getclimberPosInch(), units::length::inch_t(units::length::inch_t(20.5)));
}
else if( GetClimberManager()->GetCurrentState() == ClimberManagerGen::STATE_NAMES::STATE_AUTO_CLIMB)
{
SetTargetControl(RobotElementNames::MOTOR_CONTROLLER_USAGE::CLIMBER_MANAGER_LEFT_CLIMBER, GetClimberManager()->getclimberPosInch(), units::length::inch_t(units::length::inch_t(28)));
SetTargetControl(RobotElementNames::MOTOR_CONTROLLER_USAGE::CLIMBER_MANAGER_RIGHT_CLIMBER, GetClimberManager()->getclimberPosInch(), units::length::inch_t(units::length::inch_t(28)));
}
else if( GetClimberManager()->GetCurrentState() == ClimberManagerGen::STATE_NAMES::STATE_HOLD)
{
SetTargetControl(RobotElementNames::MOTOR_CONTROLLER_USAGE::CLIMBER_MANAGER_LEFT_CLIMBER, GetClimberManager()->getclimberPosInch(), units::length::inch_t(units::length::meter_t(20.5)));
SetTargetControl(RobotElementNames::MOTOR_CONTROLLER_USAGE::CLIMBER_MANAGER_RIGHT_CLIMBER, GetClimberManager()->getclimberPosInch(), units::length::inch_t(units::length::meter_t(20.5)));
}

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("ClimberManagerAllStatesStateGen"), string("init"));

    ClimberManagerBaseStateGen::Init();
}

void ClimberManagerAllStatesStateGen::Run()
{
    // Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("ClimberManagerAllStatesStateGen"), string("run"));
    ClimberManagerBaseStateGen::Run();
}

void ClimberManagerAllStatesStateGen::Exit()
{
    ClimberManagerBaseStateGen::Exit();
}

bool ClimberManagerAllStatesStateGen::AtTarget()
{
    return ClimberManagerBaseStateGen::AtTarget();
}
