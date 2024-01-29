
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

// FRC includes
#include "units/angle.h"

// Team 302 includes
#include "State.h"
#include "mechanisms/base/BaseMechMotor.h"
#include "mechanisms/base/BaseMechServoState.h"
#include "mechanisms/controllers/ControlData.h"
#include "mechanisms/base/BaseMech.h"
#include "utils/logging/Logger.h"

#include "teleopcontrol/TeleopControl.h"

// Third Party Includes

using namespace std;

/// @class BaseMechServoState
/// @brief information about the control (open loop, closed loop position, closed loop velocity, etc.) for a mechanism state
BaseMechServoState::BaseMechServoState(string stateName,
                                       int stateId,
                                       BaseMechServo &mech) : State(stateName, stateId),
                                                              m_mech(mech),
                                                              m_targetAngle(units::angle::degree_t(0.0))
{
}

/// @brief Set the target value for the actuator
/// @param identifier Motor Control Usage to indicate what motor to update
/// @param controlConst pid constants for controling motor
/// @param angle target value
void BaseMechServoState::SetTarget(units::angle::degree_t angle)
{
    m_targetAngle = angle;
}

void BaseMechServoState::Init()
{ // No-Op nothing to initialize
}

void BaseMechServoState::Run()
{
    m_mech.SetAngle(m_targetAngle);
}

void BaseMechServoState::Exit()
{ // No-Op nothing to do on exit
}

bool BaseMechServoState::AtTarget()
{
    return false;
}
