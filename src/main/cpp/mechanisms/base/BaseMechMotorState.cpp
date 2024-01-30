
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

// Team 302 includes
#include "State.h"
#include "mechanisms/base/BaseMechMotor.h"
#include "mechanisms/base/BaseMechMotorState.h"
#include "mechanisms/controllers/ControlData.h"
#include "mechanisms/base/BaseMech.h"
#include "utils/logging/Logger.h"

#include "teleopcontrol/TeleopControl.h"

// Third Party Includes

using namespace std;

/// @class BaseMechMotorState
/// @brief information about the control (open loop, closed loop position, closed loop velocity, etc.) for a mechanism state
BaseMechMotorState::BaseMechMotorState(string stateName,
                                       int stateId,
                                       BaseMechMotor &mech) : State(stateName, stateId),
                                                              m_mech(mech),
                                                              m_mode(motorMode::PERCENT),
                                                              m_control(),
                                                              m_target(0.0),
                                                              m_targetAngle(units::angle::degree_t(0.0)),
                                                              m_targetAngularVelocity(units::angular_velocity::revolutions_per_minute_t(0.0)),
                                                              m_targetPosition(units::length::inch_t(0.0)),
                                                              m_targetVelocity(units::velocity::feet_per_second_t(0.0))
{
}

/// @brief Set the target value for the actuator
/// @param identifier Motor Control Usage to indicate what motor to update
/// @param percentOutput target value
void BaseMechMotorState::SetTargetControl(double percentOutput)
{
    m_mode = motorMode::PERCENT;
    m_target = percentOutput;
}

/// @brief Set the target value for the actuator
/// @param identifier Motor Control Usage to indicate what motor to update
/// @param controlConst pid constants for controling motor
/// @param angle target value
void BaseMechMotorState::SetTargetControl(ControlData &controlConst, units::angle::degree_t angle)
{
    m_mode = motorMode::ANGLE;
    m_targetAngle = angle;
}

/// @brief Set the target value for the actuator
/// @param identifier Motor Control Usage to indicate what motor to update
/// @param controlConst pid constants for controling motor
/// @param angularVelocity target value
void BaseMechMotorState::SetTargetControl(ControlData &controlConst, units::angular_velocity::revolutions_per_minute_t angVel)
{
    m_mode = motorMode::ANGULAR_VELOCITY;
    m_targetAngularVelocity = angVel;
}

/// @brief Set the target value for the actuator
/// @param identifier Motor Control Usage to indicate what motor to update
/// @param controlConst pid constants for controling motor
/// @param position target value
void BaseMechMotorState::SetTargetControl(ControlData &controlConst, units::length::inch_t position)
{
    m_mode = motorMode::POSITION;
    m_targetPosition = position;
}

/// @brief Set the target value for the actuator
/// @param identifier Motor Control Usage to indicate what motor to update
/// @param controlConst pid constants for controling motor
/// @param velocity target value
void BaseMechMotorState::SetTargetControl(ControlData &controlConst, units::velocity::feet_per_second_t velocity)
{
    m_mode = motorMode::VELOCITY;
    m_targetVelocity = velocity;
}

void BaseMechMotorState::Init()
{
    m_mech.SetControlConstants(0, m_control);
    if (m_mode == motorMode::PERCENT)
    {
        m_mech.UpdateTarget(m_target);
    }
    else if (m_mode == motorMode::ANGLE)
    {
        m_mech.UpdateTarget(m_targetAngle);
    }
    else if (m_mode == motorMode::ANGULAR_VELOCITY)
    {
        m_mech.UpdateTarget(m_targetAngularVelocity);
    }
    else if (m_mode == motorMode::POSITION)
    {
        m_mech.UpdateTarget(m_targetPosition);
    }
    else if (m_mode == motorMode::VELOCITY)
    {
        m_mech.UpdateTarget(m_targetVelocity);
    }
}

void BaseMechMotorState::Run()
{
    m_mech.Update();
}

void BaseMechMotorState::Exit()
{ // NO-OP
}

bool BaseMechMotorState::AtTarget()
{
    auto pctError = 0.0;
    if (m_mode == motorMode::PERCENT)
    {
        pctError = (m_target - m_mech.GetTarget()) / m_target;
    }
    else if (m_mode == motorMode::ANGLE)
    {
        pctError = (m_targetAngle - m_mech.GetPositionDegrees()) / m_targetAngle;
    }
    else if (m_mode == motorMode::ANGULAR_VELOCITY)
    {
        pctError = (m_targetAngularVelocity - m_mech.GetRPM()) / m_targetAngularVelocity;
    }
    else if (m_mode == motorMode::POSITION)
    {
        pctError = (m_targetPosition - m_mech.GetPositionInches()) / m_targetPosition;
    }
    else if (m_mode == motorMode::VELOCITY)
    {
        pctError = (m_targetVelocity - m_mech.GetFeetPerSec()) / m_targetVelocity;
    }
    return pctError < 0.02; // TODO: this might need to be configurable
}
