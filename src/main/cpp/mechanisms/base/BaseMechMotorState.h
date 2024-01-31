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
#include <string>

#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/length.h"
#include "units/velocity.h"

#include "State.h"
#include "mechanisms/controllers/ControlData.h"
#include "mechanisms/base/BaseMechMotor.h"

class BaseMechMotorState : public State
{
public:
    BaseMechMotorState(std::string stateName,
                       int stateId,
                       BaseMechMotor &mech);
    BaseMechMotorState() = delete;
    ~BaseMechMotorState() = default;

    /// @brief Set the target value for the actuator
    /// @param identifier Motor Control Usage to indicate what motor to update
    /// @param percentOutput target value
    void SetTargetControl(double percentOutput);

    /// @brief Set the target value for the actuator
    /// @param identifier Motor Control Usage to indicate what motor to update
    /// @param controlConst pid constants for controling motor
    /// @param angle target value
    void SetTargetControl(ControlData &controlConst, units::angle::degree_t angle);

    /// @brief Set the target value for the actuator
    /// @param identifier Motor Control Usage to indicate what motor to update
    /// @param controlConst pid constants for controling motor
    /// @param angularVelocity target value
    void SetTargetControl(ControlData &controlConst, units::angular_velocity::revolutions_per_minute_t angVel);

    /// @brief Set the target value for the actuator
    /// @param identifier Motor Control Usage to indicate what motor to update
    /// @param controlConst pid constants for controling motor
    /// @param position target value
    void SetTargetControl(ControlData &controlConst, units::length::inch_t position);

    /// @brief Set the target value for the actuator
    /// @param identifier Motor Control Usage to indicate what motor to update
    /// @param controlConst pid constants for controling motor
    /// @param velocity target value
    void SetTargetControl(ControlData &controlConst, units::velocity::feet_per_second_t velocity);

    void Init() override;
    void Run() override;
    void Exit() override;
    bool AtTarget() override;

private:
    enum motorMode
    {
        PERCENT,
        ANGLE,
        ANGULAR_VELOCITY,
        POSITION,
        VELOCITY
    };
    BaseMechMotor m_mech;
    motorMode m_mode;
    ControlData m_control;
    double m_target;
    units::angle::degree_t m_targetAngle;
    units::angular_velocity::revolutions_per_minute_t m_targetAngularVelocity;
    units::length::inch_t m_targetPosition;
    units::velocity::feet_per_second_t m_targetVelocity;
};
