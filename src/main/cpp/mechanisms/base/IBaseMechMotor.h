
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

// C++ Includes
#include <string>

// FRC Includes
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/length.h"
#include "units/velocity.h"

// Team 302 includes
#include "hw/DragonDigitalInput.h"
#include "utils/logging/LoggableItem.h"

// forward declares
class IDragonMotorController;
class ControlData;

class IBaseMechMotor
{
public:
    enum EndOfTravelSensorOption
    {
        NONE,
        DIO_IN_MOTOR_CONTROLLER,
        DIGITAL_INPUT_IN_ROBORIO
    };

    /// @brief A motor that can be added to a mechanism
    /// @param [in] std::string the name of the network table for logging information
    /// @param [in] IDragonMotorController& motor controller used by this mechanism
    /// @param [in] EndOfTravelSensorOption minimum end of travel sensor option
    /// @param [in] DragonDigitalInput* minimium end of travel sensor if plugged into RoboRio otherwise ignored
    /// @param [in] EndOfTravelSensorOption maximum end of travel sensor option
    /// @param [in] DragonDigitalInput* maximum end of travel sensor if plugged into RoboRio otherwise ignored
    IBaseMechMotor() = default;
    ~IBaseMechMotor() = default;

    /// @brief update the output to the mechanism using the current controller and target value(s)
    /// @return void
    virtual void Update() = 0;

    virtual void SetTargetControl(double percentOutput) = 0;
    virtual void SetTargetControl(ControlData *controlConst, units::angle::degree_t angle) = 0;
    virtual void SetTargetControl(ControlData *controlConst, units::angular_velocity::revolutions_per_minute_t angVel) = 0;
    virtual void SetTargetControl(ControlData *controlConst, units::length::inch_t position) = 0;
    virtual void SetTargetControl(ControlData *controlConst, units::velocity::feet_per_second_t velocity) = 0;

    virtual void UpdateTarget(double target) = 0;
    virtual void UpdateTarget(units::length::inch_t target) = 0;
    virtual void UpdateTarget(units::velocity::feet_per_second_t target) = 0;
    virtual void UpdateTarget(units::angle::degree_t target) = 0;
    virtual void UpdateTarget(units::angular_velocity::revolutions_per_minute_t target) = 0;

    virtual double GetTarget() const = 0;

    /// @brief  Return the current position of the mechanism.  The value is in inches.
    /// @return units::length::inch_t	position in inches
    virtual units::length::inch_t GetPositionInches() = 0;

    /// @brief  Get the current speed of the mechanism.
    /// @return units::velocity::feet_per_second_t
    virtual units::velocity::feet_per_second_t GetFeetPerSec() = 0;

    /// @brief  Return the current position of the mechanism in degrees
    /// @return units::angle::degree_t	position in degrees
    virtual units::angle::degree_t GetPositionDegrees() = 0;

    virtual units::angular_velocity::revolutions_per_minute_t GetRPM() = 0;

    virtual bool IsAtMinTravel() const = 0;
    virtual bool IsAtMaxTravel() const = 0;

    /// @brief  Set the control constants (e.g. PIDF values).
    /// @param [in] ControlData* pid:  the control constants
    virtual void SetControlConstants(int slot, ControlData pid) = 0;

protected:
    enum MotorTargetType
    {
        PERCENT_OUTPUT,
        POSITION,
        ANGLE,
        ANGULAR_VELOCITY,
        VELOCITY
    };
};
