
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
#include <map>
#include <string>
#include <vector>

// FRC Includes
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/length.h"
#include "units/velocity.h"

// Team 302 includes
#include "configs/usages/MotorControllerUsage.h"
#include "configs/usages/ServoUsage.h"
#include "configs/usages/SolenoidUsage.h"
#include "hw/DragonAnalogInput.h"
#include "hw/DragonCanCoder.h"
#include "hw/DragonDigitalInput.h"
#include "hw/DragonServo.h"
#include "hw/DragonSolenoid.h"
#include "mechanisms/base/BaseMech.h"
#include "mechanisms/base/BaseMechMotor.h"
#include "mechanisms/base/BaseMechServo.h"
#include "mechanisms/base/BaseMechSolenoid.h"
#include "mechanisms/base/StateMgr.h"

// forward declares
class IDragonMotorController;

class ExampleGen : public BaseMech, public StateMgr
{
public:
    /// @brief  This method constructs the mechanism using composition with its various actuators and sensors.
    /// @param controlFileName The control file with the PID constants and Targets for each state
    /// @param networkTableName Location for logging information
    ExampleGen(std::string controlFileName,
               std::string networkTableName);
    ExampleGen();
    ~ExampleGen() = default;

    void AddMotor(IDragonMotorController &motor);
    void AddSolenoid(DragonSolenoid &solenoid);
    void AddServo(DragonServo &servo);
    void AddDigitalInput(DragonDigitalInput &digital);
    void AddAnalogInput(DragonAnalogInput &analog);
    void AddCanCoder(DragonCanCoder &cancoder);

    /// @brief Set the control constants (e.g. PIDF values).
    /// @param indentifier the motor controller usage to identify the motor
    /// @param slot position on the motor controller to set
    /// @param pid control data / constants
    virtual void SetControlConstants(MotorControllerUsage::MOTOR_CONTROLLER_USAGE indentifier, int slot, ControlData pid);

    /// @brief update the output to the mechanism using the current controller and target value(s)
    virtual void Update();

    /// @brief Set the target value for the actuator
    /// @param identifier Motor Control Usage to indicate what motor to update
    /// @param percentOutput target value
    virtual void UpdateTarget(MotorControllerUsage::MOTOR_CONTROLLER_USAGE identifier, double percentOutput);

    /// @brief Set the target value for the actuator
    /// @param identifier Motor Control Usage to indicate what motor to update
    /// @param angle target value
    virtual void UpdateTarget(MotorControllerUsage::MOTOR_CONTROLLER_USAGE identifier, units::angle::degree_t angle);

    /// @brief Set the target value for the actuator
    /// @param identifier Motor Control Usage to indicate what motor to update
    /// @param angularVelocity target value
    virtual void UpdateTarget(MotorControllerUsage::MOTOR_CONTROLLER_USAGE identifier, units::angular_velocity::revolutions_per_minute_t angVel);

    /// @brief Set the target value for the actuator
    /// @param identifier Motor Control Usage to indicate what motor to update
    /// @param position target value
    virtual void UpdateTarget(MotorControllerUsage::MOTOR_CONTROLLER_USAGE identifier, units::length::inch_t position);

    /// @brief Set the target value for the actuator
    /// @param identifier Motor Control Usage to indicate what motor to update
    /// @param velocity target value
    virtual void UpdateTarget(MotorControllerUsage::MOTOR_CONTROLLER_USAGE identifier, units::velocity::feet_per_second_t velocity);

    /// @brief Set the target value for the actuator
    /// @param identifier solenoid Usage to indicate what motor to update
    /// @param extend target value
    virtual void UpdateTarget(SolenoidUsage::SOLENOID_USAGE identifier, bool extend);

    virtual bool IsAtMinPosition(MotorControllerUsage::MOTOR_CONTROLLER_USAGE identifier) const;
    virtual bool IsAtMinPosition(SolenoidUsage::SOLENOID_USAGE identifier) const;
    virtual bool IsAtMaxPosition(MotorControllerUsage::MOTOR_CONTROLLER_USAGE identifier) const;
    virtual bool IsAtMaxPosition(SolenoidUsage::SOLENOID_USAGE identifier) const;

    virtual std::vector<MotorControllerUsage::MOTOR_CONTROLLER_USAGE> GetMotorUsages() const;
    virtual BaseMechMotor *GetMotorMech(MotorControllerUsage::MOTOR_CONTROLLER_USAGE usage) const;

    virtual std::vector<SolenoidUsage::SOLENOID_USAGE> GetSolenoidUsages() const;
    virtual BaseMechSolenoid *GetSolenoidMech(SolenoidUsage::SOLENOID_USAGE usage) const;

    virtual std::vector<ServoUsage::SERVO_USAGE> GetServoUsages() const;
    virtual BaseMechServo *GetServoMech(ServoUsage::SERVO_USAGE usage) const;

private:
    std::unordered_map<MotorControllerUsage::MOTOR_CONTROLLER_USAGE, BaseMechMotor *> m_motorMap;
    std::unordered_map<SolenoidUsage::SOLENOID_USAGE, BaseMechSolenoid *> m_solenoidMap;
    std::unordered_map<ServoUsage::SERVO_USAGE, BaseMechServo *> m_servoMap;
};
