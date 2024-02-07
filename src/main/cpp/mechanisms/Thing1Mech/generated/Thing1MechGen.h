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

#pragma once

#include <string>
#include <memory>

// FRC Includes
#include <networktables/NetworkTable.h>

#include "mechanisms/base/BaseMech.h"
#include "mechanisms/base/BaseMechMotor.h"
#include "mechanisms/base/BaseMechServo.h"
#include "mechanisms/base/BaseMechSolenoid.h"
#include "mechanisms/base/StateMgr.h"

#include "configs/RobotElementNames.h"
#include "configs/RobotConfigMgr.h"

#include "hw/DragonTalonSRX.h"
#include "hw/DragonTalonFX.h"
#include "hw/DragonSparkMax.h"
#include "hw/DragonSparkFlex.h"

class Thing1MechGen : public BaseMech , public StateMgr 
{
public:
    enum STATE_NAMES
    {
        STATE_LEFT_FRONT_CW, 
STATE_RIGHT_FRONT_CW, 
STATE_RIGHT_BACK_CW, 
STATE_LEFT_BACK_CW, 
STATE_SPARKY_ON, 
STATE_THING1TALON,
    };

    Thing1MechGen();

    void Create();
    void Initialize(RobotConfigMgr::RobotIdentifier robotFullName);

    void SetTheCurrentState(STATE_NAMES state, bool run);

    
    /// @brief Set the control constants (e.g. PIDF values).
    /// @param indentifier the motor controller usage to identify the motor
    /// @param slot position on the motor controller to set
    /// @param pid control data / constants
    virtual void SetControlConstants(RobotElementNames::MOTOR_CONTROLLER_USAGE indentifier, int slot, ControlData pid);

    /// @brief update the output to the mechanism using the current controller and target value(s)
    virtual void Update();

    /// @brief Set the target value for the actuator
    /// @param identifier Motor Control Usage to indicate what motor to update
    /// @param percentOutput target value
    virtual void UpdateTarget(RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, double percentOutput);

    /// @brief Set the target value for the actuator
    /// @param identifier Motor Control Usage to indicate what motor to update
    /// @param angle target value
    virtual void UpdateTarget(RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, units::angle::degree_t angle);

    /// @brief Set the target value for the actuator
    /// @param identifier Motor Control Usage to indicate what motor to update
    /// @param angularVelocity target value
    virtual void UpdateTarget(RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, units::angular_velocity::revolutions_per_minute_t angVel);

    /// @brief Set the target value for the actuator
    /// @param identifier Motor Control Usage to indicate what motor to update
    /// @param position target value
    virtual void UpdateTarget(RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, units::length::inch_t position);

    /// @brief Set the target value for the actuator
    /// @param identifier Motor Control Usage to indicate what motor to update
    /// @param velocity target value
    virtual void UpdateTarget(RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, units::velocity::feet_per_second_t velocity);

    /// @brief Set the target value for the actuator
    /// @param identifier solenoid Usage to indicate what motor to update
    /// @param extend target value
    virtual void UpdateTarget(RobotElementNames::SOLENOID_USAGE identifier, bool extend);

    virtual bool IsAtMinPosition(RobotElementNames::MOTOR_CONTROLLER_USAGE identifier) const;
    virtual bool IsAtMinPosition(RobotElementNames::SOLENOID_USAGE identifier) const;
    virtual bool IsAtMaxPosition(RobotElementNames::MOTOR_CONTROLLER_USAGE identifier) const;
    virtual bool IsAtMaxPosition(RobotElementNames::SOLENOID_USAGE identifier) const;
    

    virtual std::vector<RobotElementNames::MOTOR_CONTROLLER_USAGE> GetMotorUsages() const;
    virtual BaseMechMotor *GetMotorMech(RobotElementNames::MOTOR_CONTROLLER_USAGE usage) const;

    virtual std::vector<RobotElementNames::SOLENOID_USAGE> GetSolenoidUsages() const;
    virtual BaseMechSolenoid *GetSolenoidMech(RobotElementNames::SOLENOID_USAGE usage) const;

    virtual std::vector<RobotElementNames::SERVO_USAGE> GetServoUsages() const;
    virtual BaseMechServo *GetServoMech(RobotElementNames::SERVO_USAGE usage) const;

    void Cyclic();

    DragonTalonSRX* getBackLeftMotor() const {return BackLeftMotor;}
DragonTalonSRX* getRightBackMotor() const {return RightBackMotor;}
DragonTalonSRX* getLeftFrontMotor() const {return LeftFrontMotor;}
DragonTalonSRX* getRightFrontMotor() const {return RightFrontMotor;}
DragonTalonFX* getFlacon() const {return Flacon;}
DragonSparkMax* getNeo550() const {return Neo550;}
DragonSparkFlex* getVortex() const {return Vortex;}
ControlData* getpercentControlData() const {return percentControlData;}

protected:
    std::string m_ntName;
    std::string m_tuningIsEnabledStr;
    bool m_tuning = false;
    std::shared_ptr<nt::NetworkTable> m_table;

private:
    std::unordered_map<RobotElementNames::MOTOR_CONTROLLER_USAGE, BaseMechMotor *> m_motorMap;
    std::unordered_map<RobotElementNames::SOLENOID_USAGE, BaseMechSolenoid *> m_solenoidMap;
    std::unordered_map<RobotElementNames::SERVO_USAGE, BaseMechServo *> m_servoMap;

    DragonTalonSRX* BackLeftMotor;
DragonTalonSRX* RightBackMotor;
DragonTalonSRX* LeftFrontMotor;
DragonTalonSRX* RightFrontMotor;
DragonTalonFX* Flacon;
DragonSparkMax* Neo550;
DragonSparkFlex* Vortex;
ControlData* percentControlData;

    void CheckForTuningEnabled();
    void ReadTuningParamsFromNT();
    void PushTuningParamsToNT();

    
};