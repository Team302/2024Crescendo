$$_COPYRIGHT_$$
$$_GEN_NOTICE_$$

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

$$_INCLUDE_FILES_$$

class $$_MECHANISM_INSTANCE_NAME_$$Gen : public BaseMech _STATE_MANAGER_START_, public StateMgr _STATE_MANAGER_END_
{
public:
    enum STATE_NAMES
    {
        $$_STATE_NAMES_$$
    };

    $$_MECHANISM_INSTANCE_NAME_$$Gen(RobotConfigMgr::RobotIdentifier activeRobotId);

    $$_CREATE_FUNCTIONS_$$
    $$_INITIALZATION_FUNCTIONS_$$

    _STATE_MANAGER_START_
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
    _STATE_MANAGER_END_

    virtual std::vector<RobotElementNames::MOTOR_CONTROLLER_USAGE> GetMotorUsages() const;
    virtual BaseMechMotor *GetMotorMech(RobotElementNames::MOTOR_CONTROLLER_USAGE usage) const;

    virtual std::vector<RobotElementNames::SOLENOID_USAGE> GetSolenoidUsages() const;
    virtual BaseMechSolenoid *GetSolenoidMech(RobotElementNames::SOLENOID_USAGE usage) const;

    virtual std::vector<RobotElementNames::SERVO_USAGE> GetServoUsages() const;
    virtual BaseMechServo *GetServoMech(RobotElementNames::SERVO_USAGE usage) const;

    void Cyclic();

    $$_MECHANISM_ELEMENTS_GETTERS_$$

    static std::map<std::string, STATE_NAMES> stringToSTATE_NAMESEnumMap;

protected:
    RobotConfigMgr::RobotIdentifier m_activeRobotId;
    std::string m_ntName;
    std::string m_tuningIsEnabledStr;
    bool m_tuning = false;
    std::shared_ptr<nt::NetworkTable> m_table;

    void SetCurrentState(int state, bool run) override;

private:
    std::unordered_map<RobotElementNames::MOTOR_CONTROLLER_USAGE, BaseMechMotor *> m_motorMap;
    std::unordered_map<RobotElementNames::SOLENOID_USAGE, BaseMechSolenoid *> m_solenoidMap;
    std::unordered_map<RobotElementNames::SERVO_USAGE, BaseMechServo *> m_servoMap;

    std::unordered_map<std::string, STATE_NAMES> m_stateMap;

    $$_MECHANISM_ELEMENTS_$$

    void CheckForTuningEnabled();
    void ReadTuningParamsFromNT();
    void PushTuningParamsToNT();

    $$_TUNABLE_PARAMETERS_$$
};