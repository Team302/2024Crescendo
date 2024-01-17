$$_COPYRIGHT_$$
$$_GEN_NOTICE_$$

#pragma once
#include <string>

#include "State.h"
#include "mechanisms/base/BaseMechMotorState.h"
#include "mechanisms/base/BaseMechServoState.h"
#include "mechanisms/base/BaseMechSolenoidState.h"
#include "mechanisms/controllers/MechanismTargetData.h"
#include "mechanisms/controllers/ControlData.h"
#include "configs/RobotElementNames.h"
#include "mechanisms/$$_MECHANISM_INSTANCE_NAME_$$/generated/$$_MECHANISM_INSTANCE_NAME_$$_gen.h"

class $$_MECHANISM_INSTANCE_NAME_$$BaseStateGen : public State
{
public:
    $$_MECHANISM_INSTANCE_NAME_$$BaseStateGen(std::string stateName,
                                              int stateId,
                                              $$_MECHANISM_INSTANCE_NAME_$$_gen &mechanism);
    $$_MECHANISM_INSTANCE_NAME_$$BaseStateGen() = delete;
    ~$$_MECHANISM_INSTANCE_NAME_$$BaseStateGen() = default;

    /// @brief Set the target value for the actuator
    /// @param identifier Motor Control Usage to indicate what motor to update
    /// @param percentOutput target value
    void SetTargetControl(RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, double percentOutput);

    /// @brief Set the target value for the actuator
    /// @param identifier Motor Control Usage to indicate what motor to update
    /// @param controlConst pid constants for controling motor
    /// @param angle target value
    void SetTargetControl(RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, ControlData &controlConst, units::angle::degree_t angle);

    /// @brief Set the target value for the actuator
    /// @param identifier Motor Control Usage to indicate what motor to update
    /// @param controlConst pid constants for controling motor
    /// @param angularVelocity target value
    void SetTargetControl(RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, ControlData &controlConst, units::angular_velocity::revolutions_per_minute_t angVel);

    /// @brief Set the target value for the actuator
    /// @param identifier Motor Control Usage to indicate what motor to update
    /// @param controlConst pid constants for controling motor
    /// @param position target value
    void SetTargetControl(RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, ControlData &controlConst, units::length::inch_t position);

    /// @brief Set the target value for the actuator
    /// @param identifier Motor Control Usage to indicate what motor to update
    /// @param controlConst pid constants for controling motor
    /// @param velocity target value
    void SetTargetControl(RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, ControlData &controlConst, units::velocity::feet_per_second_t velocity);

    /// @brief Set the target value for the actuator
    /// @param identifier solenoid Usage to indicate what motor to update
    /// @param extend target value
    void SetTargetControl(RobotElementNames::SOLENOID_USAGE identifier, bool extend);

    /// @brief Set the target value for the actuator
    /// @param identifier solenoid Usage to indicate what motor to update
    /// @param extend target value
    void SetTargetControl(RobotElementNames::SERVO_USAGE identifier, units::angle::degree_t angle);

    void Init() override;
    virtual void InitMotorStates();
    virtual void InitSolenoidStates();
    virtual void InitServoStates();

    void Run() override;
    virtual void RunMotorStates();
    virtual void RunSolenoidStates();
    virtual void RunServoStates();

    void Exit() override;
    virtual void ExitMotorStates();
    virtual void ExitSolenoidStates();
    virtual void ExitServoStates();

    bool AtTarget() override;
    virtual bool AtTargetMotorStates() const;
    virtual bool AtTargetSolenoidStates() const;
    virtual bool AtTargetServoStates() const;

    $$_MECHANISM_INSTANCE_NAME_$$_gen Get$$_MECHANISM_INSTANCE_NAME_$$() { return m_$$_MECHANISM_INSTANCE_NAME_$$; }

protected:
    BaseMechMotorState *GetMotorMechState(RobotElementNames::MOTOR_CONTROLLER_USAGE usage) const;
    BaseMechSolenoidState *GetSolenoidMechState(RobotElementNames::SOLENOID_USAGE usage) const;
    BaseMechServoState *GetServoMechState(RobotElementNames::SERVO_USAGE usage) const;

private:
    $$_MECHANISM_INSTANCE_NAME_$$_gen m_$$_MECHANISM_INSTANCE_NAME_$$;
    std::unordered_map<RobotElementNames::MOTOR_CONTROLLER_USAGE, BaseMechMotorState *> m_motorMap;
    std::unordered_map<RobotElementNames::SOLENOID_USAGE, BaseMechSolenoidState *> m_solenoidMap;
    std::unordered_map<RobotElementNames::SERVO_USAGE, BaseMechServoState *> m_servoMap;
};
