
$$_COPYRIGHT_$$
$$_GEN_NOTICE_$$

#include <string>

// FRC Includes
#include <networktables/NetworkTableInstance.h>
#include "hw/interfaces/IDragonMotorController.h"

#include "$$_MECHANISM_INSTANCE_NAME_$$Gen.h"
#include "utils/logging/Logger.h"

$$_USING_DIRECTIVES_$$

$$_MECHANISM_INSTANCE_NAME_$$Gen::$$_MECHANISM_INSTANCE_NAME_$$Gen(RobotConfigMgr::RobotIdentifier activeRobotId) : BaseMech(MechanismTypes::MECHANISM_TYPE::$$_MECHANISM_TYPE_NAME_$$, "", std::string("$$_MECHANISM_INSTANCE_NAME_$$")),
                                                                                                                    m_activeRobotId(activeRobotId),
                                                                                                                    m_motorMap(),
                                                                                                                    m_solenoidMap(),
                                                                                                                    m_servoMap(),
                                                                                                                    m_stateMap()
{
}

std::map<std::string, $$_MECHANISM_INSTANCE_NAME_$$Gen::STATE_NAMES> $$_MECHANISM_INSTANCE_NAME_$$Gen::stringToSTATE_NAMESEnumMap{
    $$_STATE_MAP_$$};

$$_CREATE_FUNCTIONS_$$

$$_INITIALZATION_FUNCTIONS_$$

void $$_MECHANISM_INSTANCE_NAME_$$Gen::SetCurrentState(int state, bool run)
{
    StateMgr::SetCurrentState(state, run);
}

_STATE_MANAGER_START_
/// @brief  Set the control constants (e.g. PIDF values).
/// @param [in] ControlData*                                   pid:  the control constants
/// @return void
void $$_MECHANISM_INSTANCE_NAME_$$Gen::SetControlConstants(RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, int slot, ControlData pid)
{
    auto motor = GetMotorMech(identifier);
    if (motor != nullptr)
    {
        motor->SetControlConstants(slot, pid);
    }
}

/// @brief update the output to the mechanism using the current controller and target value(s)
/// @return void
void $$_MECHANISM_INSTANCE_NAME_$$Gen::Update()
{
    for (auto motor : m_motorMap)
    {
        motor.second->Update();
    }
}

void $$_MECHANISM_INSTANCE_NAME_$$Gen::UpdateTarget(RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, double percentOutput)
{
    auto motor = GetMotorMech(identifier);
    if (motor != nullptr)
    {
        motor->UpdateTarget(percentOutput);
    }
}

void $$_MECHANISM_INSTANCE_NAME_$$Gen::UpdateTarget(RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, units::angle::degree_t angle)
{
    auto motor = GetMotorMech(identifier);
    if (motor != nullptr)
    {
        motor->UpdateTarget(angle);
    }
}

void $$_MECHANISM_INSTANCE_NAME_$$Gen::UpdateTarget(RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, units::angular_velocity::revolutions_per_minute_t angVel)
{
    auto motor = GetMotorMech(identifier);
    if (motor != nullptr)
    {
        motor->UpdateTarget(angVel);
    }
}
void $$_MECHANISM_INSTANCE_NAME_$$Gen::UpdateTarget(RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, units::length::inch_t position)
{
    auto motor = GetMotorMech(identifier);
    if (motor != nullptr)
    {
        motor->UpdateTarget(position);
    }
}
void $$_MECHANISM_INSTANCE_NAME_$$Gen::UpdateTarget(RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, units::velocity::feet_per_second_t velocity)
{
    auto motor = GetMotorMech(identifier);
    if (motor != nullptr)
    {
        motor->UpdateTarget(velocity);
    }
}

bool $$_MECHANISM_INSTANCE_NAME_$$Gen::IsAtMinPosition(RobotElementNames::MOTOR_CONTROLLER_USAGE identifier) const
{
    auto motor = GetMotorMech(identifier);
    if (motor != nullptr)
    {
        return motor->IsAtMinTravel();
    }
    return false;
}

bool $$_MECHANISM_INSTANCE_NAME_$$Gen::IsAtMaxPosition(RobotElementNames::MOTOR_CONTROLLER_USAGE identifier) const
{
    auto motor = GetMotorMech(identifier);
    if (motor != nullptr)
    {
        return motor->IsAtMaxTravel();
    }
    return false;
}
_STATE_MANAGER_END_

BaseMechMotor *$$_MECHANISM_INSTANCE_NAME_$$Gen::GetMotorMech(RobotElementNames::MOTOR_CONTROLLER_USAGE usage) const
{
    auto itr = m_motorMap.find(usage);
    if (itr != m_motorMap.end())
    {
        return itr->second;
    }
    return nullptr;
}

std::vector<RobotElementNames::MOTOR_CONTROLLER_USAGE> $$_MECHANISM_INSTANCE_NAME_$$Gen::GetMotorUsages() const
{
    std::vector<RobotElementNames::MOTOR_CONTROLLER_USAGE> output;
    for (auto itr = m_motorMap.begin(); itr != m_motorMap.end(); ++itr)
    {
        output.emplace_back(itr->first);
    }
    return output;
}

_STATE_MANAGER_START_
void $$_MECHANISM_INSTANCE_NAME_$$Gen::UpdateTarget(RobotElementNames::SOLENOID_USAGE identifier, bool extend)
{
    auto sol = GetSolenoidMech(identifier);
    if (sol != nullptr)
    {
        sol->ActivateSolenoid(extend);
    }
}

bool $$_MECHANISM_INSTANCE_NAME_$$Gen::IsAtMinPosition(RobotElementNames::SOLENOID_USAGE identifier) const
{
    auto sol = GetSolenoidMech(identifier);
    if (sol != nullptr)
    {
        return !sol->IsSolenoidActivated();
    }
    return false;
}

bool $$_MECHANISM_INSTANCE_NAME_$$Gen::IsAtMaxPosition(RobotElementNames::SOLENOID_USAGE identifier) const
{
    auto sol = GetSolenoidMech(identifier);
    if (sol != nullptr)
    {
        return sol->IsSolenoidActivated();
    }
    return false;
}
_STATE_MANAGER_END_

BaseMechSolenoid *$$_MECHANISM_INSTANCE_NAME_$$Gen::GetSolenoidMech(RobotElementNames::SOLENOID_USAGE usage) const
{
    auto itr = m_solenoidMap.find(usage);
    if (itr != m_solenoidMap.end())
    {
        return itr->second;
    }
    return nullptr;
}

std::vector<RobotElementNames::SOLENOID_USAGE> $$_MECHANISM_INSTANCE_NAME_$$Gen::GetSolenoidUsages() const
{
    std::vector<RobotElementNames::SOLENOID_USAGE> output;
    for (auto itr = m_solenoidMap.begin(); itr != m_solenoidMap.end(); ++itr)
    {
        output.emplace_back(itr->first);
    }
    return output;
}

BaseMechServo *$$_MECHANISM_INSTANCE_NAME_$$Gen::GetServoMech(RobotElementNames::SERVO_USAGE usage) const
{
    auto itr = m_servoMap.find(usage);
    if (itr != m_servoMap.end())
    {
        return itr->second;
    }
    return nullptr;
}

std::vector<RobotElementNames::SERVO_USAGE> $$_MECHANISM_INSTANCE_NAME_$$Gen::GetServoUsages() const
{
    std::vector<RobotElementNames::SERVO_USAGE> output;
    for (auto itr = m_servoMap.begin(); itr != m_servoMap.end(); ++itr)
    {
        output.emplace_back(itr->first);
    }
    return output;
}

void $$_MECHANISM_INSTANCE_NAME_$$Gen::Cyclic()
{
    CheckForTuningEnabled();
    if (m_tuning)
    {
        ReadTuningParamsFromNT();
    }
}

void $$_MECHANISM_INSTANCE_NAME_$$Gen::CheckForTuningEnabled()
{
    bool pastTuning = m_tuning;
    m_tuning = m_table.get()->GetBoolean(m_tuningIsEnabledStr, false);
    if (pastTuning != m_tuning && m_tuning == true)
    {
        PushTuningParamsToNT();
    }
}

void $$_MECHANISM_INSTANCE_NAME_$$Gen::ReadTuningParamsFromNT()
{
    $$_READ_TUNABLE_PARAMETERS_$$
}

void $$_MECHANISM_INSTANCE_NAME_$$Gen::PushTuningParamsToNT()
{
    $$_PUSH_TUNABLE_PARAMETERS_$$
}