// clang-format off
$$_COPYRIGHT_$$
$$_GEN_NOTICE_$$

#include <string>

// FRC Includes
#include <networktables/NetworkTableInstance.h>
#include "hw/interfaces/IDragonMotorController.h"

#include "$$_MECHANISM_INSTANCE_NAME_$$_gen.h"

$$_STATE_CLASSES_INCLUDES_$$

$$_USING_DIRECTIVES_$$

$$_MECHANISM_INSTANCE_NAME_$$_gen::$$_MECHANISM_INSTANCE_NAME_$$_gen() : $$_MECHANISM_NAME_$$(MechanismTypes::MECHANISM_TYPE::$$_MECHANISM_TYPE_NAME_$$, std::string("$$_MECHANISM_INSTANCE_NAME_$$")),
                                                                         m_motorMap(),
                                                                         m_solenoidMap(),
                                                                         m_servoMap()
{
}

void $$_MECHANISM_INSTANCE_NAME_$$_gen::Create()
{
    m_ntName = "$$_MECHANISM_INSTANCE_NAME_$$";
    $$_OBJECT_CREATION_$$

    $$_STATE_TRANSITION_REGISTRATION_$$

    m_table = nt::NetworkTableInstance::GetDefault().GetTable(m_ntName);
    m_tuningIsEnabledStr = "Enable Tuning for " + m_ntName; // since this string is used every loop, we do not want to create the string every time
    m_table.get()->PutBoolean(m_tuningIsEnabledStr, m_tuning);
}

void $$_MECHANISM_INSTANCE_NAME_$$_gen::Initialize(RobotConfigMgr::RobotIdentifier robotFullName)
{
    $$_ELEMENT_INITIALIZATION_$$
}

_STATE_MANAGER_START_
/// @brief  Set the control constants (e.g. PIDF values).
/// @param [in] ControlData*                                   pid:  the control constants
/// @return void
void $$_MECHANISM_INSTANCE_NAME_$$_gen::SetControlConstants(RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, int slot, ControlData pid)
{
    auto motor = GetMotorMech(identifier);
    if (motor != nullptr)
    {
        motor->SetControlConstants(slot, pid);
    }
}

/// @brief update the output to the mechanism using the current controller and target value(s)
/// @return void
void $$_MECHANISM_INSTANCE_NAME_$$_gen::Update()
{
    for (auto motor : m_motorMap)
    {
        motor.second->Update();
    }
}

void $$_MECHANISM_INSTANCE_NAME_$$_gen::UpdateTarget(RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, double percentOutput)
{
    auto motor = GetMotorMech(identifier);
    if (motor != nullptr)
    {
        motor->UpdateTarget(percentOutput);
    }
}

void $$_MECHANISM_INSTANCE_NAME_$$_gen::UpdateTarget(RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, units::angle::degree_t angle)
{
    auto motor = GetMotorMech(identifier);
    if (motor != nullptr)
    {
        motor->UpdateTarget(angle);
    }
}

void $$_MECHANISM_INSTANCE_NAME_$$_gen::UpdateTarget(RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, units::angular_velocity::revolutions_per_minute_t angVel)
{
    auto motor = GetMotorMech(identifier);
    if (motor != nullptr)
    {
        motor->UpdateTarget(angVel);
    }
}
void $$_MECHANISM_INSTANCE_NAME_$$_gen::UpdateTarget(RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, units::length::inch_t position)
{
    auto motor = GetMotorMech(identifier);
    if (motor != nullptr)
    {
        motor->UpdateTarget(position);
    }
}
void $$_MECHANISM_INSTANCE_NAME_$$_gen::UpdateTarget(RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, units::velocity::feet_per_second_t velocity)
{
    auto motor = GetMotorMech(identifier);
    if (motor != nullptr)
    {
        motor->UpdateTarget(velocity);
    }
}

bool $$_MECHANISM_INSTANCE_NAME_$$_gen::IsAtMinPosition(RobotElementNames::MOTOR_CONTROLLER_USAGE identifier) const
{
    auto motor = GetMotorMech(identifier);
    if (motor != nullptr)
    {
        return motor->IsAtMinTravel();
    }
    return false;
}

bool $$_MECHANISM_INSTANCE_NAME_$$_gen::IsAtMaxPosition(RobotElementNames::MOTOR_CONTROLLER_USAGE identifier) const
{
    auto motor = GetMotorMech(identifier);
    if (motor != nullptr)
    {
        return motor->IsAtMaxTravel();
    }
    return false;
}
_STATE_MANAGER_END_

BaseMechMotor *$$_MECHANISM_INSTANCE_NAME_$$_gen::GetMotorMech(RobotElementNames::MOTOR_CONTROLLER_USAGE usage) const
{
    auto itr = m_motorMap.find(usage);
    if (itr != m_motorMap.end())
    {
        return itr->second;
    }
    return nullptr;
}

std::vector<RobotElementNames::MOTOR_CONTROLLER_USAGE> $$_MECHANISM_INSTANCE_NAME_$$_gen::GetMotorUsages() const
{
    std::vector<RobotElementNames::MOTOR_CONTROLLER_USAGE> output;
    for (auto itr = m_motorMap.begin(); itr != m_motorMap.end(); ++itr)
    {
        output.emplace_back(itr->first);
    }
    return output;
}

_STATE_MANAGER_START_
void $$_MECHANISM_INSTANCE_NAME_$$_gen::UpdateTarget(RobotElementNames::SOLENOID_USAGE identifier, bool extend)
{
    auto sol = GetSolenoidMech(identifier);
    if (sol != nullptr)
    {
        sol->ActivateSolenoid(extend);
    }
}

bool $$_MECHANISM_INSTANCE_NAME_$$_gen::IsAtMinPosition(RobotElementNames::SOLENOID_USAGE identifier) const
{
    auto sol = GetSolenoidMech(identifier);
    if (sol != nullptr)
    {
        return !sol->IsSolenoidActivated();
    }
    return false;
}


bool $$_MECHANISM_INSTANCE_NAME_$$_gen::IsAtMaxPosition(RobotElementNames::SOLENOID_USAGE identifier) const
{
    auto sol = GetSolenoidMech(identifier);
    if (sol != nullptr)
    {
        return sol->IsSolenoidActivated();
    }
    return false;
}
_STATE_MANAGER_END_

BaseMechSolenoid *$$_MECHANISM_INSTANCE_NAME_$$_gen::GetSolenoidMech(RobotElementNames::SOLENOID_USAGE usage) const
{
    auto itr = m_solenoidMap.find(usage);
    if (itr != m_solenoidMap.end())
    {
        return itr->second;
    }
    return nullptr;
}

std::vector<RobotElementNames::SOLENOID_USAGE> $$_MECHANISM_INSTANCE_NAME_$$_gen::GetSolenoidUsages() const
{
    std::vector<RobotElementNames::SOLENOID_USAGE> output;
    for (auto itr = m_solenoidMap.begin(); itr != m_solenoidMap.end(); ++itr)
    {
        output.emplace_back(itr->first);
    }
    return output;
}

BaseMechServo *$$_MECHANISM_INSTANCE_NAME_$$_gen::GetServoMech(RobotElementNames::SERVO_USAGE usage) const
{
    auto itr = m_servoMap.find(usage);
    if (itr != m_servoMap.end())
    {
        return itr->second;
    }
    return nullptr;
}

std::vector<RobotElementNames::SERVO_USAGE> $$_MECHANISM_INSTANCE_NAME_$$_gen::GetServoUsages() const
{
    std::vector<RobotElementNames::SERVO_USAGE> output;
    for (auto itr = m_servoMap.begin(); itr != m_servoMap.end(); ++itr)
    {
        output.emplace_back(itr->first);
    }
    return output;
}
