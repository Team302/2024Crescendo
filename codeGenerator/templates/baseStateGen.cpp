$$_COPYRIGHT_$$
$$_GEN_NOTICE_$$

#include <string>

// C++ Includes
#include <memory>
#include <string>
#include <vector>

// FRC includes

// Team 302 includes
#include "mechanisms/base/BaseMech.h"
#include "utils/logging/Logger.h"
#include "teleopcontrol/TeleopControl.h"
#include "mechanisms/$$_MECHANISM_INSTANCE_NAME_$$/generated/$$_MECHANISM_INSTANCE_NAME_$$_Base_StateGen.h"

// Third Party Includes

using namespace std;

/// @class ExampleBaseStateGen
/// @brief information about the control (open loop, closed loop position, closed loop velocity, etc.) for a mechanism state
$$_MECHANISM_INSTANCE_NAME_$$BaseStateGen::$$_MECHANISM_INSTANCE_NAME_$$BaseStateGen(string stateName,
                                                                                     int stateId,
                                                                                     $$_MECHANISM_INSTANCE_NAME_$$_gen &mech) : State(stateName, stateId),
                                                                                                                                m_$$_MECHANISM_INSTANCE_NAME_$$(mech),
                                                                                                                                m_motorMap(),
                                                                                                                                m_solenoidMap(),
                                                                                                                                m_servoMap()
{
    auto motorUsages = m_$$_MECHANISM_INSTANCE_NAME_$$.GetMotorUsages();
    for (auto usage : motorUsages)
    {
        auto motormech = m_$$_MECHANISM_INSTANCE_NAME_$$.GetMotorMech(usage);
        m_motorMap[usage] = new BaseMechMotorState(stateName, stateId, *motormech);
    }
    auto solUsages = m_$$_MECHANISM_INSTANCE_NAME_$$.GetSolenoidUsages();
    for (auto usage : solUsages)
    {
        auto solmech = m_$$_MECHANISM_INSTANCE_NAME_$$.GetSolenoidMech(usage);
        m_solenoidMap[usage] = new BaseMechSolenoidState(stateName, stateId, *solmech);
    }
    auto servoUsages = m_$$_MECHANISM_INSTANCE_NAME_$$.GetServoUsages();
    for (auto usage : servoUsages)
    {
        auto servoMech = m_$$_MECHANISM_INSTANCE_NAME_$$.GetServoMech(usage);
        m_servoMap[usage] = new BaseMechServoState(stateName, stateId, *servoMech);
    }
}

/// @brief Set the target value for the actuator
/// @param identifier Motor Control Usage to indicate what motor to update
/// @param percentOutput target value
void $$_MECHANISM_INSTANCE_NAME_$$BaseStateGen::SetTargetControl(RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, double percentOutput)
{
    auto motormech = GetMotorMechState(identifier);
    if (motormech != nullptr)
    {
        motormech->SetTargetControl(percentOutput);
    }
}

/// @brief Set the target value for the actuator
/// @param identifier Motor Control Usage to indicate what motor to update
/// @param controlConst pid constants for controling motor
/// @param angle target value
void $$_MECHANISM_INSTANCE_NAME_$$BaseStateGen::SetTargetControl(RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, ControlData &controlConst, units::angle::degree_t angle)
{
    auto motormech = GetMotorMechState(identifier);
    if (motormech != nullptr)
    {
        motormech->SetTargetControl(controlConst, angle);
    }
}

/// @brief Set the target value for the actuator
/// @param identifier Motor Control Usage to indicate what motor to update
/// @param controlConst pid constants for controling motor
/// @param angularVelocity target value
void $$_MECHANISM_INSTANCE_NAME_$$BaseStateGen::SetTargetControl(RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, ControlData &controlConst, units::angular_velocity::revolutions_per_minute_t angVel)
{
    auto motormech = GetMotorMechState(identifier);
    if (motormech != nullptr)
    {
        motormech->SetTargetControl(controlConst, angVel);
    }
}

/// @brief Set the target value for the actuator
/// @param identifier Motor Control Usage to indicate what motor to update
/// @param controlConst pid constants for controling motor
/// @param position target value
void $$_MECHANISM_INSTANCE_NAME_$$BaseStateGen::SetTargetControl(RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, ControlData &controlConst, units::length::inch_t position)
{
    auto motormech = GetMotorMechState(identifier);
    if (motormech != nullptr)
    {
        motormech->SetTargetControl(controlConst, position);
    }
}

/// @brief Set the target value for the actuator
/// @param identifier Motor Control Usage to indicate what motor to update
/// @param controlConst pid constants for controling motor
/// @param velocity target value
void $$_MECHANISM_INSTANCE_NAME_$$BaseStateGen::SetTargetControl(RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, ControlData &controlConst, units::velocity::feet_per_second_t velocity)
{
    auto motormech = GetMotorMechState(identifier);
    if (motormech != nullptr)
    {
        motormech->SetTargetControl(controlConst, velocity);
    }
}

/// @brief Set the target value for the actuator
/// @param identifier solenoid Usage to indicate what motor to update
/// @param extend target value
void $$_MECHANISM_INSTANCE_NAME_$$BaseStateGen::SetTargetControl(RobotElementNames::SOLENOID_USAGE identifier, bool extend)
{
    auto solmech = GetSolenoidMechState(identifier);
    if (solmech != nullptr)
    {
        solmech->SetTarget(extend);
    }
}

void $$_MECHANISM_INSTANCE_NAME_$$BaseStateGen::SetTargetControl(RobotElementNames::SERVO_USAGE identifier, units::angle::degree_t angle)
{
    auto servomech = GetServoMechState(identifier);
    if (servomech != nullptr)
    {
        servomech->SetTarget(angle);
    }
}

void $$_MECHANISM_INSTANCE_NAME_$$BaseStateGen::Init()
{
    InitMotorStates();
    InitSolenoidStates();
    InitServoStates();
}
void $$_MECHANISM_INSTANCE_NAME_$$BaseStateGen::InitMotorStates()
{
    auto motorUsages = m_$$_MECHANISM_INSTANCE_NAME_$$.GetMotorUsages();
    for (auto usage : motorUsages)
    {
        auto state = GetMotorMechState(usage);
        if (state != nullptr)
        {
            state->Init();
        }
    }
}
void $$_MECHANISM_INSTANCE_NAME_$$BaseStateGen::InitSolenoidStates()
{
    auto solUsages = m_$$_MECHANISM_INSTANCE_NAME_$$.GetSolenoidUsages();
    for (auto usage : solUsages)
    {
        auto state = GetSolenoidMechState(usage);
        if (state != nullptr)
        {
            state->Init();
        }
    }
}
void $$_MECHANISM_INSTANCE_NAME_$$BaseStateGen::InitServoStates()
{
    auto servoUsages = m_$$_MECHANISM_INSTANCE_NAME_$$.GetServoUsages();
    for (auto usage : servoUsages)
    {
        auto state = GetServoMechState(usage);
        if (state != nullptr)
        {
            state->Init();
        }
    }
}

void $$_MECHANISM_INSTANCE_NAME_$$BaseStateGen::Run()
{
    RunMotorStates();
    RunSolenoidStates();
    RunServoStates();
}
void $$_MECHANISM_INSTANCE_NAME_$$BaseStateGen::RunMotorStates()
{
    auto motorUsages = m_$$_MECHANISM_INSTANCE_NAME_$$.GetMotorUsages();
    for (auto usage : motorUsages)
    {
        auto state = GetMotorMechState(usage);
        if (state != nullptr)
        {
            state->Run();
        }
    }
}
void $$_MECHANISM_INSTANCE_NAME_$$BaseStateGen::RunSolenoidStates()
{
    auto solUsages = m_$$_MECHANISM_INSTANCE_NAME_$$.GetSolenoidUsages();
    for (auto usage : solUsages)
    {
        auto state = GetSolenoidMechState(usage);
        if (state != nullptr)
        {
            state->Run();
        }
    }
}
void $$_MECHANISM_INSTANCE_NAME_$$BaseStateGen::RunServoStates()
{
    auto servoUsages = m_$$_MECHANISM_INSTANCE_NAME_$$.GetServoUsages();
    for (auto usage : servoUsages)
    {
        auto state = GetServoMechState(usage);
        if (state != nullptr)
        {
            state->Run();
        }
    }
}

void $$_MECHANISM_INSTANCE_NAME_$$BaseStateGen::Exit()
{
    ExitMotorStates();
    ExitSolenoidStates();
    ExitServoStates();
}
void $$_MECHANISM_INSTANCE_NAME_$$BaseStateGen::ExitMotorStates()
{
    auto motorUsages = m_$$_MECHANISM_INSTANCE_NAME_$$.GetMotorUsages();
    for (auto usage : motorUsages)
    {
        auto state = GetMotorMechState(usage);
        if (state != nullptr)
        {
            state->Exit();
        }
    }
}
void $$_MECHANISM_INSTANCE_NAME_$$BaseStateGen::ExitSolenoidStates()
{
    auto solUsages = m_$$_MECHANISM_INSTANCE_NAME_$$.GetSolenoidUsages();
    for (auto usage : solUsages)
    {
        auto state = GetSolenoidMechState(usage);
        if (state != nullptr)
        {
            state->Exit();
        }
    }
}
void $$_MECHANISM_INSTANCE_NAME_$$BaseStateGen::ExitServoStates()
{
    auto servoUsages = m_$$_MECHANISM_INSTANCE_NAME_$$.GetServoUsages();
    for (auto usage : servoUsages)
    {
        auto state = GetServoMechState(usage);
        if (state != nullptr)
        {
            state->Exit();
        }
    }
}

bool $$_MECHANISM_INSTANCE_NAME_$$BaseStateGen::AtTarget()
{
    auto attarget = AtTargetMotorStates();
    if (attarget)
    {
        attarget = AtTargetSolenoidStates();
        if (attarget)
        {
            attarget = AtTargetServoStates();
        }
    }
    return attarget;
}
bool $$_MECHANISM_INSTANCE_NAME_$$BaseStateGen::AtTargetMotorStates() const
{
    auto attarget = true;
    auto motorUsages = m_$$_MECHANISM_INSTANCE_NAME_$$.GetMotorUsages();
    for (auto usage : motorUsages)
    {
        auto state = GetMotorMechState(usage);
        if (state != nullptr)
        {
            attarget = state->AtTarget();
            if (!attarget)
            {
                break;
            }
        }
    }
    return attarget;
}
bool $$_MECHANISM_INSTANCE_NAME_$$BaseStateGen::AtTargetSolenoidStates() const
{
    auto attarget = true;
    auto motorUsages = m_$$_MECHANISM_INSTANCE_NAME_$$.GetMotorUsages();
    for (auto usage : motorUsages)
    {
        auto state = GetMotorMechState(usage);
        if (state != nullptr)
        {
            attarget = state->AtTarget();
            if (!attarget)
            {
                break;
            }
        }
    }
    return attarget;
}
bool $$_MECHANISM_INSTANCE_NAME_$$BaseStateGen::AtTargetServoStates() const
{
    auto attarget = true;
    auto servoUsages = m_$$_MECHANISM_INSTANCE_NAME_$$.GetServoUsages();
    for (auto usage : servoUsages)
    {
        auto state = GetServoMechState(usage);
        if (state != nullptr)
        {
            attarget = state->AtTarget();
            if (!attarget)
            {
                break;
            }
        }
    }
    return attarget;
}

BaseMechMotorState *$$_MECHANISM_INSTANCE_NAME_$$BaseStateGen::GetMotorMechState(RobotElementNames::MOTOR_CONTROLLER_USAGE usage) const
{
    auto itr = m_motorMap.find(usage);
    if (itr != m_motorMap.end())
    {
        return itr->second;
    }
    return nullptr;
}

BaseMechSolenoidState *$$_MECHANISM_INSTANCE_NAME_$$BaseStateGen::GetSolenoidMechState(RobotElementNames::SOLENOID_USAGE usage) const
{
    auto itr = m_solenoidMap.find(usage);
    if (itr != m_solenoidMap.end())
    {
        return itr->second;
    }
    return nullptr;
}
BaseMechServoState *$$_MECHANISM_INSTANCE_NAME_$$BaseStateGen::GetServoMechState(RobotElementNames::SERVO_USAGE usage) const
{
    auto itr = m_servoMap.find(usage);
    if (itr != m_servoMap.end())
    {
        return itr->second;
    }
    return nullptr;
}
