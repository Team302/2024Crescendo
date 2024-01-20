// clang-format off
//====================================================================================================================================================
// Copyright 2023 Lake Orion Robotics FIRST Team 302
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
// This file was automatically generated by the Team 302 code generator version 1.1.0.0
// Generated on Saturday, January 20, 2024 11:58:48 AM

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
#include "mechanisms/mechanismInstanceName_1/generated/mechanismInstanceName_1_Base_StateGen.h"

// Third Party Includes

using namespace std;

/// @class ExampleBaseStateGen
/// @brief information about the control (open loop, closed loop position, closed loop velocity, etc.) for a mechanism state
mechanismInstanceName_1BaseStateGen::mechanismInstanceName_1BaseStateGen ( string stateName,
        int stateId,
        mechanismInstanceName_1_gen &mech ) : State ( stateName, stateId ),
	m_mechanismInstanceName_1 ( mech ),
	m_motorMap(),
	m_solenoidMap(),
	m_servoMap()
{
	auto motorUsages = m_mechanismInstanceName_1.GetMotorUsages();
	for ( auto usage : motorUsages )
	{
		auto motormech = m_mechanismInstanceName_1.GetMotorMech ( usage );
		m_motorMap[usage] = new BaseMechMotorState ( stateName, stateId, *motormech );
	}
	auto solUsages = m_mechanismInstanceName_1.GetSolenoidUsages();
	for ( auto usage : solUsages )
	{
		auto solmech = m_mechanismInstanceName_1.GetSolenoidMech ( usage );
		m_solenoidMap[usage] = new BaseMechSolenoidState ( stateName, stateId, *solmech );
	}
	auto servoUsages = m_mechanismInstanceName_1.GetServoUsages();
	for ( auto usage : servoUsages )
	{
		auto servoMech = m_mechanismInstanceName_1.GetServoMech ( usage );
		m_servoMap[usage] = new BaseMechServoState ( stateName, stateId, *servoMech );
	}
}

/// @brief Set the target value for the actuator
/// @param identifier Motor Control Usage to indicate what motor to update
/// @param percentOutput target value
void mechanismInstanceName_1BaseStateGen::SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, double percentOutput )
{
	auto motormech = GetMotorMechState ( identifier );
	if ( motormech != nullptr )
	{
		motormech->SetTargetControl ( percentOutput );
	}
}

/// @brief Set the target value for the actuator
/// @param identifier Motor Control Usage to indicate what motor to update
/// @param controlConst pid constants for controling motor
/// @param angle target value
void mechanismInstanceName_1BaseStateGen::SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, ControlData &controlConst, units::angle::degree_t angle )
{
	auto motormech = GetMotorMechState ( identifier );
	if ( motormech != nullptr )
	{
		motormech->SetTargetControl ( controlConst, angle );
	}
}

/// @brief Set the target value for the actuator
/// @param identifier Motor Control Usage to indicate what motor to update
/// @param controlConst pid constants for controling motor
/// @param angularVelocity target value
void mechanismInstanceName_1BaseStateGen::SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, ControlData &controlConst, units::angular_velocity::revolutions_per_minute_t angVel )
{
	auto motormech = GetMotorMechState ( identifier );
	if ( motormech != nullptr )
	{
		motormech->SetTargetControl ( controlConst, angVel );
	}
}

/// @brief Set the target value for the actuator
/// @param identifier Motor Control Usage to indicate what motor to update
/// @param controlConst pid constants for controling motor
/// @param position target value
void mechanismInstanceName_1BaseStateGen::SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, ControlData &controlConst, units::length::inch_t position )
{
	auto motormech = GetMotorMechState ( identifier );
	if ( motormech != nullptr )
	{
		motormech->SetTargetControl ( controlConst, position );
	}
}

/// @brief Set the target value for the actuator
/// @param identifier Motor Control Usage to indicate what motor to update
/// @param controlConst pid constants for controling motor
/// @param velocity target value
void mechanismInstanceName_1BaseStateGen::SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, ControlData &controlConst, units::velocity::feet_per_second_t velocity )
{
	auto motormech = GetMotorMechState ( identifier );
	if ( motormech != nullptr )
	{
		motormech->SetTargetControl ( controlConst, velocity );
	}
}

/// @brief Set the target value for the actuator
/// @param identifier solenoid Usage to indicate what motor to update
/// @param extend target value
void mechanismInstanceName_1BaseStateGen::SetTargetControl ( RobotElementNames::SOLENOID_USAGE identifier, bool extend )
{
	auto solmech = GetSolenoidMechState ( identifier );
	if ( solmech != nullptr )
	{
		solmech->SetTarget ( extend );
	}
}

void mechanismInstanceName_1BaseStateGen::SetTargetControl ( RobotElementNames::SERVO_USAGE identifier, units::angle::degree_t angle )
{
	auto servomech = GetServoMechState ( identifier );
	if ( servomech != nullptr )
	{
		servomech->SetTarget ( angle );
	}
}

void mechanismInstanceName_1BaseStateGen::Init()
{
	InitMotorStates();
	InitSolenoidStates();
	InitServoStates();
}
void mechanismInstanceName_1BaseStateGen::InitMotorStates()
{
	auto motorUsages = m_mechanismInstanceName_1.GetMotorUsages();
	for ( auto usage : motorUsages )
	{
		auto state = GetMotorMechState ( usage );
		if ( state != nullptr )
		{
			state->Init();
		}
	}
}
void mechanismInstanceName_1BaseStateGen::InitSolenoidStates()
{
	auto solUsages = m_mechanismInstanceName_1.GetSolenoidUsages();
	for ( auto usage : solUsages )
	{
		auto state = GetSolenoidMechState ( usage );
		if ( state != nullptr )
		{
			state->Init();
		}
	}
}
void mechanismInstanceName_1BaseStateGen::InitServoStates()
{
	auto servoUsages = m_mechanismInstanceName_1.GetServoUsages();
	for ( auto usage : servoUsages )
	{
		auto state = GetServoMechState ( usage );
		if ( state != nullptr )
		{
			state->Init();
		}
	}
}

void mechanismInstanceName_1BaseStateGen::Run()
{
	RunMotorStates();
	RunSolenoidStates();
	RunServoStates();
}
void mechanismInstanceName_1BaseStateGen::RunMotorStates()
{
	auto motorUsages = m_mechanismInstanceName_1.GetMotorUsages();
	for ( auto usage : motorUsages )
	{
		auto state = GetMotorMechState ( usage );
		if ( state != nullptr )
		{
			state->Run();
		}
	}
}
void mechanismInstanceName_1BaseStateGen::RunSolenoidStates()
{
	auto solUsages = m_mechanismInstanceName_1.GetSolenoidUsages();
	for ( auto usage : solUsages )
	{
		auto state = GetSolenoidMechState ( usage );
		if ( state != nullptr )
		{
			state->Run();
		}
	}
}
void mechanismInstanceName_1BaseStateGen::RunServoStates()
{
	auto servoUsages = m_mechanismInstanceName_1.GetServoUsages();
	for ( auto usage : servoUsages )
	{
		auto state = GetServoMechState ( usage );
		if ( state != nullptr )
		{
			state->Run();
		}
	}
}

void mechanismInstanceName_1BaseStateGen::Exit()
{
	ExitMotorStates();
	ExitSolenoidStates();
	ExitServoStates();
}
void mechanismInstanceName_1BaseStateGen::ExitMotorStates()
{
	auto motorUsages = m_mechanismInstanceName_1.GetMotorUsages();
	for ( auto usage : motorUsages )
	{
		auto state = GetMotorMechState ( usage );
		if ( state != nullptr )
		{
			state->Exit();
		}
	}
}
void mechanismInstanceName_1BaseStateGen::ExitSolenoidStates()
{
	auto solUsages = m_mechanismInstanceName_1.GetSolenoidUsages();
	for ( auto usage : solUsages )
	{
		auto state = GetSolenoidMechState ( usage );
		if ( state != nullptr )
		{
			state->Exit();
		}
	}
}
void mechanismInstanceName_1BaseStateGen::ExitServoStates()
{
	auto servoUsages = m_mechanismInstanceName_1.GetServoUsages();
	for ( auto usage : servoUsages )
	{
		auto state = GetServoMechState ( usage );
		if ( state != nullptr )
		{
			state->Exit();
		}
	}
}

bool mechanismInstanceName_1BaseStateGen::AtTarget()
{
	auto attarget = AtTargetMotorStates();
	if ( attarget )
	{
		attarget = AtTargetSolenoidStates();
		if ( attarget )
		{
			attarget = AtTargetServoStates();
		}
	}
	return attarget;
}
bool mechanismInstanceName_1BaseStateGen::AtTargetMotorStates() const
{
	auto attarget = true;
	auto motorUsages = m_mechanismInstanceName_1.GetMotorUsages();
	for ( auto usage : motorUsages )
	{
		auto state = GetMotorMechState ( usage );
		if ( state != nullptr )
		{
			attarget = state->AtTarget();
			if ( !attarget )
			{
				break;
			}
		}
	}
	return attarget;
}
bool mechanismInstanceName_1BaseStateGen::AtTargetSolenoidStates() const
{
	auto attarget = true;
	auto motorUsages = m_mechanismInstanceName_1.GetMotorUsages();
	for ( auto usage : motorUsages )
	{
		auto state = GetMotorMechState ( usage );
		if ( state != nullptr )
		{
			attarget = state->AtTarget();
			if ( !attarget )
			{
				break;
			}
		}
	}
	return attarget;
}
bool mechanismInstanceName_1BaseStateGen::AtTargetServoStates() const
{
	auto attarget = true;
	auto servoUsages = m_mechanismInstanceName_1.GetServoUsages();
	for ( auto usage : servoUsages )
	{
		auto state = GetServoMechState ( usage );
		if ( state != nullptr )
		{
			attarget = state->AtTarget();
			if ( !attarget )
			{
				break;
			}
		}
	}
	return attarget;
}

BaseMechMotorState *mechanismInstanceName_1BaseStateGen::GetMotorMechState ( RobotElementNames::MOTOR_CONTROLLER_USAGE usage ) const
{
	auto itr = m_motorMap.find ( usage );
	if ( itr != m_motorMap.end() )
	{
		return itr->second;
	}
	return nullptr;
}

BaseMechSolenoidState *mechanismInstanceName_1BaseStateGen::GetSolenoidMechState ( RobotElementNames::SOLENOID_USAGE usage ) const
{
	auto itr = m_solenoidMap.find ( usage );
	if ( itr != m_solenoidMap.end() )
	{
		return itr->second;
	}
	return nullptr;
}
BaseMechServoState *mechanismInstanceName_1BaseStateGen::GetServoMechState ( RobotElementNames::SERVO_USAGE usage ) const
{
	auto itr = m_servoMap.find ( usage );
	if ( itr != m_servoMap.end() )
	{
		return itr->second;
	}
	return nullptr;
}
