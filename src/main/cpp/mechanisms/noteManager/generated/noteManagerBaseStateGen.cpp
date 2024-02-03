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
// This file was automatically generated by the Team 302 code generator version 1.2.1.0
// Generated on Friday, February 2, 2024 8:15:15 PM

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
#include "mechanisms/noteManager/generated/noteManagerBaseStateGen.h"

// Third Party Includes

using namespace std;
using namespace noteManagerStates;

/// @class ExampleBaseStateGen
/// @brief information about the control (open loop, closed loop position, closed loop velocity, etc.) for a mechanism state
noteManagerBaseStateGen::noteManagerBaseStateGen ( string stateName,
        int stateId,
        noteManagerGen *mech ) : State ( stateName, stateId ),
	m_noteManager ( mech ),
	m_motorMap(),
	m_solenoidMap(),
	m_servoMap()
{
	auto motorUsages = m_noteManager->GetMotorUsages();
	for ( auto usage : motorUsages )
	{
		auto motormech = m_noteManager->GetMotorMech ( usage );
		m_motorMap[usage] = new BaseMechMotorState ( stateName, stateId, *motormech );
	}
	auto solUsages = m_noteManager->GetSolenoidUsages();
	for ( auto usage : solUsages )
	{
		auto solmech = m_noteManager->GetSolenoidMech ( usage );
		m_solenoidMap[usage] = new BaseMechSolenoidState ( stateName, stateId, *solmech );
	}
	auto servoUsages = m_noteManager->GetServoUsages();
	for ( auto usage : servoUsages )
	{
		auto servoMech = m_noteManager->GetServoMech ( usage );
		m_servoMap[usage] = new BaseMechServoState ( stateName, stateId, *servoMech );
	}
}

/// @brief Set the target value for the actuator
/// @param identifier Motor Control Usage to indicate what motor to update
/// @param percentOutput target value
void noteManagerBaseStateGen::SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, double percentOutput )
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
void noteManagerBaseStateGen::SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, ControlData &controlConst, units::angle::degree_t angle )
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
void noteManagerBaseStateGen::SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, ControlData &controlConst, units::angular_velocity::revolutions_per_minute_t angVel )
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
void noteManagerBaseStateGen::SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, ControlData &controlConst, units::length::inch_t position )
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
void noteManagerBaseStateGen::SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, ControlData &controlConst, units::velocity::feet_per_second_t velocity )
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
void noteManagerBaseStateGen::SetTargetControl ( RobotElementNames::SOLENOID_USAGE identifier, bool extend )
{
	auto solmech = GetSolenoidMechState ( identifier );
	if ( solmech != nullptr )
	{
		solmech->SetTarget ( extend );
	}
}

void noteManagerBaseStateGen::SetTargetControl ( RobotElementNames::SERVO_USAGE identifier, units::angle::degree_t angle )
{
	auto servomech = GetServoMechState ( identifier );
	if ( servomech != nullptr )
	{
		servomech->SetTarget ( angle );
	}
}

void noteManagerBaseStateGen::Init()
{
	InitMotorStates();
	InitSolenoidStates();
	InitServoStates();
}
void noteManagerBaseStateGen::InitMotorStates()
{
	auto motorUsages = m_noteManager->GetMotorUsages();
	for ( auto usage : motorUsages )
	{
		auto state = GetMotorMechState ( usage );
		if ( state != nullptr )
		{
			state->Init();
		}
	}
}
void noteManagerBaseStateGen::InitSolenoidStates()
{
	auto solUsages = m_noteManager->GetSolenoidUsages();
	for ( auto usage : solUsages )
	{
		auto state = GetSolenoidMechState ( usage );
		if ( state != nullptr )
		{
			state->Init();
		}
	}
}
void noteManagerBaseStateGen::InitServoStates()
{
	auto servoUsages = m_noteManager->GetServoUsages();
	for ( auto usage : servoUsages )
	{
		auto state = GetServoMechState ( usage );
		if ( state != nullptr )
		{
			state->Init();
		}
	}
}

void noteManagerBaseStateGen::Run()
{
	RunMotorStates();
	RunSolenoidStates();
	RunServoStates();
}
void noteManagerBaseStateGen::RunMotorStates()
{
	auto motorUsages = m_noteManager->GetMotorUsages();
	for ( auto usage : motorUsages )
	{
		auto state = GetMotorMechState ( usage );
		if ( state != nullptr )
		{
			state->Run();
		}
	}
}
void noteManagerBaseStateGen::RunSolenoidStates()
{
	auto solUsages = m_noteManager->GetSolenoidUsages();
	for ( auto usage : solUsages )
	{
		auto state = GetSolenoidMechState ( usage );
		if ( state != nullptr )
		{
			state->Run();
		}
	}
}
void noteManagerBaseStateGen::RunServoStates()
{
	auto servoUsages = m_noteManager->GetServoUsages();
	for ( auto usage : servoUsages )
	{
		auto state = GetServoMechState ( usage );
		if ( state != nullptr )
		{
			state->Run();
		}
	}
}

void noteManagerBaseStateGen::Exit()
{
	ExitMotorStates();
	ExitSolenoidStates();
	ExitServoStates();
}
void noteManagerBaseStateGen::ExitMotorStates()
{
	auto motorUsages = m_noteManager->GetMotorUsages();
	for ( auto usage : motorUsages )
	{
		auto state = GetMotorMechState ( usage );
		if ( state != nullptr )
		{
			state->Exit();
		}
	}
}
void noteManagerBaseStateGen::ExitSolenoidStates()
{
	auto solUsages = m_noteManager->GetSolenoidUsages();
	for ( auto usage : solUsages )
	{
		auto state = GetSolenoidMechState ( usage );
		if ( state != nullptr )
		{
			state->Exit();
		}
	}
}
void noteManagerBaseStateGen::ExitServoStates()
{
	auto servoUsages = m_noteManager->GetServoUsages();
	for ( auto usage : servoUsages )
	{
		auto state = GetServoMechState ( usage );
		if ( state != nullptr )
		{
			state->Exit();
		}
	}
}

bool noteManagerBaseStateGen::AtTarget()
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
bool noteManagerBaseStateGen::AtTargetMotorStates() const
{
	auto attarget = true;
	auto motorUsages = m_noteManager->GetMotorUsages();
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
bool noteManagerBaseStateGen::AtTargetSolenoidStates() const
{
	auto attarget = true;
	auto motorUsages = m_noteManager->GetMotorUsages();
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
bool noteManagerBaseStateGen::AtTargetServoStates() const
{
	auto attarget = true;
	auto servoUsages = m_noteManager->GetServoUsages();
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

BaseMechMotorState *noteManagerBaseStateGen::GetMotorMechState ( RobotElementNames::MOTOR_CONTROLLER_USAGE usage ) const
{
	auto itr = m_motorMap.find ( usage );
	if ( itr != m_motorMap.end() )
	{
		return itr->second;
	}
	return nullptr;
}

BaseMechSolenoidState *noteManagerBaseStateGen::GetSolenoidMechState ( RobotElementNames::SOLENOID_USAGE usage ) const
{
	auto itr = m_solenoidMap.find ( usage );
	if ( itr != m_solenoidMap.end() )
	{
		return itr->second;
	}
	return nullptr;
}
BaseMechServoState *noteManagerBaseStateGen::GetServoMechState ( RobotElementNames::SERVO_USAGE usage ) const
{
	auto itr = m_servoMap.find ( usage );
	if ( itr != m_servoMap.end() )
	{
		return itr->second;
	}
	return nullptr;
}
