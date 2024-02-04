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
// This file was automatically generated by the Team 302 code generator version 1.2.2.0
// Generated on Saturday, February 3, 2024 11:54:22 AM

#pragma once
#include <string>

#include "State.h"
#include "mechanisms/base/BaseMechMotorState.h"
#include "mechanisms/base/BaseMechServoState.h"
#include "mechanisms/base/BaseMechSolenoidState.h"
#include "mechanisms/controllers/ControlData.h"
#include "configs/RobotElementNames.h"
#include "mechanisms/Thing1Mech/generated/Thing1MechGen.h"

namespace Thing1MechStates
{
class Thing1MechBaseStateGen : public State
{
public:
	Thing1MechBaseStateGen ( std::string stateName,
	                         int stateId,
	                         Thing1MechGen *mechanism );
	Thing1MechBaseStateGen() = delete;
	~Thing1MechBaseStateGen() = default;

	/// @brief Set the target value for the actuator
	/// @param identifier Motor Control Usage to indicate what motor to update
	/// @param percentOutput target value
	void SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, double percentOutput );

	/// @brief Set the target value for the actuator
	/// @param identifier Motor Control Usage to indicate what motor to update
	/// @param controlConst pid constants for controling motor
	/// @param angle target value
	void SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, ControlData &controlConst, units::angle::degree_t angle );

	/// @brief Set the target value for the actuator
	/// @param identifier Motor Control Usage to indicate what motor to update
	/// @param controlConst pid constants for controling motor
	/// @param angularVelocity target value
	void SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, ControlData &controlConst, units::angular_velocity::revolutions_per_minute_t angVel );

	/// @brief Set the target value for the actuator
	/// @param identifier Motor Control Usage to indicate what motor to update
	/// @param controlConst pid constants for controling motor
	/// @param position target value
	void SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, ControlData &controlConst, units::length::inch_t position );

	/// @brief Set the target value for the actuator
	/// @param identifier Motor Control Usage to indicate what motor to update
	/// @param controlConst pid constants for controling motor
	/// @param velocity target value
	void SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, ControlData &controlConst, units::velocity::feet_per_second_t velocity );

	/// @brief Set the target value for the actuator
	/// @param identifier solenoid Usage to indicate what motor to update
	/// @param extend target value
	void SetTargetControl ( RobotElementNames::SOLENOID_USAGE identifier, bool extend );

	/// @brief Set the target value for the actuator
	/// @param identifier solenoid Usage to indicate what motor to update
	/// @param extend target value
	void SetTargetControl ( RobotElementNames::SERVO_USAGE identifier, units::angle::degree_t angle );

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

	Thing1MechGen *GetThing1Mech() { return m_Thing1Mech; }

protected:
	BaseMechMotorState *GetMotorMechState ( RobotElementNames::MOTOR_CONTROLLER_USAGE usage ) const;
	BaseMechSolenoidState *GetSolenoidMechState ( RobotElementNames::SOLENOID_USAGE usage ) const;
	BaseMechServoState *GetServoMechState ( RobotElementNames::SERVO_USAGE usage ) const;

private:
	Thing1MechGen *m_Thing1Mech;
	std::unordered_map<RobotElementNames::MOTOR_CONTROLLER_USAGE, BaseMechMotorState *> m_motorMap;
	std::unordered_map<RobotElementNames::SOLENOID_USAGE, BaseMechSolenoidState *> m_solenoidMap;
	std::unordered_map<RobotElementNames::SERVO_USAGE, BaseMechServoState *> m_servoMap;
};
}
