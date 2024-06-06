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
// This file was automatically generated by the Team 302 code generator version 1.3.0.15
// Generated on Tuesday, April 9, 2024 3:43:38 PM

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

#include "hw/DragonSparkMaxMonitored.h"
#include "hw/DragonSparkMax.h"
#include "hw/DragonSparkFlexMonitored.h"
#include "hw/DragonSparkFlex.h"
#include "hw/DragonTalonFX.h"
#include "hw/DragonDigitalInput.h"
#include "hw/DragonCanCoder.h"

class noteManagerGen : public BaseMech, public StateMgr
{
public:
	enum STATE_NAMES
	{
		STATE_OFF,
		STATE_READY,
		STATE_FEEDER_INTAKE,
		STATE_EXPEL,
		STATE_PLACER_INTAKE,
		STATE_LAUNCHER_TO_PLACER,
		STATE_HOLD_FEEDER,
		STATE_READY_AUTO_LAUNCH,
		STATE_READY_MANUAL_LAUNCH,
		STATE_PASS,
		STATE_AUTO_LAUNCH,
		STATE_MANUAL_LAUNCH,
		STATE_READY_ODOMETRY_LAUNCH,
		STATE_PREPARE_PLACE_AMP,
		STATE_PREPARE_PLACE_TRAP,
		STATE_PLACE_AMP,
		STATE_PLACE_TRAP,
		STATE_PLACER_TO_LAUNCHER,
		STATE_BACKUP_MANUAL_LAUNCH,
		STATE_BACKUP_MANUAL_PLACE,
		STATE_HOLD_PLACER,
		STATE_LOW_PASS
	};

	noteManagerGen ( RobotConfigMgr::RobotIdentifier activeRobotId );

	void CreateCompBot302();
	void CreatepracticeBot9999();
	void InitializeCompBot302();
	void InitializepracticeBot9999();

	/// @brief Set the control constants (e.g. PIDF values).
	/// @param indentifier the motor controller usage to identify the motor
	/// @param slot position on the motor controller to set
	/// @param pid control data / constants
	virtual void SetControlConstants ( RobotElementNames::MOTOR_CONTROLLER_USAGE indentifier, int slot, ControlData pid );

	/// @brief update the output to the mechanism using the current controller and target value(s)
	virtual void Update();

	/// @brief Set the target value for the actuator
	/// @param identifier Motor Control Usage to indicate what motor to update
	/// @param percentOutput target value
	virtual void UpdateTarget ( RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, double percentOutput );

	/// @brief Set the target value for the actuator
	/// @param identifier Motor Control Usage to indicate what motor to update
	/// @param angle target value
	virtual void UpdateTarget ( RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, units::angle::degree_t angle );

	/// @brief Set the target value for the actuator
	/// @param identifier Motor Control Usage to indicate what motor to update
	/// @param angularVelocity target value
	virtual void UpdateTarget ( RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, units::angular_velocity::revolutions_per_minute_t angVel );

	/// @brief Set the target value for the actuator
	/// @param identifier Motor Control Usage to indicate what motor to update
	/// @param position target value
	virtual void UpdateTarget ( RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, units::length::inch_t position );

	/// @brief Set the target value for the actuator
	/// @param identifier Motor Control Usage to indicate what motor to update
	/// @param velocity target value
	virtual void UpdateTarget ( RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, units::velocity::feet_per_second_t velocity );

	/// @brief Set the target value for the actuator
	/// @param identifier solenoid Usage to indicate what motor to update
	/// @param extend target value
	virtual void UpdateTarget ( RobotElementNames::SOLENOID_USAGE identifier, bool extend );

	virtual bool IsAtMinPosition ( RobotElementNames::MOTOR_CONTROLLER_USAGE identifier ) const;
	virtual bool IsAtMinPosition ( RobotElementNames::SOLENOID_USAGE identifier ) const;
	virtual bool IsAtMaxPosition ( RobotElementNames::MOTOR_CONTROLLER_USAGE identifier ) const;
	virtual bool IsAtMaxPosition ( RobotElementNames::SOLENOID_USAGE identifier ) const;

	virtual std::vector<RobotElementNames::MOTOR_CONTROLLER_USAGE> GetMotorUsages() const;
	virtual BaseMechMotor *GetMotorMech ( RobotElementNames::MOTOR_CONTROLLER_USAGE usage ) const;

	virtual std::vector<RobotElementNames::SOLENOID_USAGE> GetSolenoidUsages() const;
	virtual BaseMechSolenoid *GetSolenoidMech ( RobotElementNames::SOLENOID_USAGE usage ) const;

	virtual std::vector<RobotElementNames::SERVO_USAGE> GetServoUsages() const;
	virtual BaseMechServo *GetServoMech ( RobotElementNames::SERVO_USAGE usage ) const;

	void Cyclic();

	IDragonMotorController* getfrontIntake() const {return frontIntakeDragonSparkMaxMonitored;}
	IDragonMotorController* getbackIntake() const {return backIntakeDragonSparkMaxMonitored;}
	IDragonMotorController* getTransfer() const {return TransferDragonSparkMaxMonitored;}
	IDragonMotorController* getFeeder() const {return FeederDragonSparkFlexMonitored;}
	IDragonMotorController* getlauncherTop() const
	{
		if ( RobotConfigMgr::RobotIdentifier::COMP_BOT_302 == m_activeRobotId )
			return launcherTopDragonSparkFlex;
		else if ( RobotConfigMgr::RobotIdentifier::PRACTICE_BOT_9999 == m_activeRobotId )
			return launcherTopDragonTalonFX;
		return nullptr;
	}
	IDragonMotorController* getlauncherBottom() const
	{
		if ( RobotConfigMgr::RobotIdentifier::COMP_BOT_302 == m_activeRobotId )
			return launcherBottomDragonSparkFlex;
		else if ( RobotConfigMgr::RobotIdentifier::PRACTICE_BOT_9999 == m_activeRobotId )
			return launcherBottomDragonTalonFX;
		return nullptr;
	}
	IDragonMotorController* getlauncherAngle() const {return launcherAngleDragonSparkMax;}
	IDragonMotorController* getPlacer() const {return PlacerDragonSparkFlexMonitored;}
	IDragonMotorController* getElevator() const {return ElevatorDragonSparkMax;}
	DragonDigitalInput* getfrontIntakeSensor() const {return frontIntakeSensor;}
	DragonDigitalInput* getbackIntakeSensor() const {return backIntakeSensor;}
	DragonDigitalInput* getfeederSensor() const {return feederSensor;}
	DragonDigitalInput* getlauncherSensor() const {return launcherSensor;}
	DragonDigitalInput* getplacerInSensor() const {return placerInSensor;}
	DragonDigitalInput* getplacerMidSensor() const {return placerMidSensor;}
	DragonDigitalInput* getplacerOutSensor() const {return placerOutSensor;}
	DragonCanCoder* getlauncherAngleEncoder() const {return launcherAngleEncoder;}
	ControlData* getpercentOutput() const {return percentOutput;}
	ControlData* getpositionInch() const {return positionInch;}
	ControlData* getvelocityRPS() const {return velocityRPS;}
	ControlData* getposDegreeAbs() const {return posDegreeAbs;}
	ControlData* getpositionInchUp() const {return positionInchUp;}
	ControlData* getLauncherSafePositionControl() const {return LauncherSafePositionControl;}

	static std::map<std::string, STATE_NAMES> stringToSTATE_NAMESEnumMap;

protected:
	RobotConfigMgr::RobotIdentifier m_activeRobotId;
	std::string m_ntName;
	std::string m_tuningIsEnabledStr;
	bool m_tuning = false;
	std::shared_ptr<nt::NetworkTable> m_table;

	void SetCurrentState ( int state, bool run ) override;

private:
	std::unordered_map<RobotElementNames::MOTOR_CONTROLLER_USAGE, BaseMechMotor *> m_motorMap;
	std::unordered_map<RobotElementNames::SOLENOID_USAGE, BaseMechSolenoid *> m_solenoidMap;
	std::unordered_map<RobotElementNames::SERVO_USAGE, BaseMechServo *> m_servoMap;

	std::unordered_map<std::string, STATE_NAMES> m_stateMap;

	DragonSparkMaxMonitored* frontIntakeDragonSparkMaxMonitored;
	DragonSparkMaxMonitored* backIntakeDragonSparkMaxMonitored;
	DragonSparkMaxMonitored* TransferDragonSparkMaxMonitored;
	DragonSparkFlexMonitored* FeederDragonSparkFlexMonitored;
	DragonTalonFX* launcherTopDragonTalonFX;
	DragonTalonFX* launcherBottomDragonTalonFX;
	DragonSparkMax* launcherAngleDragonSparkMax;
	DragonSparkFlexMonitored* PlacerDragonSparkFlexMonitored;
	DragonSparkMax* ElevatorDragonSparkMax;
	DragonSparkFlex* launcherTopDragonSparkFlex;
	DragonSparkFlex* launcherBottomDragonSparkFlex;
	DragonDigitalInput* frontIntakeSensor;
	DragonDigitalInput* backIntakeSensor;
	DragonDigitalInput* feederSensor;
	DragonDigitalInput* launcherSensor;
	DragonDigitalInput* placerInSensor;
	DragonDigitalInput* placerMidSensor;
	DragonDigitalInput* placerOutSensor;
	DragonCanCoder* launcherAngleEncoder;
	ControlData* percentOutput;
	ControlData* positionInch;
	ControlData* velocityRPS;
	ControlData* posDegreeAbs;
	ControlData* positionInchUp;
	ControlData* LauncherSafePositionControl;

	void CheckForTuningEnabled();
	void ReadTuningParamsFromNT();
	void PushTuningParamsToNT();

};