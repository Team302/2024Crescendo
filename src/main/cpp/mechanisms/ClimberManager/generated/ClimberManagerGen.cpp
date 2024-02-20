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
// This file was automatically generated by the Team 302 code generator version 1.3.0.2
// Generated on Tuesday, February 20, 2024 10:40:07 AM

#include <string>

// FRC Includes
#include <networktables/NetworkTableInstance.h>
#include "hw/interfaces/IDragonMotorController.h"

#include "ClimberManagerGen.h"
#include "utils/logging/Logger.h"

ClimberManagerGen::ClimberManagerGen ( RobotConfigMgr::RobotIdentifier activeRobotId ) : BaseMech ( MechanismTypes::MECHANISM_TYPE::CLIMBER_MANAGER, "", std::string ( "ClimberManager" ) ),
	m_activeRobotId ( activeRobotId ),
	m_motorMap(),
	m_solenoidMap(),
	m_servoMap(),
	m_stateMap()
{
}

std::map<std::string, ClimberManagerGen::STATE_NAMES> ClimberManagerGen::stringToSTATE_NAMESEnumMap
{
	{"STATE_OFF", ClimberManagerGen::STATE_NAMES::STATE_OFF},
	{"STATE_INITIALIZE", ClimberManagerGen::STATE_NAMES::STATE_INITIALIZE},
	{"STATE_MANUAL", ClimberManagerGen::STATE_NAMES::STATE_MANUAL},
	{"STATE_AUTO_CLIMB", ClimberManagerGen::STATE_NAMES::STATE_AUTO_CLIMB},
	{"STATE_HOLD", ClimberManagerGen::STATE_NAMES::STATE_HOLD},};

void ClimberManagerGen::CreatepracticeBot9999()
{
	m_ntName = "ClimberManager";
	DistanceAngleCalcStruc leftClimberCalcStruct;
	leftClimberCalcStruct.countsPerRev = 1 ;
	leftClimberCalcStruct.gearRatio = 1 ;
	leftClimberCalcStruct.diameter = units::length::inch_t ( units::length::meter_t ( 1 ) ).to<double>() ;
	leftClimberCalcStruct.countsPerInch = 0.4135 ;
	leftClimberCalcStruct.countsPerDegree = 1 ;

	leftClimberDragonSparkMax = new DragonSparkMax ( 16,RobotElementNames::MOTOR_CONTROLLER_USAGE::CLIMBER_MANAGER_LEFT_CLIMBER,rev::CANSparkMax::MotorType::kBrushless,rev::SparkRelativeEncoder::Type::kHallSensor,rev::SparkLimitSwitch::Type::kNormallyOpen,rev::SparkLimitSwitch::Type::kNormallyOpen,leftClimberCalcStruct );

	m_motorMap[leftClimberDragonSparkMax->GetType()] = new BaseMechMotor ( m_ntName,
	    leftClimberDragonSparkMax,
	    BaseMechMotor::EndOfTravelSensorOption::NONE,
	    nullptr,
	    BaseMechMotor::EndOfTravelSensorOption::NONE,
	    nullptr );
	DistanceAngleCalcStruc rightClimberCalcStruct;
	rightClimberCalcStruct.countsPerRev = 0 ;
	rightClimberCalcStruct.gearRatio = 1 ;
	rightClimberCalcStruct.diameter = units::length::inch_t ( units::length::meter_t ( 1 ) ).to<double>() ;
	rightClimberCalcStruct.countsPerInch = 0.4135 ;
	rightClimberCalcStruct.countsPerDegree = 0 ;

	rightClimberDragonSparkMax = new DragonSparkMax ( 15,RobotElementNames::MOTOR_CONTROLLER_USAGE::CLIMBER_MANAGER_RIGHT_CLIMBER,rev::CANSparkMax::MotorType::kBrushless,rev::SparkRelativeEncoder::Type::kHallSensor,rev::SparkLimitSwitch::Type::kNormallyOpen,rev::SparkLimitSwitch::Type::kNormallyOpen,rightClimberCalcStruct );

	m_motorMap[rightClimberDragonSparkMax->GetType()] = new BaseMechMotor ( m_ntName,
	    rightClimberDragonSparkMax,
	    BaseMechMotor::EndOfTravelSensorOption::NONE,
	    nullptr,
	    BaseMechMotor::EndOfTravelSensorOption::NONE,
	    nullptr );

	climberPosInch = new ControlData (
	    ControlModes::CONTROL_TYPE::POSITION_INCH, // ControlModes::CONTROL_TYPE mode
	    ControlModes::CONTROL_RUN_LOCS::MOTOR_CONTROLLER, // ControlModes::CONTROL_RUN_LOCS server
	    "climberPosInch", // std::string indentifier
	    0.04, // double proportional
	    1E-05, // double integral
	    0, // double derivative
	    0, // double feedforward
	    ControlData::FEEDFORWARD_TYPE::VOLTAGE, // FEEDFORWARD_TYPE feedforwadType
	    0.25, // double integralZone
	    0, // double maxAcceleration
	    0, // double cruiseVelocity
	    0, // double peakValue
	    0, // double nominalValue
	    false  // bool enableFOC
	);
	climberPercetOut = new ControlData (
	    ControlModes::CONTROL_TYPE::PERCENT_OUTPUT, // ControlModes::CONTROL_TYPE mode
	    ControlModes::CONTROL_RUN_LOCS::MOTOR_CONTROLLER, // ControlModes::CONTROL_RUN_LOCS server
	    "climberPercetOut", // std::string indentifier
	    0, // double proportional
	    0, // double integral
	    0, // double derivative
	    0, // double feedforward
	    ControlData::FEEDFORWARD_TYPE::VOLTAGE, // FEEDFORWARD_TYPE feedforwadType
	    0, // double integralZone
	    0, // double maxAcceleration
	    0, // double cruiseVelocity
	    0, // double peakValue
	    0, // double nominalValue
	    false  // bool enableFOC
	);

	m_table = nt::NetworkTableInstance::GetDefault().GetTable ( m_ntName );
	m_tuningIsEnabledStr = "Enable Tuning for " + m_ntName; // since this string is used every loop, we do not want to create the string every time
	m_table.get()->PutBoolean ( m_tuningIsEnabledStr, m_tuning );
}

void ClimberManagerGen::CreateCompBot302()
{
	m_ntName = "ClimberManager";
	DistanceAngleCalcStruc leftClimberCalcStruct;
	leftClimberCalcStruct.countsPerRev = 1 ;
	leftClimberCalcStruct.gearRatio = 1 ;
	leftClimberCalcStruct.diameter = units::length::inch_t ( units::length::meter_t ( 1 ) ).to<double>() ;
	leftClimberCalcStruct.countsPerInch = 0.4135 ;
	leftClimberCalcStruct.countsPerDegree = 1 ;

	leftClimberDragonSparkMax = new DragonSparkMax ( 16,RobotElementNames::MOTOR_CONTROLLER_USAGE::CLIMBER_MANAGER_LEFT_CLIMBER,rev::CANSparkMax::MotorType::kBrushless,rev::SparkRelativeEncoder::Type::kHallSensor,rev::SparkLimitSwitch::Type::kNormallyOpen,rev::SparkLimitSwitch::Type::kNormallyOpen,leftClimberCalcStruct );

	m_motorMap[leftClimberDragonSparkMax->GetType()] = new BaseMechMotor ( m_ntName,
	    leftClimberDragonSparkMax,
	    BaseMechMotor::EndOfTravelSensorOption::NONE,
	    nullptr,
	    BaseMechMotor::EndOfTravelSensorOption::NONE,
	    nullptr );
	DistanceAngleCalcStruc rightClimberCalcStruct;
	rightClimberCalcStruct.countsPerRev = 0 ;
	rightClimberCalcStruct.gearRatio = 1 ;
	rightClimberCalcStruct.diameter = units::length::inch_t ( units::length::meter_t ( 1 ) ).to<double>() ;
	rightClimberCalcStruct.countsPerInch = 0.4135 ;
	rightClimberCalcStruct.countsPerDegree = 0 ;

	rightClimberDragonSparkMax = new DragonSparkMax ( 15,RobotElementNames::MOTOR_CONTROLLER_USAGE::CLIMBER_MANAGER_RIGHT_CLIMBER,rev::CANSparkMax::MotorType::kBrushless,rev::SparkRelativeEncoder::Type::kHallSensor,rev::SparkLimitSwitch::Type::kNormallyOpen,rev::SparkLimitSwitch::Type::kNormallyOpen,rightClimberCalcStruct );

	m_motorMap[rightClimberDragonSparkMax->GetType()] = new BaseMechMotor ( m_ntName,
	    rightClimberDragonSparkMax,
	    BaseMechMotor::EndOfTravelSensorOption::NONE,
	    nullptr,
	    BaseMechMotor::EndOfTravelSensorOption::NONE,
	    nullptr );

	climberPosInch = new ControlData (
	    ControlModes::CONTROL_TYPE::POSITION_INCH, // ControlModes::CONTROL_TYPE mode
	    ControlModes::CONTROL_RUN_LOCS::MOTOR_CONTROLLER, // ControlModes::CONTROL_RUN_LOCS server
	    "climberPosInch", // std::string indentifier
	    0.04, // double proportional
	    1E-05, // double integral
	    0, // double derivative
	    0, // double feedforward
	    ControlData::FEEDFORWARD_TYPE::VOLTAGE, // FEEDFORWARD_TYPE feedforwadType
	    0.25, // double integralZone
	    0, // double maxAcceleration
	    0, // double cruiseVelocity
	    0, // double peakValue
	    0, // double nominalValue
	    false  // bool enableFOC
	);
	climberPercetOut = new ControlData (
	    ControlModes::CONTROL_TYPE::PERCENT_OUTPUT, // ControlModes::CONTROL_TYPE mode
	    ControlModes::CONTROL_RUN_LOCS::MOTOR_CONTROLLER, // ControlModes::CONTROL_RUN_LOCS server
	    "climberPercetOut", // std::string indentifier
	    0, // double proportional
	    0, // double integral
	    0, // double derivative
	    0, // double feedforward
	    ControlData::FEEDFORWARD_TYPE::VOLTAGE, // FEEDFORWARD_TYPE feedforwadType
	    0, // double integralZone
	    0, // double maxAcceleration
	    0, // double cruiseVelocity
	    0, // double peakValue
	    0, // double nominalValue
	    false  // bool enableFOC
	);

	m_table = nt::NetworkTableInstance::GetDefault().GetTable ( m_ntName );
	m_tuningIsEnabledStr = "Enable Tuning for " + m_ntName; // since this string is used every loop, we do not want to create the string every time
	m_table.get()->PutBoolean ( m_tuningIsEnabledStr, m_tuning );
}

void ClimberManagerGen::InitializepracticeBot9999()
{
	leftClimberDragonSparkMax->SetRemoteSensor ( 0,
	        ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_Off );
	leftClimberDragonSparkMax->Invert ( false );
	leftClimberDragonSparkMax->EnableBrakeMode ( true );
	leftClimberDragonSparkMax->SetSmartCurrentLimiting ( 40 );
	leftClimberDragonSparkMax->SetSecondaryCurrentLimiting ( 40, 0 );
	leftClimberDragonSparkMax->SetDiameter ( units::length::inch_t ( units::length::meter_t ( 1 ) ).to<double>() );
	leftClimberDragonSparkMax->EnableDisableLimitSwitches ( false );
// leftClimber : Follower motor mode is not enabled
	leftClimberDragonSparkMax->SetVoltageRamping ( units::time::second_t ( units::time::second_t ( 0.2 ) ).to<double>(),
	        units::time::second_t ( units::time::second_t ( 0.2 ) ).to<double>() );
	leftClimberDragonSparkMax->SetSensorInverted ( false );

	rightClimberDragonSparkMax->SetRemoteSensor ( 0,
	        ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_Off );
	rightClimberDragonSparkMax->Invert ( true );
	rightClimberDragonSparkMax->EnableBrakeMode ( true );
	rightClimberDragonSparkMax->SetSmartCurrentLimiting ( 40 );
	rightClimberDragonSparkMax->SetSecondaryCurrentLimiting ( 40, 0 );
	rightClimberDragonSparkMax->SetDiameter ( units::length::inch_t ( units::length::meter_t ( 1 ) ).to<double>() );
	rightClimberDragonSparkMax->EnableDisableLimitSwitches ( false );
// rightClimber : Follower motor mode is not enabled
	rightClimberDragonSparkMax->SetVoltageRamping ( units::time::second_t ( units::time::second_t ( 0 ) ).to<double>(),
	        units::time::second_t ( units::time::second_t ( 0 ) ).to<double>() );
	rightClimberDragonSparkMax->SetSensorInverted ( false );

// climberPosInch : ControlData does not have initialization needs
// climberPercetOut : ControlData does not have initialization needs

//todo create initialization for Off
//todo create initialization for Initialize
//todo create initialization for Manual
//todo create initialization for autoClimb
//todo create initialization for Hold
}

void ClimberManagerGen::InitializeCompBot302()
{
	leftClimberDragonSparkMax->SetRemoteSensor ( 0,
	        ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_Off );
	leftClimberDragonSparkMax->Invert ( false );
	leftClimberDragonSparkMax->EnableBrakeMode ( true );
	leftClimberDragonSparkMax->SetSmartCurrentLimiting ( 40 );
	leftClimberDragonSparkMax->SetSecondaryCurrentLimiting ( 40, 0 );
	leftClimberDragonSparkMax->SetDiameter ( units::length::inch_t ( units::length::meter_t ( 1 ) ).to<double>() );
	leftClimberDragonSparkMax->EnableDisableLimitSwitches ( false );
// leftClimber : Follower motor mode is not enabled
	leftClimberDragonSparkMax->SetVoltageRamping ( units::time::second_t ( units::time::second_t ( 0.2 ) ).to<double>(),
	        units::time::second_t ( units::time::second_t ( 0.2 ) ).to<double>() );
	leftClimberDragonSparkMax->SetSensorInverted ( false );

	rightClimberDragonSparkMax->SetRemoteSensor ( 0,
	        ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_Off );
	rightClimberDragonSparkMax->Invert ( true );
	rightClimberDragonSparkMax->EnableBrakeMode ( true );
	rightClimberDragonSparkMax->SetSmartCurrentLimiting ( 40 );
	rightClimberDragonSparkMax->SetSecondaryCurrentLimiting ( 40, 0 );
	rightClimberDragonSparkMax->SetDiameter ( units::length::inch_t ( units::length::meter_t ( 1 ) ).to<double>() );
	rightClimberDragonSparkMax->EnableDisableLimitSwitches ( false );
// rightClimber : Follower motor mode is not enabled
	rightClimberDragonSparkMax->SetVoltageRamping ( units::time::second_t ( units::time::second_t ( 0 ) ).to<double>(),
	        units::time::second_t ( units::time::second_t ( 0 ) ).to<double>() );
	rightClimberDragonSparkMax->SetSensorInverted ( false );

// climberPosInch : ControlData does not have initialization needs
// climberPercetOut : ControlData does not have initialization needs

//todo create initialization for Off
//todo create initialization for Initialize
//todo create initialization for Manual
//todo create initialization for autoClimb
//todo create initialization for Hold
}

void ClimberManagerGen::SetCurrentState ( int state, bool run )
{
	StateMgr::SetCurrentState ( state, run );
}

/// @brief  Set the control constants (e.g. PIDF values).
/// @param [in] ControlData*                                   pid:  the control constants
/// @return void
void ClimberManagerGen::SetControlConstants ( RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, int slot, ControlData pid )
{
	auto motor = GetMotorMech ( identifier );
	if ( motor != nullptr )
	{
		motor->SetControlConstants ( slot, pid );
	}
}

/// @brief update the output to the mechanism using the current controller and target value(s)
/// @return void
void ClimberManagerGen::Update()
{
	for ( auto motor : m_motorMap )
	{
		motor.second->Update();
	}
}

void ClimberManagerGen::UpdateTarget ( RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, double percentOutput )
{
	auto motor = GetMotorMech ( identifier );
	if ( motor != nullptr )
	{
		motor->UpdateTarget ( percentOutput );
	}
}

void ClimberManagerGen::UpdateTarget ( RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, units::angle::degree_t angle )
{
	auto motor = GetMotorMech ( identifier );
	if ( motor != nullptr )
	{
		motor->UpdateTarget ( angle );
	}
}

void ClimberManagerGen::UpdateTarget ( RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, units::angular_velocity::revolutions_per_minute_t angVel )
{
	auto motor = GetMotorMech ( identifier );
	if ( motor != nullptr )
	{
		motor->UpdateTarget ( angVel );
	}
}
void ClimberManagerGen::UpdateTarget ( RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, units::length::inch_t position )
{
	auto motor = GetMotorMech ( identifier );
	if ( motor != nullptr )
	{
		motor->UpdateTarget ( position );
	}
}
void ClimberManagerGen::UpdateTarget ( RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, units::velocity::feet_per_second_t velocity )
{
	auto motor = GetMotorMech ( identifier );
	if ( motor != nullptr )
	{
		motor->UpdateTarget ( velocity );
	}
}

bool ClimberManagerGen::IsAtMinPosition ( RobotElementNames::MOTOR_CONTROLLER_USAGE identifier ) const
{
	auto motor = GetMotorMech ( identifier );
	if ( motor != nullptr )
	{
		return motor->IsAtMinTravel();
	}
	return false;
}

bool ClimberManagerGen::IsAtMaxPosition ( RobotElementNames::MOTOR_CONTROLLER_USAGE identifier ) const
{
	auto motor = GetMotorMech ( identifier );
	if ( motor != nullptr )
	{
		return motor->IsAtMaxTravel();
	}
	return false;
}

BaseMechMotor *ClimberManagerGen::GetMotorMech ( RobotElementNames::MOTOR_CONTROLLER_USAGE usage ) const
{
	auto itr = m_motorMap.find ( usage );
	if ( itr != m_motorMap.end() )
	{
		return itr->second;
	}
	return nullptr;
}

std::vector<RobotElementNames::MOTOR_CONTROLLER_USAGE> ClimberManagerGen::GetMotorUsages() const
{
	std::vector<RobotElementNames::MOTOR_CONTROLLER_USAGE> output;
	for ( auto itr = m_motorMap.begin(); itr != m_motorMap.end(); ++itr )
	{
		output.emplace_back ( itr->first );
	}
	return output;
}

void ClimberManagerGen::UpdateTarget ( RobotElementNames::SOLENOID_USAGE identifier, bool extend )
{
	auto sol = GetSolenoidMech ( identifier );
	if ( sol != nullptr )
	{
		sol->ActivateSolenoid ( extend );
	}
}

bool ClimberManagerGen::IsAtMinPosition ( RobotElementNames::SOLENOID_USAGE identifier ) const
{
	auto sol = GetSolenoidMech ( identifier );
	if ( sol != nullptr )
	{
		return !sol->IsSolenoidActivated();
	}
	return false;
}

bool ClimberManagerGen::IsAtMaxPosition ( RobotElementNames::SOLENOID_USAGE identifier ) const
{
	auto sol = GetSolenoidMech ( identifier );
	if ( sol != nullptr )
	{
		return sol->IsSolenoidActivated();
	}
	return false;
}

BaseMechSolenoid *ClimberManagerGen::GetSolenoidMech ( RobotElementNames::SOLENOID_USAGE usage ) const
{
	auto itr = m_solenoidMap.find ( usage );
	if ( itr != m_solenoidMap.end() )
	{
		return itr->second;
	}
	return nullptr;
}

std::vector<RobotElementNames::SOLENOID_USAGE> ClimberManagerGen::GetSolenoidUsages() const
{
	std::vector<RobotElementNames::SOLENOID_USAGE> output;
	for ( auto itr = m_solenoidMap.begin(); itr != m_solenoidMap.end(); ++itr )
	{
		output.emplace_back ( itr->first );
	}
	return output;
}

BaseMechServo *ClimberManagerGen::GetServoMech ( RobotElementNames::SERVO_USAGE usage ) const
{
	auto itr = m_servoMap.find ( usage );
	if ( itr != m_servoMap.end() )
	{
		return itr->second;
	}
	return nullptr;
}

std::vector<RobotElementNames::SERVO_USAGE> ClimberManagerGen::GetServoUsages() const
{
	std::vector<RobotElementNames::SERVO_USAGE> output;
	for ( auto itr = m_servoMap.begin(); itr != m_servoMap.end(); ++itr )
	{
		output.emplace_back ( itr->first );
	}
	return output;
}

void ClimberManagerGen::Cyclic()
{
	CheckForTuningEnabled();
	if ( m_tuning )
	{
		ReadTuningParamsFromNT();
	}
}

void ClimberManagerGen::CheckForTuningEnabled()
{
	bool pastTuning = m_tuning;
	m_tuning = m_table.get()->GetBoolean ( m_tuningIsEnabledStr, false );
	if ( pastTuning != m_tuning && m_tuning == true )
	{
		PushTuningParamsToNT();
	}
}

void ClimberManagerGen::ReadTuningParamsFromNT()
{
	climberPosInch->SetIZone ( m_table.get()->GetNumber ( "climberPosInch_iZone", 0.25 ) );
	climberPosInch->SetF ( m_table.get()->GetNumber ( "climberPosInch_fGain", 0 ) );
	climberPosInch->SetP ( m_table.get()->GetNumber ( "climberPosInch_pGain", 0.04 ) );
	climberPosInch->SetI ( m_table.get()->GetNumber ( "climberPosInch_iGain", 1E-05 ) );
	climberPosInch->SetD ( m_table.get()->GetNumber ( "climberPosInch_dGain", 0 ) );

}

void ClimberManagerGen::PushTuningParamsToNT()
{
	m_table.get()->PutNumber ( "climberPosInch_iZone", climberPosInch->GetIZone() );
	m_table.get()->PutNumber ( "climberPosInch_fGain", climberPosInch->GetF() );
	m_table.get()->PutNumber ( "climberPosInch_pGain", climberPosInch->GetP() );
	m_table.get()->PutNumber ( "climberPosInch_iGain", climberPosInch->GetI() );
	m_table.get()->PutNumber ( "climberPosInch_dGain", climberPosInch->GetD() );

}