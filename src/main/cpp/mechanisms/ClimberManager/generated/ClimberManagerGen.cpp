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
// Generated on Sunday, February 4, 2024 10:47:03 AM

#include <string>

// FRC Includes
#include <networktables/NetworkTableInstance.h>
#include "hw/interfaces/IDragonMotorController.h"

#include "ClimberManagerGen.h"

ClimberManagerGen::ClimberManagerGen() : BaseMech ( MechanismTypes::MECHANISM_TYPE::CLIMBER_MANAGER, "", std::string ( "ClimberManager" ) ),
	m_motorMap(),
	m_solenoidMap(),
	m_servoMap()
{
}

void ClimberManagerGen::Create()
{
	m_ntName = "ClimberManager";

	DistanceAngleCalcStruc leftClimberCalcStruct;
	leftClimberCalcStruct.countsPerRev = 1 ;
	leftClimberCalcStruct.gearRatio = 1 ;
	leftClimberCalcStruct.diameter = units::length::inch_t ( units::length::meter_t ( 1 ) ).to<double>() ;
	leftClimberCalcStruct.countsPerInch = 1 ;
	leftClimberCalcStruct.countsPerDegree = 1 ;;
	leftClimber = new DragonSparkMax ( 16,RobotElementNames::MOTOR_CONTROLLER_USAGE::CLIMBER_MANAGER_LEFT_CLIMBER,rev::CANSparkMax::MotorType::kBrushless,rev::SparkRelativeEncoder::Type::kHallSensor,rev::SparkLimitSwitch::Type::kNormallyOpen,rev::SparkLimitSwitch::Type::kNormallyOpen,leftClimberCalcStruct );
	m_motorMap[leftClimber->GetType()] = new BaseMechMotor ( m_ntName,
	    leftClimber,
	    BaseMechMotor::EndOfTravelSensorOption::NONE,
	    nullptr,
	    BaseMechMotor::EndOfTravelSensorOption::NONE,
	    nullptr );

	DistanceAngleCalcStruc rightClimberCalcStruct;
	rightClimberCalcStruct.countsPerRev = 0 ;
	rightClimberCalcStruct.gearRatio = 1 ;
	rightClimberCalcStruct.diameter = units::length::inch_t ( units::length::meter_t ( 1 ) ).to<double>() ;
	rightClimberCalcStruct.countsPerInch = 0 ;
	rightClimberCalcStruct.countsPerDegree = 0 ;;
	rightClimber = new DragonSparkMax ( 15,RobotElementNames::MOTOR_CONTROLLER_USAGE::CLIMBER_MANAGER_RIGHT_CLIMBER,rev::CANSparkMax::MotorType::kBrushless,rev::SparkRelativeEncoder::Type::kHallSensor,rev::SparkLimitSwitch::Type::kNormallyOpen,rev::SparkLimitSwitch::Type::kNormallyOpen,rightClimberCalcStruct );
	m_motorMap[rightClimber->GetType()] = new BaseMechMotor ( m_ntName,
	    rightClimber,
	    BaseMechMotor::EndOfTravelSensorOption::NONE,
	    nullptr,
	    BaseMechMotor::EndOfTravelSensorOption::NONE,
	    nullptr );

	climberPosInch = new ControlData (
	    ControlModes::CONTROL_TYPE::POSITION_INCH, // ControlModes::CONTROL_TYPE mode
	    ControlModes::CONTROL_RUN_LOCS::MOTOR_CONTROLLER, // ControlModes::CONTROL_RUN_LOCS server
	    "climberPosInch", // std::string indentifier
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

void ClimberManagerGen::Initialize ( RobotConfigMgr::RobotIdentifier robotFullName )
{
	if ( false ) {}
	else if ( RobotConfigMgr::RobotIdentifier::PRACTICE_BOT_9999 == robotFullName )
	{

		leftClimber->SetRemoteSensor ( 0,
		                               ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_Off );
		leftClimber->Invert ( true );
		leftClimber->EnableBrakeMode ( true );
		leftClimber->SetSmartCurrentLimiting ( 50 );
		leftClimber->SetSecondaryCurrentLimiting ( 50, 0 );
		leftClimber->SetDiameter ( units::length::inch_t ( units::length::meter_t ( 1 ) ).to<double>() );
		leftClimber->EnableDisableLimitSwitches ( false );
// leftClimber : Follower motor mode is not enabled
		leftClimber->SetVoltageRamping ( units::time::second_t ( units::time::second_t ( 0.2 ) ).to<double>(),
		                                 units::time::second_t ( units::time::second_t ( 0.2 ) ).to<double>() );
		leftClimber->SetSensorInverted ( false );

		rightClimber->SetRemoteSensor ( 0,
		                                ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_Off );
		rightClimber->Invert ( true );
		rightClimber->EnableBrakeMode ( true );
		rightClimber->SetSmartCurrentLimiting ( 50 );
		rightClimber->SetSecondaryCurrentLimiting ( 50, 0 );
		rightClimber->SetDiameter ( units::length::inch_t ( units::length::meter_t ( 1 ) ).to<double>() );
		rightClimber->EnableDisableLimitSwitches ( false );
// rightClimber : Follower motor mode is not enabled
		rightClimber->SetVoltageRamping ( units::time::second_t ( units::time::second_t ( 0 ) ).to<double>(),
		                                  units::time::second_t ( units::time::second_t ( 0 ) ).to<double>() );
		rightClimber->SetSensorInverted ( false );

// climberPosInch : ControlData does not have initialization needs
// climberPercetOut : ControlData does not have initialization needs

//todo create initialization for Off
//todo create initialization for Initialize
//todo create initialization for Manual
//todo create initialization for autoClimb
//todo create initialization for Hold
	}

}

void ClimberManagerGen::SetTheCurrentState ( STATE_NAMES state, bool run )
{
	SetCurrentState ( static_cast<int> ( state ), run );
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

}

void ClimberManagerGen::PushTuningParamsToNT()
{

}