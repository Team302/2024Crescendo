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
// This file was automatically generated by the Team 302 code generator version 1.2.3.1
// Generated on Monday, February 5, 2024 8:06:36 PM

#include <string>

// FRC Includes
#include <networktables/NetworkTableInstance.h>
#include "hw/interfaces/IDragonMotorController.h"

#include "Thing1MechGen.h"
#include "utils/logging/Logger.h"

using ctre::phoenix6::signals::ForwardLimitSourceValue;
using ctre::phoenix6::signals::ForwardLimitTypeValue;
using ctre::phoenix6::signals::ReverseLimitSourceValue;
using ctre::phoenix6::signals::ReverseLimitTypeValue;
using ctre::phoenix6::signals::InvertedValue;
using ctre::phoenix6::signals::NeutralModeValue;
using ctre::phoenix::motorcontrol::RemoteSensorSource;

Thing1MechGen::Thing1MechGen() : BaseMech ( MechanismTypes::MECHANISM_TYPE::THING1MECH, "", std::string ( "Thing1Mech" ) ),
	m_motorMap(),
	m_solenoidMap(),
	m_servoMap(),
	m_stateMap()
{
}

void Thing1MechGen::Create()
{
	m_ntName = "Thing1Mech";
	DistanceAngleCalcStruc BackLeftMotorCalcStruct;
	BackLeftMotorCalcStruct.countsPerRev = 0 ;
	BackLeftMotorCalcStruct.gearRatio = 1 ;
	BackLeftMotorCalcStruct.diameter = units::length::inch_t ( units::length::meter_t ( 1 ) ).to<double>() ;
	BackLeftMotorCalcStruct.countsPerInch = 0 ;
	BackLeftMotorCalcStruct.countsPerDegree = 0 ;;
	BackLeftMotor = new DragonTalonSRX ( "BackLeftMotor",
	                                     RobotElementNames::MOTOR_CONTROLLER_USAGE::THING1MECH_BACK_LEFT_MOTOR,
	                                     0,
	                                     0,
	                                     BackLeftMotorCalcStruct,
	                                     IDragonMotorController::MOTOR_TYPE::ANDYMARKNEVEREST );
	m_motorMap[BackLeftMotor->GetType()] = new BaseMechMotor ( m_ntName,
	    BackLeftMotor,
	    BaseMechMotor::EndOfTravelSensorOption::NONE,
	    nullptr,
	    BaseMechMotor::EndOfTravelSensorOption::NONE,
	    nullptr );


	DistanceAngleCalcStruc RightBackMotorCalcStruct;
	RightBackMotorCalcStruct.countsPerRev = 0 ;
	RightBackMotorCalcStruct.gearRatio = 1 ;
	RightBackMotorCalcStruct.diameter = units::length::inch_t ( units::length::meter_t ( 1 ) ).to<double>() ;
	RightBackMotorCalcStruct.countsPerInch = 0 ;
	RightBackMotorCalcStruct.countsPerDegree = 0 ;;
	RightBackMotor = new DragonTalonSRX ( "RightBackMotor",
	                                      RobotElementNames::MOTOR_CONTROLLER_USAGE::THING1MECH_RIGHT_BACK_MOTOR,
	                                      1,
	                                      0,
	                                      RightBackMotorCalcStruct,
	                                      IDragonMotorController::MOTOR_TYPE::ANDYMARKNEVEREST );
	m_motorMap[RightBackMotor->GetType()] = new BaseMechMotor ( m_ntName,
	    RightBackMotor,
	    BaseMechMotor::EndOfTravelSensorOption::NONE,
	    nullptr,
	    BaseMechMotor::EndOfTravelSensorOption::NONE,
	    nullptr );


	DistanceAngleCalcStruc LeftFrontMotorCalcStruct;
	LeftFrontMotorCalcStruct.countsPerRev = 0 ;
	LeftFrontMotorCalcStruct.gearRatio = 1 ;
	LeftFrontMotorCalcStruct.diameter = units::length::inch_t ( units::length::meter_t ( 1 ) ).to<double>() ;
	LeftFrontMotorCalcStruct.countsPerInch = 0 ;
	LeftFrontMotorCalcStruct.countsPerDegree = 0 ;;
	LeftFrontMotor = new DragonTalonSRX ( "LeftFrontMotor",
	                                      RobotElementNames::MOTOR_CONTROLLER_USAGE::THING1MECH_LEFT_FRONT_MOTOR,
	                                      2,
	                                      0,
	                                      LeftFrontMotorCalcStruct,
	                                      IDragonMotorController::MOTOR_TYPE::ANDYMARKNEVEREST );
	m_motorMap[LeftFrontMotor->GetType()] = new BaseMechMotor ( m_ntName,
	    LeftFrontMotor,
	    BaseMechMotor::EndOfTravelSensorOption::NONE,
	    nullptr,
	    BaseMechMotor::EndOfTravelSensorOption::NONE,
	    nullptr );


	DistanceAngleCalcStruc RightFrontMotorCalcStruct;
	RightFrontMotorCalcStruct.countsPerRev = 0 ;
	RightFrontMotorCalcStruct.gearRatio = 1 ;
	RightFrontMotorCalcStruct.diameter = units::length::inch_t ( units::length::meter_t ( 1 ) ).to<double>() ;
	RightFrontMotorCalcStruct.countsPerInch = 0 ;
	RightFrontMotorCalcStruct.countsPerDegree = 0 ;;
	RightFrontMotor = new DragonTalonSRX ( "RightFrontMotor",
	                                       RobotElementNames::MOTOR_CONTROLLER_USAGE::THING1MECH_RIGHT_FRONT_MOTOR,
	                                       3,
	                                       0,
	                                       RightFrontMotorCalcStruct,
	                                       IDragonMotorController::MOTOR_TYPE::ANDYMARKNEVEREST );
	m_motorMap[RightFrontMotor->GetType()] = new BaseMechMotor ( m_ntName,
	    RightFrontMotor,
	    BaseMechMotor::EndOfTravelSensorOption::NONE,
	    nullptr,
	    BaseMechMotor::EndOfTravelSensorOption::NONE,
	    nullptr );


	DistanceAngleCalcStruc FlaconCalcStruct;
	FlaconCalcStruct.countsPerRev = 0 ;
	FlaconCalcStruct.gearRatio = 1 ;
	FlaconCalcStruct.diameter = units::length::inch_t ( units::length::meter_t ( 1 ) ).to<double>() ;
	FlaconCalcStruct.countsPerInch = 0 ;
	FlaconCalcStruct.countsPerDegree = 0 ;;
	Flacon = new DragonTalonFX ( "Flacon",RobotElementNames::MOTOR_CONTROLLER_USAGE::THING1MECH_FLACON,11, FlaconCalcStruct, IDragonMotorController::MOTOR_TYPE::FALCON500, "rio" );
	m_motorMap[Flacon->GetType()] = new BaseMechMotor ( m_ntName,
	    Flacon,
	    BaseMechMotor::EndOfTravelSensorOption::NONE,
	    nullptr,
	    BaseMechMotor::EndOfTravelSensorOption::NONE,
	    nullptr );


	DistanceAngleCalcStruc Neo550CalcStruct;
	Neo550CalcStruct.countsPerRev = 0 ;
	Neo550CalcStruct.gearRatio = 1 ;
	Neo550CalcStruct.diameter = units::length::inch_t ( units::length::meter_t ( 1 ) ).to<double>() ;
	Neo550CalcStruct.countsPerInch = 0 ;
	Neo550CalcStruct.countsPerDegree = 0 ;;
	Neo550 = new DragonSparkMax ( 4,RobotElementNames::MOTOR_CONTROLLER_USAGE::THING1MECH_NEO550,rev::CANSparkMax::MotorType::kBrushless,rev::SparkRelativeEncoder::Type::kHallSensor,rev::SparkLimitSwitch::Type::kNormallyOpen,rev::SparkLimitSwitch::Type::kNormallyOpen,Neo550CalcStruct );
	m_motorMap[Neo550->GetType()] = new BaseMechMotor ( m_ntName,
	    Neo550,
	    BaseMechMotor::EndOfTravelSensorOption::NONE,
	    nullptr,
	    BaseMechMotor::EndOfTravelSensorOption::NONE,
	    nullptr );


	DistanceAngleCalcStruc VortexCalcStruct;
	VortexCalcStruct.countsPerRev = 0 ;
	VortexCalcStruct.gearRatio = 1 ;
	VortexCalcStruct.diameter = units::length::inch_t ( units::length::meter_t ( 1 ) ).to<double>() ;
	VortexCalcStruct.countsPerInch = 0 ;
	VortexCalcStruct.countsPerDegree = 0 ;;
	Vortex = new DragonSparkFlex ( 5,RobotElementNames::MOTOR_CONTROLLER_USAGE::THING1MECH_VORTEX,rev::CANSparkFlex::MotorType::kBrushless,rev::SparkRelativeEncoder::Type::kHallSensor,rev::SparkLimitSwitch::Type::kNormallyOpen,rev::SparkLimitSwitch::Type::kNormallyOpen,VortexCalcStruct );
	m_motorMap[Vortex->GetType()] = new BaseMechMotor ( m_ntName,
	    Vortex,
	    BaseMechMotor::EndOfTravelSensorOption::NONE,
	    nullptr,
	    BaseMechMotor::EndOfTravelSensorOption::NONE,
	    nullptr );








	percentControlData = new ControlData (
	    ControlModes::CONTROL_TYPE::PERCENT_OUTPUT, // ControlModes::CONTROL_TYPE mode
	    ControlModes::CONTROL_RUN_LOCS::MOTOR_CONTROLLER, // ControlModes::CONTROL_RUN_LOCS server
	    "percentControlData", // std::string indentifier
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

std::map<std::string, Thing1MechGen::STATE_NAMES> Thing1MechGen::stringToSTATE_NAMESEnumMap
{
	{"STATE_LEFT_FRONT_CW", Thing1MechGen::STATE_NAMES::STATE_LEFT_FRONT_CW},
	{"STATE_RIGHT_FRONT_CW", Thing1MechGen::STATE_NAMES::STATE_RIGHT_FRONT_CW},
	{"STATE_RIGHT_BACK_CW", Thing1MechGen::STATE_NAMES::STATE_RIGHT_BACK_CW},
	{"STATE_LEFT_BACK_CW", Thing1MechGen::STATE_NAMES::STATE_LEFT_BACK_CW},
	{"STATE_SPARKY_ON", Thing1MechGen::STATE_NAMES::STATE_SPARKY_ON},
	{"STATE_THING1TALON", Thing1MechGen::STATE_NAMES::STATE_THING1TALON},};

void Thing1MechGen::Initialize ( RobotConfigMgr::RobotIdentifier robotFullName )
{
	if ( false ) {}
	else if ( RobotConfigMgr::RobotIdentifier::THING_1 == robotFullName )
	{

		BackLeftMotor->SetRemoteSensor ( 0,
		                                 ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_Off );
		BackLeftMotor->Invert ( true );
		BackLeftMotor->EnableBrakeMode ( false );
		BackLeftMotor->EnableCurrentLimiting ( false );
		BackLeftMotor->ConfigPeakCurrentLimit ( units::current::ampere_t ( units::current::ampere_t ( 0 ) ).to<int>(),
		                                        units::time::millisecond_t ( units::time::second_t ( 0 ) ).to<int>() );
		BackLeftMotor->ConfigPeakCurrentDuration ( units::time::millisecond_t ( units::time::second_t ( 0 ) ).to<int>(),
		        units::time::millisecond_t ( units::time::second_t ( 0 ) ).to<int>() );
		BackLeftMotor->ConfigContinuousCurrentLimit ( units::current::ampere_t ( units::current::ampere_t ( 0 ) ).to<int>(),
		        units::time::millisecond_t ( units::time::second_t ( 0 ) ).to<int>() );
		BackLeftMotor->SetDiameter ( units::length::inch_t ( units::length::meter_t ( 1 ) ).to<double>() );
		BackLeftMotor->EnableDisableLimitSwitches ( false );
		BackLeftMotor->SetForwardLimitSwitch ( true );
		BackLeftMotor->SetReverseLimitSwitch ( true );
// BackLeftMotor : Follower motor mode is not enabled
		BackLeftMotor->SetVoltageRamping ( units::time::second_t ( units::time::second_t ( 0 ) ).to<double>(),
		                                   units::time::second_t ( units::time::second_t ( 0 ) ).to<double>() );
		BackLeftMotor->SetSensorInverted ( false );

		RightBackMotor->SetRemoteSensor ( 0,
		                                  ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_Off );
		RightBackMotor->Invert ( true );
		RightBackMotor->EnableBrakeMode ( false );
		RightBackMotor->EnableCurrentLimiting ( false );
		RightBackMotor->ConfigPeakCurrentLimit ( units::current::ampere_t ( units::current::ampere_t ( 0 ) ).to<int>(),
		        units::time::millisecond_t ( units::time::second_t ( 0 ) ).to<int>() );
		RightBackMotor->ConfigPeakCurrentDuration ( units::time::millisecond_t ( units::time::second_t ( 0 ) ).to<int>(),
		        units::time::millisecond_t ( units::time::second_t ( 0 ) ).to<int>() );
		RightBackMotor->ConfigContinuousCurrentLimit ( units::current::ampere_t ( units::current::ampere_t ( 0 ) ).to<int>(),
		        units::time::millisecond_t ( units::time::second_t ( 0 ) ).to<int>() );
		RightBackMotor->SetDiameter ( units::length::inch_t ( units::length::meter_t ( 1 ) ).to<double>() );
		RightBackMotor->EnableDisableLimitSwitches ( false );
		RightBackMotor->SetForwardLimitSwitch ( true );
		RightBackMotor->SetReverseLimitSwitch ( true );
// RightBackMotor : Follower motor mode is not enabled
		RightBackMotor->SetVoltageRamping ( units::time::second_t ( units::time::second_t ( 0 ) ).to<double>(),
		                                    units::time::second_t ( units::time::second_t ( 0 ) ).to<double>() );
		RightBackMotor->SetSensorInverted ( false );

		LeftFrontMotor->SetRemoteSensor ( 0,
		                                  ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_Off );
		LeftFrontMotor->Invert ( true );
		LeftFrontMotor->EnableBrakeMode ( false );
		LeftFrontMotor->EnableCurrentLimiting ( false );
		LeftFrontMotor->ConfigPeakCurrentLimit ( units::current::ampere_t ( units::current::ampere_t ( 0 ) ).to<int>(),
		        units::time::millisecond_t ( units::time::second_t ( 0 ) ).to<int>() );
		LeftFrontMotor->ConfigPeakCurrentDuration ( units::time::millisecond_t ( units::time::second_t ( 0 ) ).to<int>(),
		        units::time::millisecond_t ( units::time::second_t ( 0 ) ).to<int>() );
		LeftFrontMotor->ConfigContinuousCurrentLimit ( units::current::ampere_t ( units::current::ampere_t ( 0 ) ).to<int>(),
		        units::time::millisecond_t ( units::time::second_t ( 0 ) ).to<int>() );
		LeftFrontMotor->SetDiameter ( units::length::inch_t ( units::length::meter_t ( 1 ) ).to<double>() );
		LeftFrontMotor->EnableDisableLimitSwitches ( false );
		LeftFrontMotor->SetForwardLimitSwitch ( true );
		LeftFrontMotor->SetReverseLimitSwitch ( true );
// LeftFrontMotor : Follower motor mode is not enabled
		LeftFrontMotor->SetVoltageRamping ( units::time::second_t ( units::time::second_t ( 0 ) ).to<double>(),
		                                    units::time::second_t ( units::time::second_t ( 0 ) ).to<double>() );
		LeftFrontMotor->SetSensorInverted ( false );

		RightFrontMotor->SetRemoteSensor ( 0,
		                                   ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_Off );
		RightFrontMotor->Invert ( true );
		RightFrontMotor->EnableBrakeMode ( false );
		RightFrontMotor->EnableCurrentLimiting ( false );
		RightFrontMotor->ConfigPeakCurrentLimit ( units::current::ampere_t ( units::current::ampere_t ( 0 ) ).to<int>(),
		        units::time::millisecond_t ( units::time::second_t ( 0 ) ).to<int>() );
		RightFrontMotor->ConfigPeakCurrentDuration ( units::time::millisecond_t ( units::time::second_t ( 0 ) ).to<int>(),
		        units::time::millisecond_t ( units::time::second_t ( 0 ) ).to<int>() );
		RightFrontMotor->ConfigContinuousCurrentLimit ( units::current::ampere_t ( units::current::ampere_t ( 0 ) ).to<int>(),
		        units::time::millisecond_t ( units::time::second_t ( 0 ) ).to<int>() );
		RightFrontMotor->SetDiameter ( units::length::inch_t ( units::length::meter_t ( 1 ) ).to<double>() );
		RightFrontMotor->EnableDisableLimitSwitches ( false );
		RightFrontMotor->SetForwardLimitSwitch ( true );
		RightFrontMotor->SetReverseLimitSwitch ( true );
// RightFrontMotor : Follower motor mode is not enabled
		RightFrontMotor->SetVoltageRamping ( units::time::second_t ( units::time::second_t ( 0 ) ).to<double>(),
		                                     units::time::second_t ( units::time::second_t ( 0 ) ).to<double>() );
		RightFrontMotor->SetSensorInverted ( false );

		Flacon->SetCurrentLimits ( false,
		                           units::current::ampere_t ( 0 ),
		                           false,
		                           units::current::ampere_t ( 0 ),
		                           units::current::ampere_t ( 0 ),
		                           units::time::second_t ( 0 ) );
		Flacon->ConfigHWLimitSW ( false, // enableForward
		                          0, // remoteForwardSensorID
		                          false, // forwardResetPosition
		                          0, // forwardPosition
		                          ForwardLimitSourceValue::LimitSwitchPin, // forwardType
		                          ForwardLimitTypeValue::NormallyOpen, // forwardOpenClose
		                          false, // enableReverse
		                          0, // remoteReverseSensorID
		                          false, // reverseResetPosition
		                          0, // reversePosition
		                          ReverseLimitSourceValue::LimitSwitchPin, // revType
		                          ReverseLimitTypeValue::NormallyOpen ); // revOpenClose
		Flacon->ConfigMotorSettings ( ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive, // ctre::phoenixpro::signals::InvertedValue
		                              ctre::phoenix6::signals::NeutralModeValue::Coast, // ctre::phoenixpro::signals::NeutralModeValue
		                              0, // deadbandPercent
		                              0, // peakForwardDutyCycle
		                              0 ); // peakReverseDutyCycle
		Flacon->SetAsFollowerMotor ( 0 ); // masterCANID
		Flacon->SetRemoteSensor ( 0, // canID
		                          RemoteSensorSource::RemoteSensorSource_Off ); // ctre::phoenix::motorcontrol::RemoteSensorSource
		Flacon->SetDiameter ( 0 ); // double diameter
		Flacon->SetVoltageRamping ( units::time::second_t ( units::time::second_t ( 0 ) ).to<double>(),
		                            units::time::second_t ( units::time::second_t ( 0 ) ).to<double>() );
		Flacon->SetSensorInverted ( false );
		Neo550->SetRemoteSensor ( 0,
		                          ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_Off );
		Neo550->Invert ( true );
		Neo550->EnableBrakeMode ( false );
		Neo550->SetSmartCurrentLimiting ( 50 );
		Neo550->SetSecondaryCurrentLimiting ( 50, 0 );
		Neo550->SetDiameter ( units::length::inch_t ( units::length::meter_t ( 1 ) ).to<double>() );
		Neo550->EnableDisableLimitSwitches ( false );
// Neo550 : Follower motor mode is not enabled
		Neo550->SetVoltageRamping ( units::time::second_t ( units::time::second_t ( 0 ) ).to<double>(),
		                            units::time::second_t ( units::time::second_t ( 0 ) ).to<double>() );
		Neo550->SetSensorInverted ( false );

		Vortex->SetRemoteSensor ( 0,
		                          ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_Off );
		Vortex->Invert ( true );
		Vortex->EnableBrakeMode ( false );
		Vortex->SetSmartCurrentLimiting ( 50 );
		Vortex->SetSecondaryCurrentLimiting ( 50, 0 );
		Vortex->SetDiameter ( units::length::inch_t ( units::length::meter_t ( 1 ) ).to<double>() );
		Vortex->EnableDisableLimitSwitches ( false );
// Vortex : Follower motor mode is not enabled
		Vortex->SetVoltageRamping ( units::time::second_t ( units::time::second_t ( 0 ) ).to<double>(),
		                            units::time::second_t ( units::time::second_t ( 0 ) ).to<double>() );
		Vortex->SetSensorInverted ( false );








// percentControlData : ControlData does not have initialization needs

//todo create initialization for leftFrontCW
//todo create initialization for rightFrontCW
//todo create initialization for rightBackCW
//todo create initialization for leftBackCW
//todo create initialization for sparkyOn
//todo create initialization for thing1Talon
	}
}

void Thing1MechGen::SetCurrentState ( int state, bool run )
{
	StateMgr::SetCurrentState ( state, run );
}


/// @brief  Set the control constants (e.g. PIDF values).
/// @param [in] ControlData*                                   pid:  the control constants
/// @return void
void Thing1MechGen::SetControlConstants ( RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, int slot, ControlData pid )
{
	auto motor = GetMotorMech ( identifier );
	if ( motor != nullptr )
	{
		motor->SetControlConstants ( slot, pid );
	}
}

/// @brief update the output to the mechanism using the current controller and target value(s)
/// @return void
void Thing1MechGen::Update()
{
	for ( auto motor : m_motorMap )
	{
		motor.second->Update();
	}
}

void Thing1MechGen::UpdateTarget ( RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, double percentOutput )
{
	auto motor = GetMotorMech ( identifier );
	if ( motor != nullptr )
	{
		motor->UpdateTarget ( percentOutput );
	}
}

void Thing1MechGen::UpdateTarget ( RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, units::angle::degree_t angle )
{
	auto motor = GetMotorMech ( identifier );
	if ( motor != nullptr )
	{
		motor->UpdateTarget ( angle );
	}
}

void Thing1MechGen::UpdateTarget ( RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, units::angular_velocity::revolutions_per_minute_t angVel )
{
	auto motor = GetMotorMech ( identifier );
	if ( motor != nullptr )
	{
		motor->UpdateTarget ( angVel );
	}
}
void Thing1MechGen::UpdateTarget ( RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, units::length::inch_t position )
{
	auto motor = GetMotorMech ( identifier );
	if ( motor != nullptr )
	{
		motor->UpdateTarget ( position );
	}
}
void Thing1MechGen::UpdateTarget ( RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, units::velocity::feet_per_second_t velocity )
{
	auto motor = GetMotorMech ( identifier );
	if ( motor != nullptr )
	{
		motor->UpdateTarget ( velocity );
	}
}

bool Thing1MechGen::IsAtMinPosition ( RobotElementNames::MOTOR_CONTROLLER_USAGE identifier ) const
{
	auto motor = GetMotorMech ( identifier );
	if ( motor != nullptr )
	{
		return motor->IsAtMinTravel();
	}
	return false;
}

bool Thing1MechGen::IsAtMaxPosition ( RobotElementNames::MOTOR_CONTROLLER_USAGE identifier ) const
{
	auto motor = GetMotorMech ( identifier );
	if ( motor != nullptr )
	{
		return motor->IsAtMaxTravel();
	}
	return false;
}


BaseMechMotor *Thing1MechGen::GetMotorMech ( RobotElementNames::MOTOR_CONTROLLER_USAGE usage ) const
{
	auto itr = m_motorMap.find ( usage );
	if ( itr != m_motorMap.end() )
	{
		return itr->second;
	}
	return nullptr;
}

std::vector<RobotElementNames::MOTOR_CONTROLLER_USAGE> Thing1MechGen::GetMotorUsages() const
{
	std::vector<RobotElementNames::MOTOR_CONTROLLER_USAGE> output;
	for ( auto itr = m_motorMap.begin(); itr != m_motorMap.end(); ++itr )
	{
		output.emplace_back ( itr->first );
	}
	return output;
}


void Thing1MechGen::UpdateTarget ( RobotElementNames::SOLENOID_USAGE identifier, bool extend )
{
	auto sol = GetSolenoidMech ( identifier );
	if ( sol != nullptr )
	{
		sol->ActivateSolenoid ( extend );
	}
}

bool Thing1MechGen::IsAtMinPosition ( RobotElementNames::SOLENOID_USAGE identifier ) const
{
	auto sol = GetSolenoidMech ( identifier );
	if ( sol != nullptr )
	{
		return !sol->IsSolenoidActivated();
	}
	return false;
}

bool Thing1MechGen::IsAtMaxPosition ( RobotElementNames::SOLENOID_USAGE identifier ) const
{
	auto sol = GetSolenoidMech ( identifier );
	if ( sol != nullptr )
	{
		return sol->IsSolenoidActivated();
	}
	return false;
}


BaseMechSolenoid *Thing1MechGen::GetSolenoidMech ( RobotElementNames::SOLENOID_USAGE usage ) const
{
	auto itr = m_solenoidMap.find ( usage );
	if ( itr != m_solenoidMap.end() )
	{
		return itr->second;
	}
	return nullptr;
}

std::vector<RobotElementNames::SOLENOID_USAGE> Thing1MechGen::GetSolenoidUsages() const
{
	std::vector<RobotElementNames::SOLENOID_USAGE> output;
	for ( auto itr = m_solenoidMap.begin(); itr != m_solenoidMap.end(); ++itr )
	{
		output.emplace_back ( itr->first );
	}
	return output;
}

BaseMechServo *Thing1MechGen::GetServoMech ( RobotElementNames::SERVO_USAGE usage ) const
{
	auto itr = m_servoMap.find ( usage );
	if ( itr != m_servoMap.end() )
	{
		return itr->second;
	}
	return nullptr;
}

std::vector<RobotElementNames::SERVO_USAGE> Thing1MechGen::GetServoUsages() const
{
	std::vector<RobotElementNames::SERVO_USAGE> output;
	for ( auto itr = m_servoMap.begin(); itr != m_servoMap.end(); ++itr )
	{
		output.emplace_back ( itr->first );
	}
	return output;
}

void Thing1MechGen::Cyclic()
{
	CheckForTuningEnabled();
	if ( m_tuning )
	{
		ReadTuningParamsFromNT();
	}
}

void Thing1MechGen::CheckForTuningEnabled()
{
	bool pastTuning = m_tuning;
	m_tuning = m_table.get()->GetBoolean ( m_tuningIsEnabledStr, false );
	if ( pastTuning != m_tuning && m_tuning == true )
	{
		PushTuningParamsToNT();
	}
}

void Thing1MechGen::ReadTuningParamsFromNT()
{

}

void Thing1MechGen::PushTuningParamsToNT()
{

}