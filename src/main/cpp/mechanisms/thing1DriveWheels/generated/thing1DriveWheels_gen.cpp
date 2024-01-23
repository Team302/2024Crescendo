// clang-format off
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
// Generated on Monday, January 22, 2024 10:54:08 PM

#include <string>

// FRC Includes
#include <networktables/NetworkTableInstance.h>
#include "hw/interfaces/IDragonMotorController.h"

#include "thing1DriveWheels_gen.h"

#include "hw/DragonTalonSRX.h"
#include "hw/DragonTalonSRX.h"
#include "hw/DragonTalonSRX.h"
#include "hw/DragonTalonSRX.h"
#include "hw/DragonTalonFX.h"
#include "hw/DragonSparkMax.h"

#include "mechanisms/thing1DriveWheels/decoratormods/thing1DriveWheels_state_FR_State.h"
#include "mechanisms/thing1DriveWheels/decoratormods/thing1DriveWheels_state_FL_State.h"
#include "mechanisms/thing1DriveWheels/decoratormods/thing1DriveWheels_state_BR_State.h"
#include "mechanisms/thing1DriveWheels/decoratormods/thing1DriveWheels_state_BL_State.h"
#include "mechanisms/thing1DriveWheels/decoratormods/thing1DriveWheels_TalonOn_State.h"
#include "mechanisms/thing1DriveWheels/decoratormods/thing1DriveWheels_sparkyOn_State.h"

using ctre::phoenix6::signals::ForwardLimitSourceValue;
using ctre::phoenix6::signals::ForwardLimitTypeValue;
using ctre::phoenix6::signals::ReverseLimitSourceValue;
using ctre::phoenix6::signals::ReverseLimitTypeValue;
using ctre::phoenix6::signals::InvertedValue;
using ctre::phoenix6::signals::NeutralModeValue;
using ctre::phoenix::motorcontrol::RemoteSensorSource;

thing1DriveWheels_gen::thing1DriveWheels_gen() : DriveWheels ( MechanismTypes::MECHANISM_TYPE::THING1DRIVE_WHEELS, std::string ( "thing1DriveWheels" ) ),
	m_motorMap(),
	m_solenoidMap(),
	m_servoMap()
{
}

void thing1DriveWheels_gen::Create()
{
	m_ntName = "thing1DriveWheels";

	DistanceAngleCalcStruc TalonSRX_FrontLeftCalcStruct;
	TalonSRX_FrontLeftCalcStruct.countsPerRev = 0 ;
	TalonSRX_FrontLeftCalcStruct.gearRatio = 1 ;
	TalonSRX_FrontLeftCalcStruct.diameter = units::length::inch_t ( units::length::meter_t ( 1 ) ).to<double>() ;
	TalonSRX_FrontLeftCalcStruct.countsPerInch = 0 ;
	TalonSRX_FrontLeftCalcStruct.countsPerDegree = 0 ;;
	TalonSRX_FrontLeft = new DragonTalonSRX ( "TalonSRX_FrontLeft",
	    RobotElementNames::MOTOR_CONTROLLER_USAGE::THING1DRIVE_WHEELS_TALONSRX_FRONTLEFT,
	    2,
	    2,
	    TalonSRX_FrontLeftCalcStruct,
	    IDragonMotorController::MOTOR_TYPE::ANDYMARKNEVEREST );
	m_motorMap[TalonSRX_FrontLeft->GetType()] = new BaseMechMotor ( m_ntName,
	    *TalonSRX_FrontLeft,
	    BaseMechMotor::EndOfTravelSensorOption::NONE,
	    nullptr,
	    BaseMechMotor::EndOfTravelSensorOption::NONE,
	    nullptr );

	DistanceAngleCalcStruc TalonSRX_FrontRightCalcStruct;
	TalonSRX_FrontRightCalcStruct.countsPerRev = 0 ;
	TalonSRX_FrontRightCalcStruct.gearRatio = 1 ;
	TalonSRX_FrontRightCalcStruct.diameter = units::length::inch_t ( units::length::meter_t ( 1 ) ).to<double>() ;
	TalonSRX_FrontRightCalcStruct.countsPerInch = 0 ;
	TalonSRX_FrontRightCalcStruct.countsPerDegree = 0 ;;
	TalonSRX_FrontRight = new DragonTalonSRX ( "TalonSRX_FrontRight",
	    RobotElementNames::MOTOR_CONTROLLER_USAGE::THING1DRIVE_WHEELS_TALONSRX_FRONTRIGHT,
	    3,
	    3,
	    TalonSRX_FrontRightCalcStruct,
	    IDragonMotorController::MOTOR_TYPE::UNKNOWN_MOTOR );
	m_motorMap[TalonSRX_FrontRight->GetType()] = new BaseMechMotor ( m_ntName,
	    *TalonSRX_FrontRight,
	    BaseMechMotor::EndOfTravelSensorOption::NONE,
	    nullptr,
	    BaseMechMotor::EndOfTravelSensorOption::NONE,
	    nullptr );

	DistanceAngleCalcStruc TalonSRX_BackLeftCalcStruct;
	TalonSRX_BackLeftCalcStruct.countsPerRev = 0 ;
	TalonSRX_BackLeftCalcStruct.gearRatio = 1 ;
	TalonSRX_BackLeftCalcStruct.diameter = units::length::inch_t ( units::length::meter_t ( 1 ) ).to<double>() ;
	TalonSRX_BackLeftCalcStruct.countsPerInch = 0 ;
	TalonSRX_BackLeftCalcStruct.countsPerDegree = 0 ;;
	TalonSRX_BackLeft = new DragonTalonSRX ( "TalonSRX_BackLeft",
	    RobotElementNames::MOTOR_CONTROLLER_USAGE::THING1DRIVE_WHEELS_TALONSRX_BACKLEFT,
	    0,
	    0,
	    TalonSRX_BackLeftCalcStruct,
	    IDragonMotorController::MOTOR_TYPE::UNKNOWN_MOTOR );
	m_motorMap[TalonSRX_BackLeft->GetType()] = new BaseMechMotor ( m_ntName,
	    *TalonSRX_BackLeft,
	    BaseMechMotor::EndOfTravelSensorOption::NONE,
	    nullptr,
	    BaseMechMotor::EndOfTravelSensorOption::NONE,
	    nullptr );

	DistanceAngleCalcStruc TalonSRX_BackRightCalcStruct;
	TalonSRX_BackRightCalcStruct.countsPerRev = 0 ;
	TalonSRX_BackRightCalcStruct.gearRatio = 1 ;
	TalonSRX_BackRightCalcStruct.diameter = units::length::inch_t ( units::length::meter_t ( 1 ) ).to<double>() ;
	TalonSRX_BackRightCalcStruct.countsPerInch = 0 ;
	TalonSRX_BackRightCalcStruct.countsPerDegree = 0 ;;
	TalonSRX_BackRight = new DragonTalonSRX ( "TalonSRX_BackRight",
	    RobotElementNames::MOTOR_CONTROLLER_USAGE::THING1DRIVE_WHEELS_TALONSRX_BACKRIGHT,
	    1,
	    1,
	    TalonSRX_BackRightCalcStruct,
	    IDragonMotorController::MOTOR_TYPE::UNKNOWN_MOTOR );
	m_motorMap[TalonSRX_BackRight->GetType()] = new BaseMechMotor ( m_ntName,
	    *TalonSRX_BackRight,
	    BaseMechMotor::EndOfTravelSensorOption::NONE,
	    nullptr,
	    BaseMechMotor::EndOfTravelSensorOption::NONE,
	    nullptr );

	DistanceAngleCalcStruc TalonFX_5CalcStruct;
	TalonFX_5CalcStruct.countsPerRev = 0 ;
	TalonFX_5CalcStruct.gearRatio = 1 ;
	TalonFX_5CalcStruct.diameter = units::length::inch_t ( units::length::meter_t ( 1 ) ).to<double>() ;
	TalonFX_5CalcStruct.countsPerInch = 0 ;
	TalonFX_5CalcStruct.countsPerDegree = 0 ;;
	TalonFX_5 = new DragonTalonFX ( "TalonFX_5",RobotElementNames::MOTOR_CONTROLLER_USAGE::THING1DRIVE_WHEELS_TALONFX_5,11, TalonFX_5CalcStruct, IDragonMotorController::MOTOR_TYPE::FALCON500, "rio" );
	m_motorMap[TalonFX_5->GetType()] = new BaseMechMotor ( m_ntName,
	    *TalonFX_5,
	    BaseMechMotor::EndOfTravelSensorOption::NONE,
	    nullptr,
	    BaseMechMotor::EndOfTravelSensorOption::NONE,
	    nullptr );

	DistanceAngleCalcStruc SparkMaxCalcStruct;
	SparkMaxCalcStruct.countsPerRev = 0 ;
	SparkMaxCalcStruct.gearRatio = 1 ;
	SparkMaxCalcStruct.diameter = units::length::inch_t ( units::length::meter_t ( 1 ) ).to<double>() ;
	SparkMaxCalcStruct.countsPerInch = 0 ;
	SparkMaxCalcStruct.countsPerDegree = 0 ;;
	SparkMax = new DragonSparkMax ( 4,RobotElementNames::MOTOR_CONTROLLER_USAGE::THING1DRIVE_WHEELS_SPARK_MAX,rev::CANSparkMax::MotorType::kBrushless,rev::SparkRelativeEncoder::Type::kHallSensor,1 );
	m_motorMap[SparkMax->GetType()] = new BaseMechMotor ( m_ntName,
	    *SparkMax,
	    BaseMechMotor::EndOfTravelSensorOption::NONE,
	    nullptr,
	    BaseMechMotor::EndOfTravelSensorOption::NONE,
	    nullptr );

	FLmotorControlData = new ControlData (
	    ControlModes::CONTROL_TYPE::PERCENT_OUTPUT, // ControlModes::CONTROL_TYPE mode
	    ControlModes::CONTROL_RUN_LOCS::MOTOR_CONTROLLER, // ControlModes::CONTROL_RUN_LOCS server
	    "FLmotorControlData", // std::string indentifier
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
	FRmotorControlData = new ControlData (
	    ControlModes::CONTROL_TYPE::PERCENT_OUTPUT, // ControlModes::CONTROL_TYPE mode
	    ControlModes::CONTROL_RUN_LOCS::MOTOR_CONTROLLER, // ControlModes::CONTROL_RUN_LOCS server
	    "FRmotorControlData", // std::string indentifier
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
	BLmotorControlData = new ControlData (
	    ControlModes::CONTROL_TYPE::PERCENT_OUTPUT, // ControlModes::CONTROL_TYPE mode
	    ControlModes::CONTROL_RUN_LOCS::MOTOR_CONTROLLER, // ControlModes::CONTROL_RUN_LOCS server
	    "BLmotorControlData", // std::string indentifier
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
	BRmotorControlData = new ControlData (
	    ControlModes::CONTROL_TYPE::PERCENT_OUTPUT, // ControlModes::CONTROL_TYPE mode
	    ControlModes::CONTROL_RUN_LOCS::MOTOR_CONTROLLER, // ControlModes::CONTROL_RUN_LOCS server
	    "BRmotorControlData", // std::string indentifier
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
	talonMotorControlData = new ControlData (
	    ControlModes::CONTROL_TYPE::VELOCITY_RPS, // ControlModes::CONTROL_TYPE mode
	    ControlModes::CONTROL_RUN_LOCS::MOTOR_CONTROLLER, // ControlModes::CONTROL_RUN_LOCS server
	    "talonMotorControlData", // std::string indentifier
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
	sparkMaxMotorControlData = new ControlData (
	    ControlModes::CONTROL_TYPE::PERCENT_OUTPUT, // ControlModes::CONTROL_TYPE mode
	    ControlModes::CONTROL_RUN_LOCS::MOTOR_CONTROLLER, // ControlModes::CONTROL_RUN_LOCS server
	    "sparkMaxMotorControlData", // std::string indentifier
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

	thing1DriveWheelsstate_FRState* state_FRState = new thing1DriveWheelsstate_FRState ( string ( "state_FR" ), 0, new thing1DriveWheelsstate_FRStateGen ( string ( "state_FR" ), 0, this ) );
	AddToStateVector ( state_FRState );

	thing1DriveWheelsstate_FLState* state_FLState = new thing1DriveWheelsstate_FLState ( string ( "state_FL" ), 1, new thing1DriveWheelsstate_FLStateGen ( string ( "state_FL" ), 1, this ) );
	AddToStateVector ( state_FLState );

	thing1DriveWheelsstate_BRState* state_BRState = new thing1DriveWheelsstate_BRState ( string ( "state_BR" ), 2, new thing1DriveWheelsstate_BRStateGen ( string ( "state_BR" ), 2, this ) );
	AddToStateVector ( state_BRState );

	thing1DriveWheelsstate_BLState* state_BLState = new thing1DriveWheelsstate_BLState ( string ( "state_BL" ), 3, new thing1DriveWheelsstate_BLStateGen ( string ( "state_BL" ), 3, this ) );
	AddToStateVector ( state_BLState );

	thing1DriveWheelsTalonOnState* TalonOnState = new thing1DriveWheelsTalonOnState ( string ( "TalonOn" ), 4, new thing1DriveWheelsTalonOnStateGen ( string ( "TalonOn" ), 4, this ) );
	AddToStateVector ( TalonOnState );

	thing1DriveWheelssparkyOnState* sparkyOnState = new thing1DriveWheelssparkyOnState ( string ( "sparkyOn" ), 5, new thing1DriveWheelssparkyOnStateGen ( string ( "sparkyOn" ), 5, this ) );
	AddToStateVector ( sparkyOnState );

	state_FRState->RegisterTransitionState ( state_BRState );
	state_FRState->RegisterTransitionState ( state_BLState );
	state_FLState->RegisterTransitionState ( state_FRState );
	state_BRState->RegisterTransitionState ( state_BLState );
	state_BRState->RegisterTransitionState ( TalonOnState );
	state_BLState->RegisterTransitionState ( state_FLState );
	state_BLState->RegisterTransitionState ( sparkyOnState );
	TalonOnState->RegisterTransitionState ( state_BRState );
	sparkyOnState->RegisterTransitionState ( state_BLState );

	m_table = nt::NetworkTableInstance::GetDefault().GetTable ( m_ntName );
	m_tuningIsEnabledStr = "Enable Tuning for " + m_ntName; // since this string is used every loop, we do not want to create the string every time
	m_table.get()->PutBoolean ( m_tuningIsEnabledStr, m_tuning );
}

void thing1DriveWheels_gen::Initialize ( RobotConfigMgr::RobotIdentifier robotFullName )
{
	if ( false ) {}
	else if ( RobotConfigMgr::RobotIdentifier::Thing_1_1 == robotFullName )
	{

		TalonSRX_FrontLeft->SetRemoteSensor ( 0,
		                                      ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_Off );
		TalonSRX_FrontLeft->Invert ( true );
		TalonSRX_FrontLeft->EnableBrakeMode ( false );
		TalonSRX_FrontLeft->EnableCurrentLimiting ( false );
		TalonSRX_FrontLeft->ConfigPeakCurrentLimit ( units::current::ampere_t ( units::current::ampere_t ( 0 ) ).to<int>(),
		        units::time::millisecond_t ( units::time::second_t ( 0 ) ).to<int>() );
		TalonSRX_FrontLeft->ConfigPeakCurrentDuration ( units::time::millisecond_t ( units::time::second_t ( 0 ) ).to<int>(),
		        units::time::millisecond_t ( units::time::second_t ( 0 ) ).to<int>() );
		TalonSRX_FrontLeft->ConfigContinuousCurrentLimit ( units::current::ampere_t ( units::current::ampere_t ( 0 ) ).to<int>(),
		        units::time::millisecond_t ( units::time::second_t ( 0 ) ).to<int>() );
		TalonSRX_FrontLeft->SetDiameter ( units::length::inch_t ( units::length::meter_t ( 1 ) ).to<double>() );
		TalonSRX_FrontLeft->EnableDisableLimitSwitches ( false );
		TalonSRX_FrontLeft->SetForwardLimitSwitch ( true );
		TalonSRX_FrontLeft->SetReverseLimitSwitch ( true );
// TalonSRX_FrontLeft : Follower motor mode is not enabled
		TalonSRX_FrontLeft->SetVoltageRamping ( units::time::second_t ( units::time::second_t ( 0 ) ).to<double>(),
		                                        units::time::second_t ( units::time::second_t ( 0 ) ).to<double>() );
		TalonSRX_FrontLeft->SetSensorInverted ( false );

		TalonSRX_FrontRight->SetRemoteSensor ( 0,
		                                       ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_Off );
		TalonSRX_FrontRight->Invert ( true );
		TalonSRX_FrontRight->EnableBrakeMode ( false );
		TalonSRX_FrontRight->EnableCurrentLimiting ( false );
		TalonSRX_FrontRight->ConfigPeakCurrentLimit ( units::current::ampere_t ( units::current::ampere_t ( 0 ) ).to<int>(),
		        units::time::millisecond_t ( units::time::second_t ( 0 ) ).to<int>() );
		TalonSRX_FrontRight->ConfigPeakCurrentDuration ( units::time::millisecond_t ( units::time::second_t ( 0 ) ).to<int>(),
		        units::time::millisecond_t ( units::time::second_t ( 0 ) ).to<int>() );
		TalonSRX_FrontRight->ConfigContinuousCurrentLimit ( units::current::ampere_t ( units::current::ampere_t ( 0 ) ).to<int>(),
		        units::time::millisecond_t ( units::time::second_t ( 0 ) ).to<int>() );
		TalonSRX_FrontRight->SetDiameter ( units::length::inch_t ( units::length::meter_t ( 1 ) ).to<double>() );
		TalonSRX_FrontRight->EnableDisableLimitSwitches ( false );
		TalonSRX_FrontRight->SetForwardLimitSwitch ( true );
		TalonSRX_FrontRight->SetReverseLimitSwitch ( true );
// TalonSRX_FrontRight : Follower motor mode is not enabled
		TalonSRX_FrontRight->SetVoltageRamping ( units::time::second_t ( units::time::second_t ( 0 ) ).to<double>(),
		        units::time::second_t ( units::time::second_t ( 0 ) ).to<double>() );
		TalonSRX_FrontRight->SetSensorInverted ( false );

		TalonSRX_BackLeft->SetRemoteSensor ( 0,
		                                     ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_Off );
		TalonSRX_BackLeft->Invert ( true );
		TalonSRX_BackLeft->EnableBrakeMode ( false );
		TalonSRX_BackLeft->EnableCurrentLimiting ( false );
		TalonSRX_BackLeft->ConfigPeakCurrentLimit ( units::current::ampere_t ( units::current::ampere_t ( 0 ) ).to<int>(),
		        units::time::millisecond_t ( units::time::second_t ( 0 ) ).to<int>() );
		TalonSRX_BackLeft->ConfigPeakCurrentDuration ( units::time::millisecond_t ( units::time::second_t ( 0 ) ).to<int>(),
		        units::time::millisecond_t ( units::time::second_t ( 0 ) ).to<int>() );
		TalonSRX_BackLeft->ConfigContinuousCurrentLimit ( units::current::ampere_t ( units::current::ampere_t ( 0 ) ).to<int>(),
		        units::time::millisecond_t ( units::time::second_t ( 0 ) ).to<int>() );
		TalonSRX_BackLeft->SetDiameter ( units::length::inch_t ( units::length::meter_t ( 1 ) ).to<double>() );
		TalonSRX_BackLeft->EnableDisableLimitSwitches ( false );
		TalonSRX_BackLeft->SetForwardLimitSwitch ( true );
		TalonSRX_BackLeft->SetReverseLimitSwitch ( true );
// TalonSRX_BackLeft : Follower motor mode is not enabled
		TalonSRX_BackLeft->SetVoltageRamping ( units::time::second_t ( units::time::second_t ( 0 ) ).to<double>(),
		                                       units::time::second_t ( units::time::second_t ( 0 ) ).to<double>() );
		TalonSRX_BackLeft->SetSensorInverted ( false );

		TalonSRX_BackRight->SetRemoteSensor ( 0,
		                                      ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_Off );
		TalonSRX_BackRight->Invert ( true );
		TalonSRX_BackRight->EnableBrakeMode ( false );
		TalonSRX_BackRight->EnableCurrentLimiting ( false );
		TalonSRX_BackRight->ConfigPeakCurrentLimit ( units::current::ampere_t ( units::current::ampere_t ( 0 ) ).to<int>(),
		        units::time::millisecond_t ( units::time::second_t ( 0 ) ).to<int>() );
		TalonSRX_BackRight->ConfigPeakCurrentDuration ( units::time::millisecond_t ( units::time::second_t ( 0 ) ).to<int>(),
		        units::time::millisecond_t ( units::time::second_t ( 0 ) ).to<int>() );
		TalonSRX_BackRight->ConfigContinuousCurrentLimit ( units::current::ampere_t ( units::current::ampere_t ( 0 ) ).to<int>(),
		        units::time::millisecond_t ( units::time::second_t ( 0 ) ).to<int>() );
		TalonSRX_BackRight->SetDiameter ( units::length::inch_t ( units::length::meter_t ( 1 ) ).to<double>() );
		TalonSRX_BackRight->EnableDisableLimitSwitches ( false );
		TalonSRX_BackRight->SetForwardLimitSwitch ( true );
		TalonSRX_BackRight->SetReverseLimitSwitch ( true );
// TalonSRX_BackRight : Follower motor mode is not enabled
		TalonSRX_BackRight->SetVoltageRamping ( units::time::second_t ( units::time::second_t ( 0 ) ).to<double>(),
		                                        units::time::second_t ( units::time::second_t ( 0 ) ).to<double>() );
		TalonSRX_BackRight->SetSensorInverted ( false );

		TalonFX_5->SetCurrentLimits ( false,
		                              units::current::ampere_t ( 0 ),
		                              false,
		                              units::current::ampere_t ( 0 ),
		                              units::current::ampere_t ( 0 ),
		                              units::time::second_t ( 0 ) );
		TalonFX_5->ConfigHWLimitSW ( false, // enableForward
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
		TalonFX_5->ConfigMotorSettings ( ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive, // ctre::phoenixpro::signals::InvertedValue
		                                 ctre::phoenix6::signals::NeutralModeValue::Coast, // ctre::phoenixpro::signals::NeutralModeValue
		                                 0, // deadbandPercent
		                                 0, // peakForwardDutyCycle
		                                 0 ); // peakReverseDutyCycle
		TalonFX_5->SetAsFollowerMotor ( 0 ); // masterCANID
		TalonFX_5->SetRemoteSensor ( 0, // canID
		                             RemoteSensorSource::RemoteSensorSource_Off ); // ctre::phoenix::motorcontrol::RemoteSensorSource
		TalonFX_5->SetDiameter ( 0 ); // double diameter
		TalonFX_5->SetVoltageRamping ( units::time::second_t ( units::time::second_t ( 0 ) ).to<double>(),
		                               units::time::second_t ( units::time::second_t ( 0 ) ).to<double>() );
		TalonFX_5->SetSensorInverted ( false );
		SparkMax->SetRemoteSensor ( 0,
		                            ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_Off );
		SparkMax->Invert ( true );
		SparkMax->EnableBrakeMode ( false );
		SparkMax->EnableCurrentLimiting ( false );
		SparkMax->SetDiameter ( units::length::inch_t ( units::length::meter_t ( 1 ) ).to<double>() );
// SparkMax : Follower motor mode is not enabled
		SparkMax->SetVoltageRamping ( units::time::second_t ( units::time::second_t ( 0 ) ).to<double>(),
		                              units::time::second_t ( units::time::second_t ( 0 ) ).to<double>() );
		SparkMax->SetSensorInverted ( false );

// FLmotorControlData : ControlData does not have initialization needs
// FRmotorControlData : ControlData does not have initialization needs
// BLmotorControlData : ControlData does not have initialization needs
// BRmotorControlData : ControlData does not have initialization needs
// talonMotorControlData : ControlData does not have initialization needs
// sparkMaxMotorControlData : ControlData does not have initialization needs

//todo create initialization for state_FR
//todo create initialization for state_FL
//todo create initialization for state_BR
//todo create initialization for state_BL
//todo create initialization for TalonOn
//todo create initialization for sparkyOn
	}

}

/// @brief  Set the control constants (e.g. PIDF values).
/// @param [in] ControlData*                                   pid:  the control constants
/// @return void
void thing1DriveWheels_gen::SetControlConstants ( RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, int slot, ControlData pid )
{
	auto motor = GetMotorMech ( identifier );
	if ( motor != nullptr )
	{
		motor->SetControlConstants ( slot, pid );
	}
}

/// @brief update the output to the mechanism using the current controller and target value(s)
/// @return void
void thing1DriveWheels_gen::Update()
{
	for ( auto motor : m_motorMap )
	{
		motor.second->Update();
	}
}

void thing1DriveWheels_gen::UpdateTarget ( RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, double percentOutput )
{
	auto motor = GetMotorMech ( identifier );
	if ( motor != nullptr )
	{
		motor->UpdateTarget ( percentOutput );
	}
}

void thing1DriveWheels_gen::UpdateTarget ( RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, units::angle::degree_t angle )
{
	auto motor = GetMotorMech ( identifier );
	if ( motor != nullptr )
	{
		motor->UpdateTarget ( angle );
	}
}

void thing1DriveWheels_gen::UpdateTarget ( RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, units::angular_velocity::revolutions_per_minute_t angVel )
{
	auto motor = GetMotorMech ( identifier );
	if ( motor != nullptr )
	{
		motor->UpdateTarget ( angVel );
	}
}
void thing1DriveWheels_gen::UpdateTarget ( RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, units::length::inch_t position )
{
	auto motor = GetMotorMech ( identifier );
	if ( motor != nullptr )
	{
		motor->UpdateTarget ( position );
	}
}
void thing1DriveWheels_gen::UpdateTarget ( RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, units::velocity::feet_per_second_t velocity )
{
	auto motor = GetMotorMech ( identifier );
	if ( motor != nullptr )
	{
		motor->UpdateTarget ( velocity );
	}
}

bool thing1DriveWheels_gen::IsAtMinPosition ( RobotElementNames::MOTOR_CONTROLLER_USAGE identifier ) const
{
	auto motor = GetMotorMech ( identifier );
	if ( motor != nullptr )
	{
		return motor->IsAtMinTravel();
	}
	return false;
}

bool thing1DriveWheels_gen::IsAtMaxPosition ( RobotElementNames::MOTOR_CONTROLLER_USAGE identifier ) const
{
	auto motor = GetMotorMech ( identifier );
	if ( motor != nullptr )
	{
		return motor->IsAtMaxTravel();
	}
	return false;
}

BaseMechMotor *thing1DriveWheels_gen::GetMotorMech ( RobotElementNames::MOTOR_CONTROLLER_USAGE usage ) const
{
	auto itr = m_motorMap.find ( usage );
	if ( itr != m_motorMap.end() )
	{
		return itr->second;
	}
	return nullptr;
}

std::vector<RobotElementNames::MOTOR_CONTROLLER_USAGE> thing1DriveWheels_gen::GetMotorUsages() const
{
	std::vector<RobotElementNames::MOTOR_CONTROLLER_USAGE> output;
	for ( auto itr = m_motorMap.begin(); itr != m_motorMap.end(); ++itr )
	{
		output.emplace_back ( itr->first );
	}
	return output;
}

void thing1DriveWheels_gen::UpdateTarget ( RobotElementNames::SOLENOID_USAGE identifier, bool extend )
{
	auto sol = GetSolenoidMech ( identifier );
	if ( sol != nullptr )
	{
		sol->ActivateSolenoid ( extend );
	}
}

bool thing1DriveWheels_gen::IsAtMinPosition ( RobotElementNames::SOLENOID_USAGE identifier ) const
{
	auto sol = GetSolenoidMech ( identifier );
	if ( sol != nullptr )
	{
		return !sol->IsSolenoidActivated();
	}
	return false;
}

bool thing1DriveWheels_gen::IsAtMaxPosition ( RobotElementNames::SOLENOID_USAGE identifier ) const
{
	auto sol = GetSolenoidMech ( identifier );
	if ( sol != nullptr )
	{
		return sol->IsSolenoidActivated();
	}
	return false;
}

BaseMechSolenoid *thing1DriveWheels_gen::GetSolenoidMech ( RobotElementNames::SOLENOID_USAGE usage ) const
{
	auto itr = m_solenoidMap.find ( usage );
	if ( itr != m_solenoidMap.end() )
	{
		return itr->second;
	}
	return nullptr;
}

std::vector<RobotElementNames::SOLENOID_USAGE> thing1DriveWheels_gen::GetSolenoidUsages() const
{
	std::vector<RobotElementNames::SOLENOID_USAGE> output;
	for ( auto itr = m_solenoidMap.begin(); itr != m_solenoidMap.end(); ++itr )
	{
		output.emplace_back ( itr->first );
	}
	return output;
}

BaseMechServo *thing1DriveWheels_gen::GetServoMech ( RobotElementNames::SERVO_USAGE usage ) const
{
	auto itr = m_servoMap.find ( usage );
	if ( itr != m_servoMap.end() )
	{
		return itr->second;
	}
	return nullptr;
}

std::vector<RobotElementNames::SERVO_USAGE> thing1DriveWheels_gen::GetServoUsages() const
{
	std::vector<RobotElementNames::SERVO_USAGE> output;
	for ( auto itr = m_servoMap.begin(); itr != m_servoMap.end(); ++itr )
	{
		output.emplace_back ( itr->first );
	}
	return output;
}
