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
// This file was automatically generated by the Team 302 code generator version 1.2.3.3
// Generated on Saturday, February 10, 2024 11:54:42 AM

#include <string>

// FRC Includes
#include <networktables/NetworkTableInstance.h>
#include "hw/interfaces/IDragonMotorController.h"

#include "noteManagerGen.h"
#include "utils/logging/Logger.h"

using ctre::phoenix6::signals::ForwardLimitSourceValue;
using ctre::phoenix6::signals::ForwardLimitTypeValue;
using ctre::phoenix6::signals::ReverseLimitSourceValue;
using ctre::phoenix6::signals::ReverseLimitTypeValue;
using ctre::phoenix6::signals::InvertedValue;
using ctre::phoenix6::signals::NeutralModeValue;
using ctre::phoenix::motorcontrol::RemoteSensorSource;

noteManagerGen::noteManagerGen() : BaseMech ( MechanismTypes::MECHANISM_TYPE::NOTE_MANAGER, "", std::string ( "noteManager" ) ),
	m_motorMap(),
	m_solenoidMap(),
	m_servoMap(),
	m_stateMap()
{
}

void noteManagerGen::Create()
{
	m_ntName = "noteManager";
	DistanceAngleCalcStruc frontIntakeCalcStruct;
	frontIntakeCalcStruct.countsPerRev = 0 ;
	frontIntakeCalcStruct.gearRatio = 1 ;
	frontIntakeCalcStruct.diameter = units::length::inch_t ( units::length::meter_t ( 1 ) ).to<double>() ;
	frontIntakeCalcStruct.countsPerInch = 0 ;
	frontIntakeCalcStruct.countsPerDegree = 0 ;;
	frontIntake = new DragonSparkMax ( 13,RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_FRONT_INTAKE,rev::CANSparkMax::MotorType::kBrushless,rev::SparkRelativeEncoder::Type::kHallSensor,rev::SparkLimitSwitch::Type::kNormallyOpen,rev::SparkLimitSwitch::Type::kNormallyOpen,frontIntakeCalcStruct );
	m_motorMap[frontIntake->GetType()] = new BaseMechMotor ( m_ntName,
	    frontIntake,
	    BaseMechMotor::EndOfTravelSensorOption::NONE,
	    nullptr,
	    BaseMechMotor::EndOfTravelSensorOption::NONE,
	    nullptr );

	DistanceAngleCalcStruc backIntakeCalcStruct;
	backIntakeCalcStruct.countsPerRev = 0 ;
	backIntakeCalcStruct.gearRatio = 1 ;
	backIntakeCalcStruct.diameter = units::length::inch_t ( units::length::meter_t ( 1 ) ).to<double>() ;
	backIntakeCalcStruct.countsPerInch = 0 ;
	backIntakeCalcStruct.countsPerDegree = 0 ;;
	backIntake = new DragonSparkMax ( 17,RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_BACK_INTAKE,rev::CANSparkMax::MotorType::kBrushless,rev::SparkRelativeEncoder::Type::kHallSensor,rev::SparkLimitSwitch::Type::kNormallyOpen,rev::SparkLimitSwitch::Type::kNormallyOpen,backIntakeCalcStruct );
	m_motorMap[backIntake->GetType()] = new BaseMechMotor ( m_ntName,
	    backIntake,
	    BaseMechMotor::EndOfTravelSensorOption::NONE,
	    nullptr,
	    BaseMechMotor::EndOfTravelSensorOption::NONE,
	    nullptr );

	DistanceAngleCalcStruc TransferCalcStruct;
	TransferCalcStruct.countsPerRev = 0 ;
	TransferCalcStruct.gearRatio = 1 ;
	TransferCalcStruct.diameter = units::length::inch_t ( units::length::meter_t ( 1 ) ).to<double>() ;
	TransferCalcStruct.countsPerInch = 0 ;
	TransferCalcStruct.countsPerDegree = 0 ;;
	Transfer = new DragonSparkMax ( 14,RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_TRANSFER,rev::CANSparkMax::MotorType::kBrushless,rev::SparkRelativeEncoder::Type::kHallSensor,rev::SparkLimitSwitch::Type::kNormallyOpen,rev::SparkLimitSwitch::Type::kNormallyOpen,TransferCalcStruct );
	m_motorMap[Transfer->GetType()] = new BaseMechMotor ( m_ntName,
	    Transfer,
	    BaseMechMotor::EndOfTravelSensorOption::NONE,
	    nullptr,
	    BaseMechMotor::EndOfTravelSensorOption::NONE,
	    nullptr );

	DistanceAngleCalcStruc FeederCalcStruct;
	FeederCalcStruct.countsPerRev = 0 ;
	FeederCalcStruct.gearRatio = 1 ;
	FeederCalcStruct.diameter = units::length::inch_t ( units::length::meter_t ( 1 ) ).to<double>() ;
	FeederCalcStruct.countsPerInch = 0 ;
	FeederCalcStruct.countsPerDegree = 0 ;;
	Feeder = new DragonSparkFlex ( 12,RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_FEEDER,rev::CANSparkFlex::MotorType::kBrushless,rev::SparkRelativeEncoder::Type::kHallSensor,rev::SparkLimitSwitch::Type::kNormallyOpen,rev::SparkLimitSwitch::Type::kNormallyOpen,FeederCalcStruct );
	m_motorMap[Feeder->GetType()] = new BaseMechMotor ( m_ntName,
	    Feeder,
	    BaseMechMotor::EndOfTravelSensorOption::NONE,
	    nullptr,
	    BaseMechMotor::EndOfTravelSensorOption::NONE,
	    nullptr );

	DistanceAngleCalcStruc launcherTopCalcStruct;
	launcherTopCalcStruct.countsPerRev = 0 ;
	launcherTopCalcStruct.gearRatio = 1 ;
	launcherTopCalcStruct.diameter = units::length::inch_t ( units::length::meter_t ( 1 ) ).to<double>() ;
	launcherTopCalcStruct.countsPerInch = 0 ;
	launcherTopCalcStruct.countsPerDegree = 0 ;;
	launcherTop = new DragonTalonFX ( "launcherTop",RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_TOP,11, launcherTopCalcStruct, IDragonMotorController::MOTOR_TYPE::FALCON500, "rio" );
	m_motorMap[launcherTop->GetType()] = new BaseMechMotor ( m_ntName,
	    launcherTop,
	    BaseMechMotor::EndOfTravelSensorOption::NONE,
	    nullptr,
	    BaseMechMotor::EndOfTravelSensorOption::NONE,
	    nullptr );

	DistanceAngleCalcStruc launcherBottomCalcStruct;
	launcherBottomCalcStruct.countsPerRev = 0 ;
	launcherBottomCalcStruct.gearRatio = 1 ;
	launcherBottomCalcStruct.diameter = units::length::inch_t ( units::length::meter_t ( 1 ) ).to<double>() ;
	launcherBottomCalcStruct.countsPerInch = 0 ;
	launcherBottomCalcStruct.countsPerDegree = 0 ;;
	launcherBottom = new DragonTalonFX ( "launcherBottom",RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_BOTTOM,10, launcherBottomCalcStruct, IDragonMotorController::MOTOR_TYPE::FALCON500, "rio" );
	m_motorMap[launcherBottom->GetType()] = new BaseMechMotor ( m_ntName,
	    launcherBottom,
	    BaseMechMotor::EndOfTravelSensorOption::NONE,
	    nullptr,
	    BaseMechMotor::EndOfTravelSensorOption::NONE,
	    nullptr );

	DistanceAngleCalcStruc launcherAngleCalcStruct;
	launcherAngleCalcStruct.countsPerRev = 0 ;
	launcherAngleCalcStruct.gearRatio = 1 ;
	launcherAngleCalcStruct.diameter = units::length::inch_t ( units::length::meter_t ( 1 ) ).to<double>() ;
	launcherAngleCalcStruct.countsPerInch = 0 ;
	launcherAngleCalcStruct.countsPerDegree = 2.352 ;;
	launcherAngle = new DragonSparkMax ( 1,RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_ANGLE,rev::CANSparkMax::MotorType::kBrushless,rev::SparkRelativeEncoder::Type::kHallSensor,rev::SparkLimitSwitch::Type::kNormallyOpen,rev::SparkLimitSwitch::Type::kNormallyOpen,launcherAngleCalcStruct );
	m_motorMap[launcherAngle->GetType()] = new BaseMechMotor ( m_ntName,
	    launcherAngle,
	    BaseMechMotor::EndOfTravelSensorOption::NONE,
	    nullptr,
	    BaseMechMotor::EndOfTravelSensorOption::NONE,
	    nullptr );

	DistanceAngleCalcStruc PlacerCalcStruct;
	PlacerCalcStruct.countsPerRev = 0 ;
	PlacerCalcStruct.gearRatio = 1 ;
	PlacerCalcStruct.diameter = units::length::inch_t ( units::length::meter_t ( 1 ) ).to<double>() ;
	PlacerCalcStruct.countsPerInch = 0 ;
	PlacerCalcStruct.countsPerDegree = 0 ;;
	Placer = new DragonSparkFlex ( 19,RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_PLACER,rev::CANSparkFlex::MotorType::kBrushless,rev::SparkRelativeEncoder::Type::kHallSensor,rev::SparkLimitSwitch::Type::kNormallyOpen,rev::SparkLimitSwitch::Type::kNormallyOpen,PlacerCalcStruct );
	m_motorMap[Placer->GetType()] = new BaseMechMotor ( m_ntName,
	    Placer,
	    BaseMechMotor::EndOfTravelSensorOption::NONE,
	    nullptr,
	    BaseMechMotor::EndOfTravelSensorOption::NONE,
	    nullptr );

	DistanceAngleCalcStruc ElevatorCalcStruct;
	ElevatorCalcStruct.countsPerRev = 0 ;
	ElevatorCalcStruct.gearRatio = 1 ;
	ElevatorCalcStruct.diameter = units::length::inch_t ( units::length::meter_t ( 1 ) ).to<double>() ;
	ElevatorCalcStruct.countsPerInch = 0.146875229492546 ;
	ElevatorCalcStruct.countsPerDegree = 0 ;;
	Elevator = new DragonSparkMax ( 62,RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_ELEVATOR,rev::CANSparkMax::MotorType::kBrushless,rev::SparkRelativeEncoder::Type::kHallSensor,rev::SparkLimitSwitch::Type::kNormallyOpen,rev::SparkLimitSwitch::Type::kNormallyOpen,ElevatorCalcStruct );
	m_motorMap[Elevator->GetType()] = new BaseMechMotor ( m_ntName,
	    Elevator,
	    BaseMechMotor::EndOfTravelSensorOption::NONE,
	    nullptr,
	    BaseMechMotor::EndOfTravelSensorOption::NONE,
	    nullptr );

	frontIntakeSensor = new DragonDigitalInput ( "frontIntakeSensor",RobotElementNames::DIGITAL_INPUT_USAGE::NOTE_MANAGER_FRONT_INTAKE_SENSOR,1,true,units::time::second_t ( 0 ) );
	backIntakeSensor = new DragonDigitalInput ( "backIntakeSensor",RobotElementNames::DIGITAL_INPUT_USAGE::NOTE_MANAGER_BACK_INTAKE_SENSOR,2,true,units::time::second_t ( 0 ) );
	feederSensor = new DragonDigitalInput ( "feederSensor",RobotElementNames::DIGITAL_INPUT_USAGE::NOTE_MANAGER_FEEDER_SENSOR,3,true,units::time::second_t ( 0 ) );
	launcherSensor = new DragonDigitalInput ( "launcherSensor",RobotElementNames::DIGITAL_INPUT_USAGE::NOTE_MANAGER_LAUNCHER_SENSOR,4,true,units::time::second_t ( 0 ) );
	placerInSensor = new DragonDigitalInput ( "placerInSensor",RobotElementNames::DIGITAL_INPUT_USAGE::NOTE_MANAGER_PLACER_IN_SENSOR,5,true,units::time::second_t ( 0 ) );
	placerMidSensor = new DragonDigitalInput ( "placerMidSensor",RobotElementNames::DIGITAL_INPUT_USAGE::NOTE_MANAGER_PLACER_MID_SENSOR,6,true,units::time::second_t ( 0 ) );
	placerOutSensor = new DragonDigitalInput ( "placerOutSensor",RobotElementNames::DIGITAL_INPUT_USAGE::NOTE_MANAGER_PLACER_OUT_SENSOR,7,true,units::time::second_t ( 0 ) );

	percentOutput = new ControlData (
	    ControlModes::CONTROL_TYPE::PERCENT_OUTPUT, // ControlModes::CONTROL_TYPE mode
	    ControlModes::CONTROL_RUN_LOCS::MOTOR_CONTROLLER, // ControlModes::CONTROL_RUN_LOCS server
	    "percentOutput", // std::string indentifier
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
	positionInch = new ControlData (
	    ControlModes::CONTROL_TYPE::POSITION_INCH, // ControlModes::CONTROL_TYPE mode
	    ControlModes::CONTROL_RUN_LOCS::MOTOR_CONTROLLER, // ControlModes::CONTROL_RUN_LOCS server
	    "positionInch", // std::string indentifier
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
	velocityRPS = new ControlData (
	    ControlModes::CONTROL_TYPE::VELOCITY_RPS, // ControlModes::CONTROL_TYPE mode
	    ControlModes::CONTROL_RUN_LOCS::MOTOR_CONTROLLER, // ControlModes::CONTROL_RUN_LOCS server
	    "velocityRPS", // std::string indentifier
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
	posDegreeAbs = new ControlData (
	    ControlModes::CONTROL_TYPE::POSITION_DEGREES, // ControlModes::CONTROL_TYPE mode
	    ControlModes::CONTROL_RUN_LOCS::MOTOR_CONTROLLER, // ControlModes::CONTROL_RUN_LOCS server
	    "posDegreeAbs", // std::string indentifier
	    0.01625, // double proportional
	    1.006E-06, // double integral
	    0, // double derivative
	    0, // double feedforward
	    ControlData::FEEDFORWARD_TYPE::VOLTAGE, // FEEDFORWARD_TYPE feedforwadType
	    0.035, // double integralZone
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

std::map<std::string, noteManagerGen::STATE_NAMES> noteManagerGen::stringToSTATE_NAMESEnumMap
{
	{"STATE_OFF", noteManagerGen::STATE_NAMES::STATE_OFF},
	{"STATE_READY", noteManagerGen::STATE_NAMES::STATE_READY},
	{"STATE_FEEDER_INTAKE", noteManagerGen::STATE_NAMES::STATE_FEEDER_INTAKE},
	{"STATE_EXPEL", noteManagerGen::STATE_NAMES::STATE_EXPEL},
	{"STATE_PLACER_INTAKE", noteManagerGen::STATE_NAMES::STATE_PLACER_INTAKE},
	{"STATE_HOLD_FEEDER_FRONT", noteManagerGen::STATE_NAMES::STATE_HOLD_FEEDER_FRONT},
	{"STATE_HOLD_FEEDER_BACK", noteManagerGen::STATE_NAMES::STATE_HOLD_FEEDER_BACK},
	{"STATE_INTAKE_TO_FEEDER", noteManagerGen::STATE_NAMES::STATE_INTAKE_TO_FEEDER},
	{"STATE_LAUNCHER_TO_PLACER_FRONT", noteManagerGen::STATE_NAMES::STATE_LAUNCHER_TO_PLACER_FRONT},
	{"STATE_LAUNCHER_TO_PLACER_BACK", noteManagerGen::STATE_NAMES::STATE_LAUNCHER_TO_PLACER_BACK},
	{"STATE_HOLD_FEEDER", noteManagerGen::STATE_NAMES::STATE_HOLD_FEEDER},
	{"STATE_READY_AUTO_LAUNCH", noteManagerGen::STATE_NAMES::STATE_READY_AUTO_LAUNCH},
	{"STATE_READY_MANUAL_LAUNCH", noteManagerGen::STATE_NAMES::STATE_READY_MANUAL_LAUNCH},
	{"STATE_PASS", noteManagerGen::STATE_NAMES::STATE_PASS},
	{"STATE_AUTO_LAUNCH", noteManagerGen::STATE_NAMES::STATE_AUTO_LAUNCH},
	{"STATE_MANUAL_LAUNCH", noteManagerGen::STATE_NAMES::STATE_MANUAL_LAUNCH},
	{"STATE_READY_ODOMETRY_LAUNCH", noteManagerGen::STATE_NAMES::STATE_READY_ODOMETRY_LAUNCH},
	{"STATE_AUTO_LAUNCH_ODOMETRY", noteManagerGen::STATE_NAMES::STATE_AUTO_LAUNCH_ODOMETRY},
	{"STATE_HOLD_PLACER_FRONT", noteManagerGen::STATE_NAMES::STATE_HOLD_PLACER_FRONT},
	{"STATE_HOLD_PLACER_BACK", noteManagerGen::STATE_NAMES::STATE_HOLD_PLACER_BACK},
	{"STATE_INTAKE_TO_PLACER", noteManagerGen::STATE_NAMES::STATE_INTAKE_TO_PLACER},
	{"STATE_PREPARE_PLACE_AMP", noteManagerGen::STATE_NAMES::STATE_PREPARE_PLACE_AMP},
	{"STATE_PREPARE_PLACE_TRAP", noteManagerGen::STATE_NAMES::STATE_PREPARE_PLACE_TRAP},
	{"STATE_PLACE_AMP", noteManagerGen::STATE_NAMES::STATE_PLACE_AMP},
	{"STATE_PLACE_TRAP", noteManagerGen::STATE_NAMES::STATE_PLACE_TRAP},
	{"STATE_PLACER_TO_LAUNCHER_FRONT", noteManagerGen::STATE_NAMES::STATE_PLACER_TO_LAUNCHER_FRONT},
	{"STATE_PLACER_TO_LAUNCHER_BACK", noteManagerGen::STATE_NAMES::STATE_PLACER_TO_LAUNCHER_BACK},
	{"STATE_BACKUP_MANUAL_LAUNCH", noteManagerGen::STATE_NAMES::STATE_BACKUP_MANUAL_LAUNCH},
	{"STATE_BACKUP_MANUAL_PLACE", noteManagerGen::STATE_NAMES::STATE_BACKUP_MANUAL_PLACE},};

void noteManagerGen::Initialize ( RobotConfigMgr::RobotIdentifier robotFullName )
{
	if ( false ) {}
	else if ( RobotConfigMgr::RobotIdentifier::PRACTICE_BOT_9999 == robotFullName )
	{

		frontIntake->SetRemoteSensor ( 0,
		                               ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_Off );
		frontIntake->Invert ( false );
		frontIntake->EnableBrakeMode ( true );
		frontIntake->SetSmartCurrentLimiting ( 40 );
		frontIntake->SetSecondaryCurrentLimiting ( 40, 0 );
		frontIntake->SetDiameter ( units::length::inch_t ( units::length::meter_t ( 1 ) ).to<double>() );
		frontIntake->EnableDisableLimitSwitches ( false );
// frontIntake : Follower motor mode is not enabled
		frontIntake->SetVoltageRamping ( units::time::second_t ( units::time::second_t ( 0.25 ) ).to<double>(),
		                                 units::time::second_t ( units::time::second_t ( 0 ) ).to<double>() );
		frontIntake->SetSensorInverted ( false );

		backIntake->SetRemoteSensor ( 0,
		                              ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_Off );
		backIntake->Invert ( false );
		backIntake->EnableBrakeMode ( true );
		backIntake->SetSmartCurrentLimiting ( 40 );
		backIntake->SetSecondaryCurrentLimiting ( 40, 0 );
		backIntake->SetDiameter ( units::length::inch_t ( units::length::meter_t ( 1 ) ).to<double>() );
		backIntake->EnableDisableLimitSwitches ( false );
// backIntake : Follower motor mode is not enabled
		backIntake->SetVoltageRamping ( units::time::second_t ( units::time::second_t ( 0.25 ) ).to<double>(),
		                                units::time::second_t ( units::time::second_t ( 0 ) ).to<double>() );
		backIntake->SetSensorInverted ( false );

		Transfer->SetRemoteSensor ( 0,
		                            ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_Off );
		Transfer->Invert ( true );
		Transfer->EnableBrakeMode ( true );
		Transfer->SetSmartCurrentLimiting ( 40 );
		Transfer->SetSecondaryCurrentLimiting ( 40, 0 );
		Transfer->SetDiameter ( units::length::inch_t ( units::length::meter_t ( 1 ) ).to<double>() );
		Transfer->EnableDisableLimitSwitches ( false );
// Transfer : Follower motor mode is not enabled
		Transfer->SetVoltageRamping ( units::time::second_t ( units::time::second_t ( 0.25 ) ).to<double>(),
		                              units::time::second_t ( units::time::second_t ( 0 ) ).to<double>() );
		Transfer->SetSensorInverted ( false );

		Feeder->SetRemoteSensor ( 0,
		                          ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_Off );
		Feeder->Invert ( false );
		Feeder->EnableBrakeMode ( true );
		Feeder->SetSmartCurrentLimiting ( 50 );
		Feeder->SetSecondaryCurrentLimiting ( 50, 0 );
		Feeder->SetDiameter ( units::length::inch_t ( units::length::meter_t ( 1 ) ).to<double>() );
		Feeder->EnableDisableLimitSwitches ( false );
// Feeder : Follower motor mode is not enabled
		Feeder->SetVoltageRamping ( units::time::second_t ( units::time::second_t ( 0.25 ) ).to<double>(),
		                            units::time::second_t ( units::time::second_t ( 0 ) ).to<double>() );
		Feeder->SetSensorInverted ( false );

		launcherTop->SetCurrentLimits ( false,
		                                units::current::ampere_t ( 0 ),
		                                true,
		                                units::current::ampere_t ( 30 ),
		                                units::current::ampere_t ( 40 ),
		                                units::time::second_t ( 0.3 ) );
		launcherTop->ConfigHWLimitSW ( false, // enableForward
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
		launcherTop->ConfigMotorSettings ( ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive, // ctre::phoenixpro::signals::InvertedValue
		                                   ctre::phoenix6::signals::NeutralModeValue::Coast, // ctre::phoenixpro::signals::NeutralModeValue
		                                   0, // deadbandPercent
		                                   0, // peakForwardDutyCycle
		                                   0 ); // peakReverseDutyCycle
		launcherTop->SetAsFollowerMotor ( 0 ); // masterCANID
		launcherTop->SetRemoteSensor ( 0, // canID
		                               RemoteSensorSource::RemoteSensorSource_Off ); // ctre::phoenix::motorcontrol::RemoteSensorSource
		launcherTop->SetDiameter ( 0 ); // double diameter
		launcherTop->SetVoltageRamping ( units::time::second_t ( units::time::second_t ( 0 ) ).to<double>(),
		                                 units::time::second_t ( units::time::second_t ( 0 ) ).to<double>() );
		launcherTop->SetSensorInverted ( false );
		launcherBottom->SetCurrentLimits ( false,
		                                   units::current::ampere_t ( 0 ),
		                                   true,
		                                   units::current::ampere_t ( 30 ),
		                                   units::current::ampere_t ( 40 ),
		                                   units::time::second_t ( 0.3 ) );
		launcherBottom->ConfigHWLimitSW ( false, // enableForward
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
		launcherBottom->ConfigMotorSettings ( ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive, // ctre::phoenixpro::signals::InvertedValue
		                                      ctre::phoenix6::signals::NeutralModeValue::Coast, // ctre::phoenixpro::signals::NeutralModeValue
		                                      0, // deadbandPercent
		                                      0, // peakForwardDutyCycle
		                                      0 ); // peakReverseDutyCycle
		launcherBottom->SetAsFollowerMotor ( 0 ); // masterCANID
		launcherBottom->SetRemoteSensor ( 0, // canID
		                                  RemoteSensorSource::RemoteSensorSource_Off ); // ctre::phoenix::motorcontrol::RemoteSensorSource
		launcherBottom->SetDiameter ( 0 ); // double diameter
		launcherBottom->SetVoltageRamping ( units::time::second_t ( units::time::second_t ( 0 ) ).to<double>(),
		                                    units::time::second_t ( units::time::second_t ( 0 ) ).to<double>() );
		launcherBottom->SetSensorInverted ( false );
		launcherAngle->SetRemoteSensor ( 0,
		                                 ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_Off );
		launcherAngle->Invert ( false );
		launcherAngle->EnableBrakeMode ( true );
		launcherAngle->SetSmartCurrentLimiting ( 40 );
		launcherAngle->SetSecondaryCurrentLimiting ( 40, 0 );
		launcherAngle->SetDiameter ( units::length::inch_t ( units::length::meter_t ( 1 ) ).to<double>() );
		launcherAngle->EnableDisableLimitSwitches ( true );
// launcherAngle : Follower motor mode is not enabled
		launcherAngle->SetVoltageRamping ( units::time::second_t ( units::time::second_t ( 0 ) ).to<double>(),
		                                   units::time::second_t ( units::time::second_t ( 0 ) ).to<double>() );
		launcherAngle->SetSensorInverted ( false );

		Placer->SetRemoteSensor ( 0,
		                          ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_Off );
		Placer->Invert ( false );
		Placer->EnableBrakeMode ( true );
		Placer->SetSmartCurrentLimiting ( 80 );
		Placer->SetSecondaryCurrentLimiting ( 80, 0 );
		Placer->SetDiameter ( units::length::inch_t ( units::length::meter_t ( 1 ) ).to<double>() );
		Placer->EnableDisableLimitSwitches ( true );
// Placer : Follower motor mode is not enabled
		Placer->SetVoltageRamping ( units::time::second_t ( units::time::second_t ( 0.25 ) ).to<double>(),
		                            units::time::second_t ( units::time::second_t ( 0 ) ).to<double>() );
		Placer->SetSensorInverted ( false );

		Elevator->SetRemoteSensor ( 0,
		                            ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_Off );
		Elevator->Invert ( false );
		Elevator->EnableBrakeMode ( true );
		Elevator->SetSmartCurrentLimiting ( 40 );
		Elevator->SetSecondaryCurrentLimiting ( 40, 0 );
		Elevator->SetDiameter ( units::length::inch_t ( units::length::meter_t ( 1 ) ).to<double>() );
		Elevator->EnableDisableLimitSwitches ( true );
// Elevator : Follower motor mode is not enabled
		Elevator->SetVoltageRamping ( units::time::second_t ( units::time::second_t ( 0.25 ) ).to<double>(),
		                              units::time::second_t ( units::time::second_t ( 0.1 ) ).to<double>() );
		Elevator->SetSensorInverted ( false );

// frontIntakeSensor : Digital inputs do not have initialization needs
// backIntakeSensor : Digital inputs do not have initialization needs
// feederSensor : Digital inputs do not have initialization needs
// launcherSensor : Digital inputs do not have initialization needs
// placerInSensor : Digital inputs do not have initialization needs
// placerMidSensor : Digital inputs do not have initialization needs
// placerOutSensor : Digital inputs do not have initialization needs

// percentOutput : ControlData does not have initialization needs
// positionInch : ControlData does not have initialization needs
// velocityRPS : ControlData does not have initialization needs
// posDegreeAbs : ControlData does not have initialization needs

//todo create initialization for Off
//todo create initialization for Ready
//todo create initialization for feederIntake
//todo create initialization for Expel
//todo create initialization for placerIntake
//todo create initialization for holdFeederFront
//todo create initialization for holdFeederBack
//todo create initialization for intakeToFeeder
//todo create initialization for launcherToPlacerFront
//todo create initialization for launcherToPlacerBack
//todo create initialization for holdFeeder
//todo create initialization for readyAutoLaunch
//todo create initialization for readyManualLaunch
//todo create initialization for Pass
//todo create initialization for autoLaunch
//todo create initialization for manualLaunch
//todo create initialization for readyOdometryLaunch
//todo create initialization for autoLaunchOdometry
//todo create initialization for holdPlacerFront
//todo create initialization for holdPlacerBack
//todo create initialization for intakeToPlacer
//todo create initialization for preparePlaceAmp
//todo create initialization for preparePlaceTrap
//todo create initialization for placeAmp
//todo create initialization for placeTrap
//todo create initialization for placerToLauncherFront
//todo create initialization for placerToLauncherBack
//todo create initialization for backupManualLaunch
//todo create initialization for backupManualPlace
	}
}

void noteManagerGen::SetCurrentState ( int state, bool run )
{
	StateMgr::SetCurrentState ( state, run );
}

/// @brief  Set the control constants (e.g. PIDF values).
/// @param [in] ControlData*                                   pid:  the control constants
/// @return void
void noteManagerGen::SetControlConstants ( RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, int slot, ControlData pid )
{
	auto motor = GetMotorMech ( identifier );
	if ( motor != nullptr )
	{
		motor->SetControlConstants ( slot, pid );
	}
}

/// @brief update the output to the mechanism using the current controller and target value(s)
/// @return void
void noteManagerGen::Update()
{
	for ( auto motor : m_motorMap )
	{
		motor.second->Update();
	}
}

void noteManagerGen::UpdateTarget ( RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, double percentOutput )
{
	auto motor = GetMotorMech ( identifier );
	if ( motor != nullptr )
	{
		motor->UpdateTarget ( percentOutput );
	}
}

void noteManagerGen::UpdateTarget ( RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, units::angle::degree_t angle )
{
	auto motor = GetMotorMech ( identifier );
	if ( motor != nullptr )
	{
		motor->UpdateTarget ( angle );
	}
}

void noteManagerGen::UpdateTarget ( RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, units::angular_velocity::revolutions_per_minute_t angVel )
{
	auto motor = GetMotorMech ( identifier );
	if ( motor != nullptr )
	{
		motor->UpdateTarget ( angVel );
	}
}
void noteManagerGen::UpdateTarget ( RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, units::length::inch_t position )
{
	auto motor = GetMotorMech ( identifier );
	if ( motor != nullptr )
	{
		motor->UpdateTarget ( position );
	}
}
void noteManagerGen::UpdateTarget ( RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, units::velocity::feet_per_second_t velocity )
{
	auto motor = GetMotorMech ( identifier );
	if ( motor != nullptr )
	{
		motor->UpdateTarget ( velocity );
	}
}

bool noteManagerGen::IsAtMinPosition ( RobotElementNames::MOTOR_CONTROLLER_USAGE identifier ) const
{
	auto motor = GetMotorMech ( identifier );
	if ( motor != nullptr )
	{
		return motor->IsAtMinTravel();
	}
	return false;
}

bool noteManagerGen::IsAtMaxPosition ( RobotElementNames::MOTOR_CONTROLLER_USAGE identifier ) const
{
	auto motor = GetMotorMech ( identifier );
	if ( motor != nullptr )
	{
		return motor->IsAtMaxTravel();
	}
	return false;
}

BaseMechMotor *noteManagerGen::GetMotorMech ( RobotElementNames::MOTOR_CONTROLLER_USAGE usage ) const
{
	auto itr = m_motorMap.find ( usage );
	if ( itr != m_motorMap.end() )
	{
		return itr->second;
	}
	return nullptr;
}

std::vector<RobotElementNames::MOTOR_CONTROLLER_USAGE> noteManagerGen::GetMotorUsages() const
{
	std::vector<RobotElementNames::MOTOR_CONTROLLER_USAGE> output;
	for ( auto itr = m_motorMap.begin(); itr != m_motorMap.end(); ++itr )
	{
		output.emplace_back ( itr->first );
	}
	return output;
}

void noteManagerGen::UpdateTarget ( RobotElementNames::SOLENOID_USAGE identifier, bool extend )
{
	auto sol = GetSolenoidMech ( identifier );
	if ( sol != nullptr )
	{
		sol->ActivateSolenoid ( extend );
	}
}

bool noteManagerGen::IsAtMinPosition ( RobotElementNames::SOLENOID_USAGE identifier ) const
{
	auto sol = GetSolenoidMech ( identifier );
	if ( sol != nullptr )
	{
		return !sol->IsSolenoidActivated();
	}
	return false;
}

bool noteManagerGen::IsAtMaxPosition ( RobotElementNames::SOLENOID_USAGE identifier ) const
{
	auto sol = GetSolenoidMech ( identifier );
	if ( sol != nullptr )
	{
		return sol->IsSolenoidActivated();
	}
	return false;
}

BaseMechSolenoid *noteManagerGen::GetSolenoidMech ( RobotElementNames::SOLENOID_USAGE usage ) const
{
	auto itr = m_solenoidMap.find ( usage );
	if ( itr != m_solenoidMap.end() )
	{
		return itr->second;
	}
	return nullptr;
}

std::vector<RobotElementNames::SOLENOID_USAGE> noteManagerGen::GetSolenoidUsages() const
{
	std::vector<RobotElementNames::SOLENOID_USAGE> output;
	for ( auto itr = m_solenoidMap.begin(); itr != m_solenoidMap.end(); ++itr )
	{
		output.emplace_back ( itr->first );
	}
	return output;
}

BaseMechServo *noteManagerGen::GetServoMech ( RobotElementNames::SERVO_USAGE usage ) const
{
	auto itr = m_servoMap.find ( usage );
	if ( itr != m_servoMap.end() )
	{
		return itr->second;
	}
	return nullptr;
}

std::vector<RobotElementNames::SERVO_USAGE> noteManagerGen::GetServoUsages() const
{
	std::vector<RobotElementNames::SERVO_USAGE> output;
	for ( auto itr = m_servoMap.begin(); itr != m_servoMap.end(); ++itr )
	{
		output.emplace_back ( itr->first );
	}
	return output;
}

void noteManagerGen::Cyclic()
{
	CheckForTuningEnabled();
	if ( m_tuning )
	{
		ReadTuningParamsFromNT();
	}
}

void noteManagerGen::CheckForTuningEnabled()
{
	bool pastTuning = m_tuning;
	m_tuning = m_table.get()->GetBoolean ( m_tuningIsEnabledStr, false );
	if ( pastTuning != m_tuning && m_tuning == true )
	{
		PushTuningParamsToNT();
	}
}

void noteManagerGen::ReadTuningParamsFromNT()
{
	positionInch->SetIZone ( m_table.get()->GetNumber ( "positionInch_iZone", 0 ) );
	positionInch->SetF ( m_table.get()->GetNumber ( "positionInch_fGain", 0 ) );
	positionInch->SetP ( m_table.get()->GetNumber ( "positionInch_pGain", 0 ) );
	positionInch->SetI ( m_table.get()->GetNumber ( "positionInch_iGain", 0 ) );
	positionInch->SetD ( m_table.get()->GetNumber ( "positionInch_dGain", 0 ) );
	velocityRPS->SetIZone ( m_table.get()->GetNumber ( "velocityRPS_iZone", 0 ) );
	velocityRPS->SetF ( m_table.get()->GetNumber ( "velocityRPS_fGain", 0 ) );
	velocityRPS->SetP ( m_table.get()->GetNumber ( "velocityRPS_pGain", 0 ) );
	velocityRPS->SetI ( m_table.get()->GetNumber ( "velocityRPS_iGain", 0 ) );
	velocityRPS->SetD ( m_table.get()->GetNumber ( "velocityRPS_dGain", 0 ) );
	posDegreeAbs->SetIZone ( m_table.get()->GetNumber ( "posDegreeAbs_iZone", 0.035 ) );
	posDegreeAbs->SetF ( m_table.get()->GetNumber ( "posDegreeAbs_fGain", 0 ) );
	posDegreeAbs->SetP ( m_table.get()->GetNumber ( "posDegreeAbs_pGain", 0.01625 ) );
	posDegreeAbs->SetI ( m_table.get()->GetNumber ( "posDegreeAbs_iGain", 1.006E-06 ) );
	posDegreeAbs->SetD ( m_table.get()->GetNumber ( "posDegreeAbs_dGain", 0 ) );

}

void noteManagerGen::PushTuningParamsToNT()
{
	m_table.get()->PutNumber ( "positionInch_iZone", positionInch->GetIZone() );
	m_table.get()->PutNumber ( "positionInch_fGain", positionInch->GetF() );
	m_table.get()->PutNumber ( "positionInch_pGain", positionInch->GetP() );
	m_table.get()->PutNumber ( "positionInch_iGain", positionInch->GetI() );
	m_table.get()->PutNumber ( "positionInch_dGain", positionInch->GetD() );
	m_table.get()->PutNumber ( "velocityRPS_iZone", velocityRPS->GetIZone() );
	m_table.get()->PutNumber ( "velocityRPS_fGain", velocityRPS->GetF() );
	m_table.get()->PutNumber ( "velocityRPS_pGain", velocityRPS->GetP() );
	m_table.get()->PutNumber ( "velocityRPS_iGain", velocityRPS->GetI() );
	m_table.get()->PutNumber ( "velocityRPS_dGain", velocityRPS->GetD() );
	m_table.get()->PutNumber ( "posDegreeAbs_iZone", posDegreeAbs->GetIZone() );
	m_table.get()->PutNumber ( "posDegreeAbs_fGain", posDegreeAbs->GetF() );
	m_table.get()->PutNumber ( "posDegreeAbs_pGain", posDegreeAbs->GetP() );
	m_table.get()->PutNumber ( "posDegreeAbs_iGain", posDegreeAbs->GetI() );
	m_table.get()->PutNumber ( "posDegreeAbs_dGain", posDegreeAbs->GetD() );

}