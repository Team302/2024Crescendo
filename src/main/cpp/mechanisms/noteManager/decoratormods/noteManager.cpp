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
// This file was automatically generated by the Team 302 code generator version 1.3.0.8
// Generated on Wednesday, February 21, 2024 8:17:08 PM

// C++ Includes

// FRC Includes

// Team 302 includes
#include "PeriodicLooper.h"
#include "mechanisms/noteManager/decoratormods/noteManager.h"

#include "hw/DragonSparkMax.h"
#include "hw/DragonSparkFlex.h"
#include "hw/DragonTalonFX.h"
#include "hw/DragonDigitalInput.h"
#include "mechanisms/noteManager/decoratormods/OffState.h"
#include "mechanisms/noteManager/decoratormods/ReadyState.h"
#include "mechanisms/noteManager/decoratormods/feederIntakeState.h"
#include "mechanisms/noteManager/decoratormods/ExpelState.h"
#include "mechanisms/noteManager/decoratormods/placerIntakeState.h"
#include "mechanisms/noteManager/decoratormods/launcherToPlacerState.h"
#include "mechanisms/noteManager/decoratormods/holdFeederState.h"
#include "mechanisms/noteManager/decoratormods/readyAutoLaunchState.h"
#include "mechanisms/noteManager/decoratormods/readyManualLaunchState.h"
#include "mechanisms/noteManager/decoratormods/PassState.h"
#include "mechanisms/noteManager/decoratormods/autoLaunchState.h"
#include "mechanisms/noteManager/decoratormods/manualLaunchState.h"
#include "mechanisms/noteManager/decoratormods/readyOdometryLaunchState.h"
#include "mechanisms/noteManager/decoratormods/autoLaunchOdometryState.h"
#include "mechanisms/noteManager/decoratormods/preparePlaceAmpState.h"
#include "mechanisms/noteManager/decoratormods/preparePlaceTrapState.h"
#include "mechanisms/noteManager/decoratormods/placeAmpState.h"
#include "mechanisms/noteManager/decoratormods/placeTrapState.h"
#include "mechanisms/noteManager/decoratormods/placerToLauncherState.h"
#include "mechanisms/noteManager/decoratormods/backupManualLaunchState.h"
#include "mechanisms/noteManager/decoratormods/backupManualPlaceState.h"
#include "mechanisms/noteManager/decoratormods/holdPlacerState.h"

#include "DragonVision/DragonVision.h"
#include "robotstate/RobotState.h"
#include "utils/logging/Logger.h"
#include "utils/logging/DataTrace.h"
#include "utils/FMSData.h"
#include "chassis/ChassisConfig.h"
#include "chassis/ChassisConfigMgr.h"

#include "chassis/DragonDriveTargetFinder.h"

using std::string;
using namespace noteManagerStates;

/// @brief  This method constructs the mechanism using composition with its various actuators and sensors.
/// @param controlFileName The control file with the PID constants and Targets for each state
/// @param networkTableName Location for logging information
/// @param motor  Motor in the mechanims - code generator should probably use the usage for the variable name
/// @param otherMotor Same as previous
/// @param solenoid Solenoid in the mechanism - code generator should probably use the usage for the variable name
/// Additional actuators and sensors are also in this list.
noteManager::noteManager(noteManagerGen *base, RobotConfigMgr::RobotIdentifier activeRobotId) : noteManagerGen(activeRobotId), IRobotStateChangeSubscriber(),
																								m_noteManager(base)
{
	PeriodicLooper::GetInstance()->RegisterAll(this);

	m_scoringMode = RobotStateChanges::ScoringMode::Launcher;
	m_climbMode = RobotStateChanges::ClimbMode::ClimbModeOff;
	m_gamePeriod = RobotStateChanges::GamePeriod::Disabled;

	m_robotState = RobotState::GetInstance();

	m_robotState->RegisterForStateChanges(this, RobotStateChanges::StateChange::DesiredScoringMode);
	m_robotState->RegisterForStateChanges(this, RobotStateChanges::StateChange::ClimbModeStatus);
	m_robotState->RegisterForStateChanges(this, RobotStateChanges::StateChange::GameState);
}

void noteManager::RunCommonTasks()
{
	// This function is called once per loop before the current state Run()
	Cyclic();
	ResetLauncherAngle();
	ResetElevator();

	Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Sensors"), string("Front"), getfrontIntakeSensor()->Get());
	Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Sensors"), string("Back"), getbackIntakeSensor()->Get());
	Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Sensors"), string("Feeder"), getfeederSensor()->Get());
	Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Sensors"), string("Launcher"), getlauncherSensor()->Get());
	// Processing related to current monitor
	MonitorMotorCurrents();

#ifdef INCLUDE_DATA_TRACE
	double intakeDifferentialCurrent =
#endif
		MonitorForNoteInIntakes();

#ifdef INCLUDE_DATA_TRACE
	double wheelSetTop = units::angular_velocity::radians_per_second_t(units::angular_velocity::revolutions_per_minute_t(getlauncherTop()->GetRPS() * 60)).to<double>();
	double wheelSetBottom = units::angular_velocity::radians_per_second_t(units::angular_velocity::revolutions_per_minute_t(getlauncherBottom()->GetRPS() * 60)).to<double>();
	double angle = getlauncherAngle()->GetCounts();
	double elevator = getElevator()->GetCounts();
	DataTrace::GetInstance()->sendElevatorData(elevator);
	DataTrace::GetInstance()->sendLauncherData(wheelSetTop, wheelSetBottom, angle);

	if (true)
	{
		m_frontIntakeAverage = getfrontIntake()->GetCurrent();
		m_backIntakeAverage = getbackIntake()->GetCurrent();
		m_transferAverage = getTransfer()->GetCurrent();
		m_placerAverage = getPlacer()->GetCurrent();
		m_feederAverage = getFeeder()->GetCurrent();

		double NoteInIntake = m_noteInIntake ? 40 : 0;
		DataTrace::GetInstance()->sendNoteMotorData(m_frontIntakeAverage, m_backIntakeAverage, m_transferAverage, m_placerAverage, m_feederAverage, 0.0, intakeDifferentialCurrent, NoteInIntake);
	}

	double FrontIntakeSensor = getfrontIntakeSensor()->Get() ? 50 : 0;
	double BackIntakeSensor = getbackIntakeSensor()->Get() ? 50 : 0;
	double FeederSensor = getfeederSensor()->Get() ? 50 : 0;
	double LauncherSensor = getlauncherSensor()->Get() ? 50 : 0;
	double PlacerInSensor = getplacerInSensor()->Get() ? 50 : 0;
	double PlacerMidSensor = getplacerMidSensor()->Get() ? 50 : 0;
	double PlacerOutSensor = getplacerOutSensor()->Get() ? 50 : 0;

	DataTrace::GetInstance()->sendNoteSensorData(FrontIntakeSensor, BackIntakeSensor, FeederSensor, LauncherSensor, PlacerInSensor, PlacerMidSensor, PlacerOutSensor);
#endif
}

void noteManager::MonitorMotorCurrents()
{
	getFeeder()->MonitorCurrent();
	getfrontIntake()->MonitorCurrent();
	getbackIntake()->MonitorCurrent();
	getTransfer()->MonitorCurrent();
	getPlacer()->MonitorCurrent();
}

double noteManager::MonitorForNoteInIntakes()
{
	// In order to detect that a note is being inhaled into the robot, the differential
	// current needs to exceed incomingThreshold
	const double intakeIncomingThreshold = 10;

	// If the differential current is below forwardingThreshold, assume that the note has
	// been forwarded to the next noteManagement stage

	// const double intakeForwardingThreshold = 10;

	const double scaledTransferValueThreshold = 50;

	const double feederThreshold = 40;

	m_frontIntakeAverage = getfrontIntake()->GetCurrent();
	m_backIntakeAverage = getbackIntake()->GetCurrent();
	m_transferAverage = getTransfer()->GetCurrent();
	m_feederAverage = getFeeder()->GetCurrent();
	double intakeDifferenceAvg = std::abs(m_frontIntakeAverage - m_backIntakeAverage);

	double scaledTransferValue = m_transferAverage * intakeDifferenceAvg;

	if (intakeDifferenceAvg > intakeIncomingThreshold)
		m_noteInIntake = true;

	if (scaledTransferValue > scaledTransferValueThreshold)
	{
		m_noteInIntake = false;
		m_noteInFeeder = true;
	}
	if (m_feederAverage > feederThreshold)
	{
		m_noteInFeeder = false;
	}
	return intakeDifferenceAvg;
}

void noteManager::ResetElevator()
{
	if (getElevator()->IsReverseLimitSwitchClosed())
		getElevator()->SetSelectedSensorPosition(0);
}

void noteManager::ResetLauncherAngle()
{
	if (getlauncherAngle()->IsReverseLimitSwitchClosed())
		getlauncherAngle()->SetSelectedSensorPosition(0);
}

void noteManager::SetCurrentState(int state, bool run)
{
	noteManagerGen::SetCurrentState(state, run);
	Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("State Transition"), string("Note Manager Current State"), GetCurrentStatePtr()->GetStateName());
}

units::length::meter_t noteManager::GetVisionDistance()
{
	units::length::meter_t distance{units::length::meter_t(0)};
	std::optional<VisionData> optionalVisionData = DragonVision::GetDragonVision()->GetVisionData(DragonVision::VISION_ELEMENT::SPEAKER);
	if (optionalVisionData)
	{
		frc::Translation3d translate{optionalVisionData.value().translationToTarget};
		distance = optionalVisionData.value().translationToTarget.X();
		Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Launcher"), string("X"), optionalVisionData.value().translationToTarget.X().to<double>());
	}
	return distance - units::length::meter_t(0.1);
}

bool noteManager::HasVisionTarget()
{
	std::optional<VisionData> optionalVisionData = DragonVision::GetDragonVision()->GetVisionData(DragonVision::VISION_ELEMENT::SPEAKER);
	if (optionalVisionData)
	{
		return true;
	}
	return false;
}

void noteManager::Update(RobotStateChanges::StateChange change, int value)
{

	if (change == RobotStateChanges::DesiredScoringMode)
		m_scoringMode = static_cast<RobotStateChanges::ScoringMode>(value);
	else if (change == RobotStateChanges::ClimbModeStatus)
		m_climbMode = static_cast<RobotStateChanges::ClimbMode>(value);
	else if (change == RobotStateChanges::GameState)
		m_gamePeriod = static_cast<RobotStateChanges::GamePeriod>(value);
}

double noteManager::GetRequiredLaunchAngle()
{
	frc::DriverStation::Alliance allianceColor = FMSData::GetInstance()->GetAllianceColor();
	double distanceFromTarget = 3.5;
	double launchAngle = 0;
	frc::Pose3d fieldElementPose = frc::Pose3d{};
	auto config = ChassisConfigMgr::GetInstance()->GetCurrentConfig();
	auto chassis = config != nullptr ? config->GetSwerveChassis() : nullptr;
	frc::Pose2d chassisPos = frc::Pose2d();

	if (HasVisionTarget())
	{
		distanceFromTarget = GetVisionDistance().to<double>();
	}
	else if (chassis != nullptr)
	{
		fieldElementPose = allianceColor == frc::DriverStation::Alliance::kRed ? frc::Pose3d{FieldConstants::GetInstance()->GetFieldElement(FieldConstants::FIELD_ELEMENT::RED_SPEAKER)} /*load red speaker*/ : frc::Pose3d{FieldConstants::GetInstance()->GetFieldElement(FieldConstants::FIELD_ELEMENT::BLUE_SPEAKER)}; /*load blue speaker*/
		chassisPos = chassis->GetPose();

		distanceFromTarget = sqrt(pow((fieldElementPose.X() - chassisPos.X()).to<double>(), 2) + pow((fieldElementPose.Y() - chassisPos.Y()).to<double>(), 2));
	}

	launchAngle = 80.0 + (-44.2 * distanceFromTarget) + (6.09 * distanceFromTarget * distanceFromTarget);

	if (launchAngle > 40)
	{
		launchAngle = 40;
	}

	return launchAngle;
}

// todo delete this function once Joe creates and tests it
std::tuple<DragonDriveTargetFinder::TARGET_INFO, units::length::meter_t> noteManager::GetDistance(FINDER_OPTION option, DragonVision::VISION_ELEMENT item)
{
	tuple<DragonDriveTargetFinder::TARGET_INFO, units::length::meter_t> targetInfo;
	targetInfo = make_tuple(DragonDriveTargetFinder::TARGET_INFO::VISION_BASED, units::length::meter_t(1.5));

	return targetInfo;
}

void noteManager::SetLauncherTargetsForAutoLaunch()
{
	std::tuple<units::angular_velocity::radians_per_second_t, units::angular_velocity::radians_per_second_t, units::angle::degree_t> launchParameters = GetRequiredLaunchParameters();

	SetLauncherTopWheelsTarget(std::get<0>(launchParameters));
	SetLauncherBottomWheelsTarget(std::get<1>(launchParameters));
	SetLauncherAngleTarget(std::get<2>(launchParameters));

	UpdateTarget(RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_TOP, units::angular_velocity::revolutions_per_minute_t(GetLauncherTopWheelsTarget()));
	UpdateTarget(RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_BOTTOM, units::angular_velocity::revolutions_per_minute_t(GetLauncherBottomWheelsTarget()));
	UpdateTarget(RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_ANGLE, GetLauncherAngleTarget());
}

void noteManager::MaintainCurrentLauncherTargetsForAutoLaunch()
{
	UpdateTarget(RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_TOP, units::angular_velocity::revolutions_per_minute_t(GetLauncherTopWheelsTarget()));
	UpdateTarget(RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_BOTTOM, units::angular_velocity::revolutions_per_minute_t(GetLauncherBottomWheelsTarget()));
	UpdateTarget(RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_ANGLE, GetLauncherAngleTarget());
}

bool noteManager::LauncherTargetsForAutoLaunchAchieved() const
{
	units::angular_velocity::revolutions_per_minute_t topSpeed = units::angular_velocity::revolutions_per_minute_t(getlauncherTop()->GetRPS() * 60); // RPS is revs/sec not rad/sec
	units::angular_velocity::revolutions_per_minute_t botSpeed = units::angular_velocity::revolutions_per_minute_t(getlauncherBottom()->GetRPS() * 60);
	units::angle::degree_t launcherAngle = units::angle::degree_t(getlauncherAngle()->GetCounts());

	bool wheelTargetSpeedAchieved = (topSpeed > GetLauncherTopWheelsTarget()) && (botSpeed > GetLauncherBottomWheelsTarget());
	bool launcherTargetAngleAchieved = std::abs((launcherAngle - GetLauncherAngleTarget()).to<double>()) <= 0.5;

	return launcherTargetAngleAchieved && wheelTargetSpeedAchieved;
}

/// @brief
/// @return top wheel speed, bottom wheel speed, launcher angle
std::tuple<units::angular_velocity::radians_per_second_t, units::angular_velocity::radians_per_second_t, units::angle::degree_t> noteManager::GetRequiredLaunchParameters()
{
	double launcherAngle = 50;		// 50 is the angle for manualLaunch
	double topLaunchSpeed = 400;	// 400 is the default for manualLaunch
	double bottomLaunchSpeed = 400; // 400 is the default for manualLaunch

	// todo uncomment the next line and delete the line after it once Joe creates GetDistance
	// std::tuple<TARGET_INFO, units::length::meter_t> distanceToSpeaker = DragonDriveTargetFinder::GetInstance()->GetDistance(DragonDriveTargetFinder::FINDER_OPTION::FUSE_IF_POSSIBLE, DragonVision::VISION_ELEMENT::SPEAKER);
	std::tuple<DragonDriveTargetFinder::TARGET_INFO, units::length::meter_t>
		distanceToSpeaker = GetDistance(FINDER_OPTION::FUSE_IF_POSSIBLE, DragonVision::VISION_ELEMENT::SPEAKER);

	// todo uncomment the next line and delete the line after it once Joe creates GetDistance
	// if (std::get<0>(distanceToSpeaker) != DragonDriveTargetFinder::TARGET_INFO::NOT_FOUND)
	if (std::get<0>(distanceToSpeaker) != DragonDriveTargetFinder::TARGET_INFO::NOT_FOUND)
	{
		double distanceFromTarget_in = units::length::inch_t(std::get<0>(distanceToSpeaker)).to<double>();

		launcherAngle = 77.6721 + (-0.616226 * distanceFromTarget_in) + (0.00121458 * distanceFromTarget_in * distanceFromTarget_in);

		// limit the resulting launcher angle
		launcherAngle = launcherAngle > 50 ? 50 : launcherAngle;
		launcherAngle = launcherAngle < 0 ? 0 : launcherAngle;

		topLaunchSpeed = distanceFromTarget_in < 90 /*inch*/ ? 400 : 500;
		bottomLaunchSpeed = distanceFromTarget_in < 90 /*inch*/ ? 400 : 500;
	}

	return make_tuple(units::angular_velocity::radians_per_second_t(topLaunchSpeed), units::angular_velocity::radians_per_second_t(bottomLaunchSpeed), units::angle::degree_t(launcherAngle));
}

bool noteManager::autoLaunchReady()
{
	return false;

	std::optional<VisionData> optionalVisionData = DragonVision::GetDragonVision()->GetVisionData(DragonVision::VISION_ELEMENT::SPEAKER);
	if (optionalVisionData.has_value())
	{
		VisionData visionData = optionalVisionData.value();
		if (visionData.transformToTarget.Y().to<double>() <= 0.5 && GetVisionDistance().to<double>() <= 3.5)
		{
			return true;
		}
	}
}

double noteManager::GetFilteredValue(double latestValue, std::deque<double> &previousValues, double previousAverage)
{
	double average = 0.0;

	double previousTotal = previousAverage * previousValues.size();
	previousTotal -= previousValues.back();

	previousValues.push_front(latestValue);
	previousValues.pop_back();

	previousTotal += latestValue;

	average = previousTotal / previousValues.size();

	return average;
}

bool noteManager::HasNote() const
{
	auto currentState = GetCurrentState();
	if (GetCurrentState() == noteManager::STATE_NAMES::STATE_READY)
	{
		return false;
	}
	else if (currentState == noteManager::STATE_NAMES::STATE_FEEDER_INTAKE || currentState == noteManager::STATE_NAMES::STATE_PLACER_INTAKE)
	{
		return (getbackIntakeSensor()->Get() || getfrontIntakeSensor()->Get());
	}
	return true;
}
