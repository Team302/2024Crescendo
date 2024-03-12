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

	// Processing related to current monitor
	MonitorMotorCurrents();
	double intakeDifferentialCurrent = MonitorForNoteInIntakes();

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
	const double incomingThreshold = 10;

	// If the differential current is below forwardingThreshold, assume that the note has
	// been forwarded to the next noteManagement stage
	const double forwardingThreshold = 10;

	m_frontIntakeAverage = getfrontIntake()->GetCurrent();
	m_backIntakeAverage = getbackIntake()->GetCurrent();
	double intakeDifferenceAvg = std::abs(m_frontIntakeAverage - m_backIntakeAverage);

	if (intakeDifferenceAvg > incomingThreshold)
		m_noteInIntake = true;

	if (m_noteInIntake && (intakeDifferenceAvg < forwardingThreshold))
		m_noteInIntake = false;

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
	return distance;
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
	double distanceFromTarget = 3.5;
	double launchAngle = 0;

	if (HasVisionTarget())
	{
		distanceFromTarget = GetVisionDistance().to<double>();

		launchAngle = 80.0 + (-44.2 * distanceFromTarget) + (6.09 * distanceFromTarget * distanceFromTarget);
	}
	if (launchAngle > 40)
	{
		launchAngle = 0;
	}
	return launchAngle;
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
