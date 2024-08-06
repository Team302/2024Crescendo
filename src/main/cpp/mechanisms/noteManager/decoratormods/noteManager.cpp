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
#include "mechanisms/noteManager/decoratormods/holdFeederState.h"
#include "mechanisms/noteManager/decoratormods/readyAutoLaunchState.h"
#include "mechanisms/noteManager/decoratormods/readyManualLaunchState.h"
#include "mechanisms/noteManager/decoratormods/PassState.h"
#include "mechanisms/noteManager/decoratormods/autoLaunchState.h"
#include "mechanisms/noteManager/decoratormods/manualLaunchState.h"
#include "mechanisms/noteManager/decoratormods/readyOdometryLaunchState.h"
#include "mechanisms/noteManager/decoratormods/preparePlaceAmpState.h"
#include "mechanisms/noteManager/decoratormods/preparePlaceTrapState.h"
#include "mechanisms/noteManager/decoratormods/placeAmpState.h"
#include "mechanisms/noteManager/decoratormods/placeTrapState.h"
#include "mechanisms/noteManager/decoratormods/holdPlacerState.h"
#include "mechanisms/noteManager/decoratormods/readyPassState.h"
#include "teleopcontrol/TeleopControl.h"

#include "DragonVision/DragonVision.h"
#include "robotstate/RobotState.h"
#include "utils/logging/Logger.h"
#include "utils/logging/DataTrace.h"
#include "utils/FMSData.h"
#include "chassis/ChassisConfig.h"
#include "chassis/ChassisConfigMgr.h"
#include "frc/kinematics/ChassisSpeeds.h"

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

	SetLauncherTopWheelsTarget(units::angular_velocity::radians_per_second_t(0));
	SetLauncherBottomWheelsTarget(units::angular_velocity::radians_per_second_t(0));
	SetLauncherAngleTarget(units::angle::degree_t(0));

	m_robotState = RobotState::GetInstance();

	m_robotState->RegisterForStateChanges(this, RobotStateChanges::StateChange::DesiredScoringMode);
	m_robotState->RegisterForStateChanges(this, RobotStateChanges::StateChange::ClimbModeStatus);
	m_robotState->RegisterForStateChanges(this, RobotStateChanges::StateChange::GameState);

	m_launcherAnglePID.SetIZone(1.5);
	m_launcherAnglePIDLowAngle.SetIZone(1.0);
	m_launcherAnglePID.EnableContinuousInput(0.0, 360.0); // Enables continuous input on a range from 0 to 360, allows the CANCoder to roll over)
	m_launcherAnglePIDLowAngle.EnableContinuousInput(0.0, 360.0);
}

void noteManager::RunCommonTasks()
{
	// This function is called once per loop before the current state Run()
	Cyclic();
	ResetLauncherAngle();
	ResetElevator();
	SetManualLaunchTarget();

	auto currentState = static_cast<noteManagerGen::STATE_NAMES>(GetCurrentState());

	bool protectLauncher = !((currentState == m_noteManager->STATE_READY_AUTO_LAUNCH) ||
							 (currentState == m_noteManager->STATE_READY_ODOMETRY_LAUNCH) ||
							 (currentState == m_noteManager->STATE_AUTO_LAUNCH) ||
							 (currentState == m_noteManager->STATE_PASS) ||
							 (currentState == m_noteManager->STATE_READY_PASS) ||
							 (currentState == m_noteManager->STATE_READY_MANUAL_LAUNCH) ||
							 (currentState == m_noteManager->STATE_MANUAL_LAUNCH));

	if (protectLauncher)
		SetLauncherToProtectedPosition();

	// Processing related to current monitor

	Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Launcher"), string("Angle"), GetLauncherAngleFromEncoder().value());
	Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Launcher"), string("Target"), m_LauncherAngleTarget.value());
	Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Launcher"), string("Top Speed"), units::angular_velocity::radians_per_second_t(units::angular_velocity::revolutions_per_minute_t(getlauncherTop()->GetRPS() * 60.0)).value());
	Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Launcher"), string("Bottom Speed"), units::angular_velocity::radians_per_second_t(units::angular_velocity::revolutions_per_minute_t(getlauncherBottom()->GetRPS() * 60.0)).value());
	// MonitorMotorCurrents();
	UpdateLauncherAngleTarget();

#ifdef INCLUDE_DATA_TRACE
	double intakeDifferentialCurrent = MonitorForNoteInIntakes();

	// double wheelSetTop = units::angular_velocity::radians_per_second_t(units::angular_velocity::revolutions_per_minute_t(getlauncherTop()->GetRPS() * 60)).to<double>();
	double wheelSetTop = GetLauncherAngleTarget().to<double>();

	double wheelSetBottom = units::angular_velocity::radians_per_second_t(units::angular_velocity::revolutions_per_minute_t(getlauncherBottom()->GetRPS() * 60)).to<double>();
	double angle = getlauncherAngle()->GetCounts();
	double launcherTopCurrent = getlauncherTop()->GetCurrent();
	double launcherBottomCurrent = getlauncherBottom()->GetCurrent();
	int theCurrentState = GetCurrentState();
	int XButton = 0; // TeleopControl::GetInstance()->IsButtonPressed(TeleopControlFunctions::MANUAL_MODE) ? 30 : 0;
	DataTrace::GetInstance()->sendLauncherData(wheelSetTop, wheelSetBottom, angle, launcherTopCurrent, launcherBottomCurrent, theCurrentState, XButton);

	double elevator = getElevator()->GetCounts();
	double iState = getElevator()->GetIState();

	DataTrace::GetInstance()->sendElevatorData(elevator, iState);

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

bool noteManager::HasVisionTarget()
{
	auto finder = DragonDriveTargetFinder::GetInstance();
	if (finder != nullptr)
	{
		auto distinfo = finder->GetDistance(DragonDriveTargetFinder::FINDER_OPTION::VISION_ONLY, DragonVision::VISION_ELEMENT::SPEAKER);
		auto type = get<0>(distinfo);
		if (type == DragonDriveTargetFinder::TARGET_INFO::VISION_BASED && get<1>(distinfo) < units::length::meter_t(5.0))
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

void noteManager::SetLauncherTargetsForAutoLaunch(DragonDriveTargetFinder::FINDER_OPTION option)
{
	std::tuple<units::angular_velocity::radians_per_second_t, units::angular_velocity::radians_per_second_t, units::angle::degree_t> launchParameters = GetRequiredLaunchParameters(option);

	Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Launcher"), string("Top Speed Target"), std::get<0>(launchParameters).value());
	Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Launcher"), string("Bottom Speed Target"), std::get<1>(launchParameters).value());

	SetLauncherTopWheelsTarget(std::get<0>(launchParameters));
	SetLauncherBottomWheelsTarget(std::get<1>(launchParameters));
	SetLauncherAngleTarget(std::get<2>(launchParameters));

	UpdateTarget(RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_TOP, units::angular_velocity::revolutions_per_minute_t(GetLauncherTopWheelsTarget()));
	UpdateTarget(RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_BOTTOM, units::angular_velocity::revolutions_per_minute_t(GetLauncherBottomWheelsTarget()));
}

void noteManager::MaintainCurrentLauncherTargetsForAutoLaunch()
{
	UpdateTarget(RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_TOP, units::angular_velocity::revolutions_per_minute_t(GetLauncherTopWheelsTarget()));
	UpdateTarget(RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_BOTTOM, units::angular_velocity::revolutions_per_minute_t(GetLauncherBottomWheelsTarget()));
}

bool noteManager::LauncherTargetsForAutoLaunchAchieved()
{
	units::angular_velocity::revolutions_per_minute_t topSpeed = units::angular_velocity::revolutions_per_minute_t(getlauncherTop()->GetRPS() * 60); // RPS is revs/sec not rad/sec
	units::angular_velocity::revolutions_per_minute_t botSpeed = units::angular_velocity::revolutions_per_minute_t(getlauncherBottom()->GetRPS() * 60);
	units::angle::degree_t launcherAngle = GetLauncherAngleFromEncoder();

	bool wheelTargetSpeedAchieved = (topSpeed > (GetLauncherTopWheelsTarget() * 0.9)) && (botSpeed > (GetLauncherBottomWheelsTarget() * 0.9));
	bool launcherTargetAngleAchieved = std::abs((launcherAngle - GetLauncherAngleTarget()).to<double>()) <= 0.5;

	return launcherTargetAngleAchieved && wheelTargetSpeedAchieved;
}

/// @brief
/// @return top wheel speed, bottom wheel speed, launcher angle
std::tuple<units::angular_velocity::radians_per_second_t, units::angular_velocity::radians_per_second_t, units::angle::degree_t> noteManager::GetRequiredLaunchParameters(DragonDriveTargetFinder::FINDER_OPTION option)
{
	units::length::meter_t distanceFromTarget_m = GetDistanceFromSpeaker(option);

	// The following values are the coefficients for the polynomial that determines the launcher angle based on distance from the target
	m_autoLaunchTarget = units::angle::degree_t(m_autoLaunchCalcYOffset +
												m_autoLaunchCalcFirstDegree * distanceFromTarget_m.value() +
												m_autoLaunchCalcSecondDegree * units::math::pow<2>(distanceFromTarget_m).value() +
												m_autoLaunchCalcThirdDegree * units::math::pow<3>(distanceFromTarget_m).value() +
												m_autoLaunchCalcFourthDegree * units::math::pow<4>(distanceFromTarget_m).value());

	// limit the resulting launcher angle
	m_autoLaunchTarget = m_autoLaunchTarget > units::angle::degree_t(55.0) ? units::angle::degree_t(55.0) : m_autoLaunchTarget;
	m_autoLaunchTarget = m_autoLaunchTarget < units::angle::degree_t(14.5) ? units::angle::degree_t(14.5) : m_autoLaunchTarget;

	m_topLaunchSpeed = distanceFromTarget_m < m_transitionMeters ? m_manualLaunchingSpeed : m_autoLaunchingSpeed;
	m_bottomLaunchSpeed = distanceFromTarget_m < m_transitionMeters ? m_manualLaunchingSpeed : m_autoLaunchingSpeed;

	// keep for tuning purposes
	// if (abs(TeleopControl::GetInstance()->GetAxisValue(TeleopControlFunctions::LAUNCH_ANGLE)) > 0.05) // Allows manual cotrol of the elevator if you need to adujst
	//{
	//	units::angle::degree_t delta = units::angle::degree_t(6.0 * 0.1 * (TeleopControl::GetInstance()->GetAxisValue(TeleopControlFunctions::LAUNCH_ANGLE))); // changing by 6 in/s * 0.05 for 20 ms loop time * controller input
	//	m_autoLaunchTarget += delta;
	//	if (m_autoLaunchTarget > units::angle::degree_t(55.0)) // limiting the travel to 0 through 16.5
	//		m_autoLaunchTarget = units::angle::degree_t(55.0);
	//}
	return make_tuple(m_topLaunchSpeed, m_bottomLaunchSpeed, m_autoLaunchTarget);
}

units::length::meter_t noteManager::GetDistanceFromSpeaker(DragonDriveTargetFinder::FINDER_OPTION option) const
{
	auto distanceFromTarget = units::length::meter_t(6.0);
	auto finder = DragonDriveTargetFinder::GetInstance();
	if (finder != nullptr)
	{
		auto distinfo = finder->GetDistance(option, DragonVision::VISION_ELEMENT::SPEAKER);
		auto type = get<0>(distinfo);
		if (type != DragonDriveTargetFinder::TARGET_INFO::NOT_FOUND)
		{
			auto visionDist = get<1>(distinfo);
			distanceFromTarget = visionDist;
		}
	}
	Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Launcher"), string("Distance"), distanceFromTarget.to<double>());

	return distanceFromTarget;
}

void noteManager::SetManualLaunchTarget()
{

	if (TeleopControl::GetInstance()->IsButtonPressed(TeleopControlFunctions::MANUAL_LAUNCH_INC))
	{
		if (m_manualTargetChangeAllowed)
		{
			m_manualLaunchTarget++;
			m_manualTargetChangeAllowed = false;
		}
	}
	else if (TeleopControl::GetInstance()->IsButtonPressed(TeleopControlFunctions::MANUAL_LAUNCH_DEC))
	{
		if (m_manualTargetChangeAllowed)
		{
			m_manualLaunchTarget--;
			m_manualTargetChangeAllowed = false;
		}
	}
	else
	{
		m_manualTargetChangeAllowed = true;
	}
}

void noteManager::UpdateLauncherAngleTarget()
{
	// if (GetLauncherAngleTarget().to<double>() < m_lowAnglePIDThreshold)
	// {
	// 	double percentOut = std::clamp(m_launcherAnglePIDLowAngle.Calculate(GetLauncherAngleFromEncoder().to<double>(), GetLauncherAngleTarget().to<double>()), -1.0, 1.0);
	// 	UpdateTarget(RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_ANGLE, percentOut);
	// }
	// else
	// {
	double percentOut = std::clamp(m_launcherAnglePID.Calculate(GetLauncherAngleFromEncoder().to<double>(), GetLauncherAngleTarget().to<double>()), -1.0, 1.0);
	UpdateTarget(RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_ANGLE, percentOut);
	// }
}

void noteManager::SetLauncherToProtectedPosition()
{
	SetLauncherAngleTarget(units::angle::degree_t(0.0));
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

bool noteManager::isLauncherAtTargert()
{
	return units::math::abs((m_LauncherAngleTarget - GetLauncherAngleFromEncoder())) < m_angleTolerance;
}

units::angular_velocity::radians_per_second_t noteManager::getlauncherTargetSpeed()
{

	units::angular_velocity::radians_per_second_t targetSpeed = (GetLauncherAngleFromEncoder() > units::angle::degree_t(10.0)) ? units::angular_velocity::radians_per_second_t(400.0) : units::angular_velocity::radians_per_second_t(500.0); // rad/sec based on passing with no chassis speed
	/*auto chassisConfig = ChassisConfigMgr::GetInstance()->GetCurrentConfig();

	if (chassisConfig != nullptr)
	{
		auto chassis = chassisConfig->GetSwerveChassis();
		auto chassisSpeeds = chassis->GetChassisSpeeds();
		auto rot2d = frc::Rotation2d(chassis->GetYaw());

		auto fieldSpeeds = frc::ChassisSpeeds::FromRobotRelativeSpeeds(chassisSpeeds, rot2d); // don't know if you need to this?

		units::velocity::meters_per_second_t vTan = units::velocity::meters_per_second_t(targetSpeed.value() * 0.0508); // 2 in radius of wheel in meters
		units::velocity::meters_per_second_t vTanX = (FMSData::GetInstance()->GetAllianceColor() == frc::DriverStation::kBlue) ? (vTan * units::math::cos(GetLauncherAngleFromEncoder()) + fieldSpeeds.vx / units::velocity::meters_per_second_t(2.0)) : (vTan * units::math::cos(GetLauncherAngleFromEncoder()) - fieldSpeeds.vx / units::velocity::meters_per_second_t(2.0));

		targetSpeed = units::angular_velocity::radians_per_second_t((vTanX / units::math::cos(GetLauncherAngleFromEncoder())).value() / 0.0508); // 2 in radius of wheel in meters
	}*/

	return targetSpeed;
}