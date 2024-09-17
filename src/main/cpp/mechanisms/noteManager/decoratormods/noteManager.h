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
// This file was automatically generated by the Team 302 code generator version 1.3.0.10
// Generated on Tuesday, February 27, 2024 6:47:29 PM

#pragma once

// C++ Includes
#include <string>
#include <deque>
#include <tuple>

// FRC Includes
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/length.h"
#include "frc/controller/PIDController.h"

// Team 302 includes
#include "mechanisms/noteManager/generated/noteManagerGen.h"
#include "mechanisms/base/StateMgr.h"
#include "robotstate/IRobotStateChangeSubscriber.h"
#include "robotstate/RobotStateChanges.h"
#include "robotstate/RobotState.h"

#include "chassis/DragonDriveTargetFinder.h"
// forward declares

class noteManager : public noteManagerGen,
					public IRobotStateChangeSubscriber,
					public DragonDataLogger
{
public:
	// todo delete this once Joe creates and tests the functionality
	enum FINDER_OPTION
	{
		VISION_ONLY,
		ODOMETRY_ONLY,
		FUSE_IF_POSSIBLE
	};

	/// @brief  This method constructs the mechanism using composition with its various actuators and sensors.
	/// @param controlFileName The control file with the PID constants and Targets for each state
	/// @param networkTableName Location for logging information
	/// @param motor  Motor in the mechanims - code generator should probably use the usage for the variable name
	/// @param otherMotor Same as previous
	/// @param solenoid Solenoid in the mechanism - code generator should probably use the usage for the variable name
	/// Additional actuators and sensors are also in this list.
	noteManager(noteManagerGen *generatedMech, RobotConfigMgr::RobotIdentifier activeRobotId);
	noteManager() = delete;
	~noteManager() = default;

	void RunCommonTasks() override;
	void SetCurrentState(int state, bool run) override;
	void CreateAndRegisterStates();

	RobotConfigMgr::RobotIdentifier getActiveRobotId() { return m_activeRobotId; }
	void ResetElevator();
	void ResetLauncherAngle();
	bool HasVisionTarget();

	bool IsLauncherMode() const { return m_scoringMode == RobotStateChanges::ScoringMode::Launcher; }
	bool IsPlacerMode() const { return m_scoringMode == RobotStateChanges::ScoringMode::Placer; }
	bool IsClimbMode() const { return m_climbMode == RobotStateChanges::ClimbMode::ClimbModeOn; }
	bool IsEnabled() const { return m_gamePeriod != RobotStateChanges::GamePeriod::Disabled; }

	void Update(RobotStateChanges::StateChange change, int value) override;
	bool HasNote() const;

	void SetLauncherTargetsForAutoLaunch(DragonDriveTargetFinder::FINDER_OPTION option);
	void MaintainCurrentLauncherTargetsForAutoLaunch();
	bool LauncherTargetsForAutoLaunchAchieved();
	bool isLauncherAtTarget();
	units::angular_velocity::radians_per_second_t getlauncherTargetSpeed();

	units::angle::degree_t GetLauncherAngleTarget() const { return m_LauncherAngleTarget; }
	units::angle::degree_t GetLauncherAngleFromEncoder() { return getlauncherAngleEncoder()->GetAbsolutePosition(); }

	units::angular_velocity::radians_per_second_t GetLauncherTopWheelsTarget() const { return m_LauncherTopWheelsTarget; }
	units::angular_velocity::radians_per_second_t GetLauncherBottomWheelsTarget() const { return m_LauncherBottomWheelsTarget; }

	void SetManualLaunchTarget();
	units::angle::degree_t GetManualLaunchTarget() { return m_manualLaunchTarget; }

	void SetLauncherToProtectedPosition();

	void SetLauncherAngleTarget(units::angle::degree_t valueDeg) { m_LauncherAngleTarget = valueDeg; }
	void SetLauncherTopWheelsTarget(units::angular_velocity::radians_per_second_t valueRadPerSec) { m_LauncherTopWheelsTarget = valueRadPerSec; }
	void SetLauncherBottomWheelsTarget(units::angular_velocity::radians_per_second_t valueRadPerSec) { m_LauncherBottomWheelsTarget = valueRadPerSec; }
	bool GetTransitionFromHoldFeedToReady() { return m_TransitionFromHoldFeedToReady; }
	void SetTransitionFromHoldFeedToReady(bool state) { m_TransitionFromHoldFeedToReady = state; }

	units::length::meter_t GetDistanceFromSpeaker(DragonDriveTargetFinder::FINDER_OPTION option) const;
	void DataLog() override;

private:
	std::tuple<units::angular_velocity::radians_per_second_t, units::angular_velocity::radians_per_second_t, units::angle::degree_t> GetRequiredLaunchParameters(DragonDriveTargetFinder::FINDER_OPTION option);
	void UpdateLauncherAngleTarget();

	double GetFilteredValue(double latestValue, std::deque<double> &previousValues, double previousAverage);

	noteManagerGen *m_noteManager;
	RobotStateChanges::ScoringMode m_scoringMode;
	RobotStateChanges::ClimbMode m_climbMode;
	RobotStateChanges::GamePeriod m_gamePeriod;
	RobotState *m_robotState;

	double m_frontIntakeAverage;
	double m_backIntakeAverage;
	double m_transferAverage;
	double m_placerAverage;
	double m_feederAverage;

	void MonitorMotorCurrents();

	double MonitorForNoteInIntakes();
	bool m_noteInIntake = false;
	bool m_noteInFeeder = false;

	units::angle::degree_t m_manualLaunchTarget = units::angle::degree_t(50.0);
	units::angle::degree_t m_LauncherAngleTarget = units::angle::degree_t(0.0);
	units::angle::degree_t m_autoLaunchTarget = units::angle::degree_t(50.0);
	units::length::meter_t m_transitionMeters = units::length::meter_t(1.5);
	units::angular_velocity::radians_per_second_t m_topLaunchSpeed = units::angular_velocity::radians_per_second_t(450.0);
	units::angular_velocity::radians_per_second_t m_bottomLaunchSpeed = units::angular_velocity::radians_per_second_t(450.0);
	units::angular_velocity::radians_per_second_t m_LauncherTopWheelsTarget;
	units::angular_velocity::radians_per_second_t m_LauncherBottomWheelsTarget;
	const double m_similarDistToleranceMeters = 0.5;
	bool m_TransitionFromHoldFeedToReady = false;
	bool m_manualTargetChangeAllowed = true;
	units::angle::degree_t m_angleTolerance = units::angle::degree_t(0.5);
	units::angular_velocity::radians_per_second_t m_manualLaunchingSpeed = units::angular_velocity::radians_per_second_t(450);
	units::angular_velocity::radians_per_second_t m_autoLaunchingSpeed = units::angular_velocity::radians_per_second_t(620);

	// auto launch function parameters
	const double m_autoLaunchCalcYOffset = 103;
	const double m_autoLaunchCalcFirstDegree = -55.8;
	const double m_autoLaunchCalcSecondDegree = 15.1;
	const double m_autoLaunchCalcThirdDegree = -1.98;
	const double m_autoLaunchCalcFourthDegree = 0.109;
	const double m_lowAnglePIDThreshold = 10.0;
	const double m_rollOverAngle = 350.0;

	frc::PIDController m_launcherAnglePID = frc::PIDController(0.0275, 0.000035, 0.0);
};
