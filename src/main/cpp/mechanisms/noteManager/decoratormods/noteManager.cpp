
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
// Generated on Monday, February 5, 2024 10:04:45 PM

// C++ Includes
#include <string>

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
#include "mechanisms/noteManager/decoratormods/holdFeederFrontState.h"
#include "mechanisms/noteManager/decoratormods/holdFeederBackState.h"
#include "mechanisms/noteManager/decoratormods/intakeToFeederState.h"
#include "mechanisms/noteManager/decoratormods/launcherToPlacerFrontState.h"
#include "mechanisms/noteManager/decoratormods/launcherToPlacerBackState.h"
#include "mechanisms/noteManager/decoratormods/holdFeederState.h"
#include "mechanisms/noteManager/decoratormods/readyAutoLaunchState.h"
#include "mechanisms/noteManager/decoratormods/readyManualLaunchState.h"
#include "mechanisms/noteManager/decoratormods/PassState.h"
#include "mechanisms/noteManager/decoratormods/autoLaunchState.h"
#include "mechanisms/noteManager/decoratormods/manualLaunchState.h"
#include "mechanisms/noteManager/decoratormods/readyOdometryLaunchState.h"
#include "mechanisms/noteManager/decoratormods/autoLaunchOdometryState.h"
#include "mechanisms/noteManager/decoratormods/holdPlacerFrontState.h"
#include "mechanisms/noteManager/decoratormods/holdPlacerBackState.h"
#include "mechanisms/noteManager/decoratormods/intakeToPlacerState.h"
#include "mechanisms/noteManager/decoratormods/preparePlaceAmpState.h"
#include "mechanisms/noteManager/decoratormods/preparePlaceTrapState.h"
#include "mechanisms/noteManager/decoratormods/placeAmpState.h"
#include "mechanisms/noteManager/decoratormods/placeTrapState.h"
#include "mechanisms/noteManager/decoratormods/placerToLauncherFrontState.h"
#include "mechanisms/noteManager/decoratormods/placerToLauncherBackState.h"
#include "mechanisms/noteManager/decoratormods/backupManualLaunchState.h"
#include "mechanisms/noteManager/decoratormods/backupManualPlaceState.h"

#include "robotstate/RobotState.h"

using std::string;
using namespace noteManagerStates;

/// @brief  This method constructs the mechanism using composition with its various actuators and sensors.
/// @param controlFileName The control file with the PID constants and Targets for each state
/// @param networkTableName Location for logging information
/// @param motor  Motor in the mechanims - code generator should probably use the usage for the variable name
/// @param otherMotor Same as previous
/// @param solenoid Solenoid in the mechanism - code generator should probably use the usage for the variable name
/// Additional actuators and sensors are also in this list.
noteManager::noteManager(noteManagerGen *base) : noteManagerGen(), IRobotStateChangeSubscriber(),
												 m_noteManager(base)
{
	PeriodicLooper::GetInstance()->RegisterAll(this);

	m_scoringMode = RobotStateChanges::ScoringMode::Launcher;
	m_climbMode = RobotStateChanges::ClimbMode::ClimbModeOff;
	m_gamePeriod = RobotStateChanges::GamePeriod::Disabled;

	RobotState *RobotStates = RobotState::GetInstance();

	RobotStates->RegisterForStateChanges(this, RobotStateChanges::StateChange::DesiredScoringMode);
	RobotStates->RegisterForStateChanges(this, RobotStateChanges::StateChange::ClimbModeStatus);
	RobotStates->RegisterForStateChanges(this, RobotStateChanges::StateChange::GameState);
}

void noteManager::RunCommonTasks()
{
	// This function is called once per loop before the current state Run()
	Cyclic();
}

void noteManager::SetCurrentState(int state, bool run)
{
	noteManagerGen::SetCurrentState(state, run);
}

void noteManager::CreateAndRegisterStates()
{
	OffState *OffStateInst = new OffState(string("Off"), 0, new noteManagerAllStatesStateGen(string("Off"), 0, this), this);
	AddToStateVector(OffStateInst);

	ReadyState *ReadyStateInst = new ReadyState(string("Ready"), 1, new noteManagerAllStatesStateGen(string("Ready"), 1, this), this);
	AddToStateVector(ReadyStateInst);

	feederIntakeState *feederIntakeStateInst = new feederIntakeState(string("feederIntake"), 2, new noteManagerAllStatesStateGen(string("feederIntake"), 2, this), this);
	AddToStateVector(feederIntakeStateInst);

	ExpelState *ExpelStateInst = new ExpelState(string("Expel"), 3, new noteManagerAllStatesStateGen(string("Expel"), 3, this), this);
	AddToStateVector(ExpelStateInst);

	placerIntakeState *placerIntakeStateInst = new placerIntakeState(string("placerIntake"), 4, new noteManagerAllStatesStateGen(string("placerIntake"), 4, this), this);
	AddToStateVector(placerIntakeStateInst);

	holdFeederFrontState *holdFeederFrontStateInst = new holdFeederFrontState(string("holdFeederFront"), 5, new noteManagerAllStatesStateGen(string("holdFeederFront"), 5, this), this);
	AddToStateVector(holdFeederFrontStateInst);

	holdFeederBackState *holdFeederBackStateInst = new holdFeederBackState(string("holdFeederBack"), 6, new noteManagerAllStatesStateGen(string("holdFeederBack"), 6, this), this);
	AddToStateVector(holdFeederBackStateInst);

	intakeToFeederState *intakeToFeederStateInst = new intakeToFeederState(string("intakeToFeeder"), 7, new noteManagerAllStatesStateGen(string("intakeToFeeder"), 7, this), this);
	AddToStateVector(intakeToFeederStateInst);

	launcherToPlacerFrontState *launcherToPlacerFrontStateInst = new launcherToPlacerFrontState(string("launcherToPlacerFront"), 8, new noteManagerAllStatesStateGen(string("launcherToPlacerFront"), 8, this), this);
	AddToStateVector(launcherToPlacerFrontStateInst);

	launcherToPlacerBackState *launcherToPlacerBackStateInst = new launcherToPlacerBackState(string("launcherToPlacerBack"), 9, new noteManagerAllStatesStateGen(string("launcherToPlacerBack"), 9, this), this);
	AddToStateVector(launcherToPlacerBackStateInst);

	holdFeederState *holdFeederStateInst = new holdFeederState(string("holdFeeder"), 10, new noteManagerAllStatesStateGen(string("holdFeeder"), 10, this), this);
	AddToStateVector(holdFeederStateInst);

	readyAutoLaunchState *readyAutoLaunchStateInst = new readyAutoLaunchState(string("readyAutoLaunch"), 11, new noteManagerAllStatesStateGen(string("readyAutoLaunch"), 11, this), this);
	AddToStateVector(readyAutoLaunchStateInst);

	readyManualLaunchState *readyManualLaunchStateInst = new readyManualLaunchState(string("readyManualLaunch"), 12, new noteManagerAllStatesStateGen(string("readyManualLaunch"), 12, this), this);
	AddToStateVector(readyManualLaunchStateInst);

	PassState *PassStateInst = new PassState(string("Pass"), 13, new noteManagerAllStatesStateGen(string("Pass"), 13, this), this);
	AddToStateVector(PassStateInst);

	autoLaunchState *autoLaunchStateInst = new autoLaunchState(string("autoLaunch"), 14, new noteManagerAllStatesStateGen(string("autoLaunch"), 14, this), this);
	AddToStateVector(autoLaunchStateInst);

	manualLaunchState *manualLaunchStateInst = new manualLaunchState(string("manualLaunch"), 15, new noteManagerAllStatesStateGen(string("manualLaunch"), 15, this), this);
	AddToStateVector(manualLaunchStateInst);

	readyOdometryLaunchState *readyOdometryLaunchStateInst = new readyOdometryLaunchState(string("readyOdometryLaunch"), 16, new noteManagerAllStatesStateGen(string("readyOdometryLaunch"), 16, this), this);
	AddToStateVector(readyOdometryLaunchStateInst);

	autoLaunchOdometryState *autoLaunchOdometryStateInst = new autoLaunchOdometryState(string("autoLaunchOdometry"), 17, new noteManagerAllStatesStateGen(string("autoLaunchOdometry"), 17, this), this);
	AddToStateVector(autoLaunchOdometryStateInst);

	holdPlacerFrontState *holdPlacerFrontStateInst = new holdPlacerFrontState(string("holdPlacerFront"), 18, new noteManagerAllStatesStateGen(string("holdPlacerFront"), 18, this), this);
	AddToStateVector(holdPlacerFrontStateInst);

	holdPlacerBackState *holdPlacerBackStateInst = new holdPlacerBackState(string("holdPlacerBack"), 19, new noteManagerAllStatesStateGen(string("holdPlacerBack"), 19, this), this);
	AddToStateVector(holdPlacerBackStateInst);

	intakeToPlacerState *intakeToPlacerStateInst = new intakeToPlacerState(string("intakeToPlacer"), 20, new noteManagerAllStatesStateGen(string("intakeToPlacer"), 20, this), this);
	AddToStateVector(intakeToPlacerStateInst);

	preparePlaceAmpState *preparePlaceAmpStateInst = new preparePlaceAmpState(string("preparePlaceAmp"), 21, new noteManagerAllStatesStateGen(string("preparePlaceAmp"), 21, this), this);
	AddToStateVector(preparePlaceAmpStateInst);

	preparePlaceTrapState *preparePlaceTrapStateInst = new preparePlaceTrapState(string("preparePlaceTrap"), 22, new noteManagerAllStatesStateGen(string("preparePlaceTrap"), 22, this), this);
	AddToStateVector(preparePlaceTrapStateInst);

	placeAmpState *placeAmpStateInst = new placeAmpState(string("placeAmp"), 23, new noteManagerAllStatesStateGen(string("placeAmp"), 23, this), this);
	AddToStateVector(placeAmpStateInst);

	placeTrapState *placeTrapStateInst = new placeTrapState(string("placeTrap"), 24, new noteManagerAllStatesStateGen(string("placeTrap"), 24, this), this);
	AddToStateVector(placeTrapStateInst);

	placerToLauncherFrontState *placerToLauncherFrontStateInst = new placerToLauncherFrontState(string("placerToLauncherFront"), 25, new noteManagerAllStatesStateGen(string("placerToLauncherFront"), 25, this), this);
	AddToStateVector(placerToLauncherFrontStateInst);

	placerToLauncherBackState *placerToLauncherBackStateInst = new placerToLauncherBackState(string("placerToLauncherBack"), 26, new noteManagerAllStatesStateGen(string("placerToLauncherBack"), 26, this), this);
	AddToStateVector(placerToLauncherBackStateInst);

	backupManualLaunchState *backupManualLaunchStateInst = new backupManualLaunchState(string("backupManualLaunch"), 27, new noteManagerAllStatesStateGen(string("backupManualLaunch"), 27, this), this);
	AddToStateVector(backupManualLaunchStateInst);

	backupManualPlaceState *backupManualPlaceStateInst = new backupManualPlaceState(string("backupManualPlace"), 28, new noteManagerAllStatesStateGen(string("backupManualPlace"), 28, this), this);
	AddToStateVector(backupManualPlaceStateInst);

	OffStateInst->RegisterTransitionState(ReadyStateInst);
	ReadyStateInst->RegisterTransitionState(feederIntakeStateInst);
	ReadyStateInst->RegisterTransitionState(ExpelStateInst);
	ReadyStateInst->RegisterTransitionState(placerIntakeStateInst);
	ReadyStateInst->RegisterTransitionState(backupManualLaunchStateInst);
	ReadyStateInst->RegisterTransitionState(backupManualPlaceStateInst);
	feederIntakeStateInst->RegisterTransitionState(ReadyStateInst);
	feederIntakeStateInst->RegisterTransitionState(holdFeederFrontStateInst);
	feederIntakeStateInst->RegisterTransitionState(holdFeederBackStateInst);
	feederIntakeStateInst->RegisterTransitionState(ExpelStateInst);
	ExpelStateInst->RegisterTransitionState(ReadyStateInst);
	ExpelStateInst->RegisterTransitionState(feederIntakeStateInst);
	ExpelStateInst->RegisterTransitionState(placerIntakeStateInst);
	placerIntakeStateInst->RegisterTransitionState(ReadyStateInst);
	placerIntakeStateInst->RegisterTransitionState(holdPlacerFrontStateInst);
	placerIntakeStateInst->RegisterTransitionState(holdPlacerBackStateInst);
	holdFeederFrontStateInst->RegisterTransitionState(ReadyStateInst);
	holdFeederFrontStateInst->RegisterTransitionState(launcherToPlacerFrontStateInst);
	holdFeederFrontStateInst->RegisterTransitionState(intakeToFeederStateInst);
	holdFeederBackStateInst->RegisterTransitionState(ReadyStateInst);
	holdFeederBackStateInst->RegisterTransitionState(intakeToFeederStateInst);
	holdFeederBackStateInst->RegisterTransitionState(launcherToPlacerBackStateInst);
	intakeToFeederStateInst->RegisterTransitionState(ReadyStateInst);
	intakeToFeederStateInst->RegisterTransitionState(holdFeederStateInst);
	launcherToPlacerFrontStateInst->RegisterTransitionState(ReadyStateInst);
	launcherToPlacerFrontStateInst->RegisterTransitionState(placerIntakeStateInst);
	launcherToPlacerBackStateInst->RegisterTransitionState(ReadyStateInst);
	launcherToPlacerBackStateInst->RegisterTransitionState(holdPlacerBackStateInst);
	holdFeederStateInst->RegisterTransitionState(ReadyStateInst);
	holdFeederStateInst->RegisterTransitionState(readyAutoLaunchStateInst);
	holdFeederStateInst->RegisterTransitionState(readyManualLaunchStateInst);
	holdFeederStateInst->RegisterTransitionState(PassStateInst);
	readyAutoLaunchStateInst->RegisterTransitionState(ReadyStateInst);
	readyAutoLaunchStateInst->RegisterTransitionState(holdFeederStateInst);
	readyAutoLaunchStateInst->RegisterTransitionState(autoLaunchStateInst);
	readyManualLaunchStateInst->RegisterTransitionState(ReadyStateInst);
	readyManualLaunchStateInst->RegisterTransitionState(manualLaunchStateInst);
	PassStateInst->RegisterTransitionState(ReadyStateInst);
	autoLaunchStateInst->RegisterTransitionState(ReadyStateInst);
	manualLaunchStateInst->RegisterTransitionState(ReadyStateInst);
	readyOdometryLaunchStateInst->RegisterTransitionState(ReadyStateInst);
	readyOdometryLaunchStateInst->RegisterTransitionState(readyAutoLaunchStateInst);
	readyOdometryLaunchStateInst->RegisterTransitionState(autoLaunchOdometryStateInst);
	autoLaunchOdometryStateInst->RegisterTransitionState(ReadyStateInst);
	holdPlacerFrontStateInst->RegisterTransitionState(ReadyStateInst);
	holdPlacerFrontStateInst->RegisterTransitionState(intakeToPlacerStateInst);
	holdPlacerFrontStateInst->RegisterTransitionState(placerToLauncherFrontStateInst);
	holdPlacerBackStateInst->RegisterTransitionState(ReadyStateInst);
	holdPlacerBackStateInst->RegisterTransitionState(placerToLauncherBackStateInst);
	holdPlacerBackStateInst->RegisterTransitionState(intakeToPlacerStateInst);
	intakeToPlacerStateInst->RegisterTransitionState(ReadyStateInst);
	intakeToPlacerStateInst->RegisterTransitionState(preparePlaceAmpStateInst);
	intakeToPlacerStateInst->RegisterTransitionState(preparePlaceTrapStateInst);
	preparePlaceAmpStateInst->RegisterTransitionState(ReadyStateInst);
	preparePlaceAmpStateInst->RegisterTransitionState(preparePlaceTrapStateInst);
	preparePlaceAmpStateInst->RegisterTransitionState(placeAmpStateInst);
	preparePlaceTrapStateInst->RegisterTransitionState(ReadyStateInst);
	preparePlaceTrapStateInst->RegisterTransitionState(preparePlaceAmpStateInst);
	preparePlaceTrapStateInst->RegisterTransitionState(placeTrapStateInst);
	placeAmpStateInst->RegisterTransitionState(ReadyStateInst);
	placeTrapStateInst->RegisterTransitionState(ReadyStateInst);
	placerToLauncherFrontStateInst->RegisterTransitionState(ReadyStateInst);
	placerToLauncherFrontStateInst->RegisterTransitionState(feederIntakeStateInst);
	placerToLauncherBackStateInst->RegisterTransitionState(ReadyStateInst);
	placerToLauncherBackStateInst->RegisterTransitionState(holdFeederBackStateInst);
	backupManualLaunchStateInst->RegisterTransitionState(ReadyStateInst);
	backupManualPlaceStateInst->RegisterTransitionState(ReadyStateInst);
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
