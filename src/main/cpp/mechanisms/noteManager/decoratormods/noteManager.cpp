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
// This file was automatically generated by the Team 302 code generator version 1.2.1.0
// Generated on Sunday, January 28, 2024 10:39:19 AM

// C++ Includes
#include <string>

// FRC Includes

// Team 302 includes
#include "PeriodicLooper.h"
#include "mechanisms/noteManager/generated/noteManager_gen.h"
#include "mechanisms/noteManager/decoratormods/noteManager.h"

#include "hw/DragonSparkMax.h"
#include "hw/DragonSparkMax.h"
#include "hw/DragonSparkMax.h"
#include "hw/DragonTalonFX.h"
#include "hw/DragonSparkFlex.h"
#include "hw/DragonSparkFlex.h"
#include "hw/DragonTalonFX.h"
#include "hw/DragonTalonFX.h"
#include "hw/DragonTalonSRX.h"

#include "hw/DragonDigitalInput.h"
#include "hw/DragonDigitalInput.h"
#include "hw/DragonDigitalInput.h"
#include "hw/DragonDigitalInput.h"
#include "hw/DragonDigitalInput.h"
#include "hw/DragonDigitalInput.h"
#include "hw/DragonDigitalInput.h"

#include "hw/DragonCanCoder.h"

#include "mechanisms/noteManager/decoratormods/noteManager_Off_State.h"
#include "mechanisms/noteManager/decoratormods/noteManager_Ready_State.h"
#include "mechanisms/noteManager/decoratormods/noteManager_feederIntake_State.h"
#include "mechanisms/noteManager/decoratormods/noteManager_Expel_State.h"
#include "mechanisms/noteManager/decoratormods/noteManager_placerIntake_State.h"
#include "mechanisms/noteManager/decoratormods/noteManager_holdFeederFront_State.h"
#include "mechanisms/noteManager/decoratormods/noteManager_holdFeederBack_State.h"
#include "mechanisms/noteManager/decoratormods/noteManager_intakeToFeeder_State.h"
#include "mechanisms/noteManager/decoratormods/noteManager_launcherToPlacerFront_State.h"
#include "mechanisms/noteManager/decoratormods/noteManager_launcherToPlacerBack_State.h"
#include "mechanisms/noteManager/decoratormods/noteManager_holdFeeder_State.h"
#include "mechanisms/noteManager/decoratormods/noteManager_readyAutoLaunch_State.h"
#include "mechanisms/noteManager/decoratormods/noteManager_readyManualLaunch_State.h"
#include "mechanisms/noteManager/decoratormods/noteManager_Pass_State.h"
#include "mechanisms/noteManager/decoratormods/noteManager_autoLaunch_State.h"
#include "mechanisms/noteManager/decoratormods/noteManager_manualLaunch_State.h"
#include "mechanisms/noteManager/decoratormods/noteManager_readyOdometryLaunch_State.h"
#include "mechanisms/noteManager/decoratormods/noteManager_autoLaunchOdometry_State.h"
#include "mechanisms/noteManager/decoratormods/noteManager_holdPlacerFront_State.h"
#include "mechanisms/noteManager/decoratormods/noteManager_holdPlacerBack_State.h"
#include "mechanisms/noteManager/decoratormods/noteManager_intakeToPlacer_State.h"
#include "mechanisms/noteManager/decoratormods/noteManager_preparePlaceAmp_State.h"
#include "mechanisms/noteManager/decoratormods/noteManager_preparePlaceTrap_State.h"
#include "mechanisms/noteManager/decoratormods/noteManager_placeAmp_State.h"
#include "mechanisms/noteManager/decoratormods/noteManager_placeTrap_State.h"
#include "mechanisms/noteManager/decoratormods/noteManager_placerToLauncherFront_State.h"
#include "mechanisms/noteManager/decoratormods/noteManager_placerToLauncherBack_State.h"
#include "mechanisms/noteManager/decoratormods/noteManager_backupManualLaunch_State.h"
#include "mechanisms/noteManager/decoratormods/noteManager_backupManualPlace_State.h"

#include "robotstate/RobotState.h"

using std::string;

/// @brief  This method constructs the mechanism using composition with its various actuators and sensors.
/// @param controlFileName The control file with the PID constants and Targets for each state
/// @param networkTableName Location for logging information
/// @param motor  Motor in the mechanims - code generator should probably use the usage for the variable name
/// @param otherMotor Same as previous
/// @param solenoid Solenoid in the mechanism - code generator should probably use the usage for the variable name
/// Additional actuators and sensors are also in this list.
noteManager::noteManager(noteManager_gen *base) : noteManager_gen(), IRobotStateChangeSubscriber(),
												  m_noteManager(base)
{
	m_scoringMode = RobotStateChanges::ScoringMode::Launcher;
	m_climbMode = RobotStateChanges::ClimbMode::ClimbModeOff;
	RobotState::GetInstance()->RegisterForStateChanges(this, RobotStateChanges::StateChange::DesiredScoringMode);
	RobotState::GetInstance()->RegisterForStateChanges(this, RobotStateChanges::StateChange::ClimbModeStatus);
}

void noteManager::createAndRegisterStates()
{
	noteManagerOffState *OffState = new noteManagerOffState(string("Off"), 0, new noteManagerOffStateGen(string("Off"), 0, this), this);
	AddToStateVector(OffState);

	noteManagerReadyState *ReadyState = new noteManagerReadyState(string("Ready"), 1, new noteManagerReadyStateGen(string("Ready"), 1, this), this);
	AddToStateVector(ReadyState);

	noteManagerfeederIntakeState *feederIntakeState = new noteManagerfeederIntakeState(string("feederIntake"), 2, new noteManagerfeederIntakeStateGen(string("feederIntake"), 2, this), this);
	AddToStateVector(feederIntakeState);

	noteManagerExpelState *ExpelState = new noteManagerExpelState(string("Expel"), 3, new noteManagerExpelStateGen(string("Expel"), 3, this), this);
	AddToStateVector(ExpelState);

	noteManagerplacerIntakeState *placerIntakeState = new noteManagerplacerIntakeState(string("placerIntake"), 4, new noteManagerplacerIntakeStateGen(string("placerIntake"), 4, this), this);
	AddToStateVector(placerIntakeState);

	noteManagerholdFeederFrontState *holdFeederFrontState = new noteManagerholdFeederFrontState(string("holdFeederFront"), 5, new noteManagerholdFeederFrontStateGen(string("holdFeederFront"), 5, this), this);
	AddToStateVector(holdFeederFrontState);

	noteManagerholdFeederBackState *holdFeederBackState = new noteManagerholdFeederBackState(string("holdFeederBack"), 6, new noteManagerholdFeederBackStateGen(string("holdFeederBack"), 6, this), this);
	AddToStateVector(holdFeederBackState);

	noteManagerintakeToFeederState *intakeToFeederState = new noteManagerintakeToFeederState(string("intakeToFeeder"), 7, new noteManagerintakeToFeederStateGen(string("intakeToFeeder"), 7, this), this);
	AddToStateVector(intakeToFeederState);

	noteManagerlauncherToPlacerFrontState *launcherToPlacerFrontState = new noteManagerlauncherToPlacerFrontState(string("launcherToPlacerFront"), 8, new noteManagerlauncherToPlacerFrontStateGen(string("launcherToPlacerFront"), 8, this), this);
	AddToStateVector(launcherToPlacerFrontState);

	noteManagerlauncherToPlacerBackState *launcherToPlacerBackState = new noteManagerlauncherToPlacerBackState(string("launcherToPlacerBack"), 9, new noteManagerlauncherToPlacerBackStateGen(string("launcherToPlacerBack"), 9, this), this);
	AddToStateVector(launcherToPlacerBackState);

	noteManagerholdFeederState *holdFeederState = new noteManagerholdFeederState(string("holdFeeder"), 10, new noteManagerholdFeederStateGen(string("holdFeeder"), 10, this), this);
	AddToStateVector(holdFeederState);

	noteManagerreadyAutoLaunchState *readyAutoLaunchState = new noteManagerreadyAutoLaunchState(string("readyAutoLaunch"), 11, new noteManagerreadyAutoLaunchStateGen(string("readyAutoLaunch"), 11, this), this);
	AddToStateVector(readyAutoLaunchState);

	noteManagerreadyManualLaunchState *readyManualLaunchState = new noteManagerreadyManualLaunchState(string("readyManualLaunch"), 12, new noteManagerreadyManualLaunchStateGen(string("readyManualLaunch"), 12, this), this);
	AddToStateVector(readyManualLaunchState);

	noteManagerPassState *PassState = new noteManagerPassState(string("Pass"), 13, new noteManagerPassStateGen(string("Pass"), 13, this), this);
	AddToStateVector(PassState);

	noteManagerautoLaunchState *autoLaunchState = new noteManagerautoLaunchState(string("autoLaunch"), 14, new noteManagerautoLaunchStateGen(string("autoLaunch"), 14, this), this);
	AddToStateVector(autoLaunchState);

	noteManagermanualLaunchState *manualLaunchState = new noteManagermanualLaunchState(string("manualLaunch"), 15, new noteManagermanualLaunchStateGen(string("manualLaunch"), 15, this), this);
	AddToStateVector(manualLaunchState);

	noteManagerreadyOdometryLaunchState *readyOdometryLaunchState = new noteManagerreadyOdometryLaunchState(string("readyOdometryLaunch"), 16, new noteManagerreadyOdometryLaunchStateGen(string("readyOdometryLaunch"), 16, this), this);
	AddToStateVector(readyOdometryLaunchState);

	noteManagerautoLaunchOdometryState *autoLaunchOdometryState = new noteManagerautoLaunchOdometryState(string("autoLaunchOdometry"), 17, new noteManagerautoLaunchOdometryStateGen(string("autoLaunchOdometry"), 17, this), this);
	AddToStateVector(autoLaunchOdometryState);

	noteManagerholdPlacerFrontState *holdPlacerFrontState = new noteManagerholdPlacerFrontState(string("holdPlacerFront"), 18, new noteManagerholdPlacerFrontStateGen(string("holdPlacerFront"), 18, this), this);
	AddToStateVector(holdPlacerFrontState);

	noteManagerholdPlacerBackState *holdPlacerBackState = new noteManagerholdPlacerBackState(string("holdPlacerBack"), 19, new noteManagerholdPlacerBackStateGen(string("holdPlacerBack"), 19, this), this);
	AddToStateVector(holdPlacerBackState);

	noteManagerintakeToPlacerState *intakeToPlacerState = new noteManagerintakeToPlacerState(string("intakeToPlacer"), 20, new noteManagerintakeToPlacerStateGen(string("intakeToPlacer"), 20, this), this);
	AddToStateVector(intakeToPlacerState);

	noteManagerpreparePlaceAmpState *preparePlaceAmpState = new noteManagerpreparePlaceAmpState(string("preparePlaceAmp"), 21, new noteManagerpreparePlaceAmpStateGen(string("preparePlaceAmp"), 21, this), this);
	AddToStateVector(preparePlaceAmpState);

	noteManagerpreparePlaceTrapState *preparePlaceTrapState = new noteManagerpreparePlaceTrapState(string("preparePlaceTrap"), 22, new noteManagerpreparePlaceTrapStateGen(string("preparePlaceTrap"), 22, this), this);
	AddToStateVector(preparePlaceTrapState);

	noteManagerplaceAmpState *placeAmpState = new noteManagerplaceAmpState(string("placeAmp"), 23, new noteManagerplaceAmpStateGen(string("placeAmp"), 23, this), this);
	AddToStateVector(placeAmpState);

	noteManagerplaceTrapState *placeTrapState = new noteManagerplaceTrapState(string("placeTrap"), 24, new noteManagerplaceTrapStateGen(string("placeTrap"), 24, this), this);
	AddToStateVector(placeTrapState);

	noteManagerplacerToLauncherFrontState *placerToLauncherFrontState = new noteManagerplacerToLauncherFrontState(string("placerToLauncherFront"), 25, new noteManagerplacerToLauncherFrontStateGen(string("placerToLauncherFront"), 25, this), this);
	AddToStateVector(placerToLauncherFrontState);

	noteManagerplacerToLauncherBackState *placerToLauncherBackState = new noteManagerplacerToLauncherBackState(string("placerToLauncherBack"), 26, new noteManagerplacerToLauncherBackStateGen(string("placerToLauncherBack"), 26, this), this);
	AddToStateVector(placerToLauncherBackState);

	noteManagerbackupManualLaunchState *backupManualLaunchState = new noteManagerbackupManualLaunchState(string("backupManualLaunch"), 27, new noteManagerbackupManualLaunchStateGen(string("backupManualLaunch"), 27, this), this);
	AddToStateVector(backupManualLaunchState);

	noteManagerbackupManualPlaceState *backupManualPlaceState = new noteManagerbackupManualPlaceState(string("backupManualPlace"), 28, new noteManagerbackupManualPlaceStateGen(string("backupManualPlace"), 28, this), this);
	AddToStateVector(backupManualPlaceState);

	OffState->RegisterTransitionState(ReadyState);
	ReadyState->RegisterTransitionState(feederIntakeState);
	ReadyState->RegisterTransitionState(ExpelState);
	ReadyState->RegisterTransitionState(placerIntakeState);
	ReadyState->RegisterTransitionState(backupManualLaunchState);
	ReadyState->RegisterTransitionState(backupManualPlaceState);
	feederIntakeState->RegisterTransitionState(ReadyState);
	feederIntakeState->RegisterTransitionState(holdFeederFrontState);
	feederIntakeState->RegisterTransitionState(holdFeederBackState);
	feederIntakeState->RegisterTransitionState(ExpelState);
	ExpelState->RegisterTransitionState(ReadyState);
	ExpelState->RegisterTransitionState(feederIntakeState);
	ExpelState->RegisterTransitionState(placerIntakeState);
	placerIntakeState->RegisterTransitionState(ReadyState);
	placerIntakeState->RegisterTransitionState(holdPlacerFrontState);
	placerIntakeState->RegisterTransitionState(holdPlacerBackState);
	holdFeederFrontState->RegisterTransitionState(ReadyState);
	holdFeederFrontState->RegisterTransitionState(launcherToPlacerFrontState);
	holdFeederFrontState->RegisterTransitionState(intakeToFeederState);
	holdFeederBackState->RegisterTransitionState(ReadyState);
	holdFeederBackState->RegisterTransitionState(intakeToFeederState);
	holdFeederBackState->RegisterTransitionState(launcherToPlacerBackState);
	intakeToFeederState->RegisterTransitionState(ReadyState);
	intakeToFeederState->RegisterTransitionState(holdFeederState);
	launcherToPlacerFrontState->RegisterTransitionState(ReadyState);
	launcherToPlacerFrontState->RegisterTransitionState(placerIntakeState);
	launcherToPlacerBackState->RegisterTransitionState(ReadyState);
	launcherToPlacerBackState->RegisterTransitionState(holdPlacerBackState);
	holdFeederState->RegisterTransitionState(ReadyState);
	holdFeederState->RegisterTransitionState(readyAutoLaunchState);
	holdFeederState->RegisterTransitionState(readyManualLaunchState);
	holdFeederState->RegisterTransitionState(PassState);
	readyAutoLaunchState->RegisterTransitionState(ReadyState);
	readyAutoLaunchState->RegisterTransitionState(holdFeederState);
	readyAutoLaunchState->RegisterTransitionState(autoLaunchState);
	readyManualLaunchState->RegisterTransitionState(ReadyState);
	readyManualLaunchState->RegisterTransitionState(manualLaunchState);
	PassState->RegisterTransitionState(ReadyState);
	autoLaunchState->RegisterTransitionState(ReadyState);
	manualLaunchState->RegisterTransitionState(ReadyState);
	readyOdometryLaunchState->RegisterTransitionState(ReadyState);
	readyOdometryLaunchState->RegisterTransitionState(readyAutoLaunchState);
	readyOdometryLaunchState->RegisterTransitionState(autoLaunchOdometryState);
	autoLaunchOdometryState->RegisterTransitionState(ReadyState);
	holdPlacerFrontState->RegisterTransitionState(ReadyState);
	holdPlacerFrontState->RegisterTransitionState(intakeToPlacerState);
	holdPlacerFrontState->RegisterTransitionState(placerToLauncherFrontState);
	holdPlacerBackState->RegisterTransitionState(ReadyState);
	holdPlacerBackState->RegisterTransitionState(placerToLauncherBackState);
	holdPlacerBackState->RegisterTransitionState(intakeToPlacerState);
	intakeToPlacerState->RegisterTransitionState(ReadyState);
	intakeToPlacerState->RegisterTransitionState(preparePlaceAmpState);
	intakeToPlacerState->RegisterTransitionState(preparePlaceTrapState);
	preparePlaceAmpState->RegisterTransitionState(ReadyState);
	preparePlaceAmpState->RegisterTransitionState(preparePlaceTrapState);
	preparePlaceAmpState->RegisterTransitionState(placeAmpState);
	preparePlaceTrapState->RegisterTransitionState(ReadyState);
	preparePlaceTrapState->RegisterTransitionState(preparePlaceAmpState);
	preparePlaceTrapState->RegisterTransitionState(placeTrapState);
	placeAmpState->RegisterTransitionState(ReadyState);
	placeTrapState->RegisterTransitionState(ReadyState);
	placerToLauncherFrontState->RegisterTransitionState(ReadyState);
	placerToLauncherFrontState->RegisterTransitionState(feederIntakeState);
	placerToLauncherBackState->RegisterTransitionState(ReadyState);
	placerToLauncherBackState->RegisterTransitionState(holdFeederBackState);
	backupManualLaunchState->RegisterTransitionState(ReadyState);
	backupManualPlaceState->RegisterTransitionState(ReadyState);
}
void noteManager::Update(RobotStateChanges::StateChange change, int value)
{
	if (change == RobotStateChanges::DesiredScoringMode)
		m_scoringMode = static_cast<RobotStateChanges::ScoringMode>(value);

	if (change == RobotStateChanges::ClimbModeStatus)
		m_climbMode = static_cast<RobotStateChanges::ClimbMode>(value);
}

bool noteManager::isLauncherMode()
{
	return m_scoringMode == RobotStateChanges::ScoringMode::Launcher;
}
bool noteManager::isPlacerMode()
{
	return m_scoringMode == RobotStateChanges::ScoringMode::Placer;
}
bool noteManager::isClimbMode()
{
	return m_climbMode == RobotStateChanges::ClimbMode::ClimbModeOn;
}
// todo not sure what to do with this
/*
bool noteManager::IsAtMinPosition(RobotElementNames::ROBOT_ELEMENT_NAMES identifier) const
{
	return m_noteManager->IsAtMinPosition(identifier);
}
bool noteManager::IsAtMinPosition(RobotElementNames::ROBOT_ELEMENT_NAMES identifier) const
{
	return m_noteManager->IsAtMinPosition(identifier);
}
bool noteManager::IsAtMaxPosition(RobotElementNames::ROBOT_ELEMENT_NAMES identifier) const
{
	return m_noteManager->IsAtMaxPosition(identifier);
}
bool noteManager::IsAtMaxPosition(RobotElementNames::ROBOT_ELEMENT_NAMES identifier) const
{
	return m_noteManager->IsAtMaxPosition(identifier);
}
*/
