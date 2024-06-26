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
// This file was automatically generated by the Team 302 code generator version 1.3.0.15
// Generated on Tuesday, April 9, 2024 6:48:13 PM

// C++ Includes

// FRC Includes

// Team 302 includes
#include "mechanisms/noteManager/decoratormods/noteManager.h"

#include "hw/DragonSparkMaxMonitored.h"
#include "hw/DragonSparkMax.h"
#include "hw/DragonSparkFlexMonitored.h"
#include "hw/DragonSparkFlex.h"
#include "hw/DragonTalonFX.h"
#include "hw/DragonDigitalInput.h"
#include "hw/DragonCanCoder.h"
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
#include "mechanisms/noteManager/decoratormods/preparePlaceAmpState.h"
#include "mechanisms/noteManager/decoratormods/preparePlaceTrapState.h"
#include "mechanisms/noteManager/decoratormods/placeAmpState.h"
#include "mechanisms/noteManager/decoratormods/placeTrapState.h"
#include "mechanisms/noteManager/decoratormods/placerToLauncherState.h"
#include "mechanisms/noteManager/decoratormods/backupManualLaunchState.h"
#include "mechanisms/noteManager/decoratormods/backupManualPlaceState.h"
#include "mechanisms/noteManager/decoratormods/holdPlacerState.h"
#include "mechanisms/noteManager/decoratormods/lowPassState.h"

using std::string;
using namespace noteManagerStates;

void noteManager::CreateAndRegisterStates()
{
	OffState* OffStateInst = new OffState ( string ( "Off" ), 0, new noteManagerAllStatesStateGen ( m_activeRobotId, string ( "Off" ), 0, this ), this );
	AddToStateVector ( OffStateInst );

	ReadyState* ReadyStateInst = new ReadyState ( string ( "Ready" ), 1, new noteManagerAllStatesStateGen ( m_activeRobotId, string ( "Ready" ), 1, this ), this );
	AddToStateVector ( ReadyStateInst );

	feederIntakeState* feederIntakeStateInst = new feederIntakeState ( string ( "feederIntake" ), 2, new noteManagerAllStatesStateGen ( m_activeRobotId, string ( "feederIntake" ), 2, this ), this );
	AddToStateVector ( feederIntakeStateInst );

	ExpelState* ExpelStateInst = new ExpelState ( string ( "Expel" ), 3, new noteManagerAllStatesStateGen ( m_activeRobotId, string ( "Expel" ), 3, this ), this );
	AddToStateVector ( ExpelStateInst );

	placerIntakeState* placerIntakeStateInst = new placerIntakeState ( string ( "placerIntake" ), 4, new noteManagerAllStatesStateGen ( m_activeRobotId, string ( "placerIntake" ), 4, this ), this );
	AddToStateVector ( placerIntakeStateInst );

	launcherToPlacerState* launcherToPlacerStateInst = new launcherToPlacerState ( string ( "launcherToPlacer" ), 5, new noteManagerAllStatesStateGen ( m_activeRobotId, string ( "launcherToPlacer" ), 5, this ), this );
	AddToStateVector ( launcherToPlacerStateInst );

	holdFeederState* holdFeederStateInst = new holdFeederState ( string ( "holdFeeder" ), 6, new noteManagerAllStatesStateGen ( m_activeRobotId, string ( "holdFeeder" ), 6, this ), this );
	AddToStateVector ( holdFeederStateInst );

	readyAutoLaunchState* readyAutoLaunchStateInst = new readyAutoLaunchState ( string ( "readyAutoLaunch" ), 7, new noteManagerAllStatesStateGen ( m_activeRobotId, string ( "readyAutoLaunch" ), 7, this ), this );
	AddToStateVector ( readyAutoLaunchStateInst );

	readyManualLaunchState* readyManualLaunchStateInst = new readyManualLaunchState ( string ( "readyManualLaunch" ), 8, new noteManagerAllStatesStateGen ( m_activeRobotId, string ( "readyManualLaunch" ), 8, this ), this );
	AddToStateVector ( readyManualLaunchStateInst );

	PassState* PassStateInst = new PassState ( string ( "Pass" ), 9, new noteManagerAllStatesStateGen ( m_activeRobotId, string ( "Pass" ), 9, this ), this );
	AddToStateVector ( PassStateInst );

	autoLaunchState* autoLaunchStateInst = new autoLaunchState ( string ( "autoLaunch" ), 10, new noteManagerAllStatesStateGen ( m_activeRobotId, string ( "autoLaunch" ), 10, this ), this );
	AddToStateVector ( autoLaunchStateInst );

	manualLaunchState* manualLaunchStateInst = new manualLaunchState ( string ( "manualLaunch" ), 11, new noteManagerAllStatesStateGen ( m_activeRobotId, string ( "manualLaunch" ), 11, this ), this );
	AddToStateVector ( manualLaunchStateInst );

	readyOdometryLaunchState* readyOdometryLaunchStateInst = new readyOdometryLaunchState ( string ( "readyOdometryLaunch" ), 12, new noteManagerAllStatesStateGen ( m_activeRobotId, string ( "readyOdometryLaunch" ), 12, this ), this );
	AddToStateVector ( readyOdometryLaunchStateInst );

	preparePlaceAmpState* preparePlaceAmpStateInst = new preparePlaceAmpState ( string ( "preparePlaceAmp" ), 13, new noteManagerAllStatesStateGen ( m_activeRobotId, string ( "preparePlaceAmp" ), 13, this ), this );
	AddToStateVector ( preparePlaceAmpStateInst );

	preparePlaceTrapState* preparePlaceTrapStateInst = new preparePlaceTrapState ( string ( "preparePlaceTrap" ), 14, new noteManagerAllStatesStateGen ( m_activeRobotId, string ( "preparePlaceTrap" ), 14, this ), this );
	AddToStateVector ( preparePlaceTrapStateInst );

	placeAmpState* placeAmpStateInst = new placeAmpState ( string ( "placeAmp" ), 15, new noteManagerAllStatesStateGen ( m_activeRobotId, string ( "placeAmp" ), 15, this ), this );
	AddToStateVector ( placeAmpStateInst );

	placeTrapState* placeTrapStateInst = new placeTrapState ( string ( "placeTrap" ), 16, new noteManagerAllStatesStateGen ( m_activeRobotId, string ( "placeTrap" ), 16, this ), this );
	AddToStateVector ( placeTrapStateInst );

	placerToLauncherState* placerToLauncherStateInst = new placerToLauncherState ( string ( "placerToLauncher" ), 17, new noteManagerAllStatesStateGen ( m_activeRobotId, string ( "placerToLauncher" ), 17, this ), this );
	AddToStateVector ( placerToLauncherStateInst );

	backupManualLaunchState* backupManualLaunchStateInst = new backupManualLaunchState ( string ( "backupManualLaunch" ), 18, new noteManagerAllStatesStateGen ( m_activeRobotId, string ( "backupManualLaunch" ), 18, this ), this );
	AddToStateVector ( backupManualLaunchStateInst );

	backupManualPlaceState* backupManualPlaceStateInst = new backupManualPlaceState ( string ( "backupManualPlace" ), 19, new noteManagerAllStatesStateGen ( m_activeRobotId, string ( "backupManualPlace" ), 19, this ), this );
	AddToStateVector ( backupManualPlaceStateInst );

	holdPlacerState* holdPlacerStateInst = new holdPlacerState ( string ( "holdPlacer" ), 20, new noteManagerAllStatesStateGen ( m_activeRobotId, string ( "holdPlacer" ), 20, this ), this );
	AddToStateVector ( holdPlacerStateInst );

	lowPassState* lowPassStateInst = new lowPassState ( string ( "lowPass" ), 21, new noteManagerAllStatesStateGen ( m_activeRobotId, string ( "lowPass" ), 21, this ), this );
	AddToStateVector ( lowPassStateInst );

	OffStateInst->RegisterTransitionState ( ReadyStateInst );
	ReadyStateInst->RegisterTransitionState ( feederIntakeStateInst );
	ReadyStateInst->RegisterTransitionState ( ExpelStateInst );
	ReadyStateInst->RegisterTransitionState ( placerIntakeStateInst );
	ReadyStateInst->RegisterTransitionState ( backupManualLaunchStateInst );
	ReadyStateInst->RegisterTransitionState ( backupManualPlaceStateInst );
	ReadyStateInst->RegisterTransitionState ( readyManualLaunchStateInst );
	feederIntakeStateInst->RegisterTransitionState ( ReadyStateInst );
	feederIntakeStateInst->RegisterTransitionState ( holdFeederStateInst );
	feederIntakeStateInst->RegisterTransitionState ( ExpelStateInst );
	ExpelStateInst->RegisterTransitionState ( ReadyStateInst );
	ExpelStateInst->RegisterTransitionState ( feederIntakeStateInst );
	ExpelStateInst->RegisterTransitionState ( placerIntakeStateInst );
	placerIntakeStateInst->RegisterTransitionState ( ReadyStateInst );
	placerIntakeStateInst->RegisterTransitionState ( holdPlacerStateInst );
	placerIntakeStateInst->RegisterTransitionState ( ExpelStateInst );
	launcherToPlacerStateInst->RegisterTransitionState ( ReadyStateInst );
	launcherToPlacerStateInst->RegisterTransitionState ( placerIntakeStateInst );
	holdFeederStateInst->RegisterTransitionState ( ReadyStateInst );
	holdFeederStateInst->RegisterTransitionState ( readyAutoLaunchStateInst );
	holdFeederStateInst->RegisterTransitionState ( readyManualLaunchStateInst );
	holdFeederStateInst->RegisterTransitionState ( PassStateInst );
	holdFeederStateInst->RegisterTransitionState ( launcherToPlacerStateInst );
	holdFeederStateInst->RegisterTransitionState ( lowPassStateInst );
	holdFeederStateInst->RegisterTransitionState ( readyOdometryLaunchStateInst );
	readyAutoLaunchStateInst->RegisterTransitionState ( ReadyStateInst );
	readyAutoLaunchStateInst->RegisterTransitionState ( holdFeederStateInst );
	readyAutoLaunchStateInst->RegisterTransitionState ( autoLaunchStateInst );
	readyAutoLaunchStateInst->RegisterTransitionState ( readyManualLaunchStateInst );
	readyAutoLaunchStateInst->RegisterTransitionState ( PassStateInst );
	readyManualLaunchStateInst->RegisterTransitionState ( ReadyStateInst );
	readyManualLaunchStateInst->RegisterTransitionState ( manualLaunchStateInst );
	PassStateInst->RegisterTransitionState ( ReadyStateInst );
	autoLaunchStateInst->RegisterTransitionState ( ReadyStateInst );
	manualLaunchStateInst->RegisterTransitionState ( ReadyStateInst );
	readyOdometryLaunchStateInst->RegisterTransitionState ( ReadyStateInst );
	readyOdometryLaunchStateInst->RegisterTransitionState ( readyAutoLaunchStateInst );
	readyOdometryLaunchStateInst->RegisterTransitionState ( autoLaunchStateInst );
	readyOdometryLaunchStateInst->RegisterTransitionState ( readyManualLaunchStateInst );
	readyOdometryLaunchStateInst->RegisterTransitionState ( PassStateInst );
	readyOdometryLaunchStateInst->RegisterTransitionState ( lowPassStateInst );
	readyOdometryLaunchStateInst->RegisterTransitionState ( holdFeederStateInst );
	preparePlaceAmpStateInst->RegisterTransitionState ( ReadyStateInst );
	preparePlaceAmpStateInst->RegisterTransitionState ( preparePlaceTrapStateInst );
	preparePlaceAmpStateInst->RegisterTransitionState ( placeAmpStateInst );
	preparePlaceTrapStateInst->RegisterTransitionState ( ReadyStateInst );
	preparePlaceTrapStateInst->RegisterTransitionState ( preparePlaceAmpStateInst );
	preparePlaceTrapStateInst->RegisterTransitionState ( placeTrapStateInst );
	placeAmpStateInst->RegisterTransitionState ( ReadyStateInst );
	placeTrapStateInst->RegisterTransitionState ( ReadyStateInst );
	placerToLauncherStateInst->RegisterTransitionState ( ReadyStateInst );
	placerToLauncherStateInst->RegisterTransitionState ( feederIntakeStateInst );
	backupManualLaunchStateInst->RegisterTransitionState ( ReadyStateInst );
	backupManualLaunchStateInst->RegisterTransitionState ( backupManualPlaceStateInst );
	backupManualPlaceStateInst->RegisterTransitionState ( ReadyStateInst );
	backupManualPlaceStateInst->RegisterTransitionState ( backupManualLaunchStateInst );
	holdPlacerStateInst->RegisterTransitionState ( preparePlaceAmpStateInst );
	holdPlacerStateInst->RegisterTransitionState ( preparePlaceTrapStateInst );
	holdPlacerStateInst->RegisterTransitionState ( placerToLauncherStateInst );
	holdPlacerStateInst->RegisterTransitionState ( ReadyStateInst );
	lowPassStateInst->RegisterTransitionState ( ReadyStateInst );
}
