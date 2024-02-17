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
// This file was automatically generated by the Team 302 code generator version 1.2.3.5
// Generated on Friday, February 16, 2024 7:55:36 PM

// C++ Includes
#include <string>

// FRC includes

// Team 302 includes
#include "mechanisms/noteManager/generated/noteManagerGen.h"
#include "mechanisms/noteManager/generated/noteManagerAllStatesStateGen.h"
#include "mechanisms/base/BaseMech.h"
#include "utils/logging/Logger.h"

// Third Party Includes

using std::string;
using namespace noteManagerStates;

/// @class noteManagerAllStatesStateGen
/// @brief information about the control (open loop, closed loop position, closed loop velocity, etc.) for a mechanism state
noteManagerAllStatesStateGen::noteManagerAllStatesStateGen ( string stateName,
        int stateId,
        noteManagerGen *mech ) : noteManagerBaseStateGen ( stateName, stateId, mech )
{
}

void noteManagerAllStatesStateGen::Init()
{
	if ( false ) {}
	else if ( GetnoteManager()->GetCurrentState() == noteManagerGen::STATE_NAMES::STATE_OFF )
	{
		GetnoteManager()->getfrontIntake()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_FRONT_INTAKE, 0 );
		GetnoteManager()->getbackIntake()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_BACK_INTAKE, 0 );
		GetnoteManager()->getTransfer()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_TRANSFER, 0 );
		GetnoteManager()->getPlacer()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_PLACER, 0 );
		GetnoteManager()->getElevator()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_ELEVATOR, 0 );
		GetnoteManager()->getFeeder()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_FEEDER, 0 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_TOP, 0 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_BOTTOM, 0 );
		GetnoteManager()->getlauncherAngle()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_ANGLE, 0 );
	}
	else if ( GetnoteManager()->GetCurrentState() == noteManagerGen::STATE_NAMES::STATE_READY )
	{
		GetnoteManager()->getfrontIntake()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_FRONT_INTAKE, 0 );
		GetnoteManager()->getbackIntake()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_BACK_INTAKE, 0 );
		GetnoteManager()->getTransfer()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_TRANSFER, 0 );
		GetnoteManager()->getPlacer()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_PLACER, 0 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_ELEVATOR, GetnoteManager()->getpositionInch(), units::length::inch_t ( units::length::inch_t ( 0 ) ) );
		GetnoteManager()->getFeeder()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_FEEDER, 0 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_TOP, 0.25 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_BOTTOM, 0.25 );
		GetnoteManager()->getlauncherAngle()->SetControlConstants ( -0.045,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_ANGLE, -0.045 );
	}
	else if ( GetnoteManager()->GetCurrentState() == noteManagerGen::STATE_NAMES::STATE_FEEDER_INTAKE )
	{
		GetnoteManager()->getfrontIntake()->SetControlConstants ( 1,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_FRONT_INTAKE, 1 );
		GetnoteManager()->getbackIntake()->SetControlConstants ( 1,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_BACK_INTAKE, 1 );
		GetnoteManager()->getTransfer()->SetControlConstants ( 1,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_TRANSFER, 1 );
		GetnoteManager()->getPlacer()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_PLACER, 0 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_ELEVATOR, GetnoteManager()->getpositionInch(), units::length::inch_t ( units::length::inch_t ( 0 ) ) );
		GetnoteManager()->getFeeder()->SetControlConstants ( 0.35,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_FEEDER, 0.35 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_TOP, 0.25 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_BOTTOM, 0.25 );
		GetnoteManager()->getlauncherAngle()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_ANGLE, 0 );
	}
	else if ( GetnoteManager()->GetCurrentState() == noteManagerGen::STATE_NAMES::STATE_EXPEL )
	{
		GetnoteManager()->getfrontIntake()->SetControlConstants ( -1,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_FRONT_INTAKE, -1 );
		GetnoteManager()->getbackIntake()->SetControlConstants ( -1,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_BACK_INTAKE, -1 );
		GetnoteManager()->getTransfer()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_TRANSFER, 0 );
		GetnoteManager()->getPlacer()->SetControlConstants ( -1,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_PLACER, -1 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_ELEVATOR, GetnoteManager()->getpositionInch(), units::length::inch_t ( units::length::inch_t ( 0 ) ) );
		GetnoteManager()->getFeeder()->SetControlConstants ( -1,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_FEEDER, -1 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_TOP, 0.25 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_BOTTOM, 0.25 );
		GetnoteManager()->getlauncherAngle()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_ANGLE, 0 );
	}
	else if ( GetnoteManager()->GetCurrentState() == noteManagerGen::STATE_NAMES::STATE_PLACER_INTAKE )
	{
		GetnoteManager()->getfrontIntake()->SetControlConstants ( 1,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_FRONT_INTAKE, 1 );
		GetnoteManager()->getbackIntake()->SetControlConstants ( 1,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_BACK_INTAKE, 1 );
		GetnoteManager()->getTransfer()->SetControlConstants ( -1,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_TRANSFER, -1 );
		GetnoteManager()->getPlacer()->SetControlConstants ( 0.5,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_PLACER, 0.5 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_ELEVATOR, GetnoteManager()->getpositionInch(), units::length::inch_t ( units::length::inch_t ( 0 ) ) );
		GetnoteManager()->getFeeder()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_FEEDER, 0 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_TOP, 0.25 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_BOTTOM, 0.25 );
		GetnoteManager()->getlauncherAngle()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_ANGLE, 0 );
	}
	else if ( GetnoteManager()->GetCurrentState() == noteManagerGen::STATE_NAMES::STATE_LAUNCHER_TO_PLACER )
	{
		GetnoteManager()->getfrontIntake()->SetControlConstants ( 1,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_FRONT_INTAKE, 1 );
		GetnoteManager()->getbackIntake()->SetControlConstants ( -1,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_BACK_INTAKE, -1 );
		GetnoteManager()->getTransfer()->SetControlConstants ( -1,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_TRANSFER, -1 );
		GetnoteManager()->getPlacer()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_PLACER, 0 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_ELEVATOR, GetnoteManager()->getpositionInch(), units::length::inch_t ( units::length::inch_t ( 0 ) ) );
		GetnoteManager()->getFeeder()->SetControlConstants ( -1,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_FEEDER, -1 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_TOP, 0.25 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_BOTTOM, 0.25 );
		GetnoteManager()->getlauncherAngle()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_ANGLE, 0 );
	}
	else if ( GetnoteManager()->GetCurrentState() == noteManagerGen::STATE_NAMES::STATE_HOLD_FEEDER )
	{
		GetnoteManager()->getfrontIntake()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_FRONT_INTAKE, 0 );
		GetnoteManager()->getbackIntake()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_BACK_INTAKE, 0 );
		GetnoteManager()->getTransfer()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_TRANSFER, 0 );
		GetnoteManager()->getPlacer()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_PLACER, 0 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_ELEVATOR, GetnoteManager()->getpositionInch(), units::length::inch_t ( units::length::inch_t ( 0 ) ) );
		GetnoteManager()->getFeeder()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_FEEDER, 0 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_TOP, 0.25 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_BOTTOM, 0.25 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_ANGLE, GetnoteManager()->getposDegreeAbs(), units::angle::degree_t ( units::angle::degree_t ( -26 ) ) );
	}
	else if ( GetnoteManager()->GetCurrentState() == noteManagerGen::STATE_NAMES::STATE_READY_AUTO_LAUNCH )
	{
		GetnoteManager()->getfrontIntake()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_FRONT_INTAKE, 0 );
		GetnoteManager()->getbackIntake()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_BACK_INTAKE, 0 );
		GetnoteManager()->getTransfer()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_TRANSFER, 0 );
		GetnoteManager()->getPlacer()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_PLACER, 0 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_ELEVATOR, GetnoteManager()->getpositionInch(), units::length::inch_t ( units::length::inch_t ( 0 ) ) );
		GetnoteManager()->getFeeder()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_FEEDER, 0 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_TOP, 0.75 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_BOTTOM, 0.75 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_ANGLE, GetnoteManager()->getposDegreeAbs(), units::angle::degree_t ( units::angle::degree_t ( -35 ) ) );
	}
	else if ( GetnoteManager()->GetCurrentState() == noteManagerGen::STATE_NAMES::STATE_READY_MANUAL_LAUNCH )
	{
		GetnoteManager()->getfrontIntake()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_FRONT_INTAKE, 0 );
		GetnoteManager()->getbackIntake()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_BACK_INTAKE, 0 );
		GetnoteManager()->getTransfer()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_TRANSFER, 0 );
		GetnoteManager()->getPlacer()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_PLACER, 0 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_ELEVATOR, GetnoteManager()->getpositionInch(), units::length::inch_t ( units::length::inch_t ( 0 ) ) );
		GetnoteManager()->getFeeder()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_FEEDER, 0 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_TOP, 0.75 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_BOTTOM, 0.75 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_ANGLE, GetnoteManager()->getposDegreeAbs(), units::angle::degree_t ( units::angle::degree_t ( -26 ) ) );
	}
	else if ( GetnoteManager()->GetCurrentState() == noteManagerGen::STATE_NAMES::STATE_PASS )
	{
		GetnoteManager()->getfrontIntake()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_FRONT_INTAKE, 0 );
		GetnoteManager()->getbackIntake()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_BACK_INTAKE, 0 );
		GetnoteManager()->getTransfer()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_TRANSFER, 0 );
		GetnoteManager()->getPlacer()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_PLACER, 0 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_ELEVATOR, GetnoteManager()->getpositionInch(), units::length::inch_t ( units::length::inch_t ( 0 ) ) );
		GetnoteManager()->getFeeder()->SetControlConstants ( 1,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_FEEDER, 1 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_TOP, 0.25 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_BOTTOM, 0.25 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_ANGLE, GetnoteManager()->getposDegreeAbs(), units::angle::degree_t ( units::angle::degree_t ( -50 ) ) );
	}
	else if ( GetnoteManager()->GetCurrentState() == noteManagerGen::STATE_NAMES::STATE_AUTO_LAUNCH )
	{
		GetnoteManager()->getfrontIntake()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_FRONT_INTAKE, 0 );
		GetnoteManager()->getbackIntake()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_BACK_INTAKE, 0 );
		GetnoteManager()->getTransfer()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_TRANSFER, 0 );
		GetnoteManager()->getPlacer()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_PLACER, 0 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_ELEVATOR, GetnoteManager()->getpositionInch(), units::length::inch_t ( units::length::inch_t ( 0 ) ) );
		GetnoteManager()->getFeeder()->SetControlConstants ( 1,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_FEEDER, 1 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_TOP, 0.75 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_BOTTOM, 0.75 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_ANGLE, GetnoteManager()->getposDegreeAbs(), units::angle::degree_t ( units::angle::degree_t ( -15 ) ) );
	}
	else if ( GetnoteManager()->GetCurrentState() == noteManagerGen::STATE_NAMES::STATE_MANUAL_LAUNCH )
	{
		GetnoteManager()->getfrontIntake()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_FRONT_INTAKE, 0 );
		GetnoteManager()->getbackIntake()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_BACK_INTAKE, 0 );
		GetnoteManager()->getTransfer()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_TRANSFER, 0 );
		GetnoteManager()->getPlacer()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_PLACER, 0 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_ELEVATOR, GetnoteManager()->getpositionInch(), units::length::inch_t ( units::length::inch_t ( 0 ) ) );
		GetnoteManager()->getFeeder()->SetControlConstants ( 1,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_FEEDER, 1 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_TOP, 0.75 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_BOTTOM, 0.75 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_ANGLE, GetnoteManager()->getposDegreeAbs(), units::angle::degree_t ( units::angle::degree_t ( -26 ) ) );
	}
	else if ( GetnoteManager()->GetCurrentState() == noteManagerGen::STATE_NAMES::STATE_READY_ODOMETRY_LAUNCH )
	{
		GetnoteManager()->getfrontIntake()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_FRONT_INTAKE, 0 );
		GetnoteManager()->getbackIntake()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_BACK_INTAKE, 0 );
		GetnoteManager()->getTransfer()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_TRANSFER, 0 );
		GetnoteManager()->getPlacer()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_PLACER, 0 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_ELEVATOR, GetnoteManager()->getpositionInch(), units::length::inch_t ( units::length::inch_t ( 0 ) ) );
		GetnoteManager()->getFeeder()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_FEEDER, 0 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_TOP, 0.25 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_BOTTOM, 0.25 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_ANGLE, GetnoteManager()->getposDegreeAbs(), units::angle::degree_t ( units::angle::degree_t ( -15 ) ) );
	}
	else if ( GetnoteManager()->GetCurrentState() == noteManagerGen::STATE_NAMES::STATE_AUTO_LAUNCH_ODOMETRY )
	{
		GetnoteManager()->getfrontIntake()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_FRONT_INTAKE, 0 );
		GetnoteManager()->getbackIntake()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_BACK_INTAKE, 0 );
		GetnoteManager()->getTransfer()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_TRANSFER, 0 );
		GetnoteManager()->getPlacer()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_PLACER, 0 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_ELEVATOR, GetnoteManager()->getpositionInch(), units::length::inch_t ( units::length::inch_t ( 0 ) ) );
		GetnoteManager()->getFeeder()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_FEEDER, 0 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_TOP, 0.25 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_BOTTOM, 0.25 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_ANGLE, GetnoteManager()->getposDegreeAbs(), units::angle::degree_t ( units::angle::degree_t ( -15 ) ) );
	}
	else if ( GetnoteManager()->GetCurrentState() == noteManagerGen::STATE_NAMES::STATE_PREPARE_PLACE_AMP )
	{
		GetnoteManager()->getfrontIntake()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_FRONT_INTAKE, 0 );
		GetnoteManager()->getbackIntake()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_BACK_INTAKE, 0 );
		GetnoteManager()->getTransfer()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_TRANSFER, 0 );
		GetnoteManager()->getPlacer()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_PLACER, 0 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_ELEVATOR, GetnoteManager()->getpositionInchUp(), units::length::inch_t ( units::length::inch_t ( 11.5 ) ) );
		GetnoteManager()->getFeeder()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_FEEDER, 0 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_TOP, 0.25 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_BOTTOM, 0.25 );
		GetnoteManager()->getlauncherAngle()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_ANGLE, 0 );
	}
	else if ( GetnoteManager()->GetCurrentState() == noteManagerGen::STATE_NAMES::STATE_PREPARE_PLACE_TRAP )
	{
		GetnoteManager()->getfrontIntake()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_FRONT_INTAKE, 0 );
		GetnoteManager()->getbackIntake()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_BACK_INTAKE, 0 );
		GetnoteManager()->getTransfer()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_TRANSFER, 0 );
		GetnoteManager()->getPlacer()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_PLACER, 0 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_ELEVATOR, GetnoteManager()->getpositionInchUp(), units::length::inch_t ( units::length::inch_t ( 16.5 ) ) );
		GetnoteManager()->getFeeder()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_FEEDER, 0 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_TOP, 0.25 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_BOTTOM, 0.25 );
		GetnoteManager()->getlauncherAngle()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_ANGLE, 0 );
	}
	else if ( GetnoteManager()->GetCurrentState() == noteManagerGen::STATE_NAMES::STATE_PLACE_AMP )
	{
		GetnoteManager()->getfrontIntake()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_FRONT_INTAKE, 0 );
		GetnoteManager()->getbackIntake()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_BACK_INTAKE, 0 );
		GetnoteManager()->getTransfer()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_TRANSFER, 0 );
		GetnoteManager()->getPlacer()->SetControlConstants ( 1,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_PLACER, 1 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_ELEVATOR, GetnoteManager()->getpositionInchUp(), units::length::inch_t ( units::length::inch_t ( 11.5 ) ) );
		GetnoteManager()->getFeeder()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_FEEDER, 0 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_TOP, 0.25 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_BOTTOM, 0.25 );
		GetnoteManager()->getlauncherAngle()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_ANGLE, 0 );
	}
	else if ( GetnoteManager()->GetCurrentState() == noteManagerGen::STATE_NAMES::STATE_PLACE_TRAP )
	{
		GetnoteManager()->getfrontIntake()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_FRONT_INTAKE, 0 );
		GetnoteManager()->getbackIntake()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_BACK_INTAKE, 0 );
		GetnoteManager()->getTransfer()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_TRANSFER, 0 );
		GetnoteManager()->getPlacer()->SetControlConstants ( 1,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_PLACER, 1 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_ELEVATOR, GetnoteManager()->getpositionInchUp(), units::length::inch_t ( units::length::inch_t ( 16.5 ) ) );
		GetnoteManager()->getFeeder()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_FEEDER, 0 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_TOP, 0.25 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_BOTTOM, 0.25 );
		GetnoteManager()->getlauncherAngle()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_ANGLE, 0 );
	}
	else if ( GetnoteManager()->GetCurrentState() == noteManagerGen::STATE_NAMES::STATE_PLACER_TO_LAUNCHER )
	{
		GetnoteManager()->getfrontIntake()->SetControlConstants ( -1,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_FRONT_INTAKE, -1 );
		GetnoteManager()->getbackIntake()->SetControlConstants ( 1,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_BACK_INTAKE, 1 );
		GetnoteManager()->getTransfer()->SetControlConstants ( 1,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_TRANSFER, 1 );
		GetnoteManager()->getPlacer()->SetControlConstants ( -1,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_PLACER, -1 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_ELEVATOR, GetnoteManager()->getpositionInch(), units::length::inch_t ( units::length::inch_t ( 0 ) ) );
		GetnoteManager()->getFeeder()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_FEEDER, 0 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_TOP, 0.25 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_BOTTOM, 0.25 );
		GetnoteManager()->getlauncherAngle()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_ANGLE, 0 );
	}
	else if ( GetnoteManager()->GetCurrentState() == noteManagerGen::STATE_NAMES::STATE_BACKUP_MANUAL_LAUNCH )
	{
		GetnoteManager()->getfrontIntake()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_FRONT_INTAKE, 0 );
		GetnoteManager()->getbackIntake()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_BACK_INTAKE, 0 );
		GetnoteManager()->getTransfer()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_TRANSFER, 0 );
		GetnoteManager()->getPlacer()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_PLACER, 0 );
		GetnoteManager()->getElevator()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_ELEVATOR, 0 );
		GetnoteManager()->getFeeder()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_FEEDER, 0 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_TOP, 0.25 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_BOTTOM, 0.25 );
		GetnoteManager()->getlauncherAngle()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_ANGLE, 0 );
	}
	else if ( GetnoteManager()->GetCurrentState() == noteManagerGen::STATE_NAMES::STATE_BACKUP_MANUAL_PLACE )
	{
		GetnoteManager()->getfrontIntake()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_FRONT_INTAKE, 0 );
		GetnoteManager()->getbackIntake()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_BACK_INTAKE, 0 );
		GetnoteManager()->getTransfer()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_TRANSFER, 0 );
		GetnoteManager()->getPlacer()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_PLACER, 0 );
		GetnoteManager()->getElevator()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_ELEVATOR, 0 );
		GetnoteManager()->getFeeder()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_FEEDER, 0 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_TOP, 0.25 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_BOTTOM, 0.25 );
		GetnoteManager()->getlauncherAngle()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_ANGLE, 0 );
	}
	else if ( GetnoteManager()->GetCurrentState() == noteManagerGen::STATE_NAMES::STATE_HOLD_PLACER )
	{
		GetnoteManager()->getfrontIntake()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_FRONT_INTAKE, 0 );
		GetnoteManager()->getbackIntake()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_BACK_INTAKE, 0 );
		GetnoteManager()->getTransfer()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_TRANSFER, 0 );
		GetnoteManager()->getFeeder()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_FEEDER, 0 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_TOP, 0.25 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_BOTTOM, 0.25 );
		GetnoteManager()->getlauncherAngle()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_ANGLE, 0 );
		GetnoteManager()->getPlacer()->SetControlConstants ( 0,*GetnoteManager()->getpercentOutput() );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_PLACER, 0 );
		SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_ELEVATOR, GetnoteManager()->getpositionInch(), units::length::inch_t ( units::length::inch_t ( 0 ) ) );
	}

	Logger::GetLogger()->LogData ( LOGGER_LEVEL::PRINT, string ( "ArrivedAt" ), string ( "noteManagerAllStatesStateGen" ), string ( "init" ) );

	noteManagerBaseStateGen::Init();
}

void noteManagerAllStatesStateGen::Run()
{
	// Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("noteManagerAllStatesStateGen"), string("run"));
	noteManagerBaseStateGen::Run();
}

void noteManagerAllStatesStateGen::Exit()
{
	noteManagerBaseStateGen::Exit();
}

bool noteManagerAllStatesStateGen::AtTarget()
{
	return noteManagerBaseStateGen::AtTarget();
}
