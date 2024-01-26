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
// This file was automatically generated by the Team 302 code generator version 1.2.1.0
// Generated on Thursday, January 25, 2024 8:27:53 PM

// C++ Includes
#include <string>

// FRC includes

// Team 302 includes
#include "mechanisms/noteManager/generated/noteManager_gen.h"
#include "mechanisms/noteManager/generated/noteManager_holdFeederFront_StateGen.h"
#include "mechanisms/base/BaseMech.h"

#include <utils/logging/LoggableItemMgr.h>
#include "utils/logging/Logger.h"
#include <utils/logging/LoggerData.h>
#include <utils/logging/LoggerEnums.h>

// Third Party Includes

using std::string;

/// @class noteManagerholdFeederFrontStateGen
/// @brief information about the control (open loop, closed loop position, closed loop velocity, etc.) for a mechanism state
noteManagerholdFeederFrontStateGen::noteManagerholdFeederFrontStateGen ( string stateName,
        int stateId,
        noteManager_gen *mech ) : noteManagerBaseStateGen ( stateName, stateId, mech )
{
}

void noteManagerholdFeederFrontStateGen::Init()
{
	SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_FRONT_INTAKE, 0 );
	SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_BACK_INTAKE, 0 );
	SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_TRANSFER, 0 );
	SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_PLACER, 0 );
	SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_ELEVATOR, * ( GetnoteManager()->positionInch ), units::length::inch_t ( ( 0 ) ) );
	SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_FEEDER, 0 );
	SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_TOP, * ( GetnoteManager()->velocityRPS ), units::angular_velocity::revolutions_per_minute_t ( ( 0 ) ) );
	SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_BOTTOM, * ( GetnoteManager()->velocityRPS ), units::angular_velocity::revolutions_per_minute_t ( ( 0 ) ) );
	SetTargetControl ( RobotElementNames::MOTOR_CONTROLLER_USAGE::NOTE_MANAGER_LAUNCHER_ANGLE, * ( GetnoteManager()->posDegreeAbs ), units::angle::degree_t ( ( 0 ) ) );

	Logger::GetLogger()->LogData ( LOGGER_LEVEL::PRINT, string ( "ArrivedAt" ), string ( "noteManagerholdFeederFrontStateGen" ), string ( "init" ) );

	noteManagerBaseStateGen::Init();
}

void noteManagerholdFeederFrontStateGen::Run()
{
	// Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("noteManagerholdFeederFrontStateGen"), string("run"));
	noteManagerBaseStateGen::Run();
}

void noteManagerholdFeederFrontStateGen::Exit()
{
	noteManagerBaseStateGen::Exit();
}

bool noteManagerholdFeederFrontStateGen::AtTarget()
{
	return noteManagerBaseStateGen::AtTarget();
}
