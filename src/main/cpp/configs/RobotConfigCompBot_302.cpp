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
// Generated on Saturday, July 27, 2024 6:37:03 PM

#include <string>

#include "PeriodicLooper.h"
#include "utils/logging/Logger.h"
#include "configs/RobotConfigMgr.h"
#include "configs/RobotConfigCompBot_302.h"
#include "configs/RobotElementNames.h"
#include "DragonVision/DragonLimelight.h"
#include "DragonVision/DragonVision.h"
#include "hw/DragonLeds.h"

using std::string;

void RobotConfigCompBot_302::DefineMechanisms()
{
	Logger::GetLogger()->LogData ( LOGGER_LEVEL::PRINT, string ( "Initializing mechanism" ), string ( "noteManager" ), "" );
	noteManagerGen* noteManagerGenMech = new noteManagerGen ( RobotConfigMgr::RobotIdentifier::COMP_BOT_302 );
	m_thenoteManager = new noteManager ( noteManagerGenMech, RobotConfigMgr::RobotIdentifier::COMP_BOT_302 );
	m_thenoteManager->CreateCompBot302();
	m_thenoteManager->InitializeCompBot302();
	m_thenoteManager->CreateAndRegisterStates();
	m_thenoteManager->Init ( m_thenoteManager );
	m_mechanismMap[MechanismTypes::MECHANISM_TYPE::NOTE_MANAGER] = m_thenoteManager;

	Logger::GetLogger()->LogData ( LOGGER_LEVEL::PRINT, string ( "Initializing mechanism" ), string ( "ClimberManager" ), "" );
	ClimberManagerGen* ClimberManagerGenMech = new ClimberManagerGen ( RobotConfigMgr::RobotIdentifier::COMP_BOT_302 );
	m_theClimberManager = new ClimberManager ( ClimberManagerGenMech, RobotConfigMgr::RobotIdentifier::COMP_BOT_302 );
	m_theClimberManager->CreateCompBot302();
	m_theClimberManager->InitializeCompBot302();
	m_theClimberManager->CreateAndRegisterStates();
	m_theClimberManager->Init ( m_theClimberManager );
	m_mechanismMap[MechanismTypes::MECHANISM_TYPE::CLIMBER_MANAGER] = m_theClimberManager;
}

void RobotConfigCompBot_302::DefineLEDs()
{
	DragonLeds::GetInstance()->Initialize ( 9, 60 );

}

StateMgr *RobotConfigCompBot_302::GetMechanism ( MechanismTypes::MECHANISM_TYPE mechType )
{
	auto itr = m_mechanismMap.find ( mechType );
	if ( itr != m_mechanismMap.end() )
	{
		return itr->second;
	}
	return nullptr;
}

void RobotConfigCompBot_302::DefineVisionSensors()
{
	PIntake = new DragonLimelight ( "limelight-pintake", //std::string name,                      /// <I> - network table name
	                                DragonCamera::PIPELINE::OFF, //PIPELINE initialPipeline,              /// <I> enum for starting pipeline
	                                units::length::inch_t ( -11 ), //units::length::inch_t mountingXOffset, /// <I> x offset of cam from robot center (forward relative to robot)
	                                units::length::inch_t ( 0 ), //units::length::inch_t mountingYOffset, /// <I> y offset of cam from robot center (left relative to robot)
	                                units::length::inch_t ( 17.25 ), //units::length::inch_t mountingZOffset, /// <I> z offset of cam from robot center (up relative to robot)
	                                units::angle::degree_t ( -29 ), //units::angle::degree_t pitch,          /// <I> - Pitch of camera
	                                units::angle::degree_t ( 180 ), //units::angle::degree_t yaw,            /// <I> - Yaw of camera
	                                units::angle::degree_t ( 0 ), //units::angle::degree_t roll,           /// <I> - Roll of camera
	                                DragonLimelight::LED_MODE::LED_OFF, //LED_MODE ledMode,
	                                DragonLimelight::CAM_MODE::CAM_VISION, //CAM_MODE camMode,
	                                DragonLimelight::STREAM_MODE::STREAM_DEFAULT, //STREAM_MODE streamMode,
	                                DragonLimelight::SNAPSHOT_MODE::SNAP_OFF ); //SNAPSHOT_MODE snapMode);
	DragonVision::GetDragonVision()->AddCamera ( PIntake, RobotElementNames::CAMERA_USAGE::PINTAKE );

	LIntake = new DragonLimelight ( "limelight-lintake", //std::string name,                      /// <I> - network table name
	                                DragonCamera::PIPELINE::OFF, //PIPELINE initialPipeline,              /// <I> enum for starting pipeline
	                                units::length::inch_t ( 13.5 ), //units::length::inch_t mountingXOffset, /// <I> x offset of cam from robot center (forward relative to robot)
	                                units::length::inch_t ( 7.25 ), //units::length::inch_t mountingYOffset, /// <I> y offset of cam from robot center (left relative to robot)
	                                units::length::inch_t ( 12.6 ), //units::length::inch_t mountingZOffset, /// <I> z offset of cam from robot center (up relative to robot)
	                                units::angle::degree_t ( -34 ), //units::angle::degree_t pitch,          /// <I> - Pitch of camera
	                                units::angle::degree_t ( 0 ), //units::angle::degree_t yaw,            /// <I> - Yaw of camera
	                                units::angle::degree_t ( 0 ), //units::angle::degree_t roll,           /// <I> - Roll of camera
	                                DragonLimelight::LED_MODE::LED_OFF, //LED_MODE ledMode,
	                                DragonLimelight::CAM_MODE::CAM_VISION, //CAM_MODE camMode,
	                                DragonLimelight::STREAM_MODE::STREAM_DEFAULT, //STREAM_MODE streamMode,
	                                DragonLimelight::SNAPSHOT_MODE::SNAP_OFF ); //SNAPSHOT_MODE snapMode);
	DragonVision::GetDragonVision()->AddCamera ( LIntake, RobotElementNames::CAMERA_USAGE::LINTAKE );

	Launche = new DragonLimelight ( "limelight-launche", //std::string name,                      /// <I> - network table name
	                                DragonCamera::PIPELINE::OFF, //PIPELINE initialPipeline,              /// <I> enum for starting pipeline
	                                units::length::inch_t ( -6 ), //units::length::inch_t mountingXOffset, /// <I> x offset of cam from robot center (forward relative to robot)
	                                units::length::inch_t ( 10 ), //units::length::inch_t mountingYOffset, /// <I> y offset of cam from robot center (left relative to robot)
	                                units::length::inch_t ( 24.5 ), //units::length::inch_t mountingZOffset, /// <I> z offset of cam from robot center (up relative to robot)
	                                units::angle::degree_t ( 16 ), //units::angle::degree_t pitch,          /// <I> - Pitch of camera
	                                units::angle::degree_t ( 0 ), //units::angle::degree_t yaw,            /// <I> - Yaw of camera
	                                units::angle::degree_t ( 90 ), //units::angle::degree_t roll,           /// <I> - Roll of camera
	                                DragonLimelight::LED_MODE::LED_OFF, //LED_MODE ledMode,
	                                DragonLimelight::CAM_MODE::CAM_VISION, //CAM_MODE camMode,
	                                DragonLimelight::STREAM_MODE::STREAM_DEFAULT, //STREAM_MODE streamMode,
	                                DragonLimelight::SNAPSHOT_MODE::SNAP_OFF ); //SNAPSHOT_MODE snapMode);
	DragonVision::GetDragonVision()->AddCamera ( Launche, RobotElementNames::CAMERA_USAGE::LAUNCHE );
}
