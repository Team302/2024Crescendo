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
// This file was automatically generated by the Team 302 code generator version 1.3.0.13
// Generated on Saturday, March 23, 2024 11:13:23 AM

#include <string>

#include "PeriodicLooper.h"
#include "utils/logging/Logger.h"
#include "configs/RobotConfigMgr.h"
#include "configs/RobotConfigpracticeBot_9999.h"
#include "configs/RobotElementNames.h"
#include "DragonVision/DragonLimelight.h"
#include "DragonVision/DragonVision.h"
#include "DragonVision/DragonPhotonCam.h"
#include "hw/DragonLeds.h"

using std::string;

void RobotConfigpracticeBot_9999::DefineMechanisms()
{
	Logger::GetLogger()->LogData ( LOGGER_LEVEL::PRINT, string ( "Initializing mechanism" ), string ( "noteManager" ), "" );
	noteManagerGen* noteManagerGenMech = new noteManagerGen ( RobotConfigMgr::RobotIdentifier::PRACTICE_BOT_9999 );
	m_thenoteManager = new noteManager ( noteManagerGenMech, RobotConfigMgr::RobotIdentifier::PRACTICE_BOT_9999 );
	m_thenoteManager->CreatepracticeBot9999();
	m_thenoteManager->InitializepracticeBot9999();
	m_thenoteManager->CreateAndRegisterStates();
	m_thenoteManager->Init ( m_thenoteManager );
	m_mechanismMap[MechanismTypes::MECHANISM_TYPE::NOTE_MANAGER] = m_thenoteManager;

	Logger::GetLogger()->LogData ( LOGGER_LEVEL::PRINT, string ( "Initializing mechanism" ), string ( "ClimberManager" ), "" );
	ClimberManagerGen* ClimberManagerGenMech = new ClimberManagerGen ( RobotConfigMgr::RobotIdentifier::PRACTICE_BOT_9999 );
	m_theClimberManager = new ClimberManager ( ClimberManagerGenMech, RobotConfigMgr::RobotIdentifier::PRACTICE_BOT_9999 );
	m_theClimberManager->CreatepracticeBot9999();
	m_theClimberManager->InitializepracticeBot9999();
	m_theClimberManager->CreateAndRegisterStates();
	m_theClimberManager->Init ( m_theClimberManager );
	m_mechanismMap[MechanismTypes::MECHANISM_TYPE::CLIMBER_MANAGER] = m_theClimberManager;
}

void RobotConfigpracticeBot_9999::DefineLEDs()
{
	DragonLeds::GetInstance()->Initialize ( 8, 15 );

}

StateMgr *RobotConfigpracticeBot_9999::GetMechanism ( MechanismTypes::MECHANISM_TYPE mechType )
{
	auto itr = m_mechanismMap.find ( mechType );
	if ( itr != m_mechanismMap.end() )
	{
		return itr->second;
	}
	return nullptr;
}

void RobotConfigpracticeBot_9999::DefineVisionSensors()
{
	PIntake = new DragonLimelight ( "limelight-pintake", //std::string name,                      /// <I> - network table name
	                                DragonCamera::PIPELINE::OFF, //PIPELINE initialPipeline,              /// <I> enum for starting pipeline
	                                units::length::inch_t ( -14 ), //units::length::inch_t mountingXOffset, /// <I> x offset of cam from robot center (forward relative to robot)
	                                units::length::inch_t ( 1.0 ), //units::length::inch_t mountingYOffset, /// <I> y offset of cam from robot center (left relative to robot)
	                                units::length::inch_t ( 19.25 ), //units::length::inch_t mountingZOffset, /// <I> z offset of cam from robot center (up relative to robot)
	                                units::angle::degree_t ( -51 ), //units::angle::degree_t pitch,          /// <I> - Pitch of camera
	                                units::angle::degree_t ( 180 ), //units::angle::degree_t yaw,            /// <I> - Yaw of camera
	                                units::angle::degree_t ( 0 ), //units::angle::degree_t roll,           /// <I> - Roll of camera
	                                DragonLimelight::LED_MODE::LED_OFF, //LED_MODE ledMode,
	                                DragonLimelight::CAM_MODE::CAM_VISION, //CAM_MODE camMode,
	                                DragonLimelight::STREAM_MODE::STREAM_DEFAULT, //STREAM_MODE streamMode,
	                                DragonLimelight::SNAPSHOT_MODE::SNAP_OFF ); //SNAPSHOT_MODE snapMode);
	DragonVision::GetDragonVision()->AddCamera ( PIntake, RobotElementNames::CAMERA_USAGE::PINTAKE );

	Placer = new DragonPhotonCam ( "Placer", //std::string name,                      /// <I> - network table name
	                               DragonCamera::PIPELINE::OFF, //PIPELINE initialPipeline,              /// <I> enum for starting pipeline
	                               units::length::inch_t ( -13.402 ), //units::length::inch_t mountingXOffset, /// <I> x offset of cam from robot center (forward relative to robot)
	                               units::length::meter_t ( 0 ), //units::length::inch_t mountingYOffset, /// <I> y offset of cam from robot center (left relative to robot)
	                               units::length::inch_t ( 21.607 ), //units::length::inch_t mountingZOffset, /// <I> z offset of cam from robot center (up relative to robot)
	                               units::angle::degree_t ( 19.3 ), //units::angle::degree_t pitch,          /// <I> - Pitch of camera
	                               units::angle::degree_t ( 0 ), //units::angle::degree_t yaw,            /// <I> - Yaw of camera
	                               units::angle::degree_t ( 0 ) ); //units::angle::degree_t roll,           /// <I> - Roll of camera
	DragonVision::GetDragonVision()->AddCamera ( Placer, RobotElementNames::CAMERA_USAGE::PLACER );

	LIntake = new DragonLimelight ( "limelight-lintake", //std::string name,                      /// <I> - network table name
	                                DragonCamera::PIPELINE::OFF, //PIPELINE initialPipeline,              /// <I> enum for starting pipeline
	                                units::length::inch_t ( 12.75 ), //units::length::inch_t mountingXOffset, /// <I> x offset of cam from robot center (forward relative to robot)
	                                units::length::inch_t ( 0.5 ), //units::length::inch_t mountingYOffset, /// <I> y offset of cam from robot center (left relative to robot)
	                                units::length::inch_t ( 13.5 ), //units::length::inch_t mountingZOffset, /// <I> z offset of cam from robot center (up relative to robot)
	                                units::angle::degree_t ( -30 ), //units::angle::degree_t pitch,          /// <I> - Pitch of camera
	                                units::angle::degree_t ( 0 ), //units::angle::degree_t yaw,            /// <I> - Yaw of camera
	                                units::angle::degree_t ( 0 ), //units::angle::degree_t roll,           /// <I> - Roll of camera
	                                DragonLimelight::LED_MODE::LED_OFF, //LED_MODE ledMode,
	                                DragonLimelight::CAM_MODE::CAM_VISION, //CAM_MODE camMode,
	                                DragonLimelight::STREAM_MODE::STREAM_DEFAULT, //STREAM_MODE streamMode,
	                                DragonLimelight::SNAPSHOT_MODE::SNAP_OFF ); //SNAPSHOT_MODE snapMode);
	DragonVision::GetDragonVision()->AddCamera ( LIntake, RobotElementNames::CAMERA_USAGE::LINTAKE );

	Launcher = new DragonPhotonCam ( "Launcher", //std::string name,                      /// <I> - network table name
	                                 DragonCamera::PIPELINE::OFF, //PIPELINE initialPipeline,              /// <I> enum for starting pipeline
	                                 units::length::inch_t ( 7.31 ), //units::length::inch_t mountingXOffset, /// <I> x offset of cam from robot center (forward relative to robot)
	                                 units::length::inch_t ( 9.614 ), //units::length::inch_t mountingYOffset, /// <I> y offset of cam from robot center (left relative to robot)
	                                 units::length::inch_t ( 22.931 ), //units::length::inch_t mountingZOffset, /// <I> z offset of cam from robot center (up relative to robot)
	                                 units::angle::degree_t ( 0 ), //units::angle::degree_t pitch,          /// <I> - Pitch of camera
	                                 units::angle::degree_t ( 1 ), //units::angle::degree_t yaw,            /// <I> - Yaw of camera
	                                 units::angle::degree_t ( 0 ) ); //units::angle::degree_t roll,           /// <I> - Roll of camera
	DragonVision::GetDragonVision()->AddCamera ( Launcher, RobotElementNames::CAMERA_USAGE::LAUNCHER );
}
