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
// This file was automatically generated by the Team 302 code generator version 1.3.0.11
// Generated on Saturday, March 2, 2024 6:50:11 AM

#include <string>

#include "PeriodicLooper.h"
#include "utils/logging/Logger.h"
#include "configs/RobotConfigMgr.h"
#include "configs/RobotConfigCompBot_302.h"
#include "configs/RobotElementNames.h"
#include "DragonVision/DragonPhotonCam.h"
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
	Launcher = new DragonPhotonCam ( "Launcher", //std::string name,                      /// <I> - network table name
	                                 DragonCamera::PIPELINE::OFF, //PIPELINE initialPipeline,              /// <I> enum for starting pipeline
	                                 units::length::inch_t ( units::length::inch_t ( -7.673 ) ), //units::length::inch_t mountingXOffset, /// <I> x offset of cam from robot center (forward relative to robot)
	                                 units::length::inch_t ( units::length::inch_t ( 9.614 ) ), //units::length::inch_t mountingYOffset, /// <I> y offset of cam from robot center (left relative to robot)
	                                 units::length::inch_t ( units::length::inch_t ( 24.664 ) ), //units::length::inch_t mountingZOffset, /// <I> z offset of cam from robot center (up relative to robot)
	                                 units::angle::degree_t ( units::angle::degree_t ( -5 ) ), //units::angle::degree_t pitch,          /// <I> - Pitch of camera
	                                 units::angle::degree_t ( units::angle::degree_t ( 0 ) ), //units::angle::degree_t yaw,            /// <I> - Yaw of camera
	                                 units::angle::degree_t ( units::angle::degree_t ( 0 ) ) ); //units::angle::degree_t roll,           /// <I> - Roll of camera
	DragonVision::GetDragonVision()->AddCamera ( Launcher, RobotElementNames::CAMERA_USAGE::LAUNCHER );
}
