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
// This file was automatically generated by the Team 302 code generator version 1.2.2.0
// Generated on Saturday, February 3, 2024 11:54:22 AM

#include <string>

#include "PeriodicLooper.h"
#include "utils/logging/Logger.h"
#include "configs/RobotConfigMgr.h"
#include "configs/RobotConfigpracticeBot_9999.h"

using std::string;

void RobotConfigpracticeBot_9999::DefineMechanisms()
{
	Logger::GetLogger()->LogData ( LOGGER_LEVEL::PRINT, string ( "Initializing mechanism" ), string ( "noteManager" ), "" );
	noteManagerGen* noteManagerGenMech = new noteManagerGen();
	m_thenoteManager = new noteManager ( noteManagerGenMech );
	m_thenoteManager->Create();
	m_thenoteManager->Initialize ( RobotConfigMgr::RobotIdentifier::PRACTICE_BOT_9999 );
	m_thenoteManager->createAndRegisterStates();
	m_thenoteManager->Init ( m_thenoteManager );

	Logger::GetLogger()->LogData ( LOGGER_LEVEL::PRINT, string ( "Initializing mechanism" ), string ( "ClimberManager" ), "" );
	ClimberManagerGen* ClimberManagerGenMech = new ClimberManagerGen();
	m_theClimberManager = new ClimberManager ( ClimberManagerGenMech );
	m_theClimberManager->Create();
	m_theClimberManager->Initialize ( RobotConfigMgr::RobotIdentifier::PRACTICE_BOT_9999 );
	m_theClimberManager->createAndRegisterStates();
	m_theClimberManager->Init ( m_theClimberManager );


}
