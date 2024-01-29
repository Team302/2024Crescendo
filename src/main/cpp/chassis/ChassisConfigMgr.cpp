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
// Generated on Thursday, January 25, 2024 8:27:55 PM

#include <string>

#include "utils/logging/Logger.h"
#include "chassis/ChassisConfigMgr.h"
#include "chassis/ChassisConfig.h"
#include "chassis/ChassisConfigChassis_9998.h"

using namespace std;

ChassisConfigMgr *ChassisConfigMgr::m_instance = nullptr;
ChassisConfigMgr *ChassisConfigMgr::GetInstance()
{
	if ( ChassisConfigMgr::m_instance == nullptr )
	{
		ChassisConfigMgr::m_instance = new ChassisConfigMgr();
	}
	return ChassisConfigMgr::m_instance;
}

ChassisConfigMgr::ChassisConfigMgr() : m_config ( nullptr )
{
}

void ChassisConfigMgr::InitChassis( RobotConfigMgr::RobotIdentifier id )
{
	switch ( id )
	{
	case RobotConfigMgr::RobotIdentifier::CHASSISBOT_9998:
		Logger::GetLogger()->LogData ( LOGGER_LEVEL::PRINT, string ( "Initializing chassis " ), string ( "CHASSISBOT_9998" ), string ( "Yes" ) );
		m_config = new ChassisConfigChassis_9998();
		break;

	default:
		Logger::GetLogger()->LogData ( LOGGER_LEVEL::PRINT, string ( "Skipping chassis initialization because of unknown robot id " ), string ( "" ), id );
		break;
	}

	if ( m_config != nullptr )
	{
		m_config->BuildChassis();
		Logger::GetLogger()->LogData ( LOGGER_LEVEL::PRINT, string ( "Initialization completed for robot " ), string ( "" ), id );
	}
}
