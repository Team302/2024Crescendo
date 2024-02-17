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
// This file was automatically generated by the Team 302 code generator version 1.2.3.6
// Generated on Saturday, February 17, 2024 3:19:18 PM

#include <string>

#include "PeriodicLooper.h"
#include "utils/logging/Logger.h"
#include "configs/RobotConfigMgr.h"
#include "configs/RobotConfigThing_1.h"
#include "configs/RobotElementNames.h"


using std::string;

void RobotConfigThing_1::DefineMechanisms()
{
	Logger::GetLogger()->LogData ( LOGGER_LEVEL::PRINT, string ( "Initializing mechanism" ), string ( "Thing1Mech" ), "" );
	Thing1MechGen* Thing1MechGenMech = new Thing1MechGen ( RobotConfigMgr::RobotIdentifier::THING_1 );
	m_theThing1Mech = new Thing1Mech ( Thing1MechGenMech, RobotConfigMgr::RobotIdentifier::THING_1 );
	m_theThing1Mech->Create();
	m_theThing1Mech->Initialize();
	m_theThing1Mech->CreateAndRegisterStates();
	m_theThing1Mech->Init ( m_theThing1Mech );
	m_mechanismMap[MechanismTypes::MECHANISM_TYPE::THING1MECH] = m_theThing1Mech;
}

void RobotConfigThing_1::DefineLEDs()
{


}

StateMgr *RobotConfigThing_1::GetMechanism ( MechanismTypes::MECHANISM_TYPE mechType )
{
	auto itr = m_mechanismMap.find ( mechType );
	if ( itr != m_mechanismMap.end() )
	{
		return itr->second;
	}
	return nullptr;
}

void RobotConfigThing_1::DefineVisionSensors()
{

}
