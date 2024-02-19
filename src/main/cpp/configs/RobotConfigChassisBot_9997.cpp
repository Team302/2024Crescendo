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
// This file was automatically generated by the Team 302 code generator version 1.3.0.1
// Generated on Sunday, February 18, 2024 10:15:34 PM

#include <string>

#include "PeriodicLooper.h"
#include "utils/logging/Logger.h"
#include "configs/RobotConfigMgr.h"
#include "configs/RobotConfigChassisBot_9997.h"
#include "configs/RobotElementNames.h"
#include "DragonVision/DragonPhotonCam.h"
#include "DragonVision/DragonVision.h"

using std::string;

void RobotConfigChassisBot_9997::DefineMechanisms()
{

}

void RobotConfigChassisBot_9997::DefineLEDs()
{


}

StateMgr *RobotConfigChassisBot_9997::GetMechanism ( MechanismTypes::MECHANISM_TYPE mechType )
{
	auto itr = m_mechanismMap.find ( mechType );
	if ( itr != m_mechanismMap.end() )
	{
		return itr->second;
	}
	return nullptr;
}

void RobotConfigChassisBot_9997::DefineVisionSensors()
{
	Launcher = new DragonPhotonCam ( "Launcher", //std::string name,                      /// <I> - network table name
	                                 DragonCamera::PIPELINE::OFF, //PIPELINE initialPipeline,              /// <I> enum for starting pipeline
	                                 units::length::inch_t ( units::length::meter_t ( 0 ) ), //units::length::inch_t mountingXOffset, /// <I> x offset of cam from robot center (forward relative to robot)
	                                 units::length::inch_t ( units::length::meter_t ( 0 ) ), //units::length::inch_t mountingYOffset, /// <I> y offset of cam from robot center (left relative to robot)
	                                 units::length::inch_t ( units::length::meter_t ( 0 ) ), //units::length::inch_t mountingZOffset, /// <I> z offset of cam from robot center (up relative to robot)
	                                 units::angle::degree_t ( units::angle::degree_t ( 0 ) ), //units::angle::degree_t pitch,          /// <I> - Pitch of camera
	                                 units::angle::degree_t ( units::angle::degree_t ( 0 ) ), //units::angle::degree_t yaw,            /// <I> - Yaw of camera
	                                 units::angle::degree_t ( units::angle::degree_t ( 0 ) ) ); //units::angle::degree_t roll,           /// <I> - Roll of camera
	DragonVision::GetDragonVision()->AddCamera ( Launcher, RobotElementNames::CAMERA_USAGE::LAUNCHER );

	Placer = new DragonPhotonCam ( "Placer", //std::string name,                      /// <I> - network table name
	                               DragonCamera::PIPELINE::OFF, //PIPELINE initialPipeline,              /// <I> enum for starting pipeline
	                               units::length::inch_t ( units::length::meter_t ( 0 ) ), //units::length::inch_t mountingXOffset, /// <I> x offset of cam from robot center (forward relative to robot)
	                               units::length::inch_t ( units::length::meter_t ( 0 ) ), //units::length::inch_t mountingYOffset, /// <I> y offset of cam from robot center (left relative to robot)
	                               units::length::inch_t ( units::length::meter_t ( 0 ) ), //units::length::inch_t mountingZOffset, /// <I> z offset of cam from robot center (up relative to robot)
	                               units::angle::degree_t ( units::angle::degree_t ( 0 ) ), //units::angle::degree_t pitch,          /// <I> - Pitch of camera
	                               units::angle::degree_t ( units::angle::degree_t ( 180 ) ), //units::angle::degree_t yaw,            /// <I> - Yaw of camera
	                               units::angle::degree_t ( units::angle::degree_t ( 0 ) ) ); //units::angle::degree_t roll,           /// <I> - Roll of camera
	DragonVision::GetDragonVision()->AddCamera ( Placer, RobotElementNames::CAMERA_USAGE::PLACER );
}
