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
// This file was automatically generated by the Team 302 code generator version 1.2.3.2
// Generated on Thursday, February 8, 2024 7:01:27 PM

#pragma once
#include "configs/RobotConfig.h"
#include "mechanisms/MechanismTypes.h"
#include "mechanisms/base/StateMgr.h"
#include "DragonVision/DragonPhotonCam.h"

class RobotConfigChassisBot_99979997 : public RobotConfig
{
public:
	RobotConfigChassisBot_99979997() = default;
	~RobotConfigChassisBot_99979997() = default;

	StateMgr *GetMechanism ( MechanismTypes::MECHANISM_TYPE mechType );

protected:
	void DefineMechanisms() override;
	void DefineVisionSensors() override;

private:


	std::unordered_map<MechanismTypes::MECHANISM_TYPE, StateMgr *> m_mechanismMap;

	DragonPhotonCam* Launcher;
	DragonPhotonCam* Placer;
};