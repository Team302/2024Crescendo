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
// This file was automatically generated by the Team 302 code generator version 1.3.0.0
// Generated on Sunday, February 18, 2024 6:27:44 PM

#pragma once
#include "configs/RobotConfig.h"
#include "mechanisms/MechanismTypes.h"
#include "mechanisms/base/StateMgr.h"
#include "mechanisms/noteManager/decoratormods/noteManager.h"
#include "mechanisms/ClimberManager/decoratormods/ClimberManager.h"
#include "DragonVision/DragonLimelight.h"
#include "DragonVision/DragonVision.h"
#include "DragonVision/DragonPhotonCam.h"

class RobotConfigCompBot_302 : public RobotConfig
{
public:
	RobotConfigCompBot_302() = default;
	~RobotConfigCompBot_302() = default;

	StateMgr *GetMechanism ( MechanismTypes::MECHANISM_TYPE mechType );

protected:
	void DefineMechanisms() override;
	void DefineVisionSensors() override;
	void DefineLEDs() override;

private:
	noteManager* m_thenoteManager = nullptr;
	ClimberManager* m_theClimberManager = nullptr;

	std::unordered_map<MechanismTypes::MECHANISM_TYPE, StateMgr *> m_mechanismMap;

	DragonLimelight* PIntake;
	DragonPhotonCam* Placer;
	DragonLimelight* LIntake;
	DragonPhotonCam* Launcher;
};