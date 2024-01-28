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
// Generated on Sunday, January 28, 2024 11:23:10 AM

#pragma once

#include <string>
#include <memory>

// FRC Includes
#include <networktables/NetworkTable.h>

#include "mechanisms/base/BaseMech.h"
#include "configs/RobotConfigMgr.h"

#include "hw/DragonTalonSRX.h"

class Thing1Equipment : public BaseMech
{
public:
	Thing1Equipment() = delete;
	~Thing1Equipment() = default;
	Thing1Equipment ( MechanismTypes::MECHANISM_TYPE type, std::string networkTableName );

	virtual void Initialize ( RobotConfigMgr::RobotIdentifier robotFullName ) = 0;
	void Cyclic();

	DragonTalonSRX* TalonSRX_1;
	ControlData* percentControlData;
//state* state_1;

protected:
	std::string m_ntName;
	std::string m_tuningIsEnabledStr;
	bool m_tuning = false;
	std::shared_ptr<nt::NetworkTable> m_table;

private:
	void CheckForTuningEnabled();
	void ReadTuningParamsFromNT();
	void PushTuningParamsToNT();

};