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

#pragma once

#include "configs/RobotConfig.h"

class RobotConfigMgr
{
public:
	enum RobotIdentifier
	{
		MinimalRobot_302 = 302,
		CHASSISBOT_9998 = 9998,
		practiceBot_9999 = 9999
		MAX_ROBOT_IDENTIFIERS
	};

	static RobotConfigMgr* GetInstance();
	RobotConfig* GetCurrentConfig() const { return m_config; }
	void InitRobot ( RobotIdentifier );

private:
	RobotConfigMgr();
	~RobotConfigMgr() = default;

	static RobotConfigMgr* m_instance;
	RobotConfig* m_config;
};
