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

// C++ Includes
#include <map>
#include <memory>
#include <string>

// FRC includes

// Team 302 includes
#include "configs/usages/MotorControllerUsage.h"
#include "utils/logging/Logger.h"

// Third Party Includes

using namespace std;

MotorControllerUsage *MotorControllerUsage::m_instance = nullptr;
MotorControllerUsage *MotorControllerUsage::GetInstance()
{
	if (m_instance == nullptr)
	{
		m_instance = new MotorControllerUsage();
	}
	return m_instance;
}

MotorControllerUsage::MotorControllerUsage()
{

	m_usageMap["LEFT_FRONT_SWERVE_DRIVE"] = MOTOR_CONTROLLER_USAGE::LEFT_FRONT_SWERVE_DRIVE;
	m_usageMap["LEFT_FRONT_SWERVE_TURN"] = MOTOR_CONTROLLER_USAGE::LEFT_FRONT_SWERVE_TURN;
	m_usageMap["RIGHT_FRONT_SWERVE_DRIVE"] = MOTOR_CONTROLLER_USAGE::RIGHT_FRONT_SWERVE_DRIVE;
	m_usageMap["RIGHT_FRONT_SWERVE_TURN"] = MOTOR_CONTROLLER_USAGE::RIGHT_FRONT_SWERVE_TURN;
	m_usageMap["LEFT_BACK_SWERVE_DRIVE"] = MOTOR_CONTROLLER_USAGE::LEFT_BACK_SWERVE_DRIVE;
	m_usageMap["LEFT_BACK_SWERVE_TURN"] = MOTOR_CONTROLLER_USAGE::LEFT_BACK_SWERVE_TURN;
	m_usageMap["RIGHT_BACK_SWERVE_DRIVE"] = MOTOR_CONTROLLER_USAGE::RIGHT_BACK_SWERVE_DRIVE;
	m_usageMap["RIGHT_BACK_SWERVE_TURN"] = MOTOR_CONTROLLER_USAGE::RIGHT_BACK_SWERVE_TURN;
	m_usageMap["ARM"] = MOTOR_CONTROLLER_USAGE::ARM;
	m_usageMap["Extender"] = MOTOR_CONTROLLER_USAGE::Extender;
	m_usageMap["INTAKE1"] = MOTOR_CONTROLLER_USAGE::INTAKE1;
	m_usageMap["INTAKE2"] = MOTOR_CONTROLLER_USAGE::INTAKE2;
	m_usageMap["EXAMPLE_MOTOR1"] = MOTOR_CONTROLLER_USAGE::EXAMPLE_MOTOR1;
	m_usageMap["EXAMPLE_MOTOR2"] = MOTOR_CONTROLLER_USAGE::EXAMPLE_MOTOR2;
}

MotorControllerUsage::~MotorControllerUsage()
{
	m_usageMap.clear();
}

MotorControllerUsage::MOTOR_CONTROLLER_USAGE MotorControllerUsage::GetUsage(
	string usageString)
{
	auto it = m_usageMap.find(usageString);
	if (it != m_usageMap.end())
	{
		return it->second;
	}
	Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("MotorControllerUsage::GetUsage"), string("unknown usage"), usageString);
	return MotorControllerUsage::MOTOR_CONTROLLER_USAGE::UNKNOWN_MOTOR_CONTROLLER_USAGE;
}

std::string MotorControllerUsage::GetUsage(MotorControllerUsage::MOTOR_CONTROLLER_USAGE usage)
{
	for (auto thisUsage : m_usageMap)
	{
		if (thisUsage.second == usage)
		{
			return thisUsage.first;
		}
	}
	return string("");
}