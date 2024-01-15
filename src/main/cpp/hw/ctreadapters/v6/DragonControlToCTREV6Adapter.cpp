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
#include <string>

// FRC includes

// Team 302 includes
#include "hw/interfaces/IDragonControlToVendorControlAdapter.h"
#include "hw/ctreadapters/v6/DragonControlToCTREV6Adapter.h"
#include "mechanisms/controllers/ControlData.h"
#include "mechanisms/controllers/ControlModes.h"
#include "utils/logging/Logger.h"
#include "hw/DragonTalonFX.h"

// Third Party Includes

using std::string;
using std::to_string;

DragonControlToCTREV6Adapter::DragonControlToCTREV6Adapter(string networkTableName,
															 int controllerSlot,
															 const ControlData &controlInfo,
															 const DistanceAngleCalcStruc &calcStruc,
															 ctre::phoenix6::hardware::TalonFX &controller) : IDragonControlToVendorControlAdapter(), m_networkTableName(networkTableName),
																												m_controllerSlot(controllerSlot),
																												m_controlData(controlInfo),
																												m_calcStruc(calcStruc),
																												m_controller(controller),
																												m_isDuty(controlInfo.GetFType() == ControlData::FEEDFORWARD_TYPE::DUTY_CYCLE),
																												m_dutyFeedForward(controlInfo.GetF()),
																												m_isVoltage(controlInfo.GetFType() == ControlData::FEEDFORWARD_TYPE::VOLTAGE),
																												m_voltageFeedForward(units::voltage::volt_t(controlInfo.GetF())),
																												m_isTorque(controlInfo.GetFType() == ControlData::FEEDFORWARD_TYPE::TORQUE_CURRENT),
																												m_torqueCurrentFeedForward(units::current::ampere_t(controlInfo.GetF())),
																												m_enableFOC(controlInfo.IsFOCEnabled())

{
	SetPeakAndNominalValues(networkTableName, controlInfo);

	if (controlInfo.GetMode() == ControlModes::CONTROL_TYPE::POSITION_ABS_TICKS ||
		controlInfo.GetMode() == ControlModes::CONTROL_TYPE::POSITION_DEGREES ||
		controlInfo.GetMode() == ControlModes::CONTROL_TYPE::POSITION_DEGREES_ABSOLUTE ||
		controlInfo.GetMode() == ControlModes::CONTROL_TYPE::POSITION_INCH ||
		controlInfo.GetMode() == ControlModes::CONTROL_TYPE::VELOCITY_DEGREES ||
		controlInfo.GetMode() == ControlModes::CONTROL_TYPE::VELOCITY_INCH ||
		controlInfo.GetMode() == ControlModes::CONTROL_TYPE::VELOCITY_RPS ||
		controlInfo.GetMode() == ControlModes::CONTROL_TYPE::VOLTAGE ||
		controlInfo.GetMode() == ControlModes::CONTROL_TYPE::TRAPEZOID)
	{
		SetPIDConstants(networkTableName, controllerSlot, controlInfo);
	}

	if ( // controlInfo.GetMode() == ControlModes::CONTROL_TYPE::POSITION_ABS_TICKS ||
		controlInfo.GetMode() == ControlModes::CONTROL_TYPE::POSITION_DEGREES_ABSOLUTE ||
		controlInfo.GetMode() == ControlModes::CONTROL_TYPE::TRAPEZOID)
	{
		SetMaxVelocityAcceleration(networkTableName, controlInfo);
	}
}

void DragonControlToCTREV6Adapter::InitializeDefaults()
{
	m_controller.GetConfigurator().Apply(ctre::phoenix6::configs::TalonFXConfiguration{});
}

string DragonControlToCTREV6Adapter::GetErrorPrompt() const
{
	auto prompt = string("CTRE CAN motor controller ");
	prompt += to_string(m_controller.GetDeviceID());
	return prompt;
}

void DragonControlToCTREV6Adapter::SetPeakAndNominalValues(std::string networkTableName,
															const ControlData &controlInfo)
{
	// TODO:  Implement Phoenix Pro methods
}

void DragonControlToCTREV6Adapter::SetMaxVelocityAcceleration(string networkTableName,
															   const ControlData &controlInfo)
{
	// TODO:  Implement Phoenix Pro methods
}

void DragonControlToCTREV6Adapter::SetPIDConstants(std::string networkTableName,
													int controllerSlot,
													const ControlData &controlInfo)
{
	// TODO:  Implement Phoenix Pro methods
}