
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
#include <memory>
#include <string>
#include <utility>
// #include <vector>

// FRC includes

// Team 302 includes

// Third Party Includes
#include <string>
#include <frc/GenericHID.h>
#include <gamepad/IDragonGamepad.h>
#include <gamepad/DragonXBox.h>
#include <gamepad/DragonGamepad.h>
#include "teleopcontrol/TeleopControl.h"
#include <teleopcontrol/TeleopControlFunctions.h>
#include <teleopcontrol/TeleopControlMap.h>
#include <frc/DriverStation.h>
#include "utils/logging/Logger.h"

// using namespace frc;
// using namespace std;

using frc::DriverStation;
using frc::GenericHID;
using std::make_pair;
using std::pair;
using std::string;
using std::vector;

//----------------------------------------------------------------------------------
// Method:      GetInstance
// Description: If there isn't an instance of this class, it will create one.  The
//              single class instance will be returned.
// Returns:     OperatorInterface*  instance of this class
//----------------------------------------------------------------------------------
TeleopControl *TeleopControl::m_instance = nullptr; // initialize the instance variable to nullptr
TeleopControl *TeleopControl::GetInstance()
{
	if (TeleopControl::m_instance == nullptr)
	{
		TeleopControl::m_instance = new TeleopControl();
	}
	if (TeleopControl::m_instance != nullptr && !TeleopControl::m_instance->IsInitialized())
	{
		TeleopControl::m_instance->Initialize();
	}
	return TeleopControl::m_instance;
}
//----------------------------------------------------------------------------------
// Method:      OperatorInterface <<constructor>>
// Description: This will construct and initialize the object.
//              It maps the functions to the buttons/axis.
//---------------------------------------------------------------------------------
TeleopControl::TeleopControl() : m_controller(),
								 m_numControllers(0)

{
	for (auto i = 0; i < DriverStation::kJoystickPorts; ++i)
	{
		m_controller[i] = nullptr;
	}
	Initialize();
}

bool TeleopControl::IsInitialized() const
{
	return m_numControllers > 0;
}
void TeleopControl::Initialize()
{
	InitializeControllers();
}

void TeleopControl::InitializeControllers()
{
	for (int inx = 0; inx < DriverStation::kJoystickPorts; ++inx)
	{
		InitializeController(inx);
	}
}

void TeleopControl::InitializeController(int port)
{

	if (m_controller[port] == nullptr)
	{
		if (DriverStation::GetJoystickIsXbox(port))
		{
			auto xbox = new DragonXBox(port);
			m_controller[port] = xbox;
			m_numControllers++;
		}
		else if (DriverStation::GetJoystickType(port) == GenericHID::kHID1stPerson)
		{
			auto gamepad = new DragonGamepad(port);
			m_controller[port] = gamepad;
			m_numControllers++;
		}

		if (m_controller[port] != nullptr)
		{
			InitializeAxes(port);
			InitializeButtons(port);
		}
	}
}

void TeleopControl::InitializeAxes(int port)
{
	if (m_controller[port] != nullptr)
	{
		auto functions = GetAxisFunctionsOnController(port);
		for (auto function : functions)
		{
			auto itr = teleopControlMapAxisMap.find(function);
			if (itr != teleopControlMapAxisMap.end())
			{
				auto axisInfo = itr->second;
				m_controller[port]->SetAxisDeadband(axisInfo.axisId, axisInfo.deadbandType);
				m_controller[port]->SetAxisProfile(axisInfo.axisId, axisInfo.profile);
				m_controller[port]->SetAxisScale(axisInfo.axisId, axisInfo.scaleFactor);
				m_controller[port]->SetAxisFlipped(axisInfo.axisId, axisInfo.direction != TeleopControlMappingEnums::AXIS_DIRECTION::SYNCED);
			}
		}
	}
}

void TeleopControl::InitializeButtons(int port)
{
	if (m_controller[port] != nullptr)
	{
		auto functions = GetButtonFunctionsOnController(port);
		for (auto function : functions)
		{
			auto itr = teleopControlMapButtonMap.find(function);
			if (itr != teleopControlMapButtonMap.end())
			{
				auto buttonInfo = itr->second;
				if (buttonInfo.mode != TeleopControlMappingEnums::BUTTON_MODE::STANDARD)
				{
					m_controller[port]->SetButtonMode(buttonInfo.buttonId, buttonInfo.mode);
				}
			}
		}
	}
}
vector<TeleopControlFunctions::FUNCTION> TeleopControl::GetAxisFunctionsOnController(int controller)
{
	vector<TeleopControlFunctions::FUNCTION> functions;

	for (auto itr = teleopControlMapAxisMap.begin(); itr != teleopControlMapAxisMap.end(); ++itr)
	{
		if (itr->second.controllerNumber == controller)
		{
			functions.emplace_back(itr->first);
		}
	}
	return functions;
}

vector<TeleopControlFunctions::FUNCTION> TeleopControl::GetButtonFunctionsOnController(int controller)
{
	vector<TeleopControlFunctions::FUNCTION> functions;

	for (auto itr = teleopControlMapButtonMap.begin(); itr != teleopControlMapButtonMap.end(); ++itr)
	{
		if (itr->second.controllerNumber == controller)
		{
			functions.emplace_back(itr->first);
		}
	}
	return functions;
}

pair<IDragonGamepad *, TeleopControlMappingEnums::AXIS_IDENTIFIER> TeleopControl::GetAxisInfo(
	TeleopControlFunctions::FUNCTION function // <I> - controller with this function
)
{
	IDragonGamepad *controller = nullptr;
	TeleopControlMappingEnums::AXIS_IDENTIFIER axis = TeleopControlMappingEnums::AXIS_IDENTIFIER::UNDEFINED_AXIS;

	if (!IsInitialized())
	{
		Initialize();
	}

	auto itr = teleopControlMapAxisMap.find(function);
	if (itr != teleopControlMapAxisMap.end())
	{
		auto axisInfo = itr->second;
		if (m_controller[axisInfo.controllerNumber] != nullptr)
		{
			controller = m_controller[axisInfo.controllerNumber];
			axis = axisInfo.axisId;
		}
	}
	return make_pair(controller, axis);
}

pair<IDragonGamepad *, TeleopControlMappingEnums::BUTTON_IDENTIFIER> TeleopControl::GetButtonInfo(
	TeleopControlFunctions::FUNCTION function // <I> - controller with this function
)
{
	IDragonGamepad *controller = nullptr;
	TeleopControlMappingEnums::BUTTON_IDENTIFIER btn = TeleopControlMappingEnums::UNDEFINED_BUTTON;

	if (!IsInitialized())
	{
		Initialize();
	}

	auto itr = teleopControlMapButtonMap.find(function);
	if (itr != teleopControlMapButtonMap.end())
	{
		auto buttonInfo = itr->second;
		if (m_controller[buttonInfo.controllerNumber] != nullptr)
		{
			controller = m_controller[buttonInfo.controllerNumber];
			btn = buttonInfo.buttonId;
		}
	}
	return make_pair(controller, btn);
}

//------------------------------------------------------------------
// Method:      SetAxisScaleFactor
// Description: Allow the range of values to be set smaller than
//              -1.0 to 1.0.  By providing a scale factor between 0.0
//              and 1.0, the range can be made smaller.  If a value
//              outside the range is provided, then the value will
//              be set to the closest bounding value (e.g. 1.5 will
//              become 1.0)
// Returns:     void
//------------------------------------------------------------------
void TeleopControl::SetAxisScaleFactor(
	TeleopControlFunctions::FUNCTION function, // <I> - function that will update an axis
	double scaleFactor						   // <I> - scale factor used to limit the range
)
{
	auto info = GetAxisInfo(function);
	if (info.first != nullptr && info.second != TeleopControlMappingEnums::AXIS_IDENTIFIER::UNDEFINED_AXIS)
	{
		info.first->SetAxisScale(info.second, scaleFactor);
	}
}

void TeleopControl::SetDeadBand(
	TeleopControlFunctions::FUNCTION function,
	TeleopControlMappingEnums::AXIS_DEADBAND deadband)
{
	auto info = GetAxisInfo(function);
	if (info.first != nullptr && info.second != TeleopControlMappingEnums::AXIS_IDENTIFIER::UNDEFINED_AXIS)
	{
		info.first->SetAxisDeadband(info.second, deadband);
	}
}

//------------------------------------------------------------------
// Method:      SetAxisProfile
// Description: Sets the axis profile for the specifed axis
// Returns:     void
//------------------------------------------------------------------
void TeleopControl::SetAxisProfile(
	TeleopControlFunctions::FUNCTION function,		// <I> - function that will update an axis
	TeleopControlMappingEnums::AXIS_PROFILE profile // <I> - profile to use
)
{
	auto info = GetAxisInfo(function);
	if (info.first != nullptr && info.second != TeleopControlMappingEnums::AXIS_IDENTIFIER::UNDEFINED_AXIS)
	{
		info.first->SetAxisProfile(info.second, profile);
	}
}

//------------------------------------------------------------------
// Method:      GetAxisValue
// Description: Reads the joystick axis, removes any deadband (small
//              value) and then scales as requested.
// Returns:     double   -  scaled axis value
//------------------------------------------------------------------
double TeleopControl::GetAxisValue(
	TeleopControlFunctions::FUNCTION function // <I> - function that whose axis will be read
)
{
	double value = 0.0;
	auto info = GetAxisInfo(function);
	if (info.first != nullptr && info.second != TeleopControlMappingEnums::AXIS_IDENTIFIER::UNDEFINED_AXIS)
	{
		value = info.first->GetAxisValue(info.second);
	}
	return value;
}

//------------------------------------------------------------------
// Method:      IsButtonPressed
// Description: Reads the button value.  Also allows POV, bumpers,
//              and triggers to be treated as buttons.
// Returns:     bool   -  scaled axis value
//------------------------------------------------------------------
bool TeleopControl::IsButtonPressed(
	TeleopControlFunctions::FUNCTION function // <I> - function that whose button will be read
)
{
	bool isSelected = false;
	auto info = GetButtonInfo(function);
	if (info.first != nullptr && info.second != TeleopControlMappingEnums::UNDEFINED_BUTTON)
	{
		isSelected = info.first->IsButtonPressed(info.second);
	}
	return isSelected;
}

void TeleopControl::SetRumble(
	TeleopControlFunctions::FUNCTION function, // <I> - controller with this function
	bool leftRumble,						   // <I> - rumble left
	bool rightRumble						   // <I> - rumble right
)
{

	auto info = GetButtonInfo(function);
	if (info.first != nullptr)
	{
		info.first->SetRumble(leftRumble, rightRumble);
	}
	else
	{
		auto info2 = GetAxisInfo(function);
		if (info2.first != nullptr)
		{
			info2.first->SetRumble(leftRumble, rightRumble);
		}
	}
}

void TeleopControl::SetRumble(
	int controller,	 // <I> - controller to rumble
	bool leftRumble, // <I> - rumble left
	bool rightRumble // <I> - rumble right
)
{
	if (m_controller[controller] != nullptr)
	{
		m_controller[controller]->SetRumble(leftRumble, rightRumble);
	}
}

void TeleopControl::LogInformation()
{
	auto self = const_cast<TeleopControl *>(this);
	for (int inx = 0; inx < DriverStation::kJoystickPorts; ++inx)
	{
		if (m_controller[inx] != nullptr)
		{
			auto functions = self->GetAxisFunctionsOnController(inx);
			for (auto function : functions)
			{
				Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("TeleopControl-axis"), std::to_string(function), self->GetAxisValue(function));
			}

			functions.clear();
			functions = self->GetButtonFunctionsOnController(inx);
			for (auto function : functions)
			{
				Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("TeleopControl-button"), std::to_string(function), self->IsButtonPressed(function));
			}
		}
	}
}
