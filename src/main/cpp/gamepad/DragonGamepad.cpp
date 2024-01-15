
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

#include <string>

#include <frc/GenericHID.h>
#include <frc/Joystick.h>

#include <gamepad/axis/AnalogAxis.h>
#include <gamepad/axis/IDeadband.h>
#include <gamepad/axis/IProfile.h>
#include <gamepad/button/AnalogButton.h>
#include <gamepad/button/DigitalButton.h>
#include <gamepad/button/ToggleButton.h>
#include <gamepad/DragonGamepad.h>
#include <teleopcontrol/TeleopControlMappingEnums.h>

#include "utils/logging/Logger.h"

using namespace std;
using namespace frc;

DragonGamepad::DragonGamepad(
    int port) : m_gamepad(new Joystick(port)),
                m_axis(),
                m_axisScale(),
                m_axisInversionFactor(),
                m_axisProfile(),
                // m_button(),
                m_button()
{
    // device type is 24
    // 8 axis
    // 11 buttons
    m_axis.resize(TeleopControlMappingEnums::MAX_AXIS);
    m_axisScale.resize(TeleopControlMappingEnums::MAX_AXIS);
    m_axisInversionFactor.resize(TeleopControlMappingEnums::MAX_AXIS);
    m_axisProfile.resize(TeleopControlMappingEnums::MAX_AXIS);

    for (auto inx = 0; inx < TeleopControlMappingEnums::MAX_AXIS; ++inx)
    {
        m_axisScale[inx] = 1.0;
        m_axisInversionFactor[inx] = 1.0;
        m_axisProfile[inx] = TeleopControlMappingEnums::AXIS_PROFILE::LINEAR;
        m_axis[inx] = nullptr;
    }

    // Create Axis objects
    m_axis[TeleopControlMappingEnums::GAMEPAD_AXIS_16] = new AnalogAxis(m_gamepad, LEFT_JOYSTICK, false);
    m_axis[TeleopControlMappingEnums::GAMEPAD_AXIS_16]->SetDeadBand(TeleopControlMappingEnums::AXIS_DEADBAND::NONE);
    m_axis[TeleopControlMappingEnums::GAMEPAD_AXIS_16]->SetAxisScaleFactor(JOYSTICK_SCALE);

    m_axis[TeleopControlMappingEnums::GAMEPAD_AXIS_17] = new AnalogAxis(m_gamepad, RIGHT_JOYSTICK, false);
    m_axis[TeleopControlMappingEnums::GAMEPAD_AXIS_17]->SetDeadBand(TeleopControlMappingEnums::AXIS_DEADBAND::NONE);
    m_axis[TeleopControlMappingEnums::GAMEPAD_AXIS_17]->SetAxisScaleFactor(JOYSTICK_SCALE);

    m_axis[TeleopControlMappingEnums::LEFT_ANALOG_BUTTON_AXIS] = new AnalogAxis(m_gamepad, LEFT_BUTTON_AXIS_ID, false);
    m_axis[TeleopControlMappingEnums::LEFT_ANALOG_BUTTON_AXIS]->SetDeadBand(TeleopControlMappingEnums::AXIS_DEADBAND::NONE);

    m_axis[TeleopControlMappingEnums::RIGHT_ANALOG_BUTTON_AXIS] = new AnalogAxis(m_gamepad, RIGHT_BUTTON_AXIS_ID, false);
    m_axis[TeleopControlMappingEnums::RIGHT_ANALOG_BUTTON_AXIS]->SetDeadBand(TeleopControlMappingEnums::AXIS_DEADBAND::NONE);

    m_axis[TeleopControlMappingEnums::DIAL_ANALOG_BUTTON_AXIS] = new AnalogAxis(m_gamepad, DIAL_BUTTON_AXIS_ID, false);
    m_axis[TeleopControlMappingEnums::DIAL_ANALOG_BUTTON_AXIS]->SetDeadBand(TeleopControlMappingEnums::AXIS_DEADBAND::NONE);

    // Create Button objects
    m_button.resize(TeleopControlMappingEnums::MAX_BUTTONS);
    for (auto inx = 0; inx < TeleopControlMappingEnums::MAX_BUTTONS; ++inx)
    {
        m_button[inx] = nullptr;
    }

    m_button[TeleopControlMappingEnums::GAMEPAD_BUTTON_1] = new AnalogButton(m_axis[TeleopControlMappingEnums::LEFT_ANALOG_BUTTON_AXIS], BUTTON_1_LOWERBOUND, BUTTON_1_UPPERBOUND);
    m_button[TeleopControlMappingEnums::GAMEPAD_BUTTON_3] = new AnalogButton(m_axis[TeleopControlMappingEnums::LEFT_ANALOG_BUTTON_AXIS], BUTTON_3_LOWERBOUND, BUTTON_3_UPPERBOUND);
    m_button[TeleopControlMappingEnums::GAMEPAD_BUTTON_6] = new AnalogButton(m_axis[TeleopControlMappingEnums::LEFT_ANALOG_BUTTON_AXIS], BUTTON_6_LOWERBOUND, BUTTON_6_UPPERBOUND);
    m_button[TeleopControlMappingEnums::GAMEPAD_BUTTON_8] = new AnalogButton(m_axis[TeleopControlMappingEnums::LEFT_ANALOG_BUTTON_AXIS], BUTTON_8_LOWERBOUND, BUTTON_8_UPPERBOUND);
    m_button[TeleopControlMappingEnums::GAMEPAD_BUTTON_10] = new AnalogButton(m_axis[TeleopControlMappingEnums::LEFT_ANALOG_BUTTON_AXIS], BUTTON_10_LOWERBOUND, BUTTON_10_UPPERBOUND);
    m_button[TeleopControlMappingEnums::GAMEPAD_BUTTON_12] = new AnalogButton(m_axis[TeleopControlMappingEnums::LEFT_ANALOG_BUTTON_AXIS], BUTTON_12_LOWERBOUND, BUTTON_12_UPPERBOUND);
    m_button[TeleopControlMappingEnums::GAMEPAD_BUTTON_2] = new AnalogButton(m_axis[TeleopControlMappingEnums::RIGHT_ANALOG_BUTTON_AXIS], BUTTON_2_LOWERBOUND, BUTTON_2_UPPERBOUND);
    m_button[TeleopControlMappingEnums::GAMEPAD_BUTTON_4] = new AnalogButton(m_axis[TeleopControlMappingEnums::RIGHT_ANALOG_BUTTON_AXIS], BUTTON_4_LOWERBOUND, BUTTON_4_UPPERBOUND);
    m_button[TeleopControlMappingEnums::GAMEPAD_BUTTON_5] = new AnalogButton(m_axis[TeleopControlMappingEnums::RIGHT_ANALOG_BUTTON_AXIS], BUTTON_5_LOWERBOUND, BUTTON_5_UPPERBOUND);
    m_button[TeleopControlMappingEnums::GAMEPAD_BUTTON_7] = new AnalogButton(m_axis[TeleopControlMappingEnums::RIGHT_ANALOG_BUTTON_AXIS], BUTTON_7_LOWERBOUND, BUTTON_7_UPPERBOUND);
    m_button[TeleopControlMappingEnums::GAMEPAD_BUTTON_9] = new AnalogButton(m_axis[TeleopControlMappingEnums::RIGHT_ANALOG_BUTTON_AXIS], BUTTON_9_LOWERBOUND, BUTTON_9_UPPERBOUND);
    m_button[TeleopControlMappingEnums::GAMEPAD_BUTTON_11] = new AnalogButton(m_axis[TeleopControlMappingEnums::RIGHT_ANALOG_BUTTON_AXIS], BUTTON_11_LOWERBOUND, BUTTON_11_UPPERBOUND);
    m_button[TeleopControlMappingEnums::GAMEPAD_BUTTON_13] = new AnalogButton(m_axis[TeleopControlMappingEnums::RIGHT_ANALOG_BUTTON_AXIS], BUTTON_13_LOWERBOUND, BUTTON_13_UPPERBOUND);
    // m_button[GAMEPAD_BIG_RED_BUTTON] = new AnalogButton(m_gamepad, GAMEPAD_BIG_RED_BUTTON,);
    m_button[TeleopControlMappingEnums::GAMEPAD_DIAL_22] = new AnalogButton(m_axis[TeleopControlMappingEnums::DIAL_ANALOG_BUTTON_AXIS], BUTTON_22_LOWERBOUND, BUTTON_22_UPPERBOUND);
    m_button[TeleopControlMappingEnums::GAMEPAD_DIAL_23] = new AnalogButton(m_axis[TeleopControlMappingEnums::DIAL_ANALOG_BUTTON_AXIS], BUTTON_23_LOWERBOUND, BUTTON_23_UPPERBOUND);
    m_button[TeleopControlMappingEnums::GAMEPAD_DIAL_24] = new AnalogButton(m_axis[TeleopControlMappingEnums::DIAL_ANALOG_BUTTON_AXIS], BUTTON_24_LOWERBOUND, BUTTON_24_UPPERBOUND);
    m_button[TeleopControlMappingEnums::GAMEPAD_DIAL_25] = new AnalogButton(m_axis[TeleopControlMappingEnums::DIAL_ANALOG_BUTTON_AXIS], BUTTON_25_LOWERBOUND, BUTTON_25_UPPERBOUND);
    m_button[TeleopControlMappingEnums::GAMEPAD_DIAL_26] = new AnalogButton(m_axis[TeleopControlMappingEnums::DIAL_ANALOG_BUTTON_AXIS], BUTTON_26_LOWERBOUND, BUTTON_26_UPPERBOUND);
    m_button[TeleopControlMappingEnums::GAMEPAD_DIAL_27] = new AnalogButton(m_axis[TeleopControlMappingEnums::DIAL_ANALOG_BUTTON_AXIS], BUTTON_27_LOWERBOUND, BUTTON_27_UPPERBOUND);

    m_button[TeleopControlMappingEnums::GAMEPAD_SWITCH_18] = new DigitalButton(m_gamepad, SWITCH_18_DIGITAL_ID);
    m_button[TeleopControlMappingEnums::GAMEPAD_SWITCH_19] = new DigitalButton(m_gamepad, SWITCH_19_DIGITAL_ID);
    m_button[TeleopControlMappingEnums::GAMEPAD_SWITCH_20] = new DigitalButton(m_gamepad, SWITCH_20_DIGITAL_ID);
    m_button[TeleopControlMappingEnums::GAMEPAD_SWITCH_21] = new DigitalButton(m_gamepad, SWITCH_21_DIGITAL_ID);
    m_button[TeleopControlMappingEnums::GAMEPAD_BUTTON_14_UP] = new DigitalButton(m_gamepad, LEVER_14_UP_DIGITAL_ID);
    m_button[TeleopControlMappingEnums::GAMEPAD_BUTTON_14_DOWN] = new DigitalButton(m_gamepad, LEVER_14_DOWN_DIGITAL_ID);
    m_button[TeleopControlMappingEnums::GAMEPAD_BUTTON_15_UP] = new DigitalButton(m_gamepad, LEVER_15_UP_DIGITAL_ID);
    m_button[TeleopControlMappingEnums::GAMEPAD_BUTTON_15_DOWN] = new DigitalButton(m_gamepad, LEVER_15_DOWN_DIGITAL_ID);
}

DragonGamepad::~DragonGamepad()
{
    delete m_gamepad;
    m_gamepad = nullptr;
}

bool DragonGamepad::IsButtonPressed(
    TeleopControlMappingEnums::BUTTON_IDENTIFIER button) const
{
    if (m_button[button] != nullptr)
    {
        return m_button[button]->IsButtonPressed();
    }
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT_ONCE, string("IsButtonPressed"), to_string(button), string("button is Nullptr"));

    return false;
}

void DragonGamepad::SetButtonMode(
    TeleopControlMappingEnums::BUTTON_IDENTIFIER button,
    TeleopControlMappingEnums::BUTTON_MODE mode)
{
    if (m_button[button] != nullptr)
    {
        if (mode == TeleopControlMappingEnums::BUTTON_MODE::TOGGLE)
        {
            auto btn = new ToggleButton(m_button[button]);
            m_button[button] = btn;
        }
        else
        {
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT_ONCE, string("SetButtonMode"), to_string(button), string("button is Nullptr"));
        }
        // TODO: should have else to re-create the button or remove the toggle decorator
    }
}

bool DragonGamepad::WasButtonPressed(
    TeleopControlMappingEnums::BUTTON_IDENTIFIER button) const
{
    if (m_button[button] != nullptr)
    {
        return m_button[button]->WasButtonPressed();
    }
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT_ONCE, string("WasButtonPressed"), to_string(button), string("button is Nullptr"));
    return false;
}

bool DragonGamepad::WasButtonReleased(
    TeleopControlMappingEnums::BUTTON_IDENTIFIER button) const
{
    if (m_button[button] != nullptr)
    {
        return m_button[button]->WasButtonReleased();
    }
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT_ONCE, string("WasButtonPressed"), to_string(button), string("button is Nullptr"));
    return false;
}

double DragonGamepad::GetAxisValue(
    TeleopControlMappingEnums::AXIS_IDENTIFIER axis) const
{
    if (m_axis[axis] != nullptr)
    {
        auto value = m_axis[axis]->GetAxisValue();
        if (axis == TeleopControlMappingEnums::GAMEPAD_AXIS_16 || axis == TeleopControlMappingEnums::GAMEPAD_AXIS_17)
        {
            value -= JOYSTICK_OFFSET;
        }
        return value;
    }
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT_ONCE, string("GetAxisValue"), to_string(axis), string("axis is Nullptr"));
    return 0.0;
}

void DragonGamepad::SetAxisDeadband(
    TeleopControlMappingEnums::AXIS_IDENTIFIER axis,
    TeleopControlMappingEnums::AXIS_DEADBAND type)
{
    if (m_axis[axis] != nullptr)
    {
        m_axis[axis]->SetDeadBand(type);
    }
    else
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT_ONCE, string("SetAxisDeadband"), to_string(axis), string("axis is Nullptr"));
    }
}

void DragonGamepad::SetAxisProfile(
    TeleopControlMappingEnums::AXIS_IDENTIFIER axis,
    TeleopControlMappingEnums::AXIS_PROFILE curve)
{
    if (m_axis[axis] != nullptr)
    {
        m_axis[axis]->SetAxisProfile(curve);
    }
    else
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT_ONCE, string("SetAxisProfile"), to_string(axis), string("axis is Nullptr"));
    }
}

void DragonGamepad::SetAxisScale(
    TeleopControlMappingEnums::AXIS_IDENTIFIER axis,
    double scaleFactor)
{
    if (m_axis[axis] != nullptr)
    {
        m_axis[axis]->SetAxisScaleFactor(scaleFactor);
    }
    else
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT_ONCE, string("SetAxisScale"), to_string(axis), string("axis is Nullptr"));
    }
}

void DragonGamepad::SetAxisFlipped(
    TeleopControlMappingEnums::AXIS_IDENTIFIER axis, /// <I> - axis to modify
    bool isInverted                                  /// <I> - deadband option
)
{
    if (m_axis[axis] != nullptr)
    {
        m_axis[axis]->SetInverted(isInverted);
    }
    else
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT_ONCE, string("SetAxisFlipped"), to_string(axis), string("axis is Nullptr"));
    }
}

void DragonGamepad::SetRumble(
    bool leftRumble, // <I> - rumble left
    bool rightRumble // <I> - rumble right
) const
{
    if (m_gamepad != nullptr)
    {
        double lrum = leftRumble ? 1.0 : 0.0;
        double rrum = rightRumble ? 1.0 : 0.0;

        m_gamepad->SetRumble(GenericHID::RumbleType::kLeftRumble, lrum);
        m_gamepad->SetRumble(GenericHID::RumbleType::kRightRumble, rrum);
    }
    else
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT_ONCE, string("SetRumble"), "Game Pad", string("Nullptr"));
    }
}