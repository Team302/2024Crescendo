
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

#include <map>
#include <string>
#include <utility>

#include <frc/GenericHID.h>
#include <frc/XboxController.h>

#include <gamepad/axis/AnalogAxis.h>
#include <gamepad/axis/IDeadband.h>
#include <gamepad/axis/IProfile.h>
#include <gamepad/button/AnalogButton.h>
#include <gamepad/button/ButtonDecorator.h>
#include <gamepad/button/DigitalButton.h>
#include <gamepad/button/POVButton.h>
#include <gamepad/button/ToggleButton.h>
#include <gamepad/DragonXBox.h>
#include "utils/logging/Logger.h"
#include <teleopcontrol/TeleopControlMappingEnums.h>

using namespace std;
using namespace frc;

/// @brief Wrapper for an XBOX controller used to control the robot in teleop mode.

DragonXBox::DragonXBox(
    int port) : m_xbox(new frc::XboxController(port))
{
    // Create Axis Objects
    m_axis[TeleopControlMappingEnums::LEFT_JOYSTICK_X] = new AnalogAxis(m_xbox, XboxController::Axis::kLeftX, false);
    m_axis[TeleopControlMappingEnums::LEFT_JOYSTICK_Y] = new AnalogAxis(m_xbox, XboxController::Axis::kLeftY, true);
    // m_axis[TeleopControlMappingEnums::LEFT_JOYSTICK_X]->DefinePerpendicularAxis(m_axis[TeleopControlMappingEnums::LEFT_JOYSTICK_Y]);
    // m_axis[TeleopControlMappingEnums::LEFT_JOYSTICK_Y]->DefinePerpendicularAxis(m_axis[TeleopControlMappingEnums::LEFT_JOYSTICK_X]);

    m_axis[TeleopControlMappingEnums::LEFT_TRIGGER] = new AnalogAxis(m_xbox, XboxController::Axis::kLeftTrigger, false);
    m_axis[TeleopControlMappingEnums::RIGHT_TRIGGER] = new AnalogAxis(m_xbox, XboxController::Axis::kRightTrigger, false);

    m_axis[TeleopControlMappingEnums::RIGHT_JOYSTICK_X] = new AnalogAxis(m_xbox, XboxController::Axis::kRightX, false);
    m_axis[TeleopControlMappingEnums::RIGHT_JOYSTICK_Y] = new AnalogAxis(m_xbox, XboxController::Axis::kRightY, true);
    m_axis[TeleopControlMappingEnums::RIGHT_JOYSTICK_X]->DefinePerpendicularAxis(m_axis[TeleopControlMappingEnums::RIGHT_JOYSTICK_Y]);
    m_axis[TeleopControlMappingEnums::RIGHT_JOYSTICK_Y]->DefinePerpendicularAxis(m_axis[TeleopControlMappingEnums::RIGHT_JOYSTICK_X]);

    // Create DigitalButton Objects for the physical buttons
    m_button[TeleopControlMappingEnums::A_BUTTON] = new DigitalButton(m_xbox, XboxController::Button::kA);
    m_button[TeleopControlMappingEnums::B_BUTTON] = new DigitalButton(m_xbox, XboxController::Button::kB);
    m_button[TeleopControlMappingEnums::X_BUTTON] = new DigitalButton(m_xbox, XboxController::Button::kX);
    m_button[TeleopControlMappingEnums::Y_BUTTON] = new DigitalButton(m_xbox, XboxController::Button::kY);
    m_button[TeleopControlMappingEnums::LEFT_BUMPER] = new DigitalButton(m_xbox, XboxController::Button::kLeftBumper);
    m_button[TeleopControlMappingEnums::RIGHT_BUMPER] = new DigitalButton(m_xbox, XboxController::Button::kRightBumper);
    m_button[TeleopControlMappingEnums::SELECT_BUTTON] = new DigitalButton(m_xbox, XboxController::Button::kBack);
    m_button[TeleopControlMappingEnums::START_BUTTON] = new DigitalButton(m_xbox, XboxController::Button::kStart);
    m_button[TeleopControlMappingEnums::LEFT_STICK_PRESSED] = new DigitalButton(m_xbox, XboxController::Button::kLeftStick);
    m_button[TeleopControlMappingEnums::RIGHT_STICK_PRESSED] = new DigitalButton(m_xbox, XboxController::Button::kRightStick);

    // Create AnalogButton Objects for the triggers
    m_button[TeleopControlMappingEnums::LEFT_TRIGGER_PRESSED] = new AnalogButton(m_axis[TeleopControlMappingEnums::LEFT_TRIGGER]);
    m_button[TeleopControlMappingEnums::RIGHT_TRIGGER_PRESSED] = new AnalogButton(m_axis[TeleopControlMappingEnums::RIGHT_TRIGGER]);

    // Create POVButton Objects for the POV

    m_button[TeleopControlMappingEnums::POV_0] = new POVButton(m_xbox, 0);
    m_button[TeleopControlMappingEnums::POV_45] = new POVButton(m_xbox, 45);
    m_button[TeleopControlMappingEnums::POV_90] = new POVButton(m_xbox, 90);
    m_button[TeleopControlMappingEnums::POV_135] = new POVButton(m_xbox, 135);
    m_button[TeleopControlMappingEnums::POV_180] = new POVButton(m_xbox, 180);
    m_button[TeleopControlMappingEnums::POV_225] = new POVButton(m_xbox, 225);
    m_button[TeleopControlMappingEnums::POV_270] = new POVButton(m_xbox, 270);
    m_button[TeleopControlMappingEnums::POV_315] = new POVButton(m_xbox, 315);
}

DragonXBox::~DragonXBox()
{
    delete m_xbox;
    m_xbox = nullptr;
}

///-------------------------------------------------------------------------------------------------
/// Method:      IsButtonPressed
/// Description: Return whether the requested button is selected (true) or not (false)
/// Returns:     bool    true  - button is pressed
///              false - button is not pressed
///-------------------------------------------------------------------------------------------------
bool DragonXBox::IsButtonPressed(
    TeleopControlMappingEnums::BUTTON_IDENTIFIER button // <I> - button to check
) const
{
    if (m_button[button] != nullptr)
    {
        return m_button[button]->IsButtonPressed();
    }
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT_ONCE, string("DragonXBox::IsButtonPressed"), to_string(button), string("button is Nullptr"));
    return false;
}

//==================================================================================
/// <summary>
/// Method:         WasButtonReleased
/// Description:    Read whether the button was released since the last query.  This
///                 is only valid for digital buttons (normal buttons and bumpers).
/// </summary>
//==================================================================================
bool DragonXBox::WasButtonReleased(
    TeleopControlMappingEnums::BUTTON_IDENTIFIER button // <I> - button to check
) const
{
    if (m_button[button] != nullptr)
    {
        return m_button[button]->WasButtonReleased();
    }
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT_ONCE, string("DragonXBox::WasButtonReleased"), to_string(button), string("button is Nullptr"));
    return false;
}

//==================================================================================
/// <summary>
/// Method:         WasButtonPressed
/// Description:    Read whether the button was pressed since the last query.  This
///                 is only valid for digital buttons (normal buttons and bumpers).
/// </summary>
//==================================================================================
bool DragonXBox::WasButtonPressed(
    TeleopControlMappingEnums::BUTTON_IDENTIFIER button // <I> - button to check
) const
{
    if (m_button[button] != nullptr)
    {
        return m_button[button]->WasButtonPressed();
    }
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT_ONCE, string("DragonXBox::WasButtonPressed"), to_string(button), string("button is Nullptr"));
    return false;
}

//==================================================================================
/// <summary>
/// Method:         SetButtonMode
/// Description:    Specify how the button should behave.  Examples include (but
///                 not limited to):
///                 - pressed / not pressed
///                 - toggle
/// Returns:        void
/// </summary>
//==================================================================================
void DragonXBox::SetButtonMode(
    TeleopControlMappingEnums::BUTTON_IDENTIFIER button, /// <I> - button to check
    TeleopControlMappingEnums::BUTTON_MODE mode          /// <I> - button behavior
)
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
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT_ONCE, string("DragonXBox::SetButtonMode"), to_string(button), string("button is Nullptr"));
        }
        // TODO: should have else to re-create the button or remove the toggle decorator
    }
}

///-------------------------------------------------------------------------------------------------
/// Method:      GetAxisValue
/// Description: Return the current value (between -1.0 and 1.0) for the requested axis.
/// Returns:     double   - current axis value
///-------------------------------------------------------------------------------------------------
double DragonXBox::GetAxisValue(
    TeleopControlMappingEnums::AXIS_IDENTIFIER axis // <I> - axis identifier to read
) const
{
    if (m_axis[axis] != nullptr)
    {
        return m_axis[axis]->GetAxisValue();
    }
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT_ONCE, string("DragonXBox::GetAxisValue"), to_string(axis), string("button is Nullptr"));

    return 0.0;
}

///-------------------------------------------------------------------------------------------------
/// Method:      SetProfile
/// Description: Specify the profile curve used for setting the sensitivity of the axis.  By default,
///              this is linear, but a cubic curve would give more control when the axis is barely
///              pressed (e.g. if it were moved 50% forward, instead of returning 0.5, it would
///              return 0.5 * 0.5 * 0.5 or .125, but when the axis was moved all the way forward,
///              it would return the same value -- 1.0.  Since it is cubed, it retains the sign.
///
///              This affects values returned from GetAxis calls.
/// Returns:     void
///-------------------------------------------------------------------------------------------------
void DragonXBox::SetAxisProfile(
    TeleopControlMappingEnums::AXIS_IDENTIFIER axis, // <I> - axis identifier to modify
    TeleopControlMappingEnums::AXIS_PROFILE curve    // <I> - the definition of the sensitivity
)
{
    if (m_axis[axis] != nullptr)
    {
        m_axis[axis]->SetAxisProfile(curve);
    }
    else
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT_ONCE, string("DragonXBox::SetAxisProfile"), to_string(axis), string("button is Nullptr"));
    }
}

///-------------------------------------------------------------------------------------------------
/// Method:      SetScale
/// Description: Scale the returned value to a range between the specified negative scale factor and
///              the scale factor.  This is used to reduce the maximum value returned.
///
///              This affects values returned from GetAxis calls.
/// Returns:     void
///-------------------------------------------------------------------------------------------------
void DragonXBox::SetAxisScale(
    TeleopControlMappingEnums::AXIS_IDENTIFIER axis, // <I> - axis identifier to modify
    double scaleFactor                               // <I> - value  (0 < scale <= 1.0) to scale the axis value
)
{
    if (m_axis[axis] != nullptr)
    {
        m_axis[axis]->SetAxisScaleFactor(scaleFactor);
    }
    else
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT_ONCE, string("DragonXBox::SetAxisScale"), to_string(axis), string("button is Nullptr"));
    }
}

//==================================================================================
/// <summary>
/// Method:         SetAxisDeadband
/// Description:    Specify what deadband behavior is desired such as none, standard,
///                 standard with scaling.
/// Returns:        void
/// </summary>
//==================================================================================
void DragonXBox::SetAxisDeadband(
    TeleopControlMappingEnums::AXIS_IDENTIFIER axis, /// <I> - axis to modify
    TeleopControlMappingEnums::AXIS_DEADBAND type    /// <I> - deadband option
)
{
    if (m_axis[axis] != nullptr)
    {
        m_axis[axis]->SetDeadBand(type);
    }
    else
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT_ONCE, string("DragonXBox::SetAxisDeadband"), to_string(axis), string("button is Nullptr"));
    }
}

void DragonXBox::SetAxisFlipped(
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
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT_ONCE, string("DragonXBox::SetAxisFlipped"), to_string(axis), string("button is Nullptr"));
    }
}

void DragonXBox::SetRumble(
    bool leftRumble, // <I> - rumble left
    bool rightRumble // <I> - rumble right
) const
{
    double lrum = leftRumble ? 1.0 : 0.0;
    double rrum = rightRumble ? 1.0 : 0.0;

    m_xbox->SetRumble(GenericHID::RumbleType::kLeftRumble, lrum);
    m_xbox->SetRumble(GenericHID::RumbleType::kRightRumble, rrum);
}
