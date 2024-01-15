
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

#include <frc/GenericHID.h>

#include <gamepad/button/POVButton.h>
#include <gamepad/IDragonGamepad.h>

//==================================================================================
/// <summary>
/// Class:          POVButton
/// Description:    This is the POV/DPAD button on a gamepad.
/// </summary>
//==================================================================================
POVButton::POVButton(
    frc::GenericHID *gamepad, // <I> - gamepad to query
    int buttonID              // <I> - button ID this maps to
    ) : m_gamepad(gamepad),   //       false axis in the expected direction
        m_button(buttonID)
{
}

//==================================================================================
/// <summary>
/// Method:         IsButtonPressed
/// Description:    Read the button and return true if it is pressed and false if
///                 it isn't.
/// </summary>
//==================================================================================
bool POVButton::IsButtonPressed() const
{
    bool pressed = false;
    if (m_gamepad != nullptr)
    {
        pressed = (m_button == m_gamepad->GetPOV());
    }
    return pressed;
}

bool POVButton::WasButtonReleased() const
{
    return !IsButtonPressed();
}

bool POVButton::WasButtonPressed() const
{
    return IsButtonPressed();
}
