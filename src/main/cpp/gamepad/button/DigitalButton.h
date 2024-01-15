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

#pragma once

#include <gamepad/button/IButton.h>

// forward declare
namespace frc
{
    class GenericHID;
}

//==================================================================================
/// <summary>
/// Class:          DigitalButton
/// Description:    This is the standard button on a gamepad.
/// </summary>
//==================================================================================
class DigitalButton : public IButton
{
public:
    DigitalButton(
        frc::GenericHID *gamepad, // <I> - gamepad to query
        int buttonID              // <I> - button ID this maps to
    );

    //==================================================================================
    /// <summary>
    /// Method:         IsButtonPressed
    /// Description:    Read the button and return true if it is pressed and false if
    ///                 it isn't.
    /// </summary>
    //==================================================================================
    bool IsButtonPressed() const override;

    //==================================================================================
    /// <summary>
    /// Method:         WasButtonReleased
    /// Description:    Read whether the button was released since the last query.  This
    ///                 is only valid for digital buttons (normal buttons and bumpers).
    /// </summary>
    //==================================================================================
    bool WasButtonReleased() const override;

    //==================================================================================
    /// <summary>
    /// Method:         WasButtonPressed
    /// Description:    Read whether the button was pressed since the last query.  This
    ///                 is only valid for digital buttons (normal buttons and bumpers).
    /// </summary>
    //==================================================================================
    bool WasButtonPressed() const override;

private:
    frc::GenericHID *m_gamepad;
    int m_button;
};
