
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

#include <gamepad/button/ToggleButton.h>
#include <gamepad/button/ButtonDecorator.h>
#include <gamepad/button/IButton.h>

ToggleButton::ToggleButton(
    IButton *button // <I> - button to decorate
    ) : ButtonDecorator(button),
        m_isPressed(false),
        m_isToggledOn(false)
{
}
bool ToggleButton::IsButtonPressed() const
{
    auto isPressed = ButtonDecorator::IsButtonPressed(); // read current button pressed state

    //------------------------------------------------------------------------------------------
    // Cases:
    //   (1) - button was previously pressed and is still pressed ==>> toggle state unchanged
    //   (2) - button was prevuiosly pressed and now it isn't ==>> toggle state unchanged
    //   (3) - button was previously not pressed and it still isn't pressed ==>> toggle state
    //         unchanged
    //   (4) - button was previously not pressed and now it is pressed ==>> toggle state changed
    //
    //  So, only case 4 changes the toggle state.  In all cases we need to save the pressed state
    //------------------------------------------------------------------------------------------
    if (!m_isPressed && isPressed) // button previously wasn't pressed and now it is
    {
        m_isToggledOn = !m_isToggledOn; // reverse the toggle
    }

    m_isPressed = isPressed; // remember what the state of the button pressing
    return m_isToggledOn;
}

bool ToggleButton::WasButtonReleased() const
{
    return !IsButtonPressed();
}

bool ToggleButton::WasButtonPressed() const
{
    return IsButtonPressed();
}