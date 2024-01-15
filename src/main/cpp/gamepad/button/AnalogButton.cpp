
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

#include <gamepad/axis/AnalogAxis.h>
#include <gamepad/button/AnalogButton.h>
#include <gamepad/IDragonGamepad.h>

using namespace std;

//==================================================================================
/// <summary>
/// Class:          AnalogButton
/// Description:    This treats a analog input as a button on a gamepad.
/// </summary>
//==================================================================================
AnalogButton::AnalogButton(
    AnalogAxis *axis, // <I> - axis to turn into a button
    double minValue,  // <I> - minimum value for true
    double maxValue   // <I> - maximum value for true
    ) : m_axis(axis),
        m_minValue(minValue),
        m_maxValue(maxValue)
{
}

AnalogButton::AnalogButton(
    AnalogAxis *axis // <I> - axis to turn into a button
    ) : m_axis(axis),
        m_minValue(0.2),
        m_maxValue(1.0)
{
}

//==================================================================================
/// <summary>
/// Method:         IsButtonPressed
/// Description:    Read the button and return true if it is pressed and false if
///                 it isn't.
/// </summary>
//==================================================================================
bool AnalogButton::IsButtonPressed() const
{
    bool pressed = false;
    if (m_axis != nullptr)
    {
        auto axisValue = m_axis->GetAxisValue();
        pressed = (axisValue > m_minValue);
    }
    return pressed;
}

bool AnalogButton::WasButtonReleased() const
{
    return !IsButtonPressed();
}

bool AnalogButton::WasButtonPressed() const
{
    return IsButtonPressed();
}
