
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

#include <gamepad/button/DigitalButton.h>
#include <gamepad/IDragonGamepad.h>
#include "utils/logging/Logger.h"

using namespace std;

//==================================================================================
/// <summary>
/// Class:          DigitalButton
/// Description:    This is the standard button on a gamepad.
/// </summary>
//==================================================================================
DigitalButton::DigitalButton(
    frc::GenericHID *gamepad, // <I> - gamepad to query
    int buttonID              // <I> - button ID this maps to
    ) : m_gamepad(gamepad),
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
bool DigitalButton::IsButtonPressed() const
{
    if (m_gamepad != nullptr)
    {
        return m_gamepad->GetRawButton(m_button);
    }
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT_ONCE, string("DigitalButton::DigitalButton::IsButtonPressed"), string("Gampad"), string("is Nullptr"));
    return false;
}

bool DigitalButton::WasButtonReleased() const
{
    if (m_gamepad != nullptr)
    {
        return m_gamepad->GetRawButtonReleased(m_button);
    }
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT_ONCE, string("DigitalButton::DigitalButton::WasButtonReleased"), string("Gampad"), string("is Nullptr"));
    return false;
}

bool DigitalButton::WasButtonPressed() const
{
    if (m_gamepad != nullptr)
    {
        return m_gamepad->GetRawButtonPressed(m_button);
    }
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT_ONCE, string("DigitalButton::WasButtonPressed"), string("Gampad"), string("is Nullptr"));
    return false;
}
