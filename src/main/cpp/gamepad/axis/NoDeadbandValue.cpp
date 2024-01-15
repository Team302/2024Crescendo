
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

#include <gamepad/axis/NoDeadbandValue.h>

#include <cmath>

//==================================================================================
/// <summary>
/// Method:         GetInstance
/// Description:    Static singleton method to create the object
/// </summary>
//==================================================================================
NoDeadbandValue *NoDeadbandValue::m_instance = nullptr;
NoDeadbandValue *NoDeadbandValue::GetInstance()
{
    if (m_instance == nullptr)
    {
        m_instance = new NoDeadbandValue();
    }
    return m_instance;
}

//==================================================================================
/// <summary>
/// Method:         ApplyDeadband
/// Description:    Apply the standard deadband
/// </summary>
//==================================================================================
void NoDeadbandValue::ApplyDeadband(
    double &inputVal // <I> - value to apply profile to
) const
{
}
