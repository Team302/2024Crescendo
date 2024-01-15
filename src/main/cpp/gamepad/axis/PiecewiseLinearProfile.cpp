

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

//========================================================================================================
/// @class PiecewiseLinearProfile
/// @brief This applies a cubic profile to the input values.
//========================================================================================================

// C++ Includes
#include <cmath>

// FRC includes

// Team 302 includes
#include <gamepad/axis/PiecewiseLinearProfile.h>

// Third Party Includes

using namespace std;

PiecewiseLinearProfile::PiecewiseLinearProfile() : IProfile(),
                                                   m_intercept(0.25),
                                                   m_inflectionX(0.8),
                                                   m_inflectionY(0.6)
{
}

//==================================================================================
/// @brief    Apply the profile
/// @param [in] value that needs the profile (scaling) applied
/// @return double profiled (scaled) value
//==================================================================================
void PiecewiseLinearProfile::ApplyProfile(
    double &inputVal // <I> - value to apply profile to
) const
{
    if (inputVal > m_intercept)
    {
        inputVal = m_inflectionY + (inputVal - m_intercept) * ((1.0 - m_inflectionY) / (1 - m_intercept));
    }
    else if (inputVal >= 0.0)
    {
        inputVal = m_intercept + ((m_inflectionY - m_intercept) / m_inflectionX) * inputVal;
    }
    else if (inputVal >= -m_intercept)
    {
        inputVal = (-m_intercept + ((m_inflectionY - m_intercept) / m_inflectionX) * inputVal);
    }
    else
    {
        inputVal = (-m_inflectionY + (inputVal + m_intercept) * ((1 - m_inflectionY) / (1 - m_intercept)));
    }
}
