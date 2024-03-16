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

#include "units/angle.h"

class AngleUtils
{
public:
    /// @brief find the angle from the startingAngle to the targetAngle
    /// @param [in] startingAngle - angle to start from
    /// @param [in] targetAngle - angle to go to
    /// @returns units::angle::degree_t the angle to traverse to get from the startingAngle
    ///                                 to the targetAngle
    static units::angle::degree_t GetDeltaAngle(units::angle::degree_t startingAngle,
                                                units::angle::degree_t targetAngle);

    /// @brief make sure the angle is between -180.0_deg and 180.0_deg and if it isn't find
    ///        its equivalent angle within that range
    /// @param [in] units::angle::degree_t  angle - angle to check
    /// @returns units::angle::degree_t angle within range
    static units::angle::degree_t GetEquivAngle(units::angle::degree_t angle);
};
