
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

#include "hw/builtinaccel/DragonBuiltInAccelerometer.h"

class DragonRoboRioAccelXRightYUp : public DragonBuiltinAccelerometer
{
public:
	DragonRoboRioAccelXRightYUp() = default;
	virtual ~DragonRoboRioAccelXRightYUp() = default;

	/// @return The acceleration of the roboRIO along the robot X axis (forward) in g-forces
	inline units::acceleration::feet_per_second_squared_t GetX() override { return -1.0 * units::acceleration::feet_per_second_squared_t(m_builtin->GetX()); }

	/// @return The acceleration of the roboRIO along the robot Y axis (left) in g-forces
	inline units::acceleration::feet_per_second_squared_t GetY() override { return -1.0 * units::acceleration::feet_per_second_squared_t(m_builtin->GetZ()); }

	/// @return The acceleration of the roboRIO along the robot Z axis (up) in g-forces
	inline units::acceleration::feet_per_second_squared_t GetZ() override { return units::acceleration::feet_per_second_squared_t(m_builtin->GetY()); }
};
