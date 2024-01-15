
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

#include "hw/builtinaccel/DragonBuiltInAccelerometer.h"

DragonBuiltinAccelerometer::DragonBuiltinAccelerometer() : m_builtin(new frc::BuiltInAccelerometer())
{
}

DragonBuiltinAccelerometer::~DragonBuiltinAccelerometer()
{
	delete m_builtin;
}

units::acceleration::feet_per_second_squared_t DragonBuiltinAccelerometer::GetX()
{
	return units::acceleration::feet_per_second_squared_t(m_builtin->GetX());
}
units::acceleration::feet_per_second_squared_t DragonBuiltinAccelerometer::GetY()
{
	return units::acceleration::feet_per_second_squared_t(m_builtin->GetY());
}
units::acceleration::feet_per_second_squared_t DragonBuiltinAccelerometer::GetZ()
{
	return units::acceleration::feet_per_second_squared_t(m_builtin->GetZ());
}
