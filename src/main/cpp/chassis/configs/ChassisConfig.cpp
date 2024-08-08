
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

#include "chassis/configs/ChassisConfig.h"
#include "chassis/SwerveChassis.h"

ChassisConfig::ChassisConfig()
{
}

void ChassisConfig::BuildChassis()
{
    DefinePigeon();
    DefineChassis();
}

ChassisConfig::~ChassisConfig()
{
}

void ChassisConfig::DefinePigeon()
{
}

void ChassisConfig::DefineChassis()
{
}

SwerveModule *ChassisConfig::GetSwerveModule(ChassisConfig::SWERVE_MODULE module) const
{
    if (module == SWERVE_MODULE::LEFT_BACK)
        return m_leftBackModule;

    if (module == SWERVE_MODULE::LEFT_FRONT)
        return m_leftFrontModule;

    if (module == SWERVE_MODULE::RIGHT_BACK)
        return m_rightBackModule;

    return m_rightFrontModule;
}
