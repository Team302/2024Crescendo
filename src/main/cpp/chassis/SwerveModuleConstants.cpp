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

// C++ Includes

// FRC Includes
#include "units/velocity.h"

// Team 302 Includes
#include "chassis/SwerveModuleConstants.h"

// Third Party Includes

SwerveModuleAttributes SwerveModuleConstants::GetSwerveModuleAttrs(ModuleType type)
{
    SwerveModuleAttributes attr;
    attr.wheelDiameter = units::length::inch_t(4.0);

    attr.sensorToMechanismRatio = 1.0;
    attr.rotorToSensorRatio = 150.0 / 7.0;

    switch (type)
    {
    case SDS_MK4_L1:
    case SDS_MK4_L1_COLSON:
    case SDS_MK4I_L1:
    case SDS_MK4I_L1_COLSON:
        attr.driveGearRatio = 8.14;
        attr.maxSpeed = units::velocity::feet_per_second_t(13.0);
        break;

    case SDS_MK4_L2:
    case SDS_MK4_L2_COLSON:
    case SDS_MK4I_L2:
    case SDS_MK4I_L2_COLSON:
        attr.driveGearRatio = 6.75;
        attr.maxSpeed = units::velocity::feet_per_second_t(15.7);
        break;

    case SDS_MK4_L3:
    case SDS_MK4_L3_COLSON:
    case SDS_MK4I_L3:
    case SDS_MK4I_L3_COLSON:
        attr.driveGearRatio = 6.12;
        attr.maxSpeed = units::velocity::feet_per_second_t(17.3);
        break;

    default:
        break;
    }

    return attr;
}