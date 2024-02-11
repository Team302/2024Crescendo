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

// FRC Includes
#include "units/angular_velocity.h"
#include "units/length.h"
#include "units/velocity.h"

// Team302 Includes
#include "chassis/SwerveModuleConstants.h"

// Third party includes

/// @brief This is used to give all neccessary data to ISwerveDriveStates

struct SwerveModuleAttributes
{
    units::length::inch_t wheelDiameter = units::length::inch_t(4.0);
    double driveGearRatio = 1.0;
    double sensorToMechanismRatio = 1.0;
    double rotorToSensorRatio = 150.0 / 7.0;
    units::velocity::feet_per_second_t maxSpeed = units::velocity::feet_per_second_t(0.0);
};