
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

// Team 302 Includes
#include <chassis/holonomic/FieldDriveUtils.h>
#include "hw/interfaces/IDragonPigeon.h"
#include "utils/ConversionUtils.h"

// frc Includes
#include <frc/kinematics/ChassisSpeeds.h>

// Third Party Includes

using namespace frc;

/// @brief convert chassis speeds specified in Field Oriented values to Robot Values
/// @param [in] ChassisSpeeds the desired kinemetics of the chassis in a field oriented frame
/// @param [in] DragonPigeon gyro which has the angle the robot is facing
/// @returns ChassisSpeeds the converted speeds in the robot frame
ChassisSpeeds FieldDriveUtils::ConvertFieldOrientedToRobot(ChassisSpeeds input,
                                                           IDragonPigeon *pigeon)
{
    auto heading = pigeon != nullptr ? pigeon->GetYaw() : units::angle::degree_t(0.0);

    ChassisSpeeds output;
    output.vx = input.vx * cos(heading.to<double>()) - input.vy * sin(heading.to<double>());
    output.vy = input.vx * sin(heading.to<double>()) + input.vy * cos(heading.to<double>());
    output.omega = input.omega;

    return output;
}
