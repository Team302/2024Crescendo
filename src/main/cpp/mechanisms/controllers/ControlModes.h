
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

///	 @class			ControlModes
///  @brief      	This contains the enum for the mechanism parameters
class ControlModes
{
public:
    enum CONTROL_TYPE
    {
        PERCENT_OUTPUT,            /// Open Loop Control - values are between -1.0 and 1.0
        POSITION_INCH,             /// Closed Loop Control - values are displacements measured in inches
        POSITION_ABS_TICKS,        /// Closed Loop Control - values are measured in ticks
        POSITION_DEGREES,          /// Closed Loop Control - values are angles measured in degrees
        POSITION_DEGREES_ABSOLUTE, /// Closed Loop Control - values are angles measured in degrees that don't need to be converted
        VELOCITY_INCH,             /// Closed Loop Control - values are linear velocity measured in inches per second
        VELOCITY_DEGREES,          /// Closed Loop Control - values are angular velocity measured in degrees per second
        VELOCITY_RPS,              /// Closed Loop Control - values are in revolutions per second
        VOLTAGE,                   /// Closed Loop Control - values are in volts
        CURRENT,                   /// Closed Loop Control - values in amps
        TRAPEZOID,                 /// Closed Loop Control - trapezoid profile (e.g. Motion Magic)
        MOTION_PROFILE,            /// Closed Loop Control - motion profile
        MOTION_PROFILE_ARC,        /// Closed Loop Control - motion profile arc
        MAX_CONTROL_TYPES
    };

    enum CONTROL_RUN_LOCS
    {
        MOTOR_CONTROLLER,
        ROBORIO,
        MAX_CONTROL_RUN_LOCS
    };
};
