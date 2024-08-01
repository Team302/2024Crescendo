
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

// C++ Includes

// FRC includes

// Team 302 includes

class TeleopControlFunctions
{
public:
    enum FUNCTION
    {
        UNKNOWN_FUNCTION,
        ROBOT_ORIENTED_DRIVE,
        HOLONOMIC_DRIVE_FORWARD,
        HOLONOMIC_DRIVE_ROTATE,
        HOLONOMIC_DRIVE_STRAFE,
        RESET_POSITION,
        HOLD_POSITION,
        SLOW_MODE,
        AUTO_TURN_FORWARD,
        AUTO_TURN_BACKWARD,
        ALIGN_TO_AMP,
        ALIGN_TO_CENTER_STAGE_TRAP,
        ALIGN_TO_LEFT_STAGE_TRAP,
        ALIGN_TO_RIGHT_STAGE_TRAP,
        ALIGN_FLOOR_GAME_PIECE,
        // Scoring states
        DEBUG_INC_P,
        DEBUG_DEC_P,
        DEBUG_INC_I,
        DEBUG_DEC_I, // Scoring states
        // tip correction controls
        TIPCORRECTION_TOGGLE,
        // Example Mechanism
        EXAMPLE_MECH_FORWARD,
        EXAMPLE_MECH_REVERSE,
        // Cresendo Modes
        CLIMB_MODE,
        MANUAL_MODE,
        SCORING_MODE,
        DRIVE_TO_NOTE,

        TURN_TO_PASS_ANGLE,
        READY,
        INTAKE,
        EXPEL,
        AUTO_LAUNCH,
        MANUAL_LAUNCH,
        PASS,
        HIGH_PASS,
        LOW_PASS,
        PREP_PLACE,
        PLACE,
        AUTO_SPEAKER,
        AUTO_AMP,
        AUTO_STAGE,
        MANUAL_CLIMB,
        MANUAL_PLACE,
        MANUAL_FEED,
        LAUNCH_ANGLE,
        ELEVATOR,
        AUTO_CLIMB,
        READY_PASS,

        MANUAL_LAUNCH_INC,
        MANUAL_LAUNCH_DEC,
    };
};
