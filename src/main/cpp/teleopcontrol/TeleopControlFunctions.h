
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
        ARCADE_THROTTLE,
        ARCADE_STEER,
        DRIVE_TO_LEFT_GRID,
        DRIVE_TO_CENTER_GRID,
        DRIVE_TO_RIGHT_GRID,
        DRIVE_TO_LEFT_NODE,
        DRIVE_TO_RIGHT_NODE,
        DRIVE_TO_HUMAN_PLAYER_RIGHT,
        DRIVE_TO_HUMAN_PLAYER_LEFT,
        BALANCE_MODE,
        AUTO_BALANCE,
        SLOW_MODE,
        AUTO_TURN_FORWARD,
        AUTO_TURN_BACKWARD,
        // Standish raw vision
        ALIGN_APRIL_TAG,
        ALIGN_FLOOR_GAME_PIECE,
        ALIGN_SUBSTATION_GAME_PIECE,
        // Scoring states
        DEBUG_INC_P,
        DEBUG_DEC_P,
        DEBUG_INC_I,
        DEBUG_DEC_I, // Scoring states
        // HOLD_POSITION, // may not need this state
        BACKROW,
        MIDROW,
        HUMAN_PLAYER_STATION,
        STARTING_POSITION,
        FLOOR_POSITION,
        // DEBUGGING Arm states, when cleaned up, leave MANUAL_ROTATE
        HOLD_POSITION_ROTATE,
        MANUAL_ROTATE,
        CUBE_BACKROW_ROTATE,
        CONE_BACKROW_ROTATE,
        CUBE_MIDROW_ROTATE,
        CONE_MIDROW_ROTATE,
        HUMAN_PLAYER_STATION_ROTATE,
        STARTING_POSITION_ROTATE,
        FLOOR_POSITION_ROTATE,
        // DEBUGGING Extender states, when cleaned up, leave MANUAL_EXTEND_RETRACT
        HOLD_POSITION_EXTEND,
        MANUAL_EXTEND_RETRACT,
        CUBE_BACKROW_EXTEND,
        CONE_BACKROW_EXTEND,
        CUBE_MIDROW_EXTEND,
        CONE_MIDROW_EXTEND,
        HUMAN_PLAYER_STATION_EXTEND,
        STARTING_POSITION_EXTEND,
        FLOOR_EXTEND,
        // Turn about point states
        FRONT_LEFT_BUMPER_TURNABOUT_POINT,
        FRONT_RIGHT_BUMPER_TURNABOUT_POINT,
        // CYCLE_GAMEPIECE states
        OPEN,
        GRAB,
        CYCLE_CYCLE_GAMEPIECE,
        // Active intake controls?
        INTAKE_OFF,
        INTAKE,
        HOLD,
        EXPEL,
        EXPEL_LOW,
        RELEASE,
        // Compressor ON/Off controls
        TOGGLE_COMPRESSER,
        // tip correction controls
        TIPCORRECTION_TOGGLE,
        // Example Mechanism
        EXAMPLE_MECH_FORWARD,
        EXAMPLE_MECH_REVERSE,

        MAX_FUNCTIONS,

    };
};
