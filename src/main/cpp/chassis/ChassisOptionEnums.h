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

class ChassisOptionEnums
{
public:
    enum HeadingOption
    {
        MAINTAIN,
        SPECIFIED_ANGLE,
        FACE_GAME_PIECE,
        FACE_SPEAKER,
        FACE_AMP,
        FACE_LEFT_STAGE,
        FACE_RIGHT_STAGE,
        FACE_CENTER_STAGE,
        FACE_STAGE,
        IGNORE
    };

    enum DriveStateType
    {
        ROBOT_DRIVE,
        FIELD_DRIVE,
        TRAJECTORY_DRIVE_PLANNER,
        DRIVE_TO_NOTE,
        HOLD_DRIVE,
        STOP_DRIVE,
        STAGE_DRIVE,
        AUTO_BALANCE
    };

    enum RELATIVE_POSITION
    {
        LEFT = 1,
        CENTER = 2,
        RIGHT = 3
    };

    enum NoMovementOption
    {
        STOP,
        HOLD_POSITION
    };

    enum AutonControllerType
    {
        RAMSETE,
        HOLONOMIC
    };

    enum AutonChassisOptions
    {
        VISION_DRIVE_NOTE,
        VISION_DRIVE_SPEAKER,
        NO_VISION
    };
    enum AutonAvoidOptions
    {
        PODIUM,
        ROBOT_COLLISION,
        NO_AVOID_OPTION
    };

    enum PathGainsType
    {
        SHORT,
        LONG
    };

    enum PathUpdateOption
    {
        NOTE,
        NONE
    };

    ChassisOptionEnums() = delete;
    ~ChassisOptionEnums() = delete;
};