
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

class TeleopControlMappingEnums
{
public:
    enum CONTROLLER
    {
        UNKNOWN_CONTROLLER = -1,
        DRIVER,
        CO_PILOT,
        EXTRA1,
        EXTRA2,
        EXTRA3,
        EXTRA4,
        MAX_CONTROLLERS
    };

    enum CONTROL_MODE
    {
        ALL,
        CUBE,
        CONE,
        MAX_CONTROL_MODES
    };

    enum BUTTON_IDENTIFIER
    {
        UNDEFINED_BUTTON = -1,
        A_BUTTON,
        B_BUTTON,
        X_BUTTON,
        Y_BUTTON,
        LEFT_BUMPER,
        RIGHT_BUMPER,
        SELECT_BUTTON,
        START_BUTTON,
        LEFT_STICK_PRESSED,
        RIGHT_STICK_PRESSED,
        LEFT_TRIGGER_PRESSED,
        RIGHT_TRIGGER_PRESSED,
        POV_0,
        POV_45,
        POV_90,
        POV_135,
        POV_180,
        POV_225,
        POV_270,
        POV_315,
        GAMEPAD_SWITCH_18,
        GAMEPAD_SWITCH_19,
        GAMEPAD_SWITCH_20,
        GAMEPAD_SWITCH_21,
        GAMEPAD_BUTTON_14_UP,
        GAMEPAD_BUTTON_14_DOWN,
        GAMEPAD_BUTTON_15_UP,
        GAMEPAD_BUTTON_15_DOWN,
        GAMEPAD_BUTTON_1,
        GAMEPAD_BUTTON_2,
        GAMEPAD_BUTTON_3,
        GAMEPAD_BUTTON_4,
        GAMEPAD_BUTTON_5,
        GAMEPAD_BUTTON_6,
        GAMEPAD_BUTTON_7,
        GAMEPAD_BUTTON_8,
        GAMEPAD_BUTTON_9,
        GAMEPAD_BUTTON_10,
        GAMEPAD_BUTTON_11,
        GAMEPAD_BUTTON_12,
        GAMEPAD_BUTTON_13,
        GAMEPAD_DIAL_22,
        GAMEPAD_DIAL_23,
        GAMEPAD_DIAL_24,
        GAMEPAD_DIAL_25,
        GAMEPAD_DIAL_26,
        GAMEPAD_DIAL_27,
        GAMEPAD_BIG_RED_BUTTON,
        MAX_BUTTONS
    };

    // TODO:  Add debouncing
    enum BUTTON_MODE
    {
        STANDARD,
        TOGGLE,
        MAX_BUTTON_MODES
    };

    enum AXIS_IDENTIFIER
    {
        UNDEFINED_AXIS = -1,
        LEFT_JOYSTICK_X,
        LEFT_JOYSTICK_Y,
        RIGHT_JOYSTICK_X,
        RIGHT_JOYSTICK_Y,
        LEFT_TRIGGER,
        RIGHT_TRIGGER,
        GAMEPAD_AXIS_16,
        GAMEPAD_AXIS_17,
        LEFT_ANALOG_BUTTON_AXIS,
        RIGHT_ANALOG_BUTTON_AXIS,
        DIAL_ANALOG_BUTTON_AXIS,
        MAX_AXIS
    };

    enum AXIS_DEADBAND
    {
        NONE,
        APPLY_STANDARD_DEADBAND,
        APPLY_SCALED_DEADBAND,
        MAX_DEADBANDS
    };

    enum AXIS_PROFILE
    {
        LINEAR,
        SQUARED,
        CUBED,
        PIECEWISE_LINEAR,
        MAX_PROFILES
    };

    enum AXIS_DIRECTION
    {
        SYNCED,
        REVERSED
    };
};
