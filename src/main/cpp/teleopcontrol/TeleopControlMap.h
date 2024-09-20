
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
// #include <gamepad/IDragonGamepad.h>
#include <teleopcontrol/TeleopControlAxis.h>
#include <teleopcontrol/TeleopControlButton.h>
#include <teleopcontrol/TeleopControlFunctions.h>

#include <RobinHood/robin_hood.h>

const TeleopControlButton driverAButton = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::A_BUTTON, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton driverBButton = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::B_BUTTON, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton driverXButton = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::X_BUTTON, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton driverYButton = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::Y_BUTTON, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton driverLBumper = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::LEFT_BUMPER, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton driverRBumper = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::RIGHT_BUMPER, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton driverSelectButton = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::SELECT_BUTTON, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton driverStartButton = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::START_BUTTON, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton driverLStickPressed = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::LEFT_STICK_PRESSED, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton driverRStickPressed = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::RIGHT_STICK_PRESSED, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton driverLTriggerPressed = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::LEFT_TRIGGER_PRESSED, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton driverRTriggerPressed = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::RIGHT_TRIGGER_PRESSED, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton driverDPad0 = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::POV_0, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton driverDPad45 = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::POV_45, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton driverDPad90 = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::POV_90, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton driverDPad135 = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::POV_135, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton driverDPad180 = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::POV_180, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton driverDPad225 = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::POV_225, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton driverDPad270 = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::POV_270, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton driverDPad315 = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::POV_315, TeleopControlMappingEnums::STANDARD};

const TeleopControlButton copilotAButton = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::A_BUTTON, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton copilotBButton = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::B_BUTTON, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton copilotXButton = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::X_BUTTON, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton copilotYButton = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::Y_BUTTON, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton copilotLBumper = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::LEFT_BUMPER, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton copilotRBumper = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::RIGHT_BUMPER, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton copilotSelectButton = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::SELECT_BUTTON, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton copilotStartButton = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::START_BUTTON, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton copilotLStickPressed = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::LEFT_STICK_PRESSED, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton copilotRStickPressed = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::RIGHT_STICK_PRESSED, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton copilotLTriggerPressed = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::LEFT_TRIGGER_PRESSED, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton copilotRTriggerPressed = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::RIGHT_TRIGGER_PRESSED, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton copilotDPad0 = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::POV_0, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton copilotDPad45 = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::POV_45, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton copilotDPad90 = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::POV_90, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton copilotDPad135 = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::POV_135, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton copilotDPad180 = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::POV_180, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton copilotDPad225 = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::POV_225, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton copilotDPad270 = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::POV_270, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton copilotDPad315 = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::POV_315, TeleopControlMappingEnums::STANDARD};

const TeleopControlButton extra1AButton = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::A_BUTTON, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra1BButton = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::B_BUTTON, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra1XButton = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::X_BUTTON, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra1YButton = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::Y_BUTTON, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra1LBumper = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::LEFT_BUMPER, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra1RBumper = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::RIGHT_BUMPER, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra1SelectButton = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::SELECT_BUTTON, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra1StartButton = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::START_BUTTON, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra1LStickPressed = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::LEFT_STICK_PRESSED, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra1RStickPressed = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::RIGHT_STICK_PRESSED, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra1LTriggerPressed = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::LEFT_TRIGGER_PRESSED, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra1RTriggerPressed = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::RIGHT_TRIGGER_PRESSED, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra1DPad0 = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::POV_0, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra1DPad45 = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::POV_45, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra1DPad90 = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::POV_90, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra1DPad135 = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::POV_135, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra1DPad180 = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::POV_180, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra1DPad225 = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::POV_225, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra1DPad270 = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::POV_270, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra1DPad315 = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::POV_315, TeleopControlMappingEnums::STANDARD};

const TeleopControlButton extra2AButton = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::A_BUTTON, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra2BButton = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::B_BUTTON, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra2XButton = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::X_BUTTON, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra2YButton = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::Y_BUTTON, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra2LBumper = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::LEFT_BUMPER, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra2RBumper = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::RIGHT_BUMPER, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra2SelectButton = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::SELECT_BUTTON, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra2StartButton = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::START_BUTTON, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra2LStickPressed = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::LEFT_STICK_PRESSED, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra2RStickPressed = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::RIGHT_STICK_PRESSED, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra2LTriggerPressed = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::LEFT_TRIGGER_PRESSED, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra2RTriggerPressed = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::RIGHT_TRIGGER_PRESSED, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra2DPad0 = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::POV_0, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra2DPad45 = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::POV_45, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra2DPad90 = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::POV_90, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra2DPad135 = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::POV_135, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra2DPad180 = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::POV_180, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra2DPad225 = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::POV_225, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra2DPad270 = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::POV_270, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra2DPad315 = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::POV_315, TeleopControlMappingEnums::STANDARD};

const TeleopControlButton extra3AButton = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::A_BUTTON, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra3BButton = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::B_BUTTON, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra3XButton = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::X_BUTTON, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra3YButton = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::Y_BUTTON, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra3LBumper = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::LEFT_BUMPER, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra3RBumper = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::RIGHT_BUMPER, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra3SelectButton = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::SELECT_BUTTON, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra3StartButton = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::START_BUTTON, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra3LStickPressed = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::LEFT_STICK_PRESSED, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra3RStickPressed = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::RIGHT_STICK_PRESSED, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra3LTriggerPressed = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::LEFT_TRIGGER_PRESSED, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra3RTriggerPressed = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::RIGHT_TRIGGER_PRESSED, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra3DPad0 = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::POV_0, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra3DPad45 = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::POV_45, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra3DPad90 = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::POV_90, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra3DPad135 = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::POV_135, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra3DPad180 = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::POV_180, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra3DPad225 = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::POV_225, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra3DPad270 = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::POV_270, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra3DPad315 = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::POV_315, TeleopControlMappingEnums::STANDARD};

const TeleopControlButton extra4AButton = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::A_BUTTON, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra4BButton = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::B_BUTTON, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra4XButton = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::X_BUTTON, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra4YButton = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::Y_BUTTON, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra4LBumper = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::LEFT_BUMPER, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra4RBumper = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::RIGHT_BUMPER, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra4SelectButton = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::SELECT_BUTTON, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra4StartButton = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::START_BUTTON, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra4LStickPressed = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::LEFT_STICK_PRESSED, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra4RStickPressed = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::RIGHT_STICK_PRESSED, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra4LTriggerPressed = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::LEFT_TRIGGER_PRESSED, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra4RTriggerPressed = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::RIGHT_TRIGGER_PRESSED, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra4DPad0 = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::POV_0, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra4DPad45 = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::POV_45, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra4DPad90 = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::POV_90, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra4DPad135 = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::POV_135, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra4DPad180 = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::POV_180, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra4DPad225 = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::POV_225, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra4DPad270 = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::POV_270, TeleopControlMappingEnums::STANDARD};
const TeleopControlButton extra4DPad315 = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::POV_315, TeleopControlMappingEnums::STANDARD};

robin_hood::unordered_map<TeleopControlFunctions::FUNCTION, const TeleopControlButton> teleopControlMapButtonMap{
    // {TeleopControlFunctions::ALIGN_FLOOR_GAME_PIECE, driverYButton},

    {TeleopControlFunctions::RESET_POSITION, driverDPad90},
    {TeleopControlFunctions::RESET_POSITION, driverDPad270},
    {TeleopControlFunctions::AUTO_STAGE, driverRStickPressed},
    //{TeleopControlFunctions::HOLD_POSITION, driverLBumper},need to re assign
    //{TeleopControlFunctions::SLOW_MODE, driverRBumper}, need to re assign

    // {TeleopControlFunctions::DEBUG_INC_P, driverDPad0},
    {TeleopControlFunctions::ROBOT_ORIENTED_DRIVE, driverDPad0},

    {TeleopControlFunctions::DEBUG_DEC_P, driverDPad180},

    {TeleopControlFunctions::HIGH_PASS, copilotDPad0},
    {TeleopControlFunctions::LOW_PASS, copilotDPad180},
    // {TeleopControlFunctions::TIPCORRECTION_TOGGLE, driverRStickPressed},

    {TeleopControlFunctions::DRIVE_TO_NOTE, driverBButton},
    {TeleopControlFunctions::CLIMB_MODE, driverStartButton},
    {TeleopControlFunctions::SCORING_MODE, copilotSelectButton},

    {TeleopControlFunctions::READY, copilotYButton},
    {TeleopControlFunctions::MANUAL_LAUNCH_INC, copilotLStickPressed},
    {TeleopControlFunctions::MANUAL_LAUNCH_DEC, copilotRStickPressed},
    {TeleopControlFunctions::INTAKE, driverRBumper},
    {TeleopControlFunctions::EXPEL, driverLBumper},
    {TeleopControlFunctions::AUTO_LAUNCH, copilotAButton},
    {TeleopControlFunctions::MANUAL_LAUNCH, copilotXButton},
    {TeleopControlFunctions::PASS, copilotBButton},
    {TeleopControlFunctions::PREP_PLACE, copilotXButton},
    {TeleopControlFunctions::PLACE, copilotRBumper},
    {TeleopControlFunctions::AUTO_TURN_BACKWARD, driverXButton},
    {TeleopControlFunctions::AUTO_SPEAKER, driverLStickPressed},
    {TeleopControlFunctions::TURN_TO_PASS_ANGLE, copilotDPad90},

    {TeleopControlFunctions::AUTO_AMP, driverAButton},
    {TeleopControlFunctions::READY_PASS, copilotRBumper},
    {TeleopControlFunctions::AUTO_CLIMB, driverYButton},
};

const TeleopControlAxis driverLJoystickX = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::LEFT_JOYSTICK_X, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::AXIS_PROFILE::LINEAR, TeleopControlMappingEnums::REVERSED, 1.0};
const TeleopControlAxis driverLJoystickY = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::LEFT_JOYSTICK_Y, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::AXIS_PROFILE::LINEAR, TeleopControlMappingEnums::REVERSED, 1.0};
const TeleopControlAxis driverRJoystickX = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::RIGHT_JOYSTICK_X, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::AXIS_PROFILE::CUBED, TeleopControlMappingEnums::REVERSED, 0.5};
const TeleopControlAxis driverRJoystickY = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::RIGHT_JOYSTICK_Y, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::AXIS_PROFILE::CUBED, TeleopControlMappingEnums::REVERSED, 1.0};
const TeleopControlAxis driverLTrigger = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::LEFT_TRIGGER, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};
const TeleopControlAxis driverRTrigger = {TeleopControlMappingEnums::DRIVER, TeleopControlMappingEnums::RIGHT_TRIGGER, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};

const TeleopControlAxis copilotLJoystickX = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::LEFT_JOYSTICK_X, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};
const TeleopControlAxis copilotLJoystickY = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::LEFT_JOYSTICK_Y, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::REVERSED, 0.5};
const TeleopControlAxis copilotRJoystickX = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::RIGHT_JOYSTICK_X, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};
const TeleopControlAxis copilotRJoystickY = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::RIGHT_JOYSTICK_Y, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::REVERSED, 1.0};
const TeleopControlAxis copilotLTrigger = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::LEFT_TRIGGER, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};
const TeleopControlAxis copilotRTrigger = {TeleopControlMappingEnums::CO_PILOT, TeleopControlMappingEnums::RIGHT_TRIGGER, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};

const TeleopControlAxis extra1LJoystickX = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::LEFT_JOYSTICK_X, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};
const TeleopControlAxis extra1LJoystickY = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::LEFT_JOYSTICK_Y, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::REVERSED, 1.0};
const TeleopControlAxis extra1RJoystickX = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::RIGHT_JOYSTICK_X, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};
const TeleopControlAxis extra1RJoystickY = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::RIGHT_JOYSTICK_X, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::REVERSED, 1.0};
const TeleopControlAxis extra1LTrigger = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::LEFT_TRIGGER, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};
const TeleopControlAxis extra1RTrigger = {TeleopControlMappingEnums::EXTRA1, TeleopControlMappingEnums::RIGHT_TRIGGER, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};

const TeleopControlAxis extra2LJoystickX = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::LEFT_JOYSTICK_X, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};
const TeleopControlAxis extra2LJoystickY = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::LEFT_JOYSTICK_Y, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::REVERSED, 1.0};
const TeleopControlAxis extra2RJoystickX = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::RIGHT_JOYSTICK_X, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};
const TeleopControlAxis extra2RJoystickY = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::RIGHT_JOYSTICK_X, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::REVERSED, 1.0};
const TeleopControlAxis extra2LTrigger = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::LEFT_TRIGGER, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};
const TeleopControlAxis extra2RTrigger = {TeleopControlMappingEnums::EXTRA2, TeleopControlMappingEnums::RIGHT_TRIGGER, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};

const TeleopControlAxis extra3LJoystickX = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::LEFT_JOYSTICK_X, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};
const TeleopControlAxis extra3LJoystickY = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::LEFT_JOYSTICK_Y, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::REVERSED, 1.0};
const TeleopControlAxis extra3RJoystickX = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::RIGHT_JOYSTICK_X, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};
const TeleopControlAxis extra3RJoystickY = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::RIGHT_JOYSTICK_X, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::REVERSED, 1.0};
const TeleopControlAxis extra3LTrigger = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::LEFT_TRIGGER, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};
const TeleopControlAxis extra3RTrigger = {TeleopControlMappingEnums::EXTRA3, TeleopControlMappingEnums::RIGHT_TRIGGER, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};

const TeleopControlAxis extra4LJoystickX = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::LEFT_JOYSTICK_X, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};
const TeleopControlAxis extra4LJoystickY = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::LEFT_JOYSTICK_Y, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::REVERSED, 1.0};
const TeleopControlAxis extra4RJoystickX = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::RIGHT_JOYSTICK_X, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};
const TeleopControlAxis extra4RJoystickY = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::RIGHT_JOYSTICK_X, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::REVERSED, 1.0};
const TeleopControlAxis extra4LTrigger = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::LEFT_TRIGGER, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};
const TeleopControlAxis extra4RTrigger = {TeleopControlMappingEnums::EXTRA4, TeleopControlMappingEnums::RIGHT_TRIGGER, TeleopControlMappingEnums::APPLY_STANDARD_DEADBAND, TeleopControlMappingEnums::CUBED, TeleopControlMappingEnums::SYNCED, 1.0};

robin_hood::unordered_map<TeleopControlFunctions::FUNCTION, const TeleopControlAxis> teleopControlMapAxisMap{
    {TeleopControlFunctions::HOLONOMIC_DRIVE_FORWARD, driverLJoystickY},
    {TeleopControlFunctions::HOLONOMIC_DRIVE_STRAFE, driverLJoystickX},
    {TeleopControlFunctions::HOLONOMIC_DRIVE_ROTATE, driverRJoystickX},
    {TeleopControlFunctions::MANUAL_CLIMB, driverRJoystickY},
    {TeleopControlFunctions::MANUAL_PLACE, copilotLTrigger},
    {TeleopControlFunctions::MANUAL_FEED, copilotRTrigger},
    {TeleopControlFunctions::LAUNCH_ANGLE, copilotRJoystickY},
    {TeleopControlFunctions::ELEVATOR, copilotLJoystickY},
};
