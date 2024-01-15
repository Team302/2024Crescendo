
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
#include <vector>

// Team 302 includes
#include <gamepad/IDragonGamepad.h>
#include <teleopcontrol/TeleopControlMappingEnums.h>

// forward declares
namespace frc
{
    class Joystick;
}
class AnalogAxis;
class IButton;

class DragonGamepad : public IDragonGamepad
{
public:
    DragonGamepad(
        int port);
    ~DragonGamepad();

    bool IsButtonPressed(
        TeleopControlMappingEnums::BUTTON_IDENTIFIER button) const override;

    void SetAxisDeadband(
        TeleopControlMappingEnums::AXIS_IDENTIFIER axis,
        TeleopControlMappingEnums::AXIS_DEADBAND type) override;

    void SetAxisProfile(
        TeleopControlMappingEnums::AXIS_IDENTIFIER axis,
        TeleopControlMappingEnums::AXIS_PROFILE curve) override;

    void SetAxisScale(
        TeleopControlMappingEnums::AXIS_IDENTIFIER axis,
        double scaleFactor) override;

    void SetAxisFlipped(
        TeleopControlMappingEnums::AXIS_IDENTIFIER axis, // <I> - axis identifier to modify
        bool isFlipped                                   // <I> - true - invert axis, false - no inversion
        ) override;

    void SetButtonMode(
        TeleopControlMappingEnums::BUTTON_IDENTIFIER button,
        TeleopControlMappingEnums::BUTTON_MODE mode) override;

    double GetAxisValue(
        TeleopControlMappingEnums::AXIS_IDENTIFIER axis) const override;

    bool WasButtonPressed(
        TeleopControlMappingEnums::BUTTON_IDENTIFIER button) const override;

    bool WasButtonReleased(
        TeleopControlMappingEnums::BUTTON_IDENTIFIER button) const override;

    void SetRumble(
        bool leftRumble, // <I> - rumble left
        bool rightRumble // <I> - rumble right
    ) const override;

private:
    frc::Joystick *m_gamepad;

    std::vector<AnalogAxis *> m_axis;
    std::vector<double> m_axisScale;
    std::vector<double> m_axisInversionFactor;
    std::vector<TeleopControlMappingEnums::AXIS_PROFILE> m_axisProfile;

    std::vector<IButton *> m_button;

    const int LEFT_BUTTON_AXIS_ID = 0;
    const int RIGHT_BUTTON_AXIS_ID = 1;
    const int DIAL_BUTTON_AXIS_ID = 2;
    const int LEFT_JOYSTICK = 3;
    const int RIGHT_JOYSTICK = 4;

    const double JOYSTICK_SCALE = 16.94;
    const double JOYSTICK_OFFSET = 1.0;

    const int SWITCH_18_DIGITAL_ID = 1;
    const int SWITCH_19_DIGITAL_ID = 2;
    const int SWITCH_20_DIGITAL_ID = 3;
    const int SWITCH_21_DIGITAL_ID = 4;

    const int LEVER_14_DOWN_DIGITAL_ID = 6;
    const int LEVER_14_UP_DIGITAL_ID = 7;
    const int LEVER_15_DOWN_DIGITAL_ID = 8;
    const int LEVER_15_UP_DIGITAL_ID = 9;

    // order by axis and in ranges for ease of getting the ranges right

    // Axis 0

    const double BUTTON_12_LOWERBOUND = 0.014;
    const double BUTTON_12_UPPERBOUND = 0.016;
    const int BUTTON_12_AXIS = 0;

    const double BUTTON_10_LOWERBOUND = 0.030;
    const double BUTTON_10_UPPERBOUND = 0.032;
    const int BUTTON_10_AXIS = 0;

    const double BUTTON_8_LOWERBOUND = 0.054;
    const double BUTTON_8_UPPERBOUND = 0.056;
    const int BUTTON_8_AXIS = 0;

    const double BUTTON_6_LOWERBOUND = 0.077;
    const double BUTTON_6_UPPERBOUND = 0.079;
    const int BUTTON_6_AXIS = 0;

    const double BUTTON_3_LOWERBOUND = 0.102;
    const double BUTTON_3_UPPERBOUND = 0.103;
    const int BUTTON_3_AXIS = 0;

    const double BUTTON_1_LOWERBOUND = 0.118;
    const double BUTTON_1_UPPERBOUND = 0.119;
    const int BUTTON_1_AXIS = 0;

    // Axis 1

    const double BUTTON_11_LOWERBOUND = 0.014;
    const double BUTTON_11_UPPERBOUND = 0.016;
    const int BUTTON_11_AXIS = 1;

    const double BUTTON_9_LOWERBOUND = 0.030;
    const double BUTTON_9_UPPERBOUND = 0.032;
    const int BUTTON_9_AXIS = 1;

    const double BUTTON_7_LOWERBOUND = 0.054;
    const double BUTTON_7_UPPERBOUND = 0.056;
    const int BUTTON_7_AXIS = 1;

    const double BUTTON_4_LOWERBOUND = 0.077;
    const double BUTTON_4_UPPERBOUND = 0.079;
    const int BUTTON_4_AXIS = 1;

    const double BUTTON_5_LOWERBOUND = 0.102;
    const double BUTTON_5_UPPERBOUND = 0.103;
    const int BUTTON_5_AXIS = 1;

    const double BUTTON_13_LOWERBOUND = 0.104;
    const double BUTTON_13_UPPERBOUND = 0.117;
    const int BUTTON_13_AXIS = 1;

    const double BUTTON_2_LOWERBOUND = 0.118;
    const double BUTTON_2_UPPERBOUND = 0.119;
    const int BUTTON_2_AXIS = 1;

    // Axis 3

    const double BUTTON_22_LOWERBOUND = 0.117;
    const double BUTTON_22_UPPERBOUND = 0.119;
    const int BUTTON_22_AXIS = 3;

    const double BUTTON_23_LOWERBOUND = 0.101;
    const double BUTTON_23_UPPERBOUND = 0.103;
    const int BUTTON_23_AXIS = 3;

    const double BUTTON_24_LOWERBOUND = 0.078;
    const double BUTTON_24_UPPERBOUND = 0.080;
    const int BUTTON_24_AXIS = 3;

    const double BUTTON_25_LOWERBOUND = 0.054;
    const double BUTTON_25_UPPERBOUND = 0.056;
    const int BUTTON_25_AXIS = 3;

    const double BUTTON_26_LOWERBOUND = 0.030;
    const double BUTTON_26_UPPERBOUND = 0.032;
    const int BUTTON_26_AXIS = 3;

    const double BUTTON_27_LOWERBOUND = 0.014;
    const double BUTTON_27_UPPERBOUND = 0.017;
    const int BUTTON_27_AXIS = 3;

    DragonGamepad() = delete;
};
