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

#include "configs/RobotElementNames.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include <array>

class MotorControlButtons
{

public:
    static MotorControlButtons *GetInstance();
    void Init();
    void Run();
    bool GetMotorEnabled(RobotElementNames::MOTOR_CONTROLLER_USAGE identifier);

private:
    MotorControlButtons() = default;
    ~MotorControlButtons() = default;

    std::array<bool, 11> m_MotorEnabledArray;
    std::array<std::string, 11> MotorKeys = {"NOTE_MANAGER_FRONT_INTAKE", "NOTE_MANAGER_BACK_INTAKE",
                                             "NOTE_MANAGER_TRANSFER", "NOTE_MANAGER_FEEDER", "NOTE_MANAGER_LAUNCHER_TOP",
                                             "NOTE_MANAGER_LAUNCHER_BOTTOM", "NOTE_MANAGER_LAUNCHER_ANGLE",
                                             "NOTE_MANAGER_PLACER", "NOTE_MANAGER_ELEVATOR",
                                             "CLIMBER_MANAGER_LEFT_CLIMBER", "CLIMBER_MANAGER_RIGHT_CLIMBER"};
};