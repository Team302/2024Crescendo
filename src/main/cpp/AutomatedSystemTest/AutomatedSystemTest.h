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

#include "LauncherTest.h"
#include "Placertest.h"
#include "SwerveTest.h"
#include "map"

class AutomatedSystemTest
{
    AutomatedSystemTest();
    ~AutomatedSystemTest() = default;

public:
    void RunAllTests();

private:
    Placertest *placertest;
    LauncherTest *launcherTest;
    SwerveTest *swerveTest;

    enum MOTORS
    {
        BACK_LEFT_DRIVE,
        BACK_LEFT_TURN,
        BACK_RIGHT_DRIVE,
        BACK_RIGHT_TURN,
        FRONT_RIGHT_DRIVE,
        FRONT_RIGHT_TURN,
        FRONT_LEFT_DRIVE,
        FRONT_LEFT_TURN,
        ELEVATOR,
        FRONT_INTAKE,
        BACK_INTAKE,
        TRANSFER,
        PLACER,
        FEEDER,
        LAUNCHER_TOP,
        LAUNCHER_BOTTOM,
        LAUNCHER_ANGLE,
        LEFT_CLIMB,
        RIGHT_CLIMB
    };

    std::map<MOTORS, int> motorsToCan;
    std::map<MOTORS, int> motorsToPDH;
};