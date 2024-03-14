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
// Author: notcharlie wirter of bad code, ben_vs_bread, the omniscent, omnipotent, andomnipresent being, and LukDaDuke the blonde guy;
#include "AutomatedSystemTest.h"
#include "utils\logging\Logger.h"
#include "hw\factories\PDPFactory.h"
#include "string.h"

using std::string;
AutomatedSystemTest::AutomatedSystemTest()
{
    auto PDPpointer = PDPFactory::GetFactory();
    motorsToCan[BACK_LEFT_DRIVE] = 1;
    motorsToCan[BACK_LEFT_TURN] = 1;
    motorsToCan[BACK_RIGHT_DRIVE] = 1;
    motorsToCan[BACK_RIGHT_TURN] = 1;
    motorsToCan[FRONT_RIGHT_DRIVE] = 1;
    motorsToCan[FRONT_RIGHT_TURN] = 1;
    motorsToCan[FRONT_LEFT_DRIVE] = 1;
    motorsToCan[FRONT_LEFT_TURN] = 1;
    motorsToCan[ELEVATOR] = 1;
    motorsToCan[FRONT_INTAKE] = 1;
    motorsToCan[BACK_INTAKE] = 1;
    motorsToCan[TRANSFER] = 1;
    motorsToCan[PLACER] = 1;
    motorsToCan[FEEDER] = 1;
    motorsToCan[LAUNCHER_TOP] = 1;
    motorsToCan[LAUNCHER_BOTTOM] = 1;
    motorsToCan[LAUNCHER_ANGLE] = 1;
    motorsToCan[LEFT_CLIMB] = 1;
    motorsToCan[RIGHT_CLIMB] = 1;

    motorsToPDH[BACK_LEFT_DRIVE] = 1;
    motorsToPDH[BACK_LEFT_TURN] = 1;
    motorsToPDH[BACK_RIGHT_DRIVE] = 1;
    motorsToPDH[BACK_RIGHT_TURN] = 1;
    motorsToPDH[FRONT_RIGHT_DRIVE] = 1;
    motorsToPDH[FRONT_RIGHT_TURN] = 1;
    motorsToPDH[FRONT_LEFT_DRIVE] = 1;
    motorsToPDH[FRONT_LEFT_TURN] = 1;
    motorsToPDH[ELEVATOR] = 1;
    motorsToPDH[FRONT_INTAKE] = 1;
    motorsToPDH[BACK_INTAKE] = 1;
    motorsToPDH[TRANSFER] = 1;
    motorsToPDH[PLACER] = 1;
    motorsToPDH[FEEDER] = 1;
    motorsToPDH[LAUNCHER_TOP] = 1;
    motorsToPDH[LAUNCHER_BOTTOM] = 1;
    motorsToPDH[LAUNCHER_ANGLE] = 1;
    motorsToPDH[LEFT_CLIMB] = 1;
    motorsToPDH[RIGHT_CLIMB] = 1;
}
void AutomatedSystemTest::RunAllTests()
{
}
