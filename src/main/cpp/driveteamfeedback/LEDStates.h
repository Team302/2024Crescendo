
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
#include "hw/DragonLeds.h"
#include <frc/DriverStation.h>
#include <utils/FMSData.h>
#include <vector>

class LEDStates
{
public:
    void setLEDsOn();
    void setLEDsOff();
    void ResetVariables();
    void ChaserPattern(DragonLeds::Colors c);
    void BlinkingPattern(DragonLeds::Colors c);
    void SolidColorPattern(DragonLeds::Colors c);
    void AlternatingColorBlinkingPattern(DragonLeds::Colors c);
    void AlternatingColorBlinkingPattern(DragonLeds::Colors c1, DragonLeds::Colors c2);
    void ClosingInChaserPattern(DragonLeds::Colors c);
    void RainbowPattern();
    void DiagnosticPattern(frc::DriverStation::Alliance alliancecolor, bool bintake, bool fintake, bool feeder, bool launcher, bool placerin, bool placermid, bool placerout);
    DragonLeds *m_LEDstring = DragonLeds::GetInstance();
    static LEDStates *GetInstance();

private:
    LEDStates() = default;
    ~LEDStates() = default;

    int loopThroughIndividualLEDs = -1;
    int colorLoop = 0;
    int timer;
    bool switchColor = false;
    std::array<int, 3U> color = m_LEDstring->getColorValues(DragonLeds::BLACK);
    static LEDStates *m_instance;

    const int blinkPatternPeriod = 10;
    const int alternatingColorBlinkPatternPeriod = 10;
};
