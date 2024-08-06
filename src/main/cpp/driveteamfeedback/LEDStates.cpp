
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

#include <driveteamfeedback/LEDStates.h>
#include <span>

void LEDStates::BlinkingPattern(DragonLeds::Colors c)
{
    if (m_LEDstring->m_ledBuffer.size() > 0)
    {
        if (timer > blinkPatternPeriod)
            timer = 0;

        int blinkState = (timer / blinkPatternPeriod) % 2;

        if (blinkState == 0)
            m_LEDstring->setBufferAllLEDsColor(m_LEDstring->getColorValues(c));
        else
            m_LEDstring->setBufferAllLEDsBlack();

        m_LEDstring->commitLedData();

        timer++;
    }
}

void LEDStates::SolidColorPattern(DragonLeds::Colors c)
{
    m_LEDstring->setBufferAllLEDsColor(m_LEDstring->getColorValues(c));
    m_LEDstring->commitLedData();
}

void LEDStates::AlternatingColorBlinkingPattern(DragonLeds::Colors c)
{
    AlternatingColorBlinkingPattern(c, m_LEDstring->BLACK);
}

void LEDStates::AlternatingColorBlinkingPattern(DragonLeds::Colors c1, DragonLeds::Colors c2)
{
    if (m_LEDstring->m_ledBuffer.size() > 0)
    {
        if (timer > 2 * alternatingColorBlinkPatternPeriod)
            timer = 0;

        int blinkState = (timer / blinkPatternPeriod) % 2;

        if (blinkState == 0)
            m_LEDstring->setBufferAllLEDsAlternatingColor(m_LEDstring->getColorValues(c1), m_LEDstring->getColorValues(c2));
        else
            m_LEDstring->setBufferAllLEDsAlternatingColor(m_LEDstring->getColorValues(c2), m_LEDstring->getColorValues(c1));

        m_LEDstring->commitLedData();

        timer++;
    }
}

void LEDStates::ClosingInChaserPattern(DragonLeds::Colors c)
{
    if (m_LEDstring->m_ledBuffer.size() > 0)
    {
        if (timer == 7)
        {
            int halfLength = (m_LEDstring->m_ledBuffer.size() - 1) / 2;
            loopThroughIndividualLEDs += loopThroughIndividualLEDs < halfLength ? 1 : -loopThroughIndividualLEDs;
            int loopout = (m_LEDstring->m_ledBuffer.size() - 1) - loopThroughIndividualLEDs;
            auto color = colorLoop >= 0 ? m_LEDstring->getColorValues(c) : m_LEDstring->getColorValues(m_LEDstring->BLACK);
            colorLoop += colorLoop < halfLength ? 1 : -((colorLoop * 2) + 1);
            m_LEDstring->m_ledBuffer[loopThroughIndividualLEDs].SetRGB(color[0], color[1], color[2]);
            m_LEDstring->m_ledBuffer[loopout].SetRGB(color[0], color[1], color[2]);

            m_LEDstring->commitLedData();
            timer = 0;
        }
        timer++;
    }
}

void LEDStates::ChaserPattern(DragonLeds::Colors c)
{
    if (m_LEDstring->m_ledBuffer.size() > 0)
    {
        loopThroughIndividualLEDs += loopThroughIndividualLEDs < static_cast<int>(m_LEDstring->m_ledBuffer.size()) - 1 ? 1 : -loopThroughIndividualLEDs;
        if (!switchColor)
        {
            color = color == m_LEDstring->getColorValues(c) ? m_LEDstring->getColorValues(DragonLeds::BLACK) : m_LEDstring->getColorValues(c);
        }
        // auto color = colorLoop >= 0 ? m_LEDstring->getColorValues(c) : m_LEDstring->getColorValues(DragonLeds::BLACK);

        // colorLoop += colorLoop < m_LEDstring->m_ledBuffer.size() - 1 ? 1 : -((colorLoop * 2) + 1);
        switchColor = loopThroughIndividualLEDs != static_cast<int>(m_LEDstring->m_ledBuffer.size()) - 1;

        m_LEDstring->m_ledBuffer[loopThroughIndividualLEDs].SetRGB(color[0], color[1], color[2]);
        m_LEDstring->commitLedData();
    }
}

void LEDStates::RainbowPattern()
{
    m_LEDstring->setBufferAllLEDsRainbow();
    m_LEDstring->commitLedData();
}

void LEDStates::DiagnosticPattern(frc::DriverStation::Alliance alliancecolor, bool bintake, bool fintake, bool feeder, bool launcher, bool placerin, bool placermid, bool placerout)
{
    if (alliancecolor == frc::DriverStation::Alliance::kBlue)
    {
        m_LEDstring->setSpecificLED(0, m_LEDstring->getColorValues(DragonLeds::BLUE));
    }
    else
    {
        m_LEDstring->setSpecificLED(0, m_LEDstring->getColorValues(DragonLeds::RED));
    }
    if (fintake)
    {
        m_LEDstring->setSpecificLED(1, m_LEDstring->getColorValues(DragonLeds::YELLOW));
    }
    else
    {
        m_LEDstring->setSpecificLED(1, m_LEDstring->getColorValues(DragonLeds::BLACK));
    }
    if (bintake)
    {
        m_LEDstring->setSpecificLED(2, m_LEDstring->getColorValues(DragonLeds::YELLOW));
    }
    else
    {
        m_LEDstring->setSpecificLED(2, m_LEDstring->getColorValues(DragonLeds::BLACK));
    }
    if (feeder)
    {
        m_LEDstring->setSpecificLED(3, m_LEDstring->getColorValues(DragonLeds::YELLOW));
    }
    else
    {
        m_LEDstring->setSpecificLED(3, m_LEDstring->getColorValues(DragonLeds::BLACK));
    }
    if (launcher)
    {
        m_LEDstring->setSpecificLED(4, m_LEDstring->getColorValues(DragonLeds::YELLOW));
    }
    else
    {
        m_LEDstring->setSpecificLED(4, m_LEDstring->getColorValues(DragonLeds::BLACK));
    }
    if (placerin)
    {
        m_LEDstring->setSpecificLED(5, m_LEDstring->getColorValues(DragonLeds::YELLOW));
    }
    else
    {
        m_LEDstring->setSpecificLED(5, m_LEDstring->getColorValues(DragonLeds::BLACK));
    }
    if (placermid)
    {
        m_LEDstring->setSpecificLED(6, m_LEDstring->getColorValues(DragonLeds::YELLOW));
    }
    else
    {
        m_LEDstring->setSpecificLED(6, m_LEDstring->getColorValues(DragonLeds::BLACK));
    }
    if (placerout)
    {
        m_LEDstring->setSpecificLED(7, m_LEDstring->getColorValues(DragonLeds::YELLOW));
    }
    else
    {
        m_LEDstring->setSpecificLED(7, m_LEDstring->getColorValues(DragonLeds::BLACK));
    }
    m_LEDstring->commitLedData();
}

void LEDStates::ResetVariables()
{
    loopThroughIndividualLEDs = -1;
    colorLoop = 0;
    timer = 0;
    switchColor = false;
}

LEDStates *LEDStates::m_instance = nullptr;
LEDStates *LEDStates::GetInstance()
{
    if (LEDStates::m_instance == nullptr)
    {
        LEDStates::m_instance = new LEDStates();
    }
    return LEDStates::m_instance;
}

void LEDStates::setLEDsOff()
{
    m_LEDstring->setOff();
}

void LEDStates::setLEDsOn()
{
    m_LEDstring->setOn();
}