
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

#include "hw/DragonLeds.h"
#include "utils/logging/Logger.h"

#include <span>
#include <string>

void DragonLeds::Initialize(int PWMport, int numLeds)
{
    if (!IsInitialized())
    {
        m_addressibleLeds = new frc::AddressableLED(PWMport);
        m_addressibleLeds->SetLength(numLeds);

        m_ledBuffer.resize(numLeds);

        setBufferAllLEDsColor(getColorValues(Colors::GREEN));
        commitLedData();
        setOn();
    }
    else
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, std::string("DragonLeds"), std::string("Already defined"), std::string("Only one allowed"));
    }
}

bool DragonLeds::IsInitialized() const
{
    return m_addressibleLeds != nullptr;
}

void DragonLeds::setOn()
{
    m_addressibleLeds->Start();
}

void DragonLeds::setOff()
{
    m_addressibleLeds->Stop();
}

DragonLeds::DragonLeds() : m_addressibleLeds()
{
}

DragonLeds *DragonLeds::m_instance = nullptr;
DragonLeds *DragonLeds::GetInstance()
{
    if (m_instance == nullptr)
    {
        m_instance = new DragonLeds();
    }
    return m_instance;
}

std::array<int, 3> DragonLeds::getColorValues(Colors c)
{
    switch (c)
    {
    case RED:
        return {255, 0, 0};
    case GREEN:
        return {0, 255, 0};
    case BLUE:
        return {0, 0, 255};
    case YELLOW:
        return {255, 160, 0};
    case PURPLE:
        return {75, 0, 130};
    case AZUL:
        return {0, 255, 255};
    case WHITE:
        return {255, 255, 180};
    case BLACK:
        return {0, 0, 0};
    default:
        return {0, 0, 0};
    }
}

void DragonLeds::commitLedData()
{
    if (m_ledBuffer.size() > 0)
    {
        std::span ledSpan{m_ledBuffer.data(), m_ledBuffer.size()};
        m_addressibleLeds->SetData(ledSpan);
    }
}

void DragonLeds::setBufferAllLEDsColor(std::array<int, 3> color)
{
    for (unsigned int i = m_numberofDiagnosticLEDs; i < m_ledBuffer.size(); i++)
    {
        m_ledBuffer[i].SetRGB(color[0], color[1], color[2]);
    }
}

void DragonLeds::setBufferAllLEDsAlternatingColor(std::array<int, 3> color1, std::array<int, 3> color2)
{
    for (unsigned int i = m_numberofDiagnosticLEDs; i < m_ledBuffer.size(); i++)
    {
        if (i % 2 == 0)
            m_ledBuffer[i].SetRGB(color1[0], color1[1], color1[2]);
        else
            m_ledBuffer[i].SetRGB(color2[0], color2[1], color2[2]);
    }
}

void DragonLeds::setBufferAllLEDsRainbow()
{
    for (unsigned int i = m_numberofDiagnosticLEDs; i < m_ledBuffer.size(); i++)
    {
        auto pixelHue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.size())) % 180;
        m_ledBuffer[i].SetHSV(pixelHue, 255, 128);
    }
    m_rainbowFirstPixelHue += 3;
    m_rainbowFirstPixelHue %= 180;
}

void DragonLeds::setSpecificLED(int id, std::array<int, 3> color)
{
    m_ledBuffer[id].SetRGB(color[0], color[1], color[2]);
}

void DragonLeds::setBufferAllLEDsBlack()
{
    setBufferAllLEDsColor(getColorValues(BLACK));
}
