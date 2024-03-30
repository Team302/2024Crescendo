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

// FRC Includes
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>

// Team 302 Includes
#include <utils/FMSData.h>

FMSData *FMSData::m_instance = nullptr;

FMSData *FMSData::GetInstance()
{
    if (FMSData::m_instance == nullptr)
    {
        FMSData::m_instance = new FMSData();
    }
    return FMSData::m_instance;
}

FMSData::FMSData() : m_allianceColorChooser(),
                     m_hasFMS(false),
                     m_color(frc::DriverStation::Alliance::kRed),
                     m_polledFMS(false)
{
    m_allianceColorChooser.SetDefaultOption("Blue Alliance", "Blue");
    m_allianceColorChooser.AddOption("Red Alliance", "Red");

    frc::SmartDashboard::PutData("Alliance If NO FMS", &m_allianceColorChooser);
}

frc::DriverStation::Alliance FMSData::GetAllianceColor()
{
    if (!m_polledFMS)
    {
        CheckForFMS();

        if (m_hasFMS)
        {
            m_polledFMS = frc::DriverStation::GetAlliance().has_value();
            if (m_polledFMS)
            {
                m_color = frc::DriverStation::GetAlliance().value();
            }
            else
            {
                m_color = frc::DriverStation::Alliance::kRed;
            }
            m_polledFMS = true;
        }
        else
        {
            if (m_allianceColorChooser.GetSelected() == "Red")
            {
                m_color = frc::DriverStation::Alliance::kRed;
            }
            else if (m_allianceColorChooser.GetSelected() == "Blue")
            {
                m_color = frc::DriverStation::Alliance::kBlue;
            }
        }
    }

    return m_color;
}

void FMSData::CheckForFMS()
{
    if (frc::DriverStation::IsFMSAttached() && !m_hasFMS)
    {
        m_hasFMS = true;
    }
}
