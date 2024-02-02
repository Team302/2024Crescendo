
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

// C++ Includes

// FRC includes
#include "frc/Compressor.h"
#include "units/pressure.h"

// Team 302 includes
#include "hw/factories/CompressorFactory.h"

// Third Party Includes

using frc::Compressor;

CompressorFactory *CompressorFactory::m_factory = nullptr;
CompressorFactory *CompressorFactory::GetFactory()
{
    if (CompressorFactory::m_factory == nullptr)
    {
        CompressorFactory::m_factory = new CompressorFactory();
    }
    return CompressorFactory::m_factory;
}

CompressorFactory::CompressorFactory() : m_compressor(nullptr), m_minPressure(units::pounds_per_square_inch_t(0.0)), m_maxPressure(units::pounds_per_square_inch_t(0.0)), m_hub(nullptr), m_pcm(nullptr)
{
}

Compressor *CompressorFactory::CreateCompressor(int canID, frc::PneumaticsModuleType type, units::pressure::pounds_per_square_inch_t minPressure, units::pressure::pounds_per_square_inch_t maxPressure)
{
    if (m_compressor == nullptr)
    {
        m_compressor = new Compressor(canID, type);
        m_minPressure = minPressure;
        m_maxPressure = maxPressure;
        m_compressor->EnableAnalog(minPressure, maxPressure);
        if (type == frc::PneumaticsModuleType::CTREPCM)
        {
            m_pcm = new frc::PneumaticsControlModule(canID);
        }
        else
        {
            m_hub = new frc::PneumaticHub(canID);
        }
    }
    return m_compressor;
}

void CompressorFactory::ToggleEnableCompressor()
{
    if (m_compressor != nullptr)
    {
        ClearStickyFaults();
        if (m_compressor->IsEnabled())
        {
            DisableCompressor();
        }
        else
        {
            EnableCompressor();
        }
    }
}
void CompressorFactory::EnableCompressor()
{
    m_compressor->EnableAnalog(m_minPressure, m_maxPressure);
}

void CompressorFactory::DisableCompressor()
{
    m_compressor->Disable();
}

void CompressorFactory::ClearStickyFaults()
{
    if (m_hub != nullptr)
    {
        m_hub->ClearStickyFaults();
    }
    if (m_pcm != nullptr)
    {
        m_pcm->ClearAllStickyFaults();
    }
}

units::pounds_per_square_inch_t CompressorFactory::GetCurrentPressure() const
{
    if (m_compressor != nullptr)
    {
        return m_compressor->GetPressure();
    }
    return units::pounds_per_square_inch_t(0.0);
}
