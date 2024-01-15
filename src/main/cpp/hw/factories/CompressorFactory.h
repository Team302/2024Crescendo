
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

// FRC includes
#include <frc/Compressor.h>
#include <units/pressure.h>
#include <frc/PneumaticHub.h>
#include <frc/PneumaticsControlModule.h>

class CompressorFactory
{
public:
    static CompressorFactory *GetFactory();

    frc::Compressor *GetCompressor() const { return m_compressor; };
    frc::Compressor *CreateCompressor(int canID, frc::PneumaticsModuleType type, units::pressure::pounds_per_square_inch_t minPressure, units::pressure::pounds_per_square_inch_t maxPressure);

    void ToggleEnableCompressor();

    units::pounds_per_square_inch_t GetMinPressure() const { return m_minPressure; }
    units::pounds_per_square_inch_t GetMaxPressure() const { return m_maxPressure; }
    units::pounds_per_square_inch_t GetCurrentPressure() const;

private:
    void EnableCompressor();
    void DisableCompressor();

    void ClearStickyFaults();
    CompressorFactory();
    virtual ~CompressorFactory() = default;

    frc::Compressor *m_compressor;
    units::pressure::pounds_per_square_inch_t m_minPressure;
    units::pressure::pounds_per_square_inch_t m_maxPressure;
    frc::PneumaticHub *m_hub;
    frc::PneumaticsControlModule *m_pcm;

    static CompressorFactory *m_factory;
};