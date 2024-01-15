
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

// Team 302 includes
#include "hw/ctreadapters/v6/DragonControlToCTREV6Adapter.h"
#include "hw/ctreadapters/v6/DragonPercentOutputToCTREV6Adapter.h"
#include "mechanisms/controllers/ControlData.h"
#include "mechanisms/controllers/ControlModes.h"

// Third Party Includes
#include "units/dimensionless.h"
#include "units/voltage.h"

using ctre::phoenix6::controls::DutyCycleOut;
using ctre::phoenix6::controls::VoltageOut;
using ctre::phoenix6::hardware::TalonFX;
using std::string;

DragonPercentOutputToCTREV6Adapter::DragonPercentOutputToCTREV6Adapter(string networkTableName,
                                                                         int controllerSlot,
                                                                         const ControlData &controlInfo,
                                                                         const DistanceAngleCalcStruc &calcStruc,
                                                                         ctre::phoenix6::hardware::TalonFX &controller) : DragonControlToCTREV6Adapter(networkTableName, controllerSlot, controlInfo, calcStruc, controller)
{
}
void DragonPercentOutputToCTREV6Adapter::Set(double value)
{
    if (m_isDuty)
    {
        DutyCycleOut out{value};
        out.WithEnableFOC(m_enableFOC);
        m_controller.SetControl(out);
    }
    else if (m_isVoltage)
    {
        VoltageOut out{units::volt_t(value)};
        out.WithEnableFOC(m_enableFOC);
        m_controller.SetControl(out);
    }
}

void DragonPercentOutputToCTREV6Adapter::SetControlConstants(int controlSlot,
                                                              const ControlData &controlInfo)
{
    // NO-OP
}
