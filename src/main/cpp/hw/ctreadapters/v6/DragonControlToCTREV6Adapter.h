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

// C++ Includes
#include <string>

// Team 302 includes
#include "hw/DistanceAngleCalcStruc.h"
#include "hw/interfaces/IDragonControlToVendorControlAdapter.h"
#include "hw/DragonTalonFX.h"
#include "mechanisms/controllers/ControlData.h"

#include "ctre/phoenix6/TalonFX.hpp"

class DragonControlToCTREV6Adapter : public IDragonControlToVendorControlAdapter
{
public:
    DragonControlToCTREV6Adapter() = delete;
    DragonControlToCTREV6Adapter(std::string networkTableName,
                                  int controllerSlot,
                                  const ControlData &controlInfo,
                                  const DistanceAngleCalcStruc &calcStruc,
                                  ctre::phoenix6::hardware::TalonFX &controller);
    void InitializeDefaults() override;
    std::string GetErrorPrompt() const;

protected:
    void SetPeakAndNominalValues(std::string networkTableName,
                                 const ControlData &controlInfo);

    void SetMaxVelocityAcceleration(std::string networkTableName,
                                    const ControlData &controlInfo);

    void SetPIDConstants(std::string networkTableName,
                         int controllerSlot,
                         const ControlData &controlInfo);

    std::string m_networkTableName;
    int m_controllerSlot;
    ControlData m_controlData;
    DistanceAngleCalcStruc m_calcStruc;
    ctre::phoenix6::hardware::TalonFX &m_controller;
    bool m_isDuty;
    double m_dutyFeedForward;
    bool m_isVoltage;
    units::voltage::volt_t m_voltageFeedForward;
    bool m_isTorque;
    units::current::ampere_t m_torqueCurrentFeedForward;
    bool m_enableFOC;
};
