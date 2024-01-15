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
#include "hw/ctreadapters/v5/DragonControlToCTREV5Adapter.h"

namespace ctre
{
    namespace phoenix
    {
        namespace motorcontrol
        {
            namespace can
            {
                class TalonSRX;
            }
        }
    }
}
class ControlData;

class DragonPercentOutputToCTREV5Adapter : public DragonControlToCTREV5Adapter
{
public:
    DragonPercentOutputToCTREV5Adapter() = delete;
    DragonPercentOutputToCTREV5Adapter(std::string networkTableName,
                                       int controllerSlot,
                                       const ControlData &controlInfo,
                                       const DistanceAngleCalcStruc &calcStruc,
                                       ctre::phoenix::motorcontrol::can::TalonSRX *controller);

    ~DragonPercentOutputToCTREV5Adapter() = default;

    void Set(double value) override;

    void SetControlConstants(int controlSlot,
                             const ControlData &controlInfo) override;
};
