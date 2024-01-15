
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

// FRC Includes
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/length.h"
#include "units/velocity.h"

// Team 302 includes
#include "mechanisms/base/BaseMechMotor.h"
#include "hw/DragonDigitalInput.h"
#include "utils/logging/LoggableItem.h"

// forward declares
class IDragonMotorController;
class ControlData;

class BaseMechMotorWithMinLimit : public IBaseMechMotor
{
public:
    /// @brief A motor that can be added to a mechanism
    /// @param [in] std::string the name of the network table for logging information
    /// @param [in] IDragonMotorController& motor controller used by this mechanism
    /// @param [in] EndOfTravelSensorOption minimum end of travel sensor option
    /// @param [in] DragonDigitalInput* minimium end of travel sensor if plugged into RoboRio otherwise ignored
    /// @param [in] EndOfTravelSensorOption maximum end of travel sensor option
    /// @param [in] DragonDigitalInput* maximum end of travel sensor if plugged into RoboRio otherwise ignored
    BaseMechMotorWithMinLimit(IBaseMechMotor &base,
                              IBaseMechMotor::EndOfTravelSensorOption minEndOfTravelOption,
                              DragonDigitalInput *minSensor);
    BaseMechMotorWithMinLimit() = delete;
    ~BaseMechMotorWithMinLimit() = default;

    bool IsAtMinTravel() const;

private:
    IBaseMechMotor::EndOfTravelSensorOption m_minEndOfTravelOption;
    DragonDigitalInput *m_minRoboRioDigital;
};
