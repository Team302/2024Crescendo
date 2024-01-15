
//====================================================================================================================================================
// Copyright 2010 Lake Orion Robotics FIRST Team 302
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
#include <map>
#include <string>

#include "units/angle.h"

// Team 302 includes
#include <configs/usages/ServoUsage.h>

class DragonServo;

/// @class DragonServoFactory
/// @brief Create DragonServos and allow external clients to retrieve created DragonServos
class DragonServoFactory
{
public:
    /// @brief  Get the factory singleton
    /// @return DragonServoFactory*    pointer to the factory
    static DragonServoFactory *GetInstance();

    /// @brief  Create a DragonServo from the inputs
    /// @param [in] DragonServo::SERVO_USAGE   deviceUsage  Usage of the servo
    /// @param [in] int                        deviceID     PWM ID of the  servo
    /// @param [in] double                     minAngle     Minimum Angle for the servo
    /// @param [in] double                     maxAngle     Maximum Angle for the servo
    /// @return DragonServo*    - could be nullptr if invalid inputs are supplied
    DragonServo *CreateDragonServo(
        std::string networkTableName,
        ServoUsage::SERVO_USAGE deviceUsage,
        int deviceID,
        units::angle::degree_t minAngle,
        units::angle::degree_t maxAngle);

    /// @brief  Get a DragonServo from its usage
    /// @param [in] DragonServo::SERVO_USAGE   deviceUsage  Usage of the servo
    /// @return DragonServo*    - could be nullptr if invalid inputs are supplied
    DragonServo *GetDragonServo(
        ServoUsage::SERVO_USAGE deviceUsage);

private:
    DragonServoFactory() = default;
    ~DragonServoFactory() = default;

    static DragonServoFactory *m_instance;

    std::map<ServoUsage::SERVO_USAGE, DragonServo *> m_servos;
};
