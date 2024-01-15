
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

// FRC includes
#include "units/angle.h"

// Team 302 includes
#include "mechanisms/base/BaseMech.h"
#include "mechanisms/MechanismTypes.h"
#include "mechanisms/base/StateMgr.h"

// forward declares
class DragonServo;

class BaseMechServo : public LoggableItem
{
public:
    /// @brief Create a generic mechanism wiht 1 servo
    /// @param [in] std::shared_ptr<DragonServo> servo used by this mechanism
    BaseMechServo(std::string networkTableName, DragonServo *servo);
    BaseMechServo() = delete;
    virtual ~BaseMechServo() = default;

    /// @brief      Move servo to the desired angle
    /// @param [in] double angle: Target angle in degrees
    /// @return     void
    void SetAngle(units::angle::degree_t angle);

    units::angle::degree_t GetAngle() const;

    /// @brief log data to the network table if it is activated and time period has past
    void LogInformation() override;

private:
    DragonServo *m_servo;
};
