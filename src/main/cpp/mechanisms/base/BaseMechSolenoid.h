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
#include "hw/DragonSolenoid.h"
#include "utils/logging/LoggableItem.h"

class BaseMechSolenoid : public LoggableItem
{
public:
    /// @brief A solenoid that can be added to a mechanism
    /// @param [in] DragonSolenoid& solenoid used by this mechanism
    BaseMechSolenoid(std::string networkTableName, DragonSolenoid &solenoid);
    BaseMechSolenoid() = delete;
    virtual ~BaseMechSolenoid() = default;

    /// @brief      Activate/deactivate pneumatic solenoid
    /// @param [in] bool - true == activate, false == deactivate
    void ActivateSolenoid(bool activate);

    /// @brief      Check if the pneumatic solenoid is activated
    /// @return     bool - true == activated, false == deactivated
    bool IsSolenoidActivated() const;

    /// @brief log data to the network table if it is activated and time period has past
    void LogInformation() override;

private:
    std::string m_networkTableName;
    DragonSolenoid m_solenoid;
};
