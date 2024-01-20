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
#include <memory>
#include <string>

// FRC includes

// Team 302 includes
#include "mechanisms/base/BaseMech.h"
#include "mechanisms/MechanismTypes.h"
#include "utils/logging/Logger.h"

// Third Party Includes
#include "units/time.h"
using namespace std;

/// @brief create the general mechanism
/// @param [in] MechanismTypes::MECHANISM_TYPE the type of mechansim
/// @param [in] std::string the name of the file that will set control parameters for this mechanism
/// @param [in] std::string the name of the network table for logging information
BaseMech::BaseMech(MechanismTypes::MECHANISM_TYPE type,
                   string controlFileName,
                   string networkTableName) : LoggableItem(),
                                              m_type(type),
                                              m_controlFile(controlFileName),
                                              m_ntName(networkTableName)
{
    if (controlFileName.empty())
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, networkTableName, string("Mech"), string("control file name is not specified"));
    }

    if (networkTableName.empty())
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("Mech"), string("Mech"), string("network table name is not specified"));
    }
}
BaseMech::BaseMech(MechanismTypes::MECHANISM_TYPE type) : LoggableItem(),
                                                          m_type(type),
                                                          m_controlFile(),
                                                          m_ntName()
{
}

/// @brief          Indicates the type of mechanism this is
/// @return         MechanismTypes::MECHANISM_TYPE
MechanismTypes::MECHANISM_TYPE BaseMech::GetType() const
{
    return m_type;
}

/// @brief indicate the file used to get the control parameters from
/// @return std::string the name of the file
string BaseMech::GetControlFileName() const
{
    return m_controlFile;
}

/// @brief indicate the network table name used to for logging parameters
/// @return std::string the name of the network table
string BaseMech::GetNetworkTableName() const
{
    return m_ntName;
}

/// @brief log data to the network table if it is activated and time period has past
void BaseMech::LogInformation()
{
    // NO-OP - subclasses override when necessary
}
