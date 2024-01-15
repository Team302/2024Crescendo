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
#include <string>

// FRC Includes
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

// Team 302 Includes
#include <utils/NetworkTableReader.h>
#include "utils/logging/Logger.h"

using namespace std;

NetworkTableReader::NetworkTableReader()
{
}

NetworkTableReader *NetworkTableReader::GetReader()
{
    if (m_reader == nullptr)
    {
        m_reader = new NetworkTableReader();
    }
    return m_reader;
}

string NetworkTableReader::GetNetworkTableString(string ntName, string ntString)
{
    auto table = m_instance.GetTable(ntName);
    if (table != nullptr)
    {
        string value = table.get()->GetString(ntString, "Invalid NT String");
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("NetworkTableReader"), string("GetNetorkTableString"), string("Error accessing NT value, invalid string"));
        return value;
    }
    else
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("NetworkTableReader"), string("GetNetorkTableString"), string("Network Table is a nullptr"));
        return string("Table is nullptr");
    }
}

double NetworkTableReader::GetNetworkTableDouble(string ntName, string ntDouble)
{
    auto table = m_instance.GetTable(ntName);
    if (table != nullptr)
    {
        double value = table.get()->GetNumber(ntDouble, 0);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("NetworkTableReader"), string("GetNetorkTableDouble"), string("Error accessing NT value, invalid number"));
        return value;
    }
    else
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("NetworkTableReader"), string("GetNetorkTableDouble"), string("Network Table is a nullptr"));
        return 0.0;
    }
}