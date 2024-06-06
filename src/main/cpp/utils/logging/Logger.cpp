
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

//========================================================================================================
/// Logger.cpp
//========================================================================================================
///
/// File Description:
///     This logs error messages
///
//========================================================================================================

// C++ Includes
#include <algorithm>
#include <iostream>
#include <locale>
#include <string>

// FRC includes
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>

// Team 302 includes
#include "utils/logging/Logger.h"

// Third Party Includes

using namespace frc;
using namespace std;

/// @brief Find or create the singleton logger
/// @returns Logger* pointer to the logger
Logger *Logger::m_instance = nullptr;
Logger *Logger::GetLogger()
{
    if (Logger::m_instance == nullptr)
    {
        Logger::m_instance = new Logger();
    }
    return Logger::m_instance;
}

/// @brief log a message
/// @param [in] LOGGER_LEVEL: message level
/// @param [in] std::string: network table name or classname to group messages.  If logging option is DASHBOARD, this will be the network table name
/// @param [in] std::string: message identifier: within a grouping multiple messages may be displayed this is the prefix/look up key
/// @param [in] std::string: message/value
void Logger::LogData(LOGGER_LEVEL level, const string &group, const string &identifier, const string &message) const
{
    if (ShouldDisplayIt(level, group, identifier, message))
    {
        switch (m_option)
        {
        case LOGGER_OPTION::CONSOLE:
        {
            cout << group << " " << identifier << ": " << message << endl;
        }
        break;

        case LOGGER_OPTION::DASHBOARD:
        {
            auto table = nt::NetworkTableInstance::GetDefault().GetTable(group);
            table.get()->PutString(identifier, message);
        }
        break;

        default: // case LOGGER_OPTION::EAT_IT:
            break;
        }
    }
}

void Logger::LogData(LOGGER_LEVEL level, const string &group, const string &identifier, const char *message) const
{
    LogData(level, group, identifier, string(message));
}
/// @brief log a message
/// @param [in] LOGGER_LEVEL: message level
/// @param [in] std::string: network table name or classname to group messages.  If logging option is DASHBOARD, this will be the network table name
/// @param [in] std::string: message identifier: within a grouping multiple messages may be displayed this is the prefix/look up key
/// @param [in] double: value to display
void Logger::LogData(LOGGER_LEVEL level, const string &group, const string &identifier, double value) const
{
    if (ShouldDisplayIt(level, group, identifier, to_string(value)))
    {
        switch (m_option)
        {
        case LOGGER_OPTION::CONSOLE:
        {
            cout << group << " " << identifier << ": " << to_string(value) << endl;
        }
        break;

        case LOGGER_OPTION::DASHBOARD:
        {
            auto table = nt::NetworkTableInstance::GetDefault().GetTable(group);
            table.get()->PutNumber(identifier, value);
        }
        break;

        default: // case LOGGER_OPTION::EAT_IT:
            break;
        }
    }
}

/// @brief log a message
/// @param [in] LOGGER_LEVEL: message level
/// @param [in] std::string: network table name or classname to group messages.  If logging option is DASHBOARD, this will be the network table name
/// @param [in] std::string: message identifier: within a grouping multiple messages may be displayed this is the prefix/look up key
/// @param [in] bool: value to display
void Logger::LogData(LOGGER_LEVEL level, const std::string &group, const std::string &identifier, bool value) const
{
    if (ShouldDisplayIt(level, group, identifier, to_string(value)))
    {
        switch (m_option)
        {
        case LOGGER_OPTION::CONSOLE:
        {
            cout << group << " " << identifier << ": " << to_string(value) << endl;
        }
        break;

        case LOGGER_OPTION::DASHBOARD:
        {
            auto table = nt::NetworkTableInstance::GetDefault().GetTable(group);
            table.get()->PutBoolean(identifier, value);
        }
        break;

        default: // case LOGGER_OPTION::EAT_IT:
            break;
        }
    }
}

/// @brief log a message
/// @param [in] LOGGER_LEVEL: message level
/// @param [in] std::string: network table name or classname to group messages.  If logging option is DASHBOARD, this will be the network table name
/// @param [in] std::string: message identifier: within a grouping multiple messages may be displayed this is the prefix/look up key
/// @param [in] int: value to display
void Logger::LogData(LOGGER_LEVEL level, const std::string &group, const std::string &identifier, int value) const
{
    if (ShouldDisplayIt(level, group, identifier, to_string(value)))
    {
        switch (m_option)
        {
        case LOGGER_OPTION::CONSOLE:
        {
            cout << group << " " << identifier << ": " << to_string(value) << endl;
        }
        break;

        case LOGGER_OPTION::DASHBOARD:
        {
            auto table = nt::NetworkTableInstance::GetDefault().GetTable(group);
            table.get()->PutNumber(identifier, value);
        }
        break;

        default: // case LOGGER_OPTION::EAT_IT:
            break;
        }
    }
}

void Logger::LogData(LoggerData &info) const
{
    for (auto boollog : info.bools)
    {
        LogData(info.level, info.group, boollog.first, boollog.second);
    }
    for (auto doublelog : info.doubles)
    {
        LogData(info.level, info.group, doublelog.first, doublelog.second);
    }
    for (auto intlog : info.ints)
    {
        LogData(info.level, info.group, intlog.first, intlog.second);
    }
    for (auto stringlog : info.strings)
    {
        LogData(info.level, info.group, stringlog.first, stringlog.second);
    }
}

/// @brief Determines whether a message should be displayed or not.   For instance if EAT_IT is the logging option, this will return false or if the level is xxx_ONCE, it may return false if the message was already logged.
/// @param [in] LOGGER_LEVEL: message level
/// @param [in] std::string: network table name or classname to group messages.  If logging option is DASHBOARD, this will be the network table name
/// @param [in] std::string: message identifier: within a grouping multiple messages may be displayed this is the prefix/look up key
/// @param [in] std::string: message/value
/// @returns bool: true - display the message, false - don't display the message
bool Logger::ShouldDisplayIt(LOGGER_LEVEL level, const string &group, const string &identifier, const string &message) const
{
    if (m_option == LOGGER_OPTION::EAT_IT)
    {
        return false;
    }
    // If the error level is *_ONCE, display it only the first time it happens
    if ((level == ERROR_ONCE) || (level == WARNING_ONCE) || (level == PRINT_ONCE))
    {
        string key = group + identifier + message;
        auto it = m_alreadyDisplayed.find(key);
        if (it == m_alreadyDisplayed.end()) // display if not already displayed
        {
            m_alreadyDisplayed.insert(key); // save the key
            return true;
        }
        return false;
    }
    return true;
}

/// @brief Display/select logging options/levels on dashboard
void Logger::PutLoggingSelectionsOnDashboard()
{
    // set up option menu
    m_optionChooser.SetDefaultOption("EAT_IT", LOGGER_OPTION::EAT_IT);
    m_optionChooser.AddOption("DASHBOARD", LOGGER_OPTION::DASHBOARD);
    m_optionChooser.AddOption("CONSOLE", LOGGER_OPTION::CONSOLE);
    frc::SmartDashboard::PutData("Logging Options", &m_optionChooser);

    // set up level menu
    m_levelChooser.SetDefaultOption("ERROR_ONCE", LOGGER_LEVEL::ERROR_ONCE);
    m_levelChooser.AddOption("ERROR", LOGGER_LEVEL::ERROR);
    m_levelChooser.AddOption("WARNING_ONCE", LOGGER_LEVEL::WARNING_ONCE);
    m_levelChooser.AddOption("WARNING", LOGGER_LEVEL::WARNING);
    m_levelChooser.AddOption("PRINT_ONCE", LOGGER_LEVEL::PRINT_ONCE);
    m_levelChooser.AddOption("PRINT", LOGGER_LEVEL::PRINT);
    frc::SmartDashboard::PutData("Logging Levels", &m_levelChooser);

    auto table = nt::NetworkTableInstance::GetDefault().GetTable("ArrivedAt");
    auto keys = table->GetKeys(0);
    for (auto key : keys)
    {
        table.get()->PutString(key, string("N/A"));
    }

    m_cyclingCounter = 0;
}

/// @brief Read logging option from dashboard, but not every 20ms
void Logger::PeriodicLog()
{
    m_cyclingCounter += 1;      // count 20ms loops
    if (m_cyclingCounter >= 25) // execute every 500ms
    {
        m_cyclingCounter = 0;

        //
        // Check for a new option selection
        //
        LOGGER_OPTION selectedOption = m_optionChooser.GetSelected();

        if (selectedOption != m_option)
        {
            // re-work so we aren't writing this out every 25 loops
            m_option = selectedOption <= LOGGER_OPTION::EAT_IT ? selectedOption : LOGGER_OPTION::EAT_IT;
            string optionAsString;
            switch (selectedOption)
            {
            case CONSOLE:
                optionAsString.assign("CONSOLE");
                break;

            case DASHBOARD:
                optionAsString.assign("DASHBOARD");
                break;

            case EAT_IT:
                optionAsString.assign("EAT_IT");
                break;

            default:
                optionAsString.assign("Out of range !");
                m_option = EAT_IT;
                break;
            }
            LogData(LOGGER_LEVEL::PRINT, string("Logger"), string("Selected Option"), optionAsString);
        }

        //
        // Check for a new level selection
        //
        LOGGER_LEVEL selectedLevel = m_levelChooser.GetSelected();

        if (selectedLevel != m_level)
        {
            // re-work so we aren't writing this out every 25 loops
            m_level = selectedLevel <= LOGGER_LEVEL::PRINT ? selectedLevel : LOGGER_LEVEL::WARNING;

            string levelAsString;
            switch (selectedLevel)
            {
            case ERROR_ONCE:
                levelAsString.assign("ERROR_ONCE");
                break;

            case ERROR:
                levelAsString.assign("ERROR");
                break;

            case WARNING_ONCE:
                levelAsString.assign("WARNING_ONCE");
                break;

            case WARNING:
                levelAsString.assign("WARNING");
                break;

            case PRINT_ONCE:
                levelAsString.assign("PRINT_ONCE");
                break;

            case PRINT:
                levelAsString.assign("PRINT");
                break;

            default:
                levelAsString.assign("Out of range !");
                m_level = WARNING;
                break;
            }
            LogData(LOGGER_LEVEL::PRINT, string("Logger"), string("Selected Level"), levelAsString);
        }
    }
}

/// @brief set the option for where the logging messages should be displayed
/// @param [in] LOGGER_OPTION:  logging option for where to log messages
void Logger::SetLoggingOption(
    LOGGER_OPTION option)
{
    m_option = option;
}

/// @brief set the level for messages that will be displayed
/// @param [in] LOGGER_LEVEL:  logging level for which messages to display
void Logger::SetLoggingLevel(
    LOGGER_LEVEL level)
{
    m_level = level;
}

Logger::Logger() : m_option(LOGGER_OPTION::DASHBOARD),
                   m_level(LOGGER_LEVEL::PRINT),
                   m_alreadyDisplayed(),
                   m_cyclingCounter(0),
                   m_optionChooser(),
                   m_levelChooser()
{
}

/// @brief log a message
/// @param [in] std::string: network table name or classname to group messages.
/// @param [in] std::string: message identifier: within a grouping multiple messages may be displayed this is the prefix/look up key
/// @param [in] double: value to display
void Logger::LogDataDirectlyOverNT(const string &group, const string &identifier, double value) const
{
    auto table = nt::NetworkTableInstance::GetDefault().GetTable(group);
    table.get()->PutNumber(identifier, value);
}

/// @brief log a message
/// @param [in] std::string: network table name or classname to group messages.
/// @param [in] std::string: message identifier: within a grouping multiple messages may be displayed this is the prefix/look up key
/// @param [in] bool: value to display
void Logger::LogDataDirectlyOverNT(const string &group, const string &identifier, bool value) const
{
    auto table = nt::NetworkTableInstance::GetDefault().GetTable(group);
    table.get()->PutBoolean(identifier, value);
}

/// @brief log a message
/// @param [in] std::string: network table name or classname to group messages.
/// @param [in] std::string: message identifier: within a grouping multiple messages may be displayed this is the prefix/look up key
/// @param [in] double: value to display
void Logger::LogDataDirectlyOverNT(const string &group, const string &identifier, const std::string &message) const
{
    auto table = nt::NetworkTableInstance::GetDefault().GetTable(group);
    table.get()->PutString(identifier, message);
}
