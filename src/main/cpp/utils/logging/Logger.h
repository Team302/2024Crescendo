
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
#include <set>

// FRC includes
#include <frc/smartdashboard/SendableChooser.h>

// Team 302 includes
#include <utils/logging/LoggerData.h>
#include <utils/logging/LoggerEnums.h>

class Logger
{
public:
    /// @brief Find or create the singleton logger
    /// @returns Logger* pointer to the logger
    static Logger *GetLogger();

    /// @brief log a message
    /// @param [in] LOGGER_LEVEL: message level
    /// @param [in] std::string: network table name or classname to group messages.  If logging option is DASHBOARD, this will be the network table name
    /// @param [in] std::string: message identifier: within a grouping multiple messages may be displayed this is the prefix/look up key
    /// @param [in] std::string: message - text of the message
    void LogData(LOGGER_LEVEL level, const std::string &group, const std::string &identifier, const std::string &message) const;
    void LogData(LOGGER_LEVEL level, const std::string &group, const std::string &identifier, const char *message) const;
    void LogData(LoggerData &info) const;

    /// @brief log a message
    /// @param [in] LOGGER_LEVEL: message level
    /// @param [in] std::string: network table name or classname to group messages.  If logging option is DASHBOARD, this will be the network table name
    /// @param [in] std::string: message identifier: within a grouping multiple messages may be displayed this is the prefix/look up key
    /// @param [in] double: value to display
    void LogData(LOGGER_LEVEL level, const std::string &group, const std::string &identifier, double value) const;

    /// @brief log a message
    /// @param [in] LOGGER_LEVEL: message level
    /// @param [in] std::string: network table name or classname to group messages.  If logging option is DASHBOARD, this will be the network table name
    /// @param [in] std::string: message identifier: within a grouping multiple messages may be displayed this is the prefix/look up key
    /// @param [in] bool: value to display
    void LogData(LOGGER_LEVEL level, const std::string &group, const std::string &identifier, bool value) const;

    /// @brief log a message
    /// @param [in] LOGGER_LEVEL: message level
    /// @param [in] std::string: network table name or classname to group messages.  If logging option is DASHBOARD, this will be the network table name
    /// @param [in] std::string: message identifier: within a grouping multiple messages may be displayed this is the prefix/look up key
    /// @param [in] int: value to display
    void LogData(LOGGER_LEVEL level, const std::string &group, const std::string &identifier, int value) const;
    /// @brief Display logging options on dashboard
    void PutLoggingSelectionsOnDashboard();

    /// @brief Read logging option from dashboard, but not every 20ms
    void PeriodicLog();

    /// @brief Logs some data directly to the network tables, so that the rest of logging does not need to be enabled
    void LogDataDirectlyOverNT(const std::string &group, const std::string &identifier, double value) const;

    /// @brief Logs some data directly to the network tables, so that the rest of logging does not need to be enabled
    void LogDataDirectlyOverNT(const std::string &group, const std::string &identifier, bool value) const;

    /// @brief Logs some motor data directly to the network tables, so that the rest of logging does not need to be enabled
    void LogDataDirectlyOverNT(const std::string &group, const std::string &identifier, const std::string &message) const;

protected:
private:
    /// @brief Determines whether a message should be displayed or not.   For instance if EAT_IT is the logging option, this will return false or if the level is xxx_ONCE, it may return false if the message was already logged.
    /// @param [in] LOGGER_LEVEL: message level
    /// @param [in] std::string: network table name or classname to group messages.  If logging option is DASHBOARD, this will be the network table name
    /// @param [in] std::string: message identifier: within a grouping multiple messages may be displayed this is the prefix/look up key
    /// @param [in] std::string: message/value
    /// @returns bool: true - display the message, false - don't display the message
    bool ShouldDisplayIt(LOGGER_LEVEL level, const std::string &group, const std::string &identifier, const std::string &message) const;

    /// @brief set the option for where the logging messages should be displayed
    /// @param [in] LOGGER_OPTION:  logging option for where to log messages
    void SetLoggingOption(
        LOGGER_OPTION option // <I> - Logging option
    );

    /// @brief set the level for messages that will be displayed
    /// @param [in] LOGGER_LEVEL:  logging level for which messages to display
    void SetLoggingLevel(
        LOGGER_LEVEL level // <I> - Logging level
    );

    LOGGER_OPTION m_option; // indicates where the message should go
    LOGGER_LEVEL m_level;   // the level at which a message is important enough to send
    mutable std::set<std::string> m_alreadyDisplayed;
    int m_cyclingCounter; // count 20ms loops
    frc::SendableChooser<LOGGER_OPTION> m_optionChooser;
    frc::SendableChooser<LOGGER_LEVEL> m_levelChooser;

    Logger();
    ~Logger() = default;

    static Logger *m_instance;
};
